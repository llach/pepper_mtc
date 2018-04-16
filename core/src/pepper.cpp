#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <stages/mirror_grasp_generator.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pepper_mtc_msgs/PepperGrasping.h>


using namespace moveit::task_constructor;

// keep global reference, so service calls from rviz plugin
// work even after the service call task trigger is done
std::shared_ptr<Task> task_ptr_;
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> action_client_ptr;

void fillTask(Stage* initial_stage, std::string object_name) {

    // some robot specific variables
    std::string tool_frame_left = "l_grasp_frame";
    std::string tool_frame_right = "r_grasp_frame";

    std::string eef_left = "left_hand";
    std::string eef_right = "right_hand";

    std::string arm_left = "left_arm";
    std::string arm_right = "right_arm";

    auto task = task_ptr_.get();

    // planner used for connect
    auto pipeline = std::make_shared<solvers::PipelinePlanner>();
    pipeline->setTimeout(8.0);
    pipeline->setPlannerId("RRTConnectkConfigDefault");

    // connect to pick
    stages::Connect::GroupPlannerVector planners = {{eef_left, pipeline}, {arm_left, pipeline},
                                                    {eef_right, pipeline}, {arm_right, pipeline}};
    auto connect = std::make_unique<stages::Connect>("connect", planners);
    connect->properties().configureInitFrom(Stage::PARENT);

    // grasp stage to be wrapped in another grasp stage
    auto simple_grasp = std::make_unique<stages::SimpleGrasp>("grasp right");
    simple_grasp->setGraspPose("close");
    simple_grasp->setProperty("eef", eef_right);
    simple_grasp->setProperty("object", object_name);

    // outer grasp stage
    auto outer_grasp = std::make_unique<stages::SimpleGrasp>("grasp left");
    outer_grasp->setGraspPose("close");
    outer_grasp->setProperty("eef", eef_left);
    outer_grasp->setProperty("object", object_name);

    {
        // actual grasp generator for an in the origin mirrored grasp pose pair
        auto gengrasp = std::make_unique<stages::MirrorGraspGenerator>("generate mirror grasp pose");
        auto grasp_generator_ = gengrasp.get();

        // parameter for the grasp generator
        grasp_generator_->setAngleDelta(.2);
        grasp_generator_->setMonitoredStage(initial_stage);
        grasp_generator_->setObject(object_name);
        grasp_generator_->setNamedPose("open");
        grasp_generator_->setYOffset(0.02);

        // properties to be exposed to ik stage
        const std::initializer_list<std::string>& grasp_prop_names = {"pregrasp", "object", "angle_delta"};

        // properties for the ik stage to inherit via interface
        std::set<std::string> forward_properties = {"target_pose_left", "target_pose_right"};

        // ik solver that wraps the grasp generator
        auto ik_inner = std::make_unique<stages::ComputeIK>("compute ik right", std::move(gengrasp));

        // inherit properties form child
        ik_inner->exposePropertiesOfChild(0, grasp_prop_names);
        ik_inner->setEndEffector(eef_right);

        // map interface property to ik stage specific target pose property
        ik_inner->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_right");
        ik_inner->setIKFrame(Eigen::Translation3d(0.0,0.03,0.0), "r_grasp_frame");

        // inner ik needs to forward properties via interface
        // in order for the outer ik stage to access them
        ik_inner->setProperty("forward_properties", forward_properties);

        // outer ik stage to solve for the opposing target pose
        auto ik_outer = std::make_unique<stages::ComputeIK>("compute ik left", std::move(ik_inner));
        ik_outer->exposePropertiesOfChild(0, grasp_prop_names);
        ik_outer->setEndEffector(eef_left);
        ik_outer->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_left");
        ik_outer->setIKFrame(Eigen::Translation3d(0.005,0.035,0.0) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()), "l_grasp_frame");


        // from the inner grasp stage, replace the default
        // ik & graspgen pair with the double ik & mirror grasp gen
        simple_grasp->remove(0);
        simple_grasp->insert(std::move(ik_outer), 0);

        simple_grasp->exposePropertiesOfChild(0, grasp_prop_names);
        simple_grasp->exposePropertiesOfChild(0, { "max_ik_solutions", "timeout", "ik_frame" });

        // also remove close gripper
        simple_grasp->remove(-2);

        // in the outer grasp stage, replace the first container
        // with the inner grasp stage
        outer_grasp->remove(0);
        outer_grasp->insert(std::move(simple_grasp), 0);

        outer_grasp->exposePropertiesOfChild(0, grasp_prop_names);
        outer_grasp->exposePropertiesOfChild(0, { "max_ik_solutions", "timeout", "ik_frame" });

        // removing the last child of one of
        // the grasp stages avoids double attaching
        outer_grasp->remove(-1);

        // also remove close gripper
        outer_grasp->remove(-1);
    }

    // pick handles approaching and lifting the object
    auto pick = std::make_unique<stages::Pick>(std::move(outer_grasp));
    pick->cartesianSolver()->setProperty("jump_threshold", 0.0); // disable jump check, see MoveIt #773
    pick->setProperty("eef", eef_left);
    pick->setProperty("object", object_name);

    // define and set approach motion as a twist
    geometry_msgs::TwistStamped approach_twist;
    approach_twist.header.frame_id = tool_frame_left;
    approach_twist.twist.linear.x = 1.0;
    approach_twist.twist.linear.y = 1.0;
    pick->setApproachMotion(approach_twist, 0.03, 0.1);

    // lifting is done via relative joint space goal
    std::map<std::string, double> left_lift_goal;
    left_lift_goal["LShoulderPitch"] = -0.25;
    pick->setRelativeJointSpaceGoal(left_lift_goal);

    // add an approach stage for the other arm
    {
        auto approach = std::make_unique<stages::MoveRelative>("approach object right", pick->cartesianSolver());
        PropertyMap& p = approach->properties();
        p.set("marker_ns", std::string("approach"));

        approach->setGroup(arm_right);
        approach->setLink(tool_frame_right);

        geometry_msgs::TwistStamped approach_twist_right;
        approach_twist_right.header.frame_id = tool_frame_right;
        approach_twist_right.twist.linear.x = 1.0;
        approach_twist_right.twist.linear.y = 1.0;

        approach->along(approach_twist_right);
        approach->setMinMaxDistance(0.03, 0.1);
        pick->insert(std::move(approach), 0);
    }

    // add a lift stage for the other arm
    {
        auto lift = std::make_unique<stages::MoveRelative>("lift object right", pick->cartesianSolver());
        PropertyMap& p = lift->properties();
        p.set("marker_ns", std::string("lift"));

        lift->setGroup(arm_right);
        lift->setLink(tool_frame_right);

        std::map<std::string, double> right_lift_goal;
        right_lift_goal["RShoulderPitch"] = -0.25;
        lift->setRelativeJointSpaceGoal(right_lift_goal);

        pick->insert(std::move(lift));
    }

    // connect will fill the gap to the current state
    pick->insert(std::move(connect), 0);

    // finally, add the pick container to the task
    task->add(std::move(pick));

}

bool runTask(pepper_mtc_msgs::PepperGrasping::Request  &req,
             pepper_mtc_msgs::PepperGrasping::Response &res){

    // delete old task, create new one
    task_ptr_.reset();
    task_ptr_.reset(new Task());

    auto task = task_ptr_.get();

    try {
        // start from the current state
        Stage* initial_stage = nullptr;
        auto initial = std::make_unique<stages::CurrentState>();
        initial_stage = initial.get();

        task->add(std::move(initial));

        // fill task with bidextral pipeline
        fillTask(initial_stage, req.object);

        // is blocking
        task->plan();
    }
    catch (const InitStageException &e) {
        std::cerr << e;
        res.success = false;
        return false;
    }

    moveit_task_constructor_msgs::Solution msg;
    Stage::SolutionProcessor processor = [&msg](const SolutionBase& s) {
        s.fillMessage(msg);
        return false;
    };

    task->processSolutions(processor);

    for(moveit_task_constructor_msgs::SubTrajectory sub_traj: msg.sub_trajectory){
        trajectory_msgs::JointTrajectory trajectory = sub_traj.trajectory.joint_trajectory;
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = trajectory;
        action_client_ptr->sendGoal(goal);
        bool finnished_before_timeout = action_client_ptr->waitForResult(ros::Duration(120.0));
        if(finnished_before_timeout){
            actionlib::SimpleClientGoalState state = action_client_ptr->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else{
            ROS_INFO("Action did not finish before the time out. Cancelling pipeline");
            action_client_ptr->cancelGoal();
            break;
        }

    }


    res.success = true;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "grasping");

    ros::NodeHandle n;

    std::string action_topic = "/pepper_dcm/LeftArm_controller/follow_joint_trajectory";
    // create the action client
    // true causes the client to spin its own thread
    action_client_ptr.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(action_topic, true));

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    action_client_ptr->waitForServer(); //will wait for infinite time

    ROS_INFO("action_server found.");

    std::cout << "... and we're spinning in the main thread!" << std::endl;
    ros::ServiceServer server = n.advertiseService("pepper_grasping", runTask);

    // needed to also ensure that in task encapsuled service calls work
    ros::MultiThreadedSpinner().spin();

    return 0;
}
