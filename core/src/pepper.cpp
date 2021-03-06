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
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/connect.h>
#include <stages/bimanual_grasp_pose.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <stages/mirror_grasp_generator.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pepper_mtc_msgs/PepperGrasping.h>


using namespace moveit::task_constructor;

// keep global reference, so service calls from rviz plugin
// work even after the service call task trigger is done
std::shared_ptr<Task> task_ptr_;
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> action_client_ptr;


std::shared_ptr<Task> createTask(const std::string& object = "object") {
    std::shared_ptr<Task> t;
    t.reset(new Task());
    t->loadRobotModel();

    const std::string eef_left = "left_hand";
    const std::string eef_right = "right_hand";

    const std::string arm_left = "left_arm";
    const std::string arm_right = "right_arm";

    geometry_msgs::PoseStamped ik_frame_left;
    ik_frame_left.header.frame_id = "l_grasp_frame";
    tf::poseEigenToMsg(Eigen::Translation3d(0.0,0.05,0.0) *
                       Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()),
                       ik_frame_left.pose);

    geometry_msgs::PoseStamped ik_frame_right;
    ik_frame_right.header.frame_id = "r_grasp_frame";
    tf::poseEigenToMsg(Eigen::Affine3d(Eigen::Translation3d(0.0,0.05,0.0)),
                       ik_frame_right.pose);


    // cartesian planner
    auto cartesian = std::make_shared<solvers::CartesianPath>();
    cartesian->setProperty("jump_threshold", 0.0); // disable jump check, see MoveIt #773

    // pipeline planner
    auto pipeline = std::make_shared<solvers::PipelinePlanner>();
    pipeline->setTimeout(8.0);
    pipeline->setPlannerId("RRTConnectkConfigDefault");


    Stage* referenced_stage = nullptr;
    {  // fetch initial state from move_group
        auto initial = new stages::CurrentState("current state");
        t->add(std::unique_ptr<Stage>(referenced_stage = initial));
    }

    {  // connect current state to pick
        stages::Connect::GroupPlannerVector planners = {{eef_left, pipeline}, {arm_left, pipeline},
                                                        {eef_right, pipeline}, {arm_right, pipeline}};
        auto connect = new stages::Connect("connect", planners);
        connect->properties().configureInitFrom(Stage::PARENT);
        t->add(std::unique_ptr<Stage>(connect));
    }

    auto pick = new SerialContainer("pick");

    {  // approach
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.y = 1.0;

        auto merger = new Merger("approach");
        for (const auto& eef : {eef_left, eef_right}) {
            auto move = new stages::MoveRelative("approach " + eef, cartesian);
            move->restrictDirection(stages::MoveRelative::BACKWARD);
            move->setProperty("marker_ns", std::string("approach"));
            const moveit::core::JointModelGroup* eef_jmg = t->getRobotModel()->getEndEffector(eef);
            const auto& group_link = eef_jmg->getEndEffectorParentGroup();
            move->setGroup(group_link.first);
            move->setLink(group_link.second);
            twist.header.frame_id = group_link.second;
            move->along(twist);
            move->setMinMaxDistance(0.05, 0.10);
            merger->insert(std::unique_ptr<Stage>(move));
        }
        pick->insert(std::unique_ptr<Stage>(merger));
    }

    {  // bimanual grasp generator
        auto gengrasp = new stages::BimanualGraspPose();
        gengrasp->setMonitoredStage(referenced_stage);
        gengrasp->setObject(object);
        gengrasp->setEndEffectorPoses({{eef_left, "open"}, {eef_right, "open"}});

        // inner IK: right hand
        auto ik_inner = new stages::ComputeIK("compute ik right", std::unique_ptr<Stage>(gengrasp));
        ik_inner->setEndEffector(eef_right);
        ik_inner->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_right");
        ik_inner->setIKFrame(ik_frame_right);
        ik_inner->setProperty("forward_properties", std::set<std::string>({"target_pose_left", "target_pose_right"}));

        // outer IK: left hand
        auto ik_outer = new stages::ComputeIK("compute ik left", std::unique_ptr<Stage>(ik_inner));
        ik_outer->setEndEffector(eef_left);
        ik_outer->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_left");
        ik_outer->setIKFrame(ik_frame_left);

        pick->insert(std::unique_ptr<Stage>(ik_outer));
    }

    {  // allow touching the object
        auto allow_touch = new stages::ModifyPlanningScene("allow object collision");
        allow_touch->allowCollisions(object, t->getRobotModel()->getJointModelGroup(eef_left)->getLinkModelNamesWithCollisionGeometry(), true);
        allow_touch->allowCollisions(object, t->getRobotModel()->getJointModelGroup(eef_right)->getLinkModelNamesWithCollisionGeometry(), true);
        pick->insert(std::unique_ptr<Stage>(allow_touch));
    }
#if 1
    {  // close hands
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.y = 1.0;

        auto merger = new Merger("close");
        for (const auto& eef : {eef_left, eef_right}) {
            auto move = new stages::MoveRelative("close " + eef, cartesian);
            move->restrictDirection(stages::MoveRelative::FORWARD);
            move->setProperty("marker_ns", std::string("close"));
            const moveit::core::JointModelGroup* eef_jmg = t->getRobotModel()->getEndEffector(eef);
            const auto& group_link = eef_jmg->getEndEffectorParentGroup();
            move->setGroup(group_link.first);
            move->setLink(group_link.second);
            twist.header.frame_id = group_link.second;
            move->along(twist);
            move->setMinMaxDistance(0.02, 0.03);
            merger->insert(std::unique_ptr<Stage>(move));
        }
        pick->insert(std::unique_ptr<Stage>(merger));
    }
#else
    {  // close grippers
        auto merger = new Merger("close grippers");
        for (const auto& eef : {eef_left, eef_right}) {
            auto move = new stages::MoveTo("close " + eef, pipeline);
            move->restrictDirection(stages::MoveTo::FORWARD);
            move->setGroup(t.getRobotModel()->getEndEffector(eef)->getName());
            move->setGoal("close");
            merger->insert(std::unique_ptr<Stage>(move));
        }
        pick->insert(std::unique_ptr<Stage>(merger));
    }
#endif
    {  // attach object
        auto attach = new stages::ModifyPlanningScene("attach object");
        attach->attachObject(object, ik_frame_left.header.frame_id);
        pick->insert(std::unique_ptr<Stage>(attach));
    }

    {  // lift
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.z = 1.0;
        twist.header.frame_id = "odom";

        auto merger = new Merger("lift");
        for (const auto& arm : {arm_left, arm_right}) {
            auto move = new stages::MoveRelative("lift " + arm, cartesian);
            move->restrictDirection(stages::MoveRelative::FORWARD);
            move->setProperty("marker_ns", std::string("lift"));
            move->setGroup(arm);
            std::map<std::string, double> goal;
            std::string shoulder_joint(1, std::toupper(arm[0]));
            shoulder_joint.append("ShoulderPitch");
            goal[shoulder_joint] = -0.25;
            move->about(goal);
            merger->insert(std::unique_ptr<Stage>(move));
        }
        pick->insert(std::unique_ptr<Stage>(merger));
    }

    t->add(std::unique_ptr<Stage>(pick));
    return t;
}


bool runTask(pepper_mtc_msgs::PepperGrasping::Request  &req,
             pepper_mtc_msgs::PepperGrasping::Response &res){

    // delete old task, create new one
    task_ptr_.reset();
    task_ptr_ = createTask();

    auto task = task_ptr_.get();

    // is blocking
    task->plan();


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
        if(trajectory.points.empty()){ // subsolution is not a trajectory (e.g. a planning-scene modification)
            continue;
        }
        action_client_ptr->sendGoal(goal);
        float timeout = 30.0;
        bool finnished_before_timeout = action_client_ptr->waitForResult(ros::Duration(timeout));
        if(finnished_before_timeout){
            actionlib::SimpleClientGoalState state = action_client_ptr->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else{
            ROS_INFO("Action did not finish before the time out.",std::to_string(timeout)," Cancelling pipeline");
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

    std::string action_topic = "/pepper_robot/motion_control/LeftArm/follow_joint_trajectory";
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
