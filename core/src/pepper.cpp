#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <stages/mirror_grasp_generator.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pepper_mtc_msgs/PepperGrasping.h>


using namespace moveit::task_constructor;

std::shared_ptr<Task> task_ptr_;

void fillTask(Stage* initial_stage, std::string object_name) {

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

    // grasp generator
    auto simple_grasp = std::make_unique<stages::SimpleGrasp>("grasp right");
    simple_grasp->setGraspPose("close");
    simple_grasp->setProperty("eef", eef_right);
    simple_grasp->setProperty("object", object_name);

    // outer grasp generator
    auto outer_grasp = std::make_unique<stages::SimpleGrasp>("grasp left");
    outer_grasp->setGraspPose("close");
    outer_grasp->setProperty("eef", eef_left);
    outer_grasp->setProperty("object", object_name);

    {
        auto gengrasp = std::make_unique<stages::MirrorGraspGenerator>("generate mirror grasp pose");
        auto grasp_generator_ = gengrasp.get();

        grasp_generator_->setAngleDelta(.2);
        grasp_generator_->setMonitoredStage(initial_stage);
        grasp_generator_->setObject(object_name);
        grasp_generator_->setNamedPose("open");
        grasp_generator_->setSafetyMargin(0.02);

        const std::initializer_list<std::string>& grasp_prop_names = {"pregrasp", "object", "angle_delta"};
        std::set<std::string> forward_properties = {"target_pose_left", "target_pose_right"};


        auto ik_inner = std::make_unique<stages::ComputeIK>("compute ik right", std::move(gengrasp));
        ik_inner->exposePropertiesOfChild(0, grasp_prop_names);
        ik_inner->setEndEffector(eef_right);
        ik_inner->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_right");
        ik_inner->setIKFrame(Eigen::Translation3d(0.0,0.03,0.0), "r_grasp_frame");

        ik_inner->setProperty("forward_properties", forward_properties);

        auto ik_outer = std::make_unique<stages::ComputeIK>("compute ik left", std::move(ik_inner));
        ik_outer->exposePropertiesOfChild(0, grasp_prop_names);
        ik_outer->setEndEffector(eef_left);
        ik_outer->properties().property("target_pose").configureInitFrom(Stage::INTERFACE, "target_pose_left");
        ik_outer->setIKFrame(Eigen::Translation3d(0.005,0.035,0.0) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()), "l_grasp_frame");


        simple_grasp->remove(0);
        simple_grasp->insert(std::move(ik_outer), 0);

        simple_grasp->exposePropertiesOfChild(0, grasp_prop_names);
        simple_grasp->exposePropertiesOfChild(0, { "max_ik_solutions", "timeout", "ik_frame" });

        outer_grasp->remove(0);
        outer_grasp->insert(std::move(simple_grasp), 0);

        outer_grasp->exposePropertiesOfChild(0, grasp_prop_names);
        outer_grasp->exposePropertiesOfChild(0, { "max_ik_solutions", "timeout", "ik_frame" });

        outer_grasp->remove(-1);
    }

    // pick container, using the generated grasp generator
    auto pick = std::make_unique<stages::Pick>(std::move(outer_grasp));
    pick->cartesianSolver()->setProperty("jump_threshold", 0.0); // disable jump check, see MoveIt #773
    pick->setProperty("eef", eef_left);
    pick->setProperty("object", object_name);

    geometry_msgs::TwistStamped approach_twist;
    approach_twist.header.frame_id = tool_frame_left;
    approach_twist.twist.linear.x = 1.0;
    approach_twist.twist.linear.y = 1.0;
    pick->setApproachMotion(approach_twist, 0.03, 0.1);

    std::map<std::string, double> left_lift_goal;
    left_lift_goal["LShoulderPitch"] = -0.25;
    pick->setRelativeJointSpaceGoal(left_lift_goal);

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

    pick->insert(std::move(connect), 0);
    task->add(std::move(pick));

}

bool runTask(pepper_mtc_msgs::PepperGrasping::Request  &req,
             pepper_mtc_msgs::PepperGrasping::Response &res){

    task_ptr_.reset();
    task_ptr_.reset(new Task());

    auto task = task_ptr_.get();

    try {
        Stage* initial_stage = nullptr;
        auto initial = std::make_unique<stages::CurrentState>();
        initial_stage = initial.get();

        task->add(std::move(initial));

        fillTask(initial_stage, req.object);

        task->plan();
    }
    catch (const InitStageException &e) {
        std::cerr << e;
        res.success = false;
        return false;
    }

    res.success = true;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "grasping");

    ros::NodeHandle n;
    std::cout << "... and we're spinning in the main thread!" << std::endl;
    ros::ServiceServer server = n.advertiseService("pepper_grasping", runTask);

    ros::MultiThreadedSpinner().spin();

    return 0;
}
