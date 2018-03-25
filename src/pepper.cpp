
#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <stages/simple_mirror_grasp.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pepper_mtc/PepperGrasping.h>


using namespace moveit::task_constructor;


std::shared_ptr<Task> task_ptr_;

void fill(ParallelContainerBase &container, Stage* initial_stage, bool right_side, std::string obj) {
    std::string side = right_side ? "right" : "left";
    std::string tool_frame = side.substr(0,1) + "_grasp_frame";
    std::string eef = side + "_hand";
    std::string arm = side + "_arm";

    // planner used for connect
    auto pipeline = std::make_shared<solvers::PipelinePlanner>();
    pipeline->setTimeout(8.0);
    pipeline->setPlannerId("RRTConnectkConfigDefault");

    // connect to pick
    stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
    auto connect = std::make_unique<stages::Connect>("connect", planners);
    connect->properties().configureInitFrom(Stage::PARENT);

    // grasp generator
    auto grasp_generator = std::make_unique<stages::SimpleMirrorGrasp>();

    if (right_side)
        grasp_generator->setToolToGraspTF(Eigen::Translation3d(0.0,0.03,0.0), tool_frame);
    else
        grasp_generator->setToolToGraspTF(Eigen::Translation3d(0.005,0.035,0.0) *
                                          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()), tool_frame);

    grasp_generator->setAngleDelta(.2);
    grasp_generator->setPreGraspPose("open");
    grasp_generator->setGraspPose("close");
    grasp_generator->setMonitoredStage(initial_stage);

    // pick container, using the generated grasp generator
    auto pick = std::make_unique<stages::Pick>(std::move(grasp_generator), side);
    pick->cartesianSolver()->setProperty("jump_threshold", 0.0); // disable jump check, see MoveIt #773
    pick->setProperty("eef", eef);
    pick->setProperty("object", obj);
    geometry_msgs::TwistStamped approach;
    approach.header.frame_id = tool_frame;
    approach.twist.linear.x = 1.0;
    approach.twist.linear.y = 1.0;
    pick->setApproachMotion(approach, 0.03, 0.1);

    geometry_msgs::TwistStamped lift;
    lift.header.frame_id = "base_link";
    lift.twist.linear.z = 1.0;
    pick->setLiftMotion(lift, 0.03, 0.05);

    pick->insert(std::move(connect), 0);
    container.insert(std::move(pick));
}

bool runTask(pepper_mtc::PepperGrasping::Request  &req,
             pepper_mtc::PepperGrasping::Response &res){

    task_ptr_.reset();
    task_ptr_.reset(new Task());

    auto task = task_ptr_.get();

    try {
        Stage* initial_stage = nullptr;
        auto initial = std::make_unique<stages::CurrentState>();
        initial_stage = initial.get();
        task->add(std::move(initial));

        auto parallel = std::make_unique<Alternatives>();
        fill(*parallel, initial_stage, true, req.object);
        //fill(*parallel, initial_stage, false, req.object);

        task->add(std::move(parallel));

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
