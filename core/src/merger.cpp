#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

const std::string DISPLAY_TRAJECOTRY_TOPIC = "/move_group/display_planned_path";
robot_state::RobotStateConstPtr const_start_state;

void printJointModelVector(std::vector<const moveit::core::JointModel*> jm_list, std::string name){
    std::cout << std::endl << "### JM Vector " << name << ": " << std::endl;
    for (auto jm : jm_list) {
        std::cout << jm->getName() << std::endl;
    }
}

void printStringVector(const std::vector<std::string>& vec){
    for (auto s : vec) {
        std::cout << s << std::endl;
    }
}

void printJMG(srdf::Model::Group gr){

    std::cout << std::endl << "### JMG " << gr.name_ << ": " << std::endl << std::endl;

    std::cout << "Chains: " << std::endl;
    for (auto c : gr.chains_){
        std::cout << c.first << " --> " << c.second << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Joints:" << std::endl;
    for (auto j : gr.joints_) {
        std::cout << j << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Links:" << std::endl;
    for (auto l : gr.links_) {
        std::cout << l << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Subgroups:" << std::endl;
    for (auto s : gr.subgroups_) {
        std::cout << s << std::endl;
    }

}

void printJMG(moveit::core::JointModelGroup jmg){
    printJMG(jmg.getConfig());
}

std::shared_ptr<moveit::core::JointModelGroup> mergeJointModelGroups( std::initializer_list<std::string> &groups, moveit::core::RobotModel &model){

    srdf::Model::Group merged_group;
    merged_group.name_ = "merged_arms";

    // vector for joint models from merged group
    std::vector<const moveit::core::JointModel*> merged_jm_list;

    for (auto g : groups){
        auto jmg =  model.getJointModelGroup(g);
        auto jmg_config = jmg->getConfig();
        auto jm_list = model.getJointModelGroup(g)->getJointModels();

        // lets look at the current jmg
        printJMG(jmg_config);
        printJointModelVector(jm_list, jmg->getName());

        // copy attributes
        merged_group.joints_.insert(merged_group.joints_.end(), jmg_config.joints_.begin(), jmg_config.joints_.end());
        merged_group.chains_.insert(merged_group.chains_.end(), jmg_config.chains_.begin(), jmg_config.chains_.end());
        merged_group.links_.insert(merged_group.links_.end(), jmg_config.links_.begin(), jmg_config.links_.end());
        merged_group.subgroups_.insert(merged_group.subgroups_.end(), jmg_config.subgroups_.begin(), jmg_config.subgroups_.end());

        // add group
        merged_group.subgroups_.insert(merged_group.subgroups_.end(), g);

        merged_jm_list.insert(merged_jm_list.end(), jm_list.begin(), jm_list.end());
    }

    // evil const casting to satisfy the JMG constructor
    const std::vector<const moveit::core::JointModel*> &jm_list = const_cast<const std::vector<const moveit::core::JointModel*>&>(merged_jm_list);
    const srdf::Model::Group &mg = const_cast<const srdf::Model::Group&>(merged_group);

//    printJMG(merged_group);
//    printJointModelVector(merged_jm_list, merged_group.name_);

    return std::make_shared<moveit::core::JointModelGroup>(merged_group.name_, mg, jm_list, &model);
}

bool getRandomTrajectory(robot_model::RobotModelPtr robot_model, std::string group,
                         robot_trajectory::RobotTrajectoryPtr traj){

    // instantiate mgi for given group
    moveit::planning_interface::MoveGroupInterface move_group(group);

    // store current robot state
    auto start_state = move_group.getCurrentState();

    // get random eef pose
    auto jmg_right = start_state->getJointModelGroup(group);
    start_state->setToRandomPositions(jmg_right);
    const Eigen::Affine3d &eef_state_right = start_state->getGlobalLinkTransform(move_group.getEndEffectorLink());

    // convert random eef pose from eigen to pose message
    geometry_msgs::PoseStamped ps;
    tf::poseEigenToMsg(eef_state_right, ps.pose);
    ps.header.frame_id = move_group.getPlanningFrame();

    // set random eef pose as group target
    move_group.setPoseTarget(ps);

    // plan to pose and store result
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO_STREAM("######### Planning succeeded for group " << group << "!");
    } else {
        ROS_INFO_STREAM("######### Planning failed for group " << group << "!");
        return false;
    }

    // construct trajectory and return
    traj->setRobotTrajectoryMsg(*(const_start_state.get()), plan.trajectory_);

    return true;
}

void mergeJMGinRobotState(robot_model::RobotModelPtr robot_model, robot_state::RobotStatePtr target,
                          robot_state::RobotStatePtr source, std::string group_name) {

    std::cout << "merging group " << group_name << std::endl;

    // get joint model names to merge
    auto jmg = robot_model->getJointModelGroup(group_name);
    auto joint_names = jmg->getJointModelNames();

    // merge all relevatna values into the target model
    for (auto j : joint_names) {
        try {
            std::cout << j << ": " << std::endl;

            double pos = source->getVariablePosition(j);
            std::cout << "  --> position: " << pos << std::endl;
            target->setVariablePosition(j, pos);

            if (source->hasVelocities()) {
                double vel = source->getVariableVelocity(j);
                std::cout << "  --> velocity: " <<vel << std::endl;
                target->setVariableVelocity(j, vel);
            }

            // a joint within a state either has effort or acceleration
            if (source->hasEffort()) {
                double eff = source->getVariableEffort(j);
                std::cout << "  --> effort: " << eff << std::endl;
                target->setVariableEffort(j, eff);
            } else {
                double acc = source->getVariableEffort(j);
                std::cout << "  --> acceleration: " << acc << std::endl;
                target->setVariableAcceleration(j, acc);
            }
        } catch (const std::exception& e)  {
                    std::cout << j << " unkown." << std::endl;
        }

    }

    // forcefully update tfs in target state
    target->update(true);

    std::cout << " --------- state merging done --------- " << std::endl;
}

void mergeRobotTrajectories(robot_model::RobotModelPtr robot_model, robot_trajectory::RobotTrajectoryPtr one,
                            robot_trajectory::RobotTrajectoryPtr two, robot_trajectory::RobotTrajectoryPtr merged) {

    // sort for longer / shorter trajectory
    robot_trajectory::RobotTrajectoryPtr longer = one->getWayPointCount() >= two->getWayPointCount() ? one : two;
    robot_trajectory::RobotTrajectoryPtr shorter = one->getWayPointCount() >= two->getWayPointCount() ? two : one;

    // get length of trajectories
    unsigned long longer_length = longer->getWayPointCount();
    unsigned long shorter_length = shorter->getWayPointCount();

    robot_state::RobotStatePtr last_shorter_state = nullptr;

    // loop over longer trajectory --> no waypoint from second trajectory is forgotten
    for(unsigned long i = 0; i < longer_length; i++){

        // get waypoint and duration from shorter trajectory
        robot_state::RobotStatePtr state = longer->getWayPointPtr(i);
        double duration = longer->getWayPointDurationFromPrevious(i);

        // if there are waypoints left in the shorter trajectory, merge into actual waypoint
        if (i < shorter_length) {

            // get waypoint and duration from shorter trajectory
            robot_state::RobotStatePtr state_two = shorter->getWayPointPtr(i);
            double duration_two = shorter->getWayPointDurationFromPrevious(i);

            // set mean as duration || TODO think about this.
            duration = (duration + duration_two) / 2;

            // merge the relevant joint values from the shorter trajectory
            // into the robot state of the longer one.
            mergeJMGinRobotState(robot_model, state, state_two, shorter->getGroupName());

            // store last waypoint
            last_shorter_state = state_two;
        } else if (last_shorter_state != nullptr) {
            // merge last state
            mergeJMGinRobotState(robot_model, state, last_shorter_state, shorter->getGroupName());
        }

        // add waypoint at end of the trajectory
        merged->addSuffixWayPoint(state, duration);
    }

    std::cout << " --------- trajectory merging done --------- " << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "merging");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_TRAJECOTRY_TOPIC, 1);

    std::string arm_left = "left_arm";
    std::string arm_right = "right_arm";

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model_(robot_model_loader.getModel());

    // sadly, this doesn't fix the double free ...
    moveit::planning_interface::MoveGroupInterface move_group(arm_left);
    auto start_state = move_group.getCurrentState();
    const_start_state.reset(start_state.get());

    robot_trajectory::RobotTrajectoryPtr r_traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, arm_right);
    robot_trajectory::RobotTrajectoryPtr l_traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, arm_left);

    /*
     *
     * group merging works, but using the merged group doesn't
     *
    // group list
    std::initializer_list<std::string> groups = {r_traj->getGroupName(), l_traj->getGroupName()};

    // merge jmgs from inocming trajectories
    auto merged_jmg = mergeJointModelGroups(groups, *(robot_model_.get()));

    // doesn't work as not all internal robot model members are updated
    bool added = robot_model_->addJointModelGroup(merged_jmg->getConfig());

    if (added) {
        std::cout << "added group successfully" << std::endl;
    } else {
        std::cout << "group not added" << std::endl;
    }

    */

    // use merged jmg, otherwise message serialization of the trajectory will only work for one group.
    robot_trajectory::RobotTrajectoryPtr merged = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, "both_arms");

    bool success_right = false;
    bool success_left = false;

    while (!success_right) {
        success_right = getRandomTrajectory(robot_model_, arm_right, r_traj);
    }

    while (!success_left) {
        success_left = getRandomTrajectory(robot_model_, arm_left, l_traj);
    }

    ROS_INFO_STREAM("Generated Trajectory for " << r_traj->getGroupName() << " has " << r_traj->getWayPointCount() << " Waypoints.");
    ROS_INFO_STREAM("Generated Trajectory for " << l_traj->getGroupName() << " has " << l_traj->getWayPointCount() << " Waypoints.");

    mergeRobotTrajectories(robot_model_, r_traj, l_traj, merged);

    moveit_msgs::RobotTrajectory merged_msg;
    merged->getRobotTrajectoryMsg(merged_msg);

    moveit_msgs::DisplayTrajectory dt;
    dt.model_id = "JulietteY20MP";
    dt.trajectory.push_back(merged_msg);
    moveit::core::robotStateToRobotStateMsg(*(const_start_state.get()), dt.trajectory_start);

    display_pub.publish(dt);

    // evil! this is only to surpress the double free or corruption error..
    exit(0);
    return 0;
}
