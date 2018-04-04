/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Luca Lach */

#include <stages/mirror_grasp_generator.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {


MirrorGraspGenerator::MirrorGraspGenerator(const std::string& name)
   : MonitoringGenerator(name)
{
    auto& p = properties();
    p.declare<std::string>("eef", "name of end-effector");
    p.declare<std::string>("pregrasp", "name of end-effector's pregrasp pose");
    p.declare<std::string>("object");
    p.declare<geometry_msgs::TransformStamped>("ik_frame_left", geometry_msgs::TransformStamped(), "transform from robot tool frame to left grasp frame");
    p.declare<geometry_msgs::TransformStamped>("ik_frame_right", geometry_msgs::TransformStamped(), "transform from robot tool frame to right grasp frame");
    p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
    p.declare<double>("safety_margin", 0.01, "distance between origin of grasp pose and object surface (cm)");


    p.declare<double>("hand_height", 0.04, "height of hand needed to avoid grasps above the object");
    p.declare<double>("linear_z_step", 0.01, "distance between grasp poses on z axis");
    p.declare<double>("linear_x_step", 0.01, "distance between grasp poses on x axis");
}


void MirrorGraspGenerator::setEndEffector(const std::string &eef){
    setProperty("eef", eef);
}

void MirrorGraspGenerator::setNamedPose(const std::string &pose_name){
    setProperty("pregrasp", pose_name);
}

void MirrorGraspGenerator::setObject(const std::string &object){
    setProperty("object", object);
}

void MirrorGraspGenerator::setIKFrameLeft(const geometry_msgs::TransformStamped &transform){
    setProperty("ik_frame_left", transform);
}

void MirrorGraspGenerator::setIKFrameLeft(const Eigen::Affine3d &transform, const std::string &link){
    geometry_msgs::TransformStamped stamped;
    stamped.header.frame_id = link;
    stamped.child_frame_id = "grasp_frame";
    tf::transformEigenToMsg(transform, stamped.transform);
    setIKFrameLeft(stamped);
}

void MirrorGraspGenerator::setIKFrameRight(const geometry_msgs::TransformStamped &transform){
    setProperty("ik_frame_right", transform);
}

void MirrorGraspGenerator::setIKFrameRight(const Eigen::Affine3d &transform, const std::string &link){
    geometry_msgs::TransformStamped stamped;
    stamped.header.frame_id = link;
    stamped.child_frame_id = "grasp_frame";
    tf::transformEigenToMsg(transform, stamped.transform);
    setIKFrameRight(stamped);
}


void MirrorGraspGenerator::setAngleDelta(double delta){
    setProperty("angle_delta", delta);
}

void MirrorGraspGenerator::setSafetyMargin(double safety_margin){
    setProperty("safety_margin", safety_margin);
}

void MirrorGraspGenerator::onNewSolution(const moveit::task_constructor::SolutionBase& s)
{
    scenes_.push_back(s.end()->scene()->diff());
}

bool MirrorGraspGenerator::canCompute() const {
    return scenes_.size() > 0;
}

bool MirrorGraspGenerator::compute(){
    const auto& props = properties();

    double safety_margin = props.get<double>("safety_margin");
    double hand_height = props.get<double>("hand_height");

    if(scenes_.empty()) throw std::runtime_error("MirrorGraspGenerator called without checking canCompute.");

    planning_scene::PlanningSceneConstPtr scene = scenes_[0];
    scenes_.pop_front();

    ROS_INFO_STREAM("generating grasps for " << props.get<std::string>("object"));

    object_ = props.get<std::string>("object");
    if (!scene->knowsFrameTransform(object_))
        throw std::runtime_error("requested object does not exist or could not be retrieved");
    const Eigen::Affine3d object_pose_eigen = scene->getFrameTransform(object_);

    geometry_msgs::Pose object_pose;
    tf::poseEigenToMsg(object_pose_eigen, object_pose);

    shapes::ShapeConstPtr shape = scene->getWorld()->getObject(object_)->shapes_.front();

    double grasp_y_left;
    double grasp_y_right;

    switch(shape->type) {
        case(shapes::ShapeType::BOX):
        {
            // box->size: depth, lenght, height [x, y, z]
            std::shared_ptr<const shapes::Box> box = std::static_pointer_cast<const shapes::Box>(shape);

            grasp_y_left = ((box->size[1] / 2) + safety_margin);
            grasp_y_right = - ((box->size[1] / 2) + safety_margin);

            x_min_ = - (box->size[0] / 2);
            x_max_ = (box->size[0] / 2);
            x_current_ = x_min_;

            z_min_ = - (box->size[2] / 2) + (hand_height / 2);
            z_max_ = (box->size[2] / 2) - (hand_height / 2);
            z_current_ = z_min_;

            break;
        }
        case(shapes::ShapeType::SPHERE):
        {
            std::shared_ptr<const shapes::Sphere> sphere = std::static_pointer_cast<const shapes::Sphere>(shape);

            grasp_y_left = (sphere->radius + safety_margin);
            grasp_y_right = - (sphere->radius + safety_margin);

            spawnPoses(0, 0, grasp_y_left, grasp_y_right, object_pose.orientation, scene);
            return true;
        }
        case(shapes::ShapeType::CYLINDER):
        {
            std::shared_ptr<const shapes::Cylinder> cylinder = std::static_pointer_cast<const shapes::Cylinder>(shape);

            grasp_y_left = (cylinder->radius + safety_margin);
            grasp_y_right = - (cylinder->radius + safety_margin);

            z_min_ = - (cylinder->length / 2) + (hand_height / 2);
            z_max_ = (cylinder->length / 2) - (hand_height / 2);
            z_current_ = z_min_;

            break;
        }
        default:
            throw std::runtime_error("requested object has unsupported shape");
    }

    double linear_z_step = props.get<double>("linear_z_step");
    double linear_x_step = props.get<double>("linear_x_step");

    while (z_current_ < z_max_) {
        std::cout << "z current: " << z_current_ << std::endl;
        if (shape->type == shapes::ShapeType::BOX) {
            while (x_current_ < x_max_) {
                std::cout << "x current: " << x_current_ << std::endl;
                spawnPoses(x_current_, z_current_, grasp_y_left, grasp_y_right, object_pose.orientation, scene);
                x_current_ += linear_x_step;
            }
            x_current_ = x_min_;
        } else {
            spawnPoses(0, z_current_, grasp_y_left, grasp_y_right, object_pose.orientation, scene);
        }

        z_current_ += linear_z_step;
    }

    return true;
}

void MirrorGraspGenerator::spawnPoses(double x, double z, double y_left, double y_right,
                geometry_msgs::Quaternion orientation, planning_scene::PlanningSceneConstPtr scene) {

    geometry_msgs::PoseStamped target_pose_left;
    geometry_msgs::PoseStamped target_pose_right;

    target_pose_left.pose.orientation = orientation;
    target_pose_right.pose.orientation = orientation;

    target_pose_left.pose.position.x = x;
    target_pose_left.pose.position.y = y_left;
    target_pose_left.pose.position.z = z;

    target_pose_right.pose.position.x = x;
    target_pose_right.pose.position.y = y_right;
    target_pose_right.pose.position.z = z;

    target_pose_left.header.frame_id = object_;
    target_pose_right.header.frame_id = object_;

    moveit::task_constructor::InterfaceState state(scene);
    state.properties().set("target_pose_right", target_pose_right);
    state.properties().set("target_pose_left", target_pose_left);

    moveit::task_constructor::SubTrajectory trajectory;
    trajectory.setCost(0.0);

    rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_right, 0.1, "pose_right frame");
    rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_left, 0.1, "pose_left frame");

    spawn(std::move(state), std::move(trajectory));
}

} } }
