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

    geometry_msgs::PoseStamped target_pose_left;
    geometry_msgs::PoseStamped target_pose_right;

    if(scenes_.empty()) throw std::runtime_error("MirrorGraspGenerator called without checking canCompute.");

    planning_scene::PlanningSceneConstPtr scene = scenes_[0];
    scenes_.pop_front();

    ROS_INFO_STREAM("generating grasps for " << props.get<std::string>("object"));

    const std::string& object_name = props.get<std::string>("object");
    if (!scene->knowsFrameTransform(object_name))
        throw std::runtime_error("requested object does not exist or could not be retrieved");
    const Eigen::Affine3d object_pose_eigen = scene->getFrameTransform(object_name);

    geometry_msgs::Pose object_pose;
    tf::poseEigenToMsg(object_pose_eigen, object_pose);

    shapes::ShapeConstPtr shape = scene->getWorld()->getObject(object_name)->shapes_.front();

    switch(shape->type) {
        case(shapes::ShapeType::BOX):
        {
            // box->size: depth, lenght, height [x, y, z]
            std::shared_ptr<const shapes::Box> box = std::static_pointer_cast<const shapes::Box>(shape);

            target_pose_left.pose.orientation = object_pose.orientation;
            target_pose_right.pose.orientation = object_pose.orientation;

            double grasp_x = object_pose.position.x;
            double grasp_z = object_pose.position.z;

            double grasp_y_left = object_pose.position.y + ((box->size[1] / 2) + safety_margin);
            double grasp_y_right = object_pose.position.y - ((box->size[1] / 2) + safety_margin);

            target_pose_left.pose.position.x = grasp_x;
            target_pose_left.pose.position.y = grasp_y_left;
            target_pose_left.pose.position.z = grasp_z;

            target_pose_right.pose.position.x = grasp_x;
            target_pose_right.pose.position.y = grasp_y_right;
            target_pose_right.pose.position.z = grasp_z;

            target_pose_left.header.frame_id = "base_footprint";
            target_pose_right.header.frame_id = "base_footprint";

            break;
        }
        case(shapes::ShapeType::SPHERE):
        {
            std::shared_ptr<const shapes::Sphere> sphere = std::static_pointer_cast<const shapes::Sphere>(shape);

            target_pose_left.pose.orientation = object_pose.orientation;
            target_pose_right.pose.orientation = object_pose.orientation;

            double grasp_x = object_pose.position.x;
            double grasp_z = object_pose.position.z;

            double grasp_y_left = object_pose.position.y + (sphere->radius + safety_margin);
            double grasp_y_right = object_pose.position.y - (sphere->radius + safety_margin);

            target_pose_left.pose.position.x = grasp_x;
            target_pose_left.pose.position.y = grasp_y_left;
            target_pose_left.pose.position.z = grasp_z;

            target_pose_right.pose.position.x = grasp_x;
            target_pose_right.pose.position.y = grasp_y_right;
            target_pose_right.pose.position.z = grasp_z;

            target_pose_left.header.frame_id = "base_footprint";
            target_pose_right.header.frame_id = "base_footprint";

            break;
        }
        case(shapes::ShapeType::CYLINDER):
        {
            std::shared_ptr<const shapes::Cylinder> cylinder = std::static_pointer_cast<const shapes::Cylinder>(shape);

            target_pose_left.pose.orientation = object_pose.orientation;
            target_pose_right.pose.orientation = object_pose.orientation;

            double grasp_x = object_pose.position.x;
            double grasp_z = object_pose.position.z;

            double grasp_y_left = object_pose.position.y + (cylinder->radius + safety_margin);
            double grasp_y_right = object_pose.position.y - (cylinder->radius + safety_margin);

            target_pose_left.pose.position.x = grasp_x;
            target_pose_left.pose.position.y = grasp_y_left;
            target_pose_left.pose.position.z = grasp_z;

            target_pose_right.pose.position.x = grasp_x;
            target_pose_right.pose.position.y = grasp_y_right;
            target_pose_right.pose.position.z = grasp_z;

            target_pose_left.header.frame_id = "base_footprint";
            target_pose_right.header.frame_id = "base_footprint";

            break;
        }
        default:
            throw std::runtime_error("requested object has unsupported shape");
    }

    moveit::task_constructor::InterfaceState state(scene);
    state.properties().set("target_pose_right", target_pose_right);
    state.properties().set("target_pose_left", target_pose_left);

    moveit::task_constructor::SubTrajectory trajectory;
    trajectory.setCost(0.0);

    rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_right, 0.1, "pose_right frame");
    rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_left, 0.1, "pose_left frame");

    spawn(std::move(state), std::move(trajectory));
    return true;
}

} } }