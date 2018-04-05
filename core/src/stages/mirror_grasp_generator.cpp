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

    p.declare<double>("y_offset", 0.01, "distance between origin of grasp pose and object surface (cm)");
    p.declare<double>("hand_height", 0.04, "height of hand needed to avoid grasps above the object");

    p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
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

void MirrorGraspGenerator::setAngleDelta(double delta){
    setProperty("angle_delta", delta);
}

void MirrorGraspGenerator::setYOffset(double y_offset){
    setProperty("y_offset", y_offset);
}

void MirrorGraspGenerator::setHandHeight(const double &hand_height) {
    setProperty("hand_height", hand_height);
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

    double y_offset = props.get<double>("y_offset");
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

    double y_left;
    double y_right;

    // based on object shape grasp pose pairs are generated
    switch(shape->type) {
        case(shapes::ShapeType::BOX):
        {
            // box->size: depth, lenght, height [x, y, z]
            std::shared_ptr<const shapes::Box> box = std::static_pointer_cast<const shapes::Box>(shape);

            y_left = ((box->size[1] / 2) + y_offset);
            y_right = - ((box->size[1] / 2) + y_offset);

            x_min_ = - (box->size[0] / 2);
            x_max_ = (box->size[0] / 2);
            x_current_ = x_min_;

            z_min_ = - (box->size[2] / 2) + (hand_height / 2);
            z_max_ = (box->size[2] / 2) - (hand_height / 2);
            z_current_ = z_min_;

            while (z_current_ < z_max_) {
                    while (x_current_ < x_max_) {
                        Eigen::Affine3d target_pose_left(Eigen::Translation3d(x_current_, y_left, z_current_));
                        Eigen::Affine3d target_pose_right(Eigen::Translation3d(x_current_, y_right, z_current_));

                        x_current_ += props.get<double>("linear_x_step");

                        spawnPoses(target_pose_right, target_pose_left, scene);
                    }
                x_current_ = x_min_;
                z_current_ += props.get<double>("linear_z_step");
            }

            break;
        }
        case(shapes::ShapeType::SPHERE):
        {
            std::shared_ptr<const shapes::Sphere> sphere = std::static_pointer_cast<const shapes::Sphere>(shape);

            y_left = (sphere->radius + y_offset);
            y_right = - (sphere->radius + y_offset);

            double current_angle_ = - (M_PI / 4);

            while (current_angle_ < (M_PI / 4)) {

                Eigen::Affine3d target_pose_left(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()) *
                                                 Eigen::Translation3d(0.0, y_left, 0.0));
                Eigen::Affine3d target_pose_right(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()) *
                                                  Eigen::Translation3d(0.0, y_right, 0.0));

                current_angle_ += props.get<double>("angle_delta");

                spawnPoses(target_pose_right, target_pose_left, scene);
            }

            break;
        }
        case(shapes::ShapeType::CYLINDER):
        {
            std::shared_ptr<const shapes::Cylinder> cylinder = std::static_pointer_cast<const shapes::Cylinder>(shape);

            y_left = (cylinder->radius + y_offset);
            y_right = - (cylinder->radius + y_offset);

            z_min_ = - (cylinder->length / 2) + (hand_height / 2);
            z_max_ = (cylinder->length / 2) - (hand_height / 2);
            z_current_ = z_min_;

            while (z_current_ < z_max_) {
                double current_angle_ = - (M_PI / 4);

                while (current_angle_ < (M_PI / 4)) {

                    Eigen::Affine3d target_pose_left(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()) *
                                                     Eigen::Translation3d(0.0, y_left, z_current_));
                    Eigen::Affine3d target_pose_right(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()) *
                                                      Eigen::Translation3d(0.0, y_right, z_current_));

                    current_angle_ += props.get<double>("angle_delta");

                    spawnPoses(target_pose_right, target_pose_left, scene);
                }

                z_current_ += props.get<double>("linear_z_step");
            }

            break;
        }
        default:
            throw std::runtime_error("requested object has unsupported shape");
    }

    return true;
}

void MirrorGraspGenerator::spawnPoses(Eigen::Affine3d right, Eigen::Affine3d left,
                                      planning_scene::PlanningSceneConstPtr scene) {

    geometry_msgs::PoseStamped target_pose_left_msg;
    target_pose_left_msg.header.frame_id = object_;

    geometry_msgs::PoseStamped target_pose_right_msg;
    target_pose_right_msg.header.frame_id = object_;

    InterfaceState state(scene);
    tf::poseEigenToMsg(right, target_pose_right_msg.pose);
    state.properties().set("target_pose_right", target_pose_right_msg);

    tf::poseEigenToMsg(left, target_pose_left_msg.pose);
    state.properties().set("target_pose_left", target_pose_left_msg);

    SubTrajectory trajectory;
    trajectory.setCost(0.0);

    // add frame at target pose
    rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_right_msg, 0.1, "pose_right frame");
    rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_left_msg, 0.1, "pose_left frame");

    spawn(std::move(state), std::move(trajectory));
}

} } }
