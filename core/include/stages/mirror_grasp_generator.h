/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Hamburg University
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

/* Authors: Luca Lach
   Desc:    Generator Stage for mirrored poses
*/

#pragma once

#include <moveit/task_constructor/stage.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>

namespace moveit { namespace task_constructor { namespace stages {

class MirrorGraspGenerator : public moveit::task_constructor::MonitoringGenerator {
public:
    MirrorGraspGenerator(const std::string& name = "mirror grasp generator");

    bool canCompute() const override;
    bool compute() override;

    void spawnPoses(Eigen::Affine3d right, Eigen::Affine3d left,
                    planning_scene::PlanningSceneConstPtr scene);

    void setEndEffector(const std::string &eef);
    void setNamedPose(const std::string &pose_name);
    void setObject(const std::string &object);

    void setAngleDelta(double delta);
    void setYOffset(double y_offset);
    void setHandHeight(const double &hand_height);

protected:
    void onNewSolution(const moveit::task_constructor::SolutionBase& s) override;

    std::deque<planning_scene::PlanningSceneConstPtr> scenes_;

private:
    double z_min_, z_max_, z_current_;
    double x_min_, x_max_, x_current_;

    std::string object_;
};

} } }
