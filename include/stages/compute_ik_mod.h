//
// Created by llach on 24.11.17.
//
// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/task_constructor/container.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace moveit {
namespace core { MOVEIT_CLASS_FORWARD(RobotState) }
}

namespace moveit { namespace task_constructor { namespace stages {

class ComputeIKMod : public WrapperBase {
public:
    ComputeIKMod(const std::string &name, pointer &&child = Stage::pointer(), bool right = true);

	void onNewSolution(const SolutionBase &s) override;

	void setTimeout(double timeout);
	void setEndEffector(const std::string& eef);

    void setTargetPoseRight(const geometry_msgs::PoseStamped &pose);
    void setTargetPoseRight(const Eigen::Affine3d& pose, const std::string& link = "");

    void setTargetPoseLeft(const geometry_msgs::PoseStamped &pose);
    void setTargetPoseLeft(const Eigen::Affine3d& pose, const std::string& link = "");

	template <typename T>
	void setTargetPose(const T& p, const std::string& link = "") {
		Eigen::Affine3d pose; pose = p;
		setTargetPose(pose, link);
	}
	void setMaxIKSolutions(uint32_t n);
	void setIgnoreCollisions(bool flag);

private:
    bool right_;
};

} } }
