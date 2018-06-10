#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <pepper_mtc_msgs/PepperFindGraspPlan.h>

void spawnObject(std::string id, int primitive_type) {
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::CollisionObject o;
    o.id= id;
    o.header.frame_id= "base_footprint";
    o.primitive_poses.resize(1);
    o.primitive_poses[0].position.x = 0.31;
    o.primitive_poses[0].position.y = -0.02;
    o.primitive_poses[0].position.z = 0.93;
    o.primitive_poses[0].orientation.w = 1.0;
//    o.primitive_poses[0].position.x = 0.31;
//    o.primitive_poses[0].position.y = 0;
//    o.primitive_poses[0].position.z = 0.93;
//    o.primitive_poses[0].orientation.w = 1.0;
    o.primitives.resize(1);

    switch(primitive_type){
        case(0):
        {
            o.primitive_poses[0].position.x = 0.32;
            o.primitive_poses[0].position.z = 0.856;
            o.primitive_poses[0].orientation.w = 1.0;

            std::cout << "spawning a floxx" << std::endl;

            o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
            o.primitives[0].dimensions.resize(3);
            o.primitives[0].dimensions[0]= 0.273;
            o.primitives[0].dimensions[1]= 0.355;
            o.primitives[0].dimensions[2]= 0.055;

            break;
        }
        case(1):
        {
            std::cout << "spawning a cylinder" << std::endl;

            o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
            o.primitives[0].dimensions.resize(2);
            o.primitives[0].dimensions[0]= 0.4;
            o.primitives[0].dimensions[1]= 0.1;

            break;
        }
        case(2):
        {
            std::cout << "spawning a sphere" << std::endl;

            o.primitives[0].type= shape_msgs::SolidPrimitive::SPHERE;
            o.primitives[0].dimensions.resize(1);
            o.primitives[0].dimensions[0]= 0.11;

            break;
        }
        default: {

            std::cout << "spawning a box" << std::endl;

            o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
            o.primitives[0].dimensions.resize(3);
            //o.primitives[0].dimensions[0]= 0.15;
            //o.primitives[0].dimensions[1]= 0.23;
            //o.primitives[0].dimensions[2]= 0.10;
            o.primitives[0].dimensions[0]= 0.15;
            o.primitives[0].dimensions[1]= 0.23;
            o.primitives[0].dimensions[2]= 0.10;

            break;
        }

    }

    psi.applyCollisionObject(o);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_spawner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::ServiceClient client = nh.serviceClient<pepper_mtc_msgs::PepperFindGraspPlan>("/pepper_grasping");

    int p = 0;
    nh_private.getParam("primitive", p);

    std::string obj = "object";
    spawnObject(obj, p);

    pepper_mtc_msgs::PepperFindGraspPlan srv;
    srv.request.object_uuid = obj;

    ROS_INFO_STREAM("Waiting for pepper_grasping service ...");
    client.waitForExistence();
    ROS_INFO_STREAM("Found pepper_grasping service!");

    client.call(srv);
    return 0;
}
