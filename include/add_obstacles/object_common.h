#pragma once
#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>

double angle2rad(double& angle)
{
    return (angle/180)*3.1415;
}


void addCollisionObjects(ros::Publisher& planning_scene_diff_publisher, moveit_msgs::PlanningScene& p,
double sx, double sy, double sz,\
double px, double py, double pz, \
double ox, double oy, double oz, \
std::string frame_id, std::string id, bool isInvisible=false)
{

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = id;
    collision_objects[0].header.frame_id = frame_id;

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = sx;
    collision_objects[0].primitives[0].dimensions[1] = sy;
    collision_objects[0].primitives[0].dimensions[2] = sz;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = px;
    collision_objects[0].primitive_poses[0].position.y = py;
    collision_objects[0].primitive_poses[0].position.z = pz;

    tf2::Quaternion orientation;
    orientation.setRPY(angle2rad(ox), angle2rad(oy), angle2rad(oz));
    collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(orientation);

    collision_objects[0].operation = collision_objects[0].ADD;

    // planning_scene_interface.applyCollisionObjects(collision_objects);


    p.world.collision_objects.push_back(collision_objects[0]);
    p.is_diff = true;
    p.robot_state.is_diff = true;
    
    if(isInvisible)
    {
        moveit_msgs::ObjectColor color;
        color.color.a = 0;
        color.id = collision_objects[0].id;
        p.object_colors.push_back(color);
    }
    planning_scene_diff_publisher.publish(p);
    ros::Duration(1).sleep();
}




