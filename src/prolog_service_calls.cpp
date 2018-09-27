#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <memory>

#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include "world_control_msgs/SpawnModel.h"
#include "world_control_msgs/AttachModelToParent.h"
#include "world_control_msgs/SpawnPhysicsConstraint.h"

#define ROSPROLOG_TF_CACHE_TIME 10.0

PREDICATE(ros_init, 0) {
  if(!ros::isInitialized()) {
    int argc=0;
    ros::init(argc, (char**)NULL, std::string("knowrob"));
  }
  return TRUE;
}


/* ID, class_name, Location, Quaternion */
PREDICATE(spawn_model, 9) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<world_control_msgs::SpawnModel>("unreal/spawn_model");

    world_control_msgs::SpawnModel srv;
    srv.request.id = (char*)PL_A1;
    srv.request.name = (char*)PL_A2;
    srv.request.pose.position.x = (double)PL_A3;
    srv.request.pose.position.y = (double)PL_A4;
    srv.request.pose.position.z = (double)PL_A5;

    srv.request.pose.orientation.x = (double)PL_A7;
    srv.request.pose.orientation.y = (double)PL_A8;
    srv.request.pose.orientation.z = (double)PL_A9;
    srv.request.pose.orientation.w = (double)PL_A6;

    client.call(srv);
    return srv.response.success ? TRUE : FALSE;
}
/*
spawn_constraint([ParentId, ChildId,
    LX, LY, LZ, RX, RY, RZ, RW,
    LMX, LMY, LMZ, LinLimit,
    LinSoftConstraint, LinStiffness, LinDamping,
    S1M, S2M, TM, S1L, S2L, TL,
    AngSwingSoftConstraint, AngSwingStiffness, AngSwingDamping,
    AngTwistSoftConstraint, AngTwistStiffness, AngTwistDamping])
*/
PREDICATE(spawn_constraint, 1) {

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<world_control_msgs::SpawnPhysicsConstraint>("unreal/spawn_physics_constraint");


    world_control_msgs::SpawnPhysicsConstraint srv;

    PlTail args(PL_A1);
    PlTerm value;
    args.next(value);
    srv.request.constraint_details.id_first_model = (char*) value; args.next(value);
    srv.request.constraint_details.id_second_model = (char*) value; args.next(value);

    srv.request.pose.position.x = (double) value; args.next(value);
    srv.request.pose.position.y = (double) value; args.next(value);
    srv.request.pose.position.z = (double) value; args.next(value);

    srv.request.pose.orientation.x = (double) value; args.next(value);
    srv.request.pose.orientation.y = (double) value; args.next(value);
    srv.request.pose.orientation.z = (double) value; args.next(value);
    srv.request.pose.orientation.w = (double) value; args.next(value);

//    srv.request.constraint_details.disable_collision = false;
//    srv.request.constraint_details.enable_projection = true;
//    srv.request.constraint_details.parent_dominates = false;

    srv.request.constraint_details.linear_limits.x_motion = (int) value; args.next(value);
    srv.request.constraint_details.linear_limits.y_motion = (int) value; args.next(value);
    srv.request.constraint_details.linear_limits.z_motion = (int) value; args.next(value);
    srv.request.constraint_details.linear_limits.limit = (double) value; args.next(value);

    srv.request.constraint_details.linear_limits.use_advanced = true;
    srv.request.constraint_details.linear_limits.soft_constraint = (value == "true"); args.next(value);
    srv.request.constraint_details.linear_limits.stiffness = (double) value; args.next(value);
    srv.request.constraint_details.linear_limits.damping = (double) value; args.next(value);

    srv.request.constraint_details.angular_limits.swing_1_motion = (int) value; args.next(value);
    srv.request.constraint_details.angular_limits.swing_2_motion = (int) value; args.next(value);
    srv.request.constraint_details.angular_limits.twist_motion = (int) value; args.next(value);

    srv.request.constraint_details.angular_limits.swing_1_limit_angle = (double) value; args.next(value);
    srv.request.constraint_details.angular_limits.swing_2_limit_angle = (double) value; args.next(value);
    srv.request.constraint_details.angular_limits.twist_limit_angle = (double) value; args.next(value);
    srv.request.constraint_details.angular_limits.use_advanced = true;

    srv.request.constraint_details.angular_limits.swing_soft_constraint = (value == "true"); args.next(value);
    srv.request.constraint_details.angular_limits.swing_stiffness = (double) value; args.next(value);
    srv.request.constraint_details.angular_limits.swing_damping = (double) value; args.next(value);

    srv.request.constraint_details.angular_limits.twist_soft_constraint = (value == "true"); args.next(value);
    srv.request.constraint_details.angular_limits.twist_stiffness = (double) value; args.next(value);
    srv.request.constraint_details.angular_limits.twist_damping = (double) value;

    uint x = srv.request.constraint_details.linear_limits.x_motion;
    uint y = srv.request.constraint_details.linear_limits.y_motion;
    uint z = srv.request.constraint_details.linear_limits.z_motion;

    client.call(srv);
    return srv.response.success ? TRUE : FALSE;
}


/* attach_model_to_parent(Parent, Child). */
PREDICATE(attach_model_to_parent, 2) {

    	ros::NodeHandle n;
    	ros::ServiceClient client = n.serviceClient<world_control_msgs::AttachModelToParent>("unreal/attach_model_to_parent");

    	world_control_msgs::AttachModelToParent srv;
    	srv.request.parent_id = (char*)PL_A1;
    	srv.request.child_id = (char*)PL_A2;

    	client.call(srv);
        return srv.response.success ? TRUE : FALSE;
}
