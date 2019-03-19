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
#include "world_control_msgs/SpawnSemanticMap.h"
#include "world_control_msgs/HighlightModel.h"

#define ROSPROLOG_TF_CACHE_TIME 10.0

PREDICATE(ros_init, 0) {
  if(!ros::isInitialized()) {
    int argc=0;
    ros::init(argc, (char**)NULL, std::string("knowrob"));
  }
  return TRUE;
}

/* spawn_semantic_map(ModelProperties, ConstraintProperties, Relations)
 *
 * ModelProperties      PL_A1 = [[Id, Name,
                                (Location)LX, LY, LZ,
                                (Rotation)QW, QX, QY, QZ,
                                [(Tags)[Type, Key, Value], .. ], Mobility,
                                (PhysicsProperties) Gravity, Gernate_overlap, Mass ], ...]
 * ConstraintProperties PL_A2 = [[Id, ParentId, ChildId, LX, LY, LZ, RW, RX, RY, RZ,
                                LMX, LMY, LMZ, LinLimit, LinSoftConstraint, LinStiffness, LinDamping,
                                S1M, S2M, TM, S1L, S2L, TL,
                                AngSwingSoftConstraint, AngSwingStiffness, AngSwingDamping,
                                AngTwistSoftConstraint, AngTwistStiffness, AngTwistDamping], ...]
 * Relations            PL_A3 = [[Parent, Child], ...]
*/
PREDICATE(spawn_semantic_map, 3){
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<world_control_msgs::SpawnSemanticMap>("unreal/spawn_semantic_map");

    world_control_msgs::SpawnSemanticMap srv;

    // Add all models
    PlTail models(PL_A1);
    PlTerm model;

    while(models.next(model)){
        /* model = [Id, Name, LX, LY, LZ, QX, QY, QZ, QW] */
        world_control_msgs::ModelDescription description;

        // "fill out" description
        PlTail parts(model);
        PlTerm part;
        parts.next(part);
        description.id = (char*)part; parts.next(part);
        description.name = (char*)part; parts.next(part);
        //Location
        description.pose.position.x = (double)part; parts.next(part);
        description.pose.position.y = (double)part; parts.next(part);
        description.pose.position.z = (double)part; parts.next(part);
        //Orientation
        description.pose.orientation.w = (double)part; parts.next(part);
        description.pose.orientation.x = (double)part; parts.next(part);
        description.pose.orientation.y = (double)part; parts.next(part);
        description.pose.orientation.z = (double)part; parts.next(part);

        //TAGS
        PlTail tags(part);
        PlTerm tag;
        while(tags.next(tag)){
            world_control_msgs::Tag tagmsg;
            PlTail tag_triple(tag);
            PlTerm tag_part;
            tag_triple.next(tag_part);
            tagmsg.type = (char*)tag_part; tag_triple.next(tag_part);
            tagmsg.key = (char*)tag_part;tag_triple.next(tag_part);
            tagmsg.value = (char*)tag_part;
            description.tags.push_back(tagmsg);
        }
        parts.next(part);

        //Check the mobility
        if((std::string)part == "static") {
            description.physics_properties.mobility = description.physics_properties.STATIC;
        } else if ((std::string)part == "dynamic") {
            description.physics_properties.mobility = description.physics_properties.DYNAMIC;
        } else if ((std::string)part == "kinematic") {
            description.physics_properties.mobility = description.physics_properties.KINEMATIC;
        } else { //Stationary as default
            description.physics_properties.mobility = description.physics_properties.STATIONARY;
        }

        parts.next(part);

        //Physics Properties
        description.physics_properties.gravity = ((std::string)part == "true"); parts.next(part);
        description.physics_properties.generate_overlap_events = ((std::string)part == "true"); parts.next(part);
        description.physics_properties.mass = (double)part; parts.next(part);

        // add description to srv
        srv.request.models.push_back(description);
    }

    // Add all Constraints
    PlTail constraints(PL_A2);
    PlTerm constraint;
    while(constraints.next(constraint)){
        /* constraint = [Id, ParentId, ChildId,
            LX, LY, LZ, RW, RX, RY, RZ,
            LMX, LMY, LMZ, LinLimit,
            LinSoftConstraint, LinStiffness, LinDamping,
            S1M, S2M, TM, S1L, S2L, TL,
            AngSwingSoftConstraint, AngSwingStiffness, AngSwingDamping,
            AngTwistSoftConstraint, AngTwistStiffness, AngTwistDamping] */
        world_control_msgs::ConstraintDescription description;

        // "fill out" description
        PlTail parts(constraint);
        PlTerm part;
        parts.next(part);
        description.id = (char*) part; parts.next(part);
        description.constraint_details.id_first_model = (char*) part; parts.next(part);
        description.constraint_details.id_second_model = (char*) part; parts.next(part);

        description.pose.position.x = (double) part; parts.next(part);
        description.pose.position.y = (double) part; parts.next(part);
        description.pose.position.z = (double) part; parts.next(part);

        description.pose.orientation.w = (double) part; parts.next(part);
        description.pose.orientation.x = (double) part; parts.next(part);
        description.pose.orientation.y = (double) part; parts.next(part);
        description.pose.orientation.z = (double) part; parts.next(part);

        //    description.constraint_details.disable_collision = false;
        //    description.constraint_details.enable_projection = true;
        //    description.constraint_details.parent_dominates = false;

        description.constraint_details.linear_limits.x_motion = (int) part; parts.next(part);
        description.constraint_details.linear_limits.y_motion = (int) part; parts.next(part);
        description.constraint_details.linear_limits.z_motion = (int) part; parts.next(part);
        description.constraint_details.linear_limits.limit = (double) part; parts.next(part);

        description.constraint_details.linear_limits.use_advanced = true;
        description.constraint_details.linear_limits.soft_constraint = ((std::string)part == "true"); parts.next(part);
        description.constraint_details.linear_limits.stiffness = (double) part; parts.next(part);
        description.constraint_details.linear_limits.damping = (double) part; parts.next(part);

        description.constraint_details.angular_limits.swing_1_motion = (int) part; parts.next(part);
        description.constraint_details.angular_limits.swing_2_motion = (int) part; parts.next(part);
        description.constraint_details.angular_limits.twist_motion = (int) part; parts.next(part);

        description.constraint_details.angular_limits.swing_1_limit_angle = (double) part; parts.next(part);
        description.constraint_details.angular_limits.swing_2_limit_angle = (double) part; parts.next(part);
        description.constraint_details.angular_limits.twist_limit_angle = (double) part; parts.next(part);
        description.constraint_details.angular_limits.use_advanced = true;

        description.constraint_details.angular_limits.swing_soft_constraint = ((std::string)part == "true"); parts.next(part);
        description.constraint_details.angular_limits.swing_stiffness = (double) part; parts.next(part);
        description.constraint_details.angular_limits.swing_damping = (double) part; parts.next(part);

        description.constraint_details.angular_limits.twist_soft_constraint = ((std::string)part == "true"); parts.next(part);
        description.constraint_details.angular_limits.twist_stiffness = (double) part; parts.next(part);
        description.constraint_details.angular_limits.twist_damping = (double) part;


        // add description to srv
        srv.request.constraints.push_back(description);
    }


    // Add all Relations
    PlTail relations(PL_A3);
    PlTerm relation;

    while(relations.next(relation)){
        /* relation = [Parent, Child] */
        world_control_msgs::RelationDescription description;

        // "fill out" description
        PlTail parts(relation);
        PlTerm part;
        parts.next(part);
        description.parent_id = (char*)part; parts.next(part);
        description.child_id = (char*)part;

        // add description to srv
        srv.request.relations.push_back(description);
    }

    // calling actual service
    client.call(srv);

    if(srv.response.success){
        ROS_INFO("SUCCESS!");
    } else {
        ROS_ERROR("Not everything was spawned correctly!");
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
    srv.request.constraint_description.constraint_details.id_first_model = (char*) value; args.next(value);
    srv.request.constraint_description.constraint_details.id_second_model = (char*) value; args.next(value);

    srv.request.constraint_description.pose.position.x = (double) value; args.next(value);
    srv.request.constraint_description.pose.position.y = (double) value; args.next(value);
    srv.request.constraint_description.pose.position.z = (double) value; args.next(value);

    srv.request.constraint_description.pose.orientation.x = (double) value; args.next(value);
    srv.request.constraint_description.pose.orientation.y = (double) value; args.next(value);
    srv.request.constraint_description.pose.orientation.z = (double) value; args.next(value);
    srv.request.constraint_description.pose.orientation.w = (double) value; args.next(value);

    //    srv.request.constraint_description.constraint_details.disable_collision = false;
    //    srv.request.constraint_description.constraint_details.enable_projection = true;
    //    srv.request.constraint_description.constraint_details.parent_dominates = false;

    srv.request.constraint_description.constraint_details.linear_limits.x_motion = (int) value; args.next(value);
    srv.request.constraint_description.constraint_details.linear_limits.y_motion = (int) value; args.next(value);
    srv.request.constraint_description.constraint_details.linear_limits.z_motion = (int) value; args.next(value);
    srv.request.constraint_description.constraint_details.linear_limits.limit = (double) value; args.next(value);

    srv.request.constraint_description.constraint_details.linear_limits.use_advanced = true;
    srv.request.constraint_description.constraint_details.linear_limits.soft_constraint = ((std::string)value == "true"); args.next(value);
    srv.request.constraint_description.constraint_details.linear_limits.stiffness = (double) value; args.next(value);
    srv.request.constraint_description.constraint_details.linear_limits.damping = (double) value; args.next(value);

    srv.request.constraint_description.constraint_details.angular_limits.swing_1_motion = (int) value; args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.swing_2_motion = (int) value; args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.twist_motion = (int) value; args.next(value);

    srv.request.constraint_description.constraint_details.angular_limits.swing_1_limit_angle = (double) value; args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.swing_2_limit_angle = (double) value; args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.twist_limit_angle = (double) value; args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.use_advanced = true;

    srv.request.constraint_description.constraint_details.angular_limits.swing_soft_constraint = ((std::string)value == "true"); args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.swing_stiffness = (double) value; args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.swing_damping = (double) value; args.next(value);

    srv.request.constraint_description.constraint_details.angular_limits.twist_soft_constraint = ((std::string)value == "true"); args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.twist_stiffness = (double) value; args.next(value);
    srv.request.constraint_description.constraint_details.angular_limits.twist_damping = (double) value;


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


/* highlight_models(Ids) */
PREDICATE(highlight_models, 1){
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<world_control_msgs::HighlightModel>("unreal/highlight_models");

    bool all_highlighted = true;

    PlTail ids(PL_A1);
    PlTerm id;

    while(ids.next(id)){

        world_control_msgs::HighlightModel srv;

        srv.request.id = (char*)id;
        srv.request.color = srv.request.GREEN;

        if(client.call(srv))
        {
            all_highlighted = all_highlighted & srv.response.success;
        } else {
            ROS_ERROR("Failed to call highlight_model service");
            return FALSE;
        }
    }


    if(all_highlighted) {
        ROS_INFO("All models were highlighted!");
        return TRUE;
    } else {
        ROS_ERROR("At least one model was not hightlighted!");
        return FALSE;
    }
}
