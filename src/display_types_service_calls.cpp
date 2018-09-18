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
#include "unreal_vis_msgs/DisplayBasicMarker.h"
#include "unreal_vis_msgs/DisplayTrajectory.h"
#include "unreal_vis_msgs/RemoveMarker.h"

#define ROSPROLOG_TF_CACHE_TIME 10.0

PREDICATE(ros_init, 0) {
  if(!ros::isInitialized()) {
    int argc=0;
    ros::init(argc, (char**)NULL, std::string("knowrob"));
  }
  return TRUE;
}


/* display_basic_marker( Markertype, Pose, Color, Scale, ID )
 *
 * Markertype PL_A1 = int
 * Pose       PL_A2 = [pX, pY, PZ, oX, oY, oZ, oW] (all double)
 * Color      PL_A3 = [R, G, B, A] (all double)
 * Scale      PL_A4 = double
 * ID         PL_A5 = char*
 *
 */
PREDICATE(display_basic_marker, 5) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<unreal_vis_msgs::DisplayBasicMarker>("unreal/display_basic_marker");

    unreal_vis_msgs::DisplayBasicMarker srv;

    PlTerm value;

    // Set Type
    srv.request.marker_type = (int)PL_A1;

    //Get pose elements out of list [pX, pY, PZ, oX, oY, oZ, oW]
    PlTail in_pose(PL_A2);
    in_pose.next(value);

    srv.request.pose.position.x = (double)value;
    in_pose.next(value);
    srv.request.pose.position.y = (double)value;
    in_pose.next(value);
    srv.request.pose.position.z = (double)value;
    in_pose.next(value);

    srv.request.pose.orientation.x = (double)value;
    in_pose.next(value);
    srv.request.pose.orientation.y = (double)value;
    in_pose.next(value);
    srv.request.pose.orientation.z = (double)value;
    in_pose.next(value);
    srv.request.pose.orientation.w = (double)value;

    //Get Color elements out of list [R, G, B, A]
    PlTail in_color(PL_A3);
    in_color.next(value);

    srv.request.color.r = (double)value;
    in_color.next(value);
    srv.request.color.g = (double)value;
    in_color.next(value);
    srv.request.color.b = (double)value;
    in_color.next(value);
    srv.request.color.a = (double)value;
    in_color.next(value);

    //set scale and id
    srv.request.scale = (double)PL_A4;
    srv.request.marker_id = (char*)PL_A5;

    // call service and give appropriate response
    if(client.call(srv)){
        return TRUE;
    } else {
        return FALSE;
    }

}

/* display_trajectory (points, ColorBegin, ColorEnd, ID)
 *
 * points     PL_A1 = [[X, Y, Z], [X, Y, Z], ...] (list of list of 3 doubles)
 * ColorBegin PL_A2 = [R, G, B, A] (all double)
 * ColorEnd   PL_A3 = [R, G, B, A] (all double)
 * ID         PL_A4 = char*
 *
 */
PREDICATE(display_trajectory_marker, 4) {

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<unreal_vis_msgs::DisplayTrajectory>("unreal/display_trajectory");

    PlTerm value;
    unreal_vis_msgs::DisplayTrajectory srv;

    // process points
    PlTerm in_point;
    PlTail in_points(PL_A1);
    in_points.next(in_point);

    //iterate over in_points list
    while(in_points.next(in_point))
    {
        //push back single point.
        PlTail point_tail(in_point);
        point_tail.next(value);
        geometry_msgs::Point point;

        point.x = (double)value;
        point_tail.next(value);
        point.y = (double)value;
        point_tail.next(value);
        point.z = (double)value;
        srv.request.points.push_back(point);

        in_points.next(in_point);
    }


    //Get Color elements out of lists [R, G, B, A]
    PlTail in_start_color(PL_A2);
    in_start_color.next(value);

    srv.request.color_begin.r = (double)value;
    in_start_color.next(value);
    srv.request.color_begin.g = (double)value;
    in_start_color.next(value);
    srv.request.color_begin.b = (double)value;
    in_start_color.next(value);
    srv.request.color_begin.a = (double)value;
    in_start_color.next(value);


    PlTail in_end_color(PL_A3);
    in_end_color.next(value);

    srv.request.color_end.r = (double)value;
    in_end_color.next(value);
    srv.request.color_end.g = (double)value;
    in_end_color.next(value);
    srv.request.color_end.b = (double)value;
    in_end_color.next(value);
    srv.request.color_end.a = (double)value;
    in_end_color.next(value);

    //ID
    srv.request.marker_id = (char*)PL_A4;

    // call service and give appropriate response
    if(client.call(srv)){
        return TRUE;
    } else {
        return FALSE;
    }
}


/* remove_marker(ID) */
PREDICATE(remove_marker, 1) {

    	ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<unreal_vis_msgs::RemoveMarker>("unreal/remove_marker");

        unreal_vis_msgs::RemoveMarker srv;
        srv.request.marker_id = (char*)PL_A1;

        // call service and give appropriate response
        if(client.call(srv)){
            return srv.response.success ? TRUE : FALSE;
        } else {
            return FALSE;
        }
}
