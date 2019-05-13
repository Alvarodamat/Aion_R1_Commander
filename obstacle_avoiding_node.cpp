#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <aion_r1/avoiding.h>
#include <sensor_msgs/LaserScan.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoidance_node");

    Avoidance avoid_node;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0);

    // wait for FCU connection
    while (ros::ok()){
      if(!avoid_node.getMavrosConnected()){
        ROS_INFO("Waiting to Pixhawk.");
      }else{
        //ROS_ERROR("entrando en step");
        avoid_node.step();
      }
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
