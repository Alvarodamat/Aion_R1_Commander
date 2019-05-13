#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/LaserScan.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>

#ifndef AVOIDING_H
#define AVOIDING_H

class Avoidance{

  private:

    ros::NodeHandle nh_;

    ros::Subscriber state_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber wp_sub;
    ros::Subscriber reached_sub;

    ros::Publisher vel_pub;
    ros::Publisher local_pos_pub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient wp_clear_client;
    ros::ServiceClient wp_pull_client;
    ros::ServiceClient wp_push_client;

    sensor_msgs::LaserScan obstacle_sensor_;

    ros::Time last_request;

    ros::Time timeflag1, timeflag2, timeflag3, time_start;

    int state_;

    static const int IDLE = 0;
    static const int FORWARD = 1;
    static const int TURNCLOCK = 2;
    static const int TURNCOUNTER = 3;

    bool obstacle_encountered_;
    bool timeflag_set_;
    bool avoiding_initialize_;
    bool auto_engaged_;

    bool waypoint_saved_;
    bool mission_set_;

    geometry_msgs::PoseStamped pose;

    geometry_msgs::TwistStamped clockwise_speed_;
    geometry_msgs::TwistStamped counterwise_speed_;
    geometry_msgs::TwistStamped forward_speed_;
    geometry_msgs::TwistStamped forward_corrected_;
    geometry_msgs::TwistStamped stop_;

    mavros_msgs::State current_state;
    mavros_msgs::WaypointReached reached_;

    mavros_msgs::WaypointClear wp_clear_srv;
    mavros_msgs::WaypointPull wp_pull_srv;
    mavros_msgs::WaypointPush wp_push_srv;

    mavros_msgs::WaypointList waypoints_;
    mavros_msgs::WaypointList mission_;
    mavros_msgs::WaypointList wp_saves;

  public:

    Avoidance();
    //~Avoidance();

    bool getMavrosConnected(){return current_state.connected;};
    mavros_msgs::State getMavrosState(){return current_state;};
    void setMavrosState(mavros_msgs::State msg){
      //ROS_ERROR("msg received");
      current_state = msg;};

    void state_cb(const mavros_msgs::State::ConstPtr& msg){setMavrosState(*msg);}
    void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan){obstacle_sensor_ = *scan;};
    void waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& waypoints){waypoints_ = *waypoints;};
    void reached_cb(const mavros_msgs::WaypointReached::ConstPtr& reached){reached_ = *reached;};

    void setModeGuided();
    void setModeAuto();

    void setTwist(geometry_msgs::TwistStamped& twist, float vx, float omegaz);
    void setMovements();

    void setObstacle(bool obstacle_encounter){obstacle_encountered_ = obstacle_encounter;};
    void setTimeflagsDone(bool timeflag_set){timeflag_set_ = timeflag_set;};
    void setInitializeAvoid(bool avoid_init){avoiding_initialize_ = avoid_init;};
    bool getObstacle(){return obstacle_encountered_;};
    bool getTimeflagsDone(){return timeflag_set_;};
    bool getInitializeAvoid(){return avoiding_initialize_;};

    void saveWaypoints();
    void getWaypoints();
    void clearWaypoints();
    void WaypointsPushback();
    void setMission();
    void EraseSaves();

    void setTimeflags();

    int getState(){return state_;};
    void setState(int state){state_ = state;};

    void step();

};
#endif
