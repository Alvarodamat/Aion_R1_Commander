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

Avoidance::Avoidance()
: nh_(),
  state_(IDLE),
  obstacle_encountered_(false),
  timeflag_set_(false),
  avoiding_initialize_(false),
  waypoint_saved_(false),
  auto_engaged_(false),
  mission_set_(false),
  current_state(),
  obstacle_sensor_(),
  waypoints_(),
  mission_(),
  wp_saves()
{
  state_sub = nh_.subscribe<mavros_msgs::State>
          ("mavros/state", 10, &Avoidance::state_cb, this);
  scan_sub = nh_.subscribe<sensor_msgs::LaserScan>
          ("/scan", 1000, &Avoidance::scan_cb, this);
  wp_sub = nh_.subscribe<mavros_msgs::WaypointList>
          ("/mavros/mission/waypoints", 2, &Avoidance::waypoint_cb, this);
  reached_sub = nh_.subscribe<mavros_msgs::WaypointReached>
          ("/mavros/mission/reached", 2, &Avoidance::reached_cb, this);

  local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);
  vel_pub = nh_.advertise<geometry_msgs::TwistStamped>
          ("mavros/setpoint_velocity/cmd_vel", 1);

  arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  wp_clear_client = nh_.serviceClient<mavros_msgs::WaypointClear>
          ("mavros/mission/clear");
  wp_pull_client = nh_.serviceClient<mavros_msgs::WaypointPull>
          ("mavros/mission/pull");
  wp_push_client = nh_.serviceClient<mavros_msgs::WaypointPush>
          ("mavros/mission/push");

  //getWaypoints();
  setMovements();
  last_request = ros::Time::now();
}

void Avoidance::setModeGuided()
{

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "GUIDED";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  //ROS_INFO("Entro en el setModeGuided?");

  if(current_state.mode != "GUIDED" &&
      (ros::Time::now() - last_request > ros::Duration(5.0))){
      //ROS_INFO("Intento cambiar a modo guided?");
      //ROS_INFO("Se hace la llamada a cambiar modo? %i", set_mode_client.call(offb_set_mode));
      //ROS_INFO("Se envia la resuesta de modo enviado? %i", offb_set_mode.response.mode_sent);
      if( set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent){
          ROS_INFO("Mode changed to GUIDED. Offboard enabled.");
      }
      last_request = ros::Time::now();
  } else {
    //ROS_INFO("Intento armarlo en guided?");
      if( !current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success){
              ROS_INFO("Vehicle armed.");
          }
          last_request = ros::Time::now();
      }
  }
}

void Avoidance::setModeAuto()
{

  mavros_msgs::SetMode auto_set_mode;
  auto_set_mode.request.custom_mode = "AUTO";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ROS_INFO("Entro en el setModeAuto?");

  if(current_state.mode != "AUTO" &&
      (ros::Time::now() - last_request > ros::Duration(5.0))){
      ROS_INFO("Intento cambiar a modo auto?");
      if( set_mode_client.call(auto_set_mode) &&
          auto_set_mode.response.mode_sent){
          ROS_INFO("AUTO Mode engaged.");
          auto_engaged_ = true;
      }
      last_request = ros::Time::now();
  } else {
    //ROS_INFO("Intento armarlo en auto?");
      if( !current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success){
              ROS_INFO("Vehicle armed.");
          }
          last_request = ros::Time::now();
      }
  }
}

void Avoidance::setTwist(geometry_msgs::TwistStamped& twist, float vx, float omegaz)
{
  twist.twist.linear.x = vx;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = 0.0;
  twist.twist.angular.y = 0.0;
  twist.twist.angular.z = omegaz;
}

void Avoidance::setMovements()
{
  float straightspeed, turnspeed;
  straightspeed = 0.3;
  turnspeed = 0.45;
  clockwise_speed_.header.frame_id = "base_footprint";
  clockwise_speed_.header.stamp = ros::Time::now();
  counterwise_speed_.header.stamp = ros::Time::now();
  counterwise_speed_.header.frame_id = "base_footprint";

  setTwist(forward_speed_, straightspeed, 0.0);
  setTwist(clockwise_speed_, 0.0, -turnspeed);
  setTwist(counterwise_speed_, 0.0, turnspeed);
  setTwist(stop_, 0.0, 0.0);
}

void Avoidance::setTimeflags()
{
  ROS_INFO("Setting timeflags.");
  ros::Duration movetime(4.0);
  ros::Duration turntime(5.0);

  time_start = ros::Time::now();
  timeflag1 = ros::Time::now() + turntime;
  timeflag2 = timeflag1 + movetime;
  timeflag3 = timeflag2 + turntime;

  timeflag_set_= true;
}

void Avoidance::saveWaypoints()
{
  std::vector<mavros_msgs::Waypoint> wp_v;
  for(int i = 0; i < waypoints_.waypoints.size(); i++){
    mavros_msgs::Waypoint w;
    w.frame = waypoints_.waypoints[i].frame;
    w.command = waypoints_.waypoints[i].command;
    w.is_current = waypoints_.waypoints[i].is_current;
    w.autocontinue = waypoints_.waypoints[i].autocontinue;
    w.param1 = waypoints_.waypoints[i].param1;
    w.param2 = waypoints_.waypoints[i].param2;
    w.param3 = waypoints_.waypoints[i].param3;
    w.param4 = waypoints_.waypoints[i].param4;
    w.x_lat = waypoints_.waypoints[i].x_lat;
    w.y_long = waypoints_.waypoints[i].y_long;
    w.z_alt = waypoints_.waypoints[i].z_alt;

    wp_v.push_back(w);
  }
  wp_saves.waypoints = wp_v;
  ROS_INFO("Waypoints saved.");
}

void Avoidance::getWaypoints()
{
  wp_pull_srv.request = {};

  if(wp_pull_client.call(wp_pull_srv))
  {
    ROS_INFO("Waypoint list pulled from Autopilot.");
  }else{
    ROS_ERROR("Unable to get Waypoints.");
  }
}

void Avoidance::clearWaypoints()
{
    wp_clear_srv.request = {};

    if(wp_clear_client.call(wp_clear_srv))
    {
      ROS_INFO("Waypoint list cleared.");
    }else{
      ROS_ERROR("Unable to clear waypoint list.");
    }
}

void Avoidance::WaypointsPushback()
{
  if(wp_saves.waypoints.size() > 0){
    ROS_INFO("Calling the push service.");
    wp_push_srv.request.start_index = 0;
    //We push all the waypoints for this case.
    for(int i = 0; i < wp_saves.waypoints.size(); i++){
      ROS_INFO("Starting pushing function.");
      wp_push_srv.request.waypoints.push_back(wp_saves.waypoints[i]);
    }
    ROS_INFO("Pushing waypoints...");
  }

  if(wp_push_client.call(wp_push_srv))
  {
    ROS_INFO("Waypoints pushed.");
  }else{
    ROS_ERROR("Error pushing waypoint.");
  }
}

void Avoidance::setMission()
{
  ROS_INFO("Setting mission...");

  std::vector<mavros_msgs::Waypoint> wp_v;

  mavros_msgs::Waypoint waypoint1, waypoint2;
  int maxwaypoints = 2;

  waypoint1.frame = 3;
  waypoint1.command = 16;
  waypoint1.is_current = false;
  waypoint1.autocontinue = true;
  waypoint1.param1 = 0.0;
  waypoint1.param2 = 0.0;
  waypoint1.param3 = 0.0;
  waypoint1.param4 = 0.0;
  waypoint1.x_lat = 40.2831230164;
  waypoint1.y_long = -3.82083773613;
  waypoint1.z_alt = 0.0;

  waypoint2.frame = 3;
  waypoint2.command = 16;
  waypoint2.is_current = false ;
  waypoint2.autocontinue = true;
  waypoint2.param1 = 0.0;
  waypoint2.param2 = 0.0;
  waypoint2.param3 = 0.0;
  waypoint2.param4 = 0.0;
  waypoint2.x_lat = 40.2830085754;
  waypoint2.y_long = -3.82083916664;
  waypoint2.z_alt = 0.0;

  wp_v.push_back(waypoint1);
  wp_v.push_back(waypoint2);

  mission_.waypoints = wp_v;

  for(int i = 0; i < maxwaypoints; i++){
    wp_push_srv.request.waypoints.push_back(mission_.waypoints[i]);
  }

  if(wp_push_client.call(wp_push_srv))
  {
    ROS_INFO("Mission set.");
  }else{
    ROS_ERROR("Error. Couldn't upload mission.");
  }

}

void Avoidance::EraseSaves()
{
  std::vector<mavros_msgs::Waypoint> void_wp;
  waypoints_.waypoints = void_wp;
  wp_saves.waypoints = void_wp;
}

void Avoidance::step()
{

  if(current_state.mode != "AUTO" && !obstacle_encountered_){
    ROS_INFO("Cambiando a Auto");
    setModeAuto();
    auto_engaged_ = true;
  }

  if(!mission_set_){
    setMission();
    mission_set_ = true;
  }

  if(!obstacle_encountered_ && obstacle_sensor_.ranges.size() != 0 && obstacle_sensor_.ranges[0] < 0.75 &&
      !waypoint_saved_){
    setModeGuided();
    //getWaypoints();
    //ROS_INFO("Saving Waypoints...");
    //saveWaypoints();
    waypoint_saved_ = true;
    obstacle_encountered_ = true;
  }


  if(waypoint_saved_ && obstacle_encountered_ && !timeflag_set_){
    ROS_INFO("Setting timeflags...");
    setTimeflags();
  }

  ROS_WARN("[Antes de avoiding]Modo de vuelo: %s", current_state.mode.c_str());

  if(current_state.mode == "GUIDED" && obstacle_encountered_ && timeflag_set_){

    if(!avoiding_initialize_){
      //clearWaypoints();
      state_ = TURNCLOCK;
      avoiding_initialize_ = true;
    }

    ROS_INFO("State: %i", state_);

    switch(state_){
      case FORWARD:
        ROS_INFO("Me muevo recto");
        vel_pub.publish(forward_speed_);
        if(ros::Time::now() > timeflag2 && ros::Time::now() < timeflag3){
          vel_pub.publish(stop_);
          state_ = TURNCOUNTER;
        }
        break;

      case TURNCLOCK:
        ROS_INFO("Girando horario");
        vel_pub.publish(clockwise_speed_);
        if(ros::Time::now() > timeflag1 && ros::Time::now() < timeflag2){
          vel_pub.publish(stop_);
          state_ = FORWARD;
        }
        break;

      case TURNCOUNTER:
        ROS_INFO("Giro antihorario");
        vel_pub.publish(counterwise_speed_);
        if(ros::Time::now() > timeflag3){
          vel_pub.publish(stop_);
          state_ = IDLE;
        }
        break;

      case IDLE:
        ROS_INFO("Estoy Idle");
        ROS_INFO("Obstacle avoided. Resuming to mission.");
        obstacle_encountered_ = false;
        timeflag_set_ = false;
        avoiding_initialize_ = false;
        waypoint_saved_ = false;
        auto_engaged_ = false;

        //WaypointsPushback();

        break;
    }

    if(reached_.wp_seq == (waypoints_.waypoints.size()-1)){
      ROS_INFO("Mission Accomplished. Erasing Waypoints.");
      clearWaypoints();
    }

  }

}
