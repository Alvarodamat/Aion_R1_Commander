#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 1);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(3.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ROS_INFO("Waiting to Pixhawk");
        ros::spinOnce();
        rate.sleep();
    }

    int state_;
    static const int IDLE = 0;
    static const int FORWARD = 1;
    static const int TURNCLOCK = 2;
    static const int TURNCOUNTER = 3;

    bool obstacle_encountered = false;
    bool timeflag_set = false;
    bool avoiding_initialize = false;


    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.2;

    geometry_msgs::Vector3 yaw_linear_vel;
    yaw_linear_vel.x = 0.0;
    yaw_linear_vel.y = 0.0;
    yaw_linear_vel.z = 0.0;
    geometry_msgs::Vector3 yaw_angular_vel;
    yaw_angular_vel.x = 0.0;
    yaw_angular_vel.y = 0.0;
    yaw_angular_vel.z = 0.2;


    geometry_msgs::Vector3 straight_linear_vel;
    straight_linear_vel.x = 0.2;
    straight_linear_vel.y = 0.0;
    straight_linear_vel.z = 0.0;
    geometry_msgs::Vector3 straight_angular_vel;
    straight_angular_vel.x = 0.0;
    straight_angular_vel.y = 0.0;
    straight_angular_vel.z = 0.0;

    geometry_msgs::Twist yaw_move;
    yaw_move.linear = yaw_linear_vel;
    yaw_move.angular = yaw_angular_vel;

    geometry_msgs::TwistStamped stamped_yaw_move;
    stamped_yaw_move.twist = yaw_move;

    geometry_msgs::Twist straight_move;
    straight_move.linear = straight_linear_vel;
    straight_move.angular = straight_angular_vel;

    geometry_msgs::TwistStamped stamped_straight_move;
    stamped_straight_move.twist = straight_move;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    state_ = IDLE;

    ros::Time timeflag1, timeflag2, timeflag3, timeflag4, timeflag5, timeflag6, timeflag7, time_start;
    ros::Time timereset(0.0);
    ros::Duration movetime(2.0);
    ros::Duration turntime(3.0);

    if(!timeflag_set){
      ROS_INFO("timeflag_set");
      time_start = ros::Time::now();
      timeflag1 = ros::Time::now() + turntime;
      timeflag2 = timeflag1 + movetime;
      timeflag3 = timeflag2 + turntime;
      timeflag4 = timeflag3 + movetime;
      timeflag5 = timeflag4 + turntime;
      timeflag6 = timeflag5 + movetime;
      timeflag7 = timeflag6 + turntime;

      timeflag_set = true;
    }

    obstacle_encountered = true;

    while(ros::ok()){
        if( current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(obstacle_encountered){
          ROS_INFO("obstacle_encountered");
          if(!avoiding_initialize){
            state_ = TURNCLOCK;
            avoiding_initialize = true;
          }

          switch(state_){
            case FORWARD:
              std::cout<<"Me muevo recto"<<std::endl;
              state_ = TURNCOUNTER;
              break;
            case TURNCLOCK:
              std::cout<<"Giro horario"<<std::endl;
              if(ros::Time::now() > time_start + ros::Duration(25)){
                state_ = FORWARD;
              }
              break;
            case TURNCOUNTER:
              std::cout<<"Giro Antihorario"<<std::endl;
              state_ = IDLE;
              break;
            case IDLE:
              std::cout<<"Estoy Idle de la life"<<std::endl;
              break;
          }
        }

        vel_pub.publish(stamped_yaw_move);
      //  vel_pub.publish(stamped_straight_move);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
