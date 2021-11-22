#include "ros/ros.h"
#include "ros/package.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream> 
#include <sstream>
#include <yaml-cpp/yaml.h>

class position_estimation
{
private:
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub;
    ros::Subscriber aruco_sub;

    tf::TransformListener listener;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped current_pose_;

    double target_angle_, current_angle_;
    std::string map_frame_ = "map";
    std::string pkg_path = ros::package::getPath("position_estimation");

public:
    position_estimation(/* args */);
    ~position_estimation();

    bool readFile(const std::string &filename);
    void ArucoCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &aruco_);
    void move();
};

position_estimation::position_estimation()
{
    /* 目標位置をyamlで読み込む */
    std::string filename = pkg_path + "/config/target_pose.yaml";

    if(filename != "")
    {
      ROS_INFO_STREAM("Read waypoints data from " << filename);

      if(!readFile(filename))
        ROS_ERROR("Failed loading waypoints file");
    }
    else
      ROS_ERROR("waypoints file doesn't have name");

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);
    aruco_sub = nh.subscribe("/ar_pose_marker", 100, &position_estimation::ArucoCallback, this);

}

position_estimation::~position_estimation()
{
}

//Function to read waypoint.yaml
bool position_estimation::readFile(const std::string &filename)
{
    double r, p, y;
    tf::Quaternion tf_quat;
    geometry_msgs::Quaternion quat;

    try
    {
      std::ifstream ifs(filename.c_str(), std::ifstream::in);
      if(ifs.good() == false){
        return false;
      }
      
      YAML::Node config = YAML::LoadFile(filename.c_str());
      const YAML::Node &wp_node_tmp = config;
      const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
      int end_waypoint = wp_node->size() - 1;
      if(wp_node != NULL)
      {
        target_pose_.header.frame_id = map_frame_;
        target_pose_.pose.position.x = (*wp_node)[end_waypoint]["pose"]["position"]["x"].as<float>() ;
        target_pose_.pose.position.y = (*wp_node)[end_waypoint]["pose"]["position"]["y"].as<float>() ;
        target_pose_.pose.position.z = (*wp_node)[end_waypoint]["pose"]["position"]["z"].as<float>() ;
        target_pose_.pose.orientation.x = (*wp_node)[end_waypoint]["pose"]["orientation"]["x"].as<float>() ;
        target_pose_.pose.orientation.y = (*wp_node)[end_waypoint]["pose"]["orientation"]["y"].as<float>() ;
        target_pose_.pose.orientation.z = (*wp_node)[end_waypoint]["pose"]["orientation"]["z"].as<float>() ;
        target_pose_.pose.orientation.w = (*wp_node)[end_waypoint]["pose"]["orientation"]["w"].as<float>() ;
        
        quat.x = target_pose_.pose.orientation.x;
        quat.y = target_pose_.pose.orientation.y;
        quat.z = target_pose_.pose.orientation.z;
        quat.w = target_pose_.pose.orientation.w;

        quaternionMsgToTF(quat, tf_quat);
        tf::Matrix3x3(tf_quat).getRPY(r, p, y); 

        target_angle_ = y * (180/M_PI);
        // std::cout << "target_angle_:=" << target_angle_ << std::endl;
        return true;

      }
      else
        return false;
    }
    catch(YAML::ParserException &e){
      return false; 
    }
    
  
}

void position_estimation::move()
{
    
    ROS_INFO("target pose x:=%3f", target_pose_.pose.position.x);
    ROS_INFO("target pose y:=%3f", target_pose_.pose.position.y);
    ROS_INFO("current robot pose x:= %3f", current_pose_.pose.position.x);
    ROS_INFO("current robot pose y:= %3f", current_pose_.pose.position.y);
    ROS_INFO("target angle:=%3f", target_angle_);
    ROS_INFO("current angle:=%3f", current_angle_);
    
    // 旋回条件
    if (abs(target_angle_ - current_angle_) <= 0.1)
    {
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);
        // x方向条件
        if (abs(target_pose_.pose.position.x - current_pose_.pose.position.x) <= 0.001)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel_pub.publish(cmd_vel);

            // y方向条件
            if (abs(target_pose_.pose.position.y - current_pose_.pose.position.y) <= 0.001)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub.publish(cmd_vel);
                std::cout << "位置補正終了します" << std::endl;

            }
            // y方向条件
            else
            {
                if ((target_pose_.pose.position.y - current_pose_.pose.position.y) >= 0)
                {
                    if (abs(target_pose_.pose.position.y - current_pose_.pose.position.y) <= 5.5 && abs(target_pose_.pose.position.y - current_pose_.pose.position.y) > 0.02)
                    {
                        std::cout << "y方向高速移動" << std::endl;
                        cmd_vel.linear.y = 0.10;
                    }   
                    else
                    {
                        std::cout << "y方向低速移動" << std::endl;
                        cmd_vel.linear.y = 0.05;
                    }
                    cmd_vel_pub.publish(cmd_vel);
                    
                }
                else if((target_pose_.pose.position.y - current_pose_.pose.position.y) < 0)
                {
                    if (abs(target_pose_.pose.position.y - current_pose_.pose.position.y) <= 5.5 && abs(target_pose_.pose.position.y - current_pose_.pose.position.y) > 0.02)
                    {
                        std::cout << "y方向低速移動" << std::endl;
                        cmd_vel.linear.y = -0.10;
                    }    
                    else
                    {
                        std::cout << "y方向低速移動" << std::endl;
                        cmd_vel.linear.y = -0.05;
                    }
                    cmd_vel_pub.publish(cmd_vel);
                }
            }
            
        }
        // x方向条件
        else
        {
            if ((target_pose_.pose.position.x - current_pose_.pose.position.x) >= 0)
            {
                if (abs(target_pose_.pose.position.x - current_pose_.pose.position.x) <= 5.5 && abs(target_pose_.pose.position.x - current_pose_.pose.position.x) > 0.02)
                {
                    std::cout << "x方向高速移動" << std::endl;
                    cmd_vel.linear.x = 0.10;
                }    
                else
                {
                    std::cout << "x方向低速移動" << std::endl;
                    cmd_vel.linear.x = 0.05;
                }
                cmd_vel_pub.publish(cmd_vel);
                
            }
            else if((target_pose_.pose.position.x - current_pose_.pose.position.x) < 0)
            {
                if (abs(target_pose_.pose.position.x - current_pose_.pose.position.x) <= 5.5 && abs(target_pose_.pose.position.x - current_pose_.pose.position.x) > 0.02)
                {
                    std::cout << "x方向高速移動" << std::endl;
                    cmd_vel.linear.x = -0.10; 
                }    
                else
                {
                    std::cout << "x方向低速移動" << std::endl;
                    cmd_vel.linear.x = -0.05;
                }
                cmd_vel_pub.publish(cmd_vel);
            }
        }
    }
    // 旋回条件
    else
    {
        if ((target_angle_ - current_angle_) >= 0)
        {
            std::cout << "左旋回中" << std::endl;
            cmd_vel.angular.z = 0.05;
            cmd_vel_pub.publish(cmd_vel);
        }
        else if((target_angle_ - current_angle_) < 0)
        {
            std::cout << "右旋回中" << std::endl;
            cmd_vel.angular.z = -0.05;
            cmd_vel_pub.publish(cmd_vel);
        }
        
    }


}

void position_estimation::ArucoCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &aruco_)
{
    uint32_t id;
    double roll, pitch, yaw;

    std::string robot_frame_ = "base_link";
    std::string aruco_frame_ = "ar_marker_";

    tf::Quaternion tf_quat_;
    tf::StampedTransform transform_ab;
    geometry_msgs::Quaternion quat_;
    geometry_msgs::PoseStamped aruco_pose_;

    for (int i = 0; i < aruco_->markers.size(); i++)
    {
        if (!aruco_->markers[i].id == 0)
        {
            id = aruco_->markers[i].id;
        }
    }

    aruco_frame_ += std::to_string(id);
    ROS_INFO("aruco_frame_:=%3s", aruco_frame_.c_str());

    try
    {   
        // marker -> base_link 
        listener.lookupTransform(aruco_frame_, robot_frame_, ros::Time(0), transform_ab);

        aruco_pose_.header.frame_id = aruco_frame_;
        aruco_pose_.pose.position.x = transform_ab.getOrigin().x();
        aruco_pose_.pose.position.y = transform_ab.getOrigin().y();
        aruco_pose_.pose.position.z = transform_ab.getOrigin().z();
        aruco_pose_.pose.orientation.x = transform_ab.getRotation().x();
        aruco_pose_.pose.orientation.y = transform_ab.getRotation().y();
        aruco_pose_.pose.orientation.z = transform_ab.getRotation().z();
        aruco_pose_.pose.orientation.w = transform_ab.getRotation().w();
        listener.transformPose(map_frame_, aruco_pose_, current_pose_);

        quat_.x = current_pose_.pose.orientation.x;
        quat_.y = current_pose_.pose.orientation.y;
        quat_.z = current_pose_.pose.orientation.z;
        quat_.w = current_pose_.pose.orientation.w;

        // yaw = marker -> base_link angle
        quaternionMsgToTF(quat_, tf_quat_);
        tf::Matrix3x3(tf_quat_).getRPY(roll, pitch, yaw);
        current_angle_ = yaw * (180/M_PI);

        move();
        
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    
}

int main(int argc, char **argv)
{
    // 初期化　wheel_clientノード                                                            
    ros::init(argc, argv, "Aruco_Position_Estimation");

    position_estimation pe;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}
