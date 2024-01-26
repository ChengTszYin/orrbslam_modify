#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

ros::Publisher vel_pub;
ros::Publisher pose_pub;
bool pose_received = false;
bool taskFinish = true;
int nfeature;
int ptrSize;
int angle;
int startAngle = 0;
ros::ServiceClient lidar_client_;
ros::ServiceClient orb_client_;
std_srvs::Trigger srv;


void angleCallback_(const std_msgs::Int16::ConstPtr& msgs)
{ 
  int old_angle = angle;
  angle = msgs->data;
  old_angle = old_angle==0 ? angle : old_angle;
  startAngle = (angle - old_angle)==0 ? startAngle : (startAngle + (angle - old_angle));
  pose_received = true;
}

void featCallback_(const std_msgs::Int16::ConstPtr& msgs)
{
  nfeature = msgs->data;
}

void ptrCallback_(const std_msgs::Int16::ConstPtr& msgs)
{
  ptrSize = msgs->data;
}

void StartLocalize()
{ 
  geometry_msgs::Twist move;
  move.angular.z = 0.4;
  ros::Rate rate(5);
  while(nfeature<2000 && startAngle >= 0)
  { 
    //std::cout << nfeature << std::endl;
    std::cout << "angle: " << startAngle << std::endl; 
    vel_pub.publish(move);
    ros::spinOnce();
    rate.sleep();
  }
  startAngle = 0;
  ROS_INFO("ESCAPER FORM WHILE LOOP");
  if(nfeature<2000)
  {
    ROS_INFO("Localization Failed, Please retry at other localization");
  }
  pose_received = false;
  move.angular.z = 0;
  vel_pub.publish(move);
  std::cout << "ptrSize: " << ptrSize << std::endl; 
  if(nfeature >= 2000 && ptrSize > 135)
  {
    if(lidar_client_.call(srv))
    {
      ROS_INFO("Service Response: %s",srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }
  }
  else if(nfeature >= 2000 && ptrSize <= 135)
  {
    if(orb_client_.call(srv))
    {
      ROS_INFO("Service Response: %s",srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }
  }
  else
  {
    ROS_INFO("Localization Failed. Please Localiza again near the wall");
  }
  ROS_INFO("Localization finished");
  taskFinish = false;
}

bool callback_(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  taskFinish = true;
  while(!pose_received)
  { 
    ROS_INFO("Service is not available. Please double check the /odom");
    ros::spinOnce();
  }

  while(taskFinish)
  {
    StartLocalize();
    if(!taskFinish)
    {
      ROS_INFO("Breaking out of the loop");
      break;
    }
  }
  ROS_INFO("Breaking out of the bool loop");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Localize_node");
  ros::NodeHandle nh;
  ros::Subscriber feature_sub_ = nh.subscribe("/featureLevel",1000,featCallback_);
  ros::Subscriber euler_sub_ = nh.subscribe("/euler_angles",1000, angleCallback_);
  ros::Subscriber ptr_sub_ = nh.subscribe("/pointcloud_size",1000,ptrCallback_);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
  ros::ServiceServer service = nh.advertiseService("Localize_service",callback_);
  lidar_client_ = nh.serviceClient<std_srvs::Trigger>("/StartLocalization");
  orb_client_ = nh.serviceClient<std_srvs::Trigger>("/orb_servicew");
  ROS_INFO("Ready to receive requests.");
  ros::spin();
  return 0;
}
