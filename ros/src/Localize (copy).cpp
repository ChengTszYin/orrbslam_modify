#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher pub;
float positionX, positionY, orientation;
float localizeX, localizeY, localizeZ;
bool pose_received = false;
bool taskFinish = true;
int nfeature;
int angle;
int startAngle = 0;

void localizeCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  localizeX = msg->pose.pose.position.x;
  localizeY = msg->pose.pose.position.y;
  localizeZ = msg->pose.pose.orientation.z;
}

void angleCallback_(const geometry_msgs::Vector3::ConstPtr& msgs)
{ 
  int old_angle = angle;
  angle = msgs->z;
  old_angle = old_angle==0 ? angle : old_angle;
  startAngle = (angle - old_angle)==0 ? startAngle : (startAngle + (angle - old_angle));
  pose_received = true;
}

void featCallback_(const std_msgs::Int16::ConstPtr& msgs)
{
  nfeature = msgs->data;
}

void vposeCallback_(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  positionX = msg->pose.position.x;
  positionY = msg->pose.position.y;
  orientation = msg->pose.orientation.z;
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
    pub.publish(move);
    ros::spinOnce();
    rate.sleep();
  }
  startAngle = 0;
  ROS_INFO("ESCAPER FORM WHILE LOOP");
  if(nfeature<2000){
    ROS_INFO("Localization Failed, Please retry at other localization");
  }
  pose_received = false;
  move.angular.z = 0;
  pub.publish(move);
  ROS_INFO("Localization finished");
  ROS_INFO("Vector3 are x: %f y: %f z: %f", localizeX,localizeY,localizeZ);
  ROS_INFO("nfeature: %d", nfeature);
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
  ros::Subscriber sub = nh.subscribe("/orb_slam2_pose",1000,vposeCallback_);
  ros::Subscriber sub2 = nh.subscribe("/featureLevel",1000,featCallback_);
  ros::Subscriber sub3 = nh.subscribe("/euler_angles",1000, angleCallback_);
  ros::Subscriber sub4 = nh.subscribe("/localizepose",1000, localizeCallback_);
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
  ros::ServiceServer service = nh.advertiseService("Localize_service",callback_);
  ROS_INFO("Ready to receive requests.");
  ros::spin();
  return 0;
}
