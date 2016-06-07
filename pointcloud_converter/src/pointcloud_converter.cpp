#include <ros/ros.h>
#include <boost/thread.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

ros::Publisher pointcloud_pub, pointcloud2_pub;

void pointcloud_callback(const sensor_msgs::PointCloudConstPtr &msg)
{
  sensor_msgs::PointCloud2 pointcloud2;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, pointcloud2);
  pointcloud2_pub.publish(pointcloud2);
}

void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  sensor_msgs::PointCloud pointcloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, pointcloud);
  pointcloud_pub.publish(pointcloud);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_converter");
  ros::NodeHandle nh("~");

  bool pointcloud_to_pointcloud2 = true;
  nh.param("pointcloud_to_pointcloud2", pointcloud_to_pointcloud2,  pointcloud_to_pointcloud2);
  ros::Subscriber pointcloud_sub, pointcloud2_sub;
  if(pointcloud_to_pointcloud2)
  {
    pointcloud_sub = nh.subscribe("pointcloud", 10, pointcloud_callback);
    pointcloud2_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud2", 10);
  }else 
  {
    pointcloud2_sub = nh.subscribe("pointcoud2", 10, pointcloud2_callback);
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud>("pointcloud", 10);
  }


  

  ros::spin();

  return (0);
}
