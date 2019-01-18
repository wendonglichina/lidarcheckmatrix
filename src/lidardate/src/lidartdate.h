#ifndef __LIDARDATE_H__
#define __LIDARDATE_H__

#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

class lidarDateGet
{
  public:
    lidarDateGet();
    ~lidarDateGet();

    bool start(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    void lidarCloudReceived(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg);

  std::string lidar_topic_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber lidardates;
};

#endif