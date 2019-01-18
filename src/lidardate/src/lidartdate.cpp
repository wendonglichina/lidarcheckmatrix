#include "lidardate.h"
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <thread>
#include <signal.h>

bool lidarDateGet::start(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
    nh_ = nh;
	private_nh_ = private_nh;
    private_nh_.param("lidar_topic", lidar_topic_, std::string("/velodyne_points"));
    lidardates = nh_.subscribe(lidar_topic_, 2, &lidarDateGet::lidarCloud, this);
}
void lidarDateGet::lidarCloud(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg)
{
    
}