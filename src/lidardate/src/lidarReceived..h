#ifndef __LIDAR_RECEIVE_H__
#define __LIDAR_RECEIVE_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include "tf/tf.h"
#include "tf2/utils.h"
#include "tf/transform_listener.h"
#include "tf2_msgs/TFMessage.h"

#include "pointc_transform.h"
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

#include "jsk_recognition_msgs/PolygonArray.h"

#define PI 3.1415926

using std::atan2;
using std::cos;
using std::sin;
typedef struct set_PointXYZ
{
  float x;
  float y;
  float z;
}Points;

typedef struct cameraPoint
{
  float x;
  float y;
}Camera;
// enum directions
// {
//   Front = 0,
//   FrontR = 1,
//   RightB = 2,
//   Back ,
//   LeftB,
//   Left,
//   FrontL,
// };
class getLidarClound
{
  public:
    getLidarClound();
    ~getLidarClound();

  void readTopicsFromRosbag();
  bool start(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  void lidarCloudReceived(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg);
  void lidar_tansform(const tf2_msgs::TFMessagePtr &msg);
  void camera_direction();
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidarToChassis(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_in);
  void checkTransformReceived();
  void Distance_label();
  visualization_msgs::Marker getMarker(pcl::PointXYZI points, std::string markernam, int ID);
  void car_extrinsic();
  std::string lidar_topic_;
  std::string matrix_topic_;
  std::string velodyne_topic_;
  std::string round_lab_;
  std::string marker_topic_;
  std::string box2d_topic_;
  std::string rosbag_path_;
  std::string camera_yaml_path_;
  std::string read_bags_;
  int direction_;

  ros::Publisher pub_pcl;
  ros::Publisher pub_round;
  ros::Publisher pub_raw_cloud;
	ros::Publisher pub_matrix_cloud;
  ros::Publisher pub_box2D;
  ros::Publisher markerArrayPub;

  rosbag::Bag rosbag_;

  bool pause_;
  bool read_rosbag_;
  bool matrix_bag_;

  boost::thread *lidar_thread_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber lidardates;

  ros::Subscriber sub_static_tf_msg_;
  geometry_msgs::Transform lidar_to_base_;

  bool get_static_tf_;
  //摄像头方向角，前右后左
  float angle;
  double line_width_;
  //摄像头到底盘坐标
  float camera_point[2];
  int space_;
  Camera camera_points[8];

  ros::Time current_time;
};
#endif
