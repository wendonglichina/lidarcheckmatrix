#ifndef __GET_MATRIX_
#define __GET_MATRIX_

#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <memory>

#include <zeromq/zmq.h>
#include <protocol/frame.pb.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "geometry_msgs/Polygon.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
//#include "Polygons_msg/Polygons.h"

#include "jsk_recognition_msgs/PolygonArray.h"

class getMatrixObs
{
private:
	void * m_context;
	void * m_requester;
	zmq_msg_t m_recv_msg;

public:
	getMatrixObs();
	~getMatrixObs();

	void init();
	void close();
	int RecvFrame();
	void * ThreadRecieve();
	void ReConnect();
	bool start(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
	geometry_msgs::PolygonStamped getBox2D(CommonProto::Point point,float width,float height,float yaw,int num);

	FrameProtocol::Frame *frame;
	char   m_endpoint[100];
	unsigned int nRecvFailCount = 0;
	bool g_bStopReceive = false;
	std::string matrix_topic_;
	std::string box2d_topic_;
	int camera_id_;
	std::string server_IP_;
	int recv_faile_count_;
	std::fstream fs_record_file;

	ros::Publisher pub_matrix;
	ros::Publisher box2D_pub;

	boost::thread *Recieve_thread_;

	ros::NodeHandle nh_;
  	ros::NodeHandle private_nh_;

	ros::Time current_time;
};
#endif // !__GET_MATRIX_
