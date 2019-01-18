#include "lidarReceived..h"
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <thread>
#include <signal.h>

#define view_DEBUG 0
#define MATRIX_TOPIC 1

#if view_DEBUG
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
#endif
bool getLidarClound::start(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
	nh_ = nh;
	private_nh_ = private_nh;
	//雷达原始topic
	private_nh_.param("lidar_topic", lidar_topic_, std::string("/velodyne_points"));
	//matrix topic,用于发布
	private_nh_.param("matrix_topic", matrix_topic_, std::string("/matrix_points"));
	private_nh_.param("box2d_topic", box2d_topic_, std::string("/matrix_box2d"));
	//bag路径
	private_nh_.param("rosbag_path", rosbag_path_, std::string("/home/bags/lidar3d_loc/map_car/matrixandlidtopic_2019-01-14-14-25-04.bag"));
	private_nh_.param("camera_yaml_path", camera_yaml_path_, std::string("/../yaml/camera_point.yaml"));
	//回放数据或在线
	private_nh_.param("read_rosbag", read_rosbag_, true);
	//回放数据包含matrix
	private_nh_.param("matrix_bag", matrix_bag_, true);
	//发布处理后雷达数据
	private_nh_.param("velodyne_topic", velodyne_topic_, std::string("/velodyne_clound"));
	//摄像头方向角
	private_nh_.param("direction", direction_, 0);
	//标志间隔距离
	private_nh_.param("space", space_, 5);
	//标志线宽度
	private_nh_.param("line_width", line_width_, 1.0);
	private_nh_.param("round_lab", round_lab_, std::string("/round_lab_point"));
	private_nh_.param("marker_topic", marker_topic_, std::string("/MarkerArray"));

	sub_static_tf_msg_ = nh_.subscribe("/tf_static", 2, &getLidarClound::lidar_tansform, this);

	pub_pcl = nh_.advertise<sensor_msgs::PointCloud2> (velodyne_topic_, 1);
	pub_round = nh_.advertise<sensor_msgs::PointCloud2> (round_lab_, 1);
	markerArrayPub = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 2);

	car_extrinsic();
	camera_direction();
	checkTransformReceived();

	if(read_rosbag_ == true)
	{
		lidar_thread_ = new boost::thread(boost::bind(&getLidarClound::readTopicsFromRosbag, this));
	}
	else
	{
		lidardates = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic_, 2, &getLidarClound::lidarCloudReceived, this);
	}
	return true;

}
void getLidarClound::lidarCloudReceived(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg)
{
	if(laser_cloud_msg->header.frame_id != "velodyne")
	{
		std::cout<<"velydone cloud is empty"<<std::endl;
		return;
	}
	sensor_msgs::PointCloud2 output;  //ROS中点云的数据格式
  	//对数据进行处理
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	current_time = ros::Time::now();
  	//output = *laser_cloud_msg;
	
    pcl::fromROSMsg(*laser_cloud_msg,*cloud);
	cloud = lidarToChassis(cloud);
    sor.setInputCloud(cloud);
    sor.setMeanK(50);//50个临近点
    sor.setStddevMulThresh(0.5);//距离大于1倍标准方差

    sor.filter(*cloud);
	// for (size_t i = 0; i < cloud->points.size (); i++) 
	// {
	// 	//if(cloud_inliner->points[i].x>0)
	// 	{		
	// 		cloud_new->push_back(cloud->points[i]);
	// 	}
	// }
	
#if view_DEBUG
    viewer.showCloud(cloud);
#endif
	pcl::toROSMsg(*cloud,output);
	output.header.frame_id = "base";
	output.header.stamp = current_time;
  	pub_pcl.publish (output);
	Distance_label();
}
visualization_msgs::Marker getLidarClound::getMarker(pcl::PointXYZI points, std::string markernam, int ID)
{
	visualization_msgs::Marker marker;
    marker.header.frame_id="base";
    marker.header.stamp = current_time;
	marker.ns = "basic_shapes";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id =ID;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.z = 1;
    marker.color.b = 0;
    marker.color.g = 1;
    marker.color.r = 1;
    marker.color.a = 1;


    geometry_msgs::Pose pose;
    pose.position.x = points.x;
    pose.position.y = points.y;
    pose.position.z = points.z;
    marker.text=markernam;
    marker.pose=pose;
	return marker;
    //markerArray.markers.push_back(marker);
}
void getLidarClound::Distance_label()
{
	float x,y;
	bool lenth = true;
	int numb = 0,markerN = 1;

	sensor_msgs::PointCloud2 roundPoint;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker marker;

	pcl::PointXYZI point1;
	point1.z = 0;
	point1.intensity = 0;
	//标定线，左右宽1m，方向角angle，宽度待优化
	for (size_t i = 0; i < 6000; i++) 
	{
		//x2 = x1 * cos(alpha) + y1 * sin(alpha);
		//y2 = -x1 * sin(alpha) + y1 * cos(alpha);
		point1.x = i*0.01*cos(angle) + line_width_*sin(angle) + camera_point[0];
		std::cout<<"aaaaaaaaaaaaaaaaa"<<point1.x<<std::endl;
		point1.y = -0.01*i*sin(angle) + line_width_*cos(angle) + camera_point[1];
		cloud->push_back(point1);
		point1.x = i*0.01*cos(angle) - line_width_*sin(angle) + camera_point[0];
		point1.y = -0.01*i*sin(angle) - line_width_*cos(angle) + camera_point[1];
		cloud->push_back(point1);

		x = pow(i*0.01*cos(angle),2);
		y = pow(i*0.01*sin(angle),2);
		//距离车辆边缘5xi米标志
		if(int(sqrt(x + y))%5 == 0 and int(sqrt(x + y)) > 0 and lenth)
		{
			for(int j = 0;j<10;j++)
			{
				point1.x = i*0.01*cos(angle) + line_width_*(1+j*0.1)*sin(angle) + camera_point[0];
				point1.y = -0.01*i*sin(angle) + line_width_*(1+j*0.1)*cos(angle) + camera_point[1];
				cloud->push_back(point1);
				point1.x = i*0.01*cos(angle) - line_width_*(1+j*0.1)*sin(angle) + camera_point[0];
				point1.y = -0.01*i*sin(angle) - line_width_*(1+j*0.1)*cos(angle) + camera_point[1];
				cloud->push_back(point1);

			}
			marker = getMarker(point1,std::to_string(markerN*5) + "m", markerN);
			markerN ++;
			markerArray.markers.push_back(marker);
			lenth = false;
			numb = i;
		}
		if (!lenth and i - numb > 100)
		{
			lenth = true;
		}
	}
	
	//车辆底盘方西标志x,y正方方向
	for (size_t i = 0; i < 60; i++) 
	{
		point1.x = 0;
		point1.y = 0;
		point1.z = 0;
		if (i%2 == 0)
		{
			point1.x = i*0.01;
		}
		else
		{
			point1.y = i*0.01;
		}
		point1.intensity = 0;
		cloud->push_back(point1);

	}
	//原点等距圆标志线
	for(int i = 5;i < 60; i +=space_)
	{
		for(float j = 0; j <2*PI;j+=0.01)
		{
			point1.x = i*sin(j);
			point1.y = i*cos(j);
			cloud->push_back(point1);
		}
		marker = getMarker(point1,std::to_string(i) + "m", markerN);
		markerN ++;
		markerArray.markers.push_back(marker);
	}
	pcl::toROSMsg(*cloud,roundPoint);
	roundPoint.header.frame_id = "base";
	roundPoint.header.stamp = current_time;
  	pub_round.publish (roundPoint);
	markerArrayPub.publish(markerArray);

}
void getLidarClound::camera_direction()
{
	switch(direction_)
	{
		//前向
		case 0:
			angle = 0;
			camera_point[0] = camera_points[0].x;
			camera_point[1] = camera_points[0].y;
			break;
		//右前
		case 1:
			angle = PI/4;
			camera_point[0] = camera_points[1].x;
			camera_point[1] = camera_points[1].y;
			break;
		//右
		case 2:
			angle = PI/2;
			camera_point[0] = camera_points[2].x;
			camera_point[1] = camera_points[2].y;
			break;
		//右后
		case 3:
			angle = 3*PI/4;
			camera_point[0] = camera_points[3].x;
			camera_point[1] = camera_points[3].y;
			break;
		//后
		case 4:
			angle = PI;
			camera_point[0] = camera_points[4].x;
			camera_point[1] = camera_points[4].y;
			break;
		//左后	
		case 5:
			angle = 5*PI/4;	
			camera_point[0] = camera_points[5].x;
			camera_point[1] = camera_points[5].y;
			break;
		//左
		case 6:
			angle = 3*PI/2;	
			camera_point[0] = camera_points[6].x;
			camera_point[1] = camera_points[6].y;
			break;
		//左前
		case 7:
			angle = 7*PI/4;	
			camera_point[0] = camera_points[7].x;
			camera_point[1] = camera_points[7].y;
			break;
		default:
			angle = 0;	
			camera_point[0] = camera_points[0].x;
			camera_point[1] = camera_points[0].y;
		
	}

}

void getLidarClound::lidar_tansform(const tf2_msgs::TFMessagePtr &msg)
{
	for (unsigned int i = 0; i < msg->transforms.size(); i++)
    {
		geometry_msgs::TransformStamped stripped = msg->transforms[i];
		if(stripped.child_frame_id == "velodyne")
		{
			lidar_to_base_ = stripped.transform;
			get_static_tf_ = true;
		}
	}
}
void getLidarClound::checkTransformReceived()
{
    std::cout << "waiting static transform from before to after !" << std::endl;

    while (ros::ok())
    {
        if (get_static_tf_)
        {
            break;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ros::spinOnce();
        }
    }

    std::cout << "get static transform from before to after !" << std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarClound::lidarToChassis(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_in)
{

	Eigen::Vector3d r;
	Eigen::Vector3d t;
	//transformToRT(lidar_to_base_, r, t);
	tf2::getEulerYPR(lidar_to_base_.rotation, r[2], r[1], r[0]);

	t[0] = lidar_to_base_.translation.x;
	t[1] = lidar_to_base_.translation.y;
	t[2] = lidar_to_base_.translation.z;
	PointTranformer3D trans(r, t);
	for (int i = 0; i < laser_cloud_in->points.size(); i++)
    {
        pcl::PointXYZI po;
        trans.transPointToWorld(&laser_cloud_in->points[i], &po);

        laser_cloud_in->points[i] = po;
    }
	return laser_cloud_in;

}

void getLidarClound::readTopicsFromRosbag()
{
	std::vector<std::string> topics;

	topics.push_back(lidar_topic_);
	topics.push_back(matrix_topic_);
	topics.push_back(box2d_topic_);

	//读入rosbag
	std::cout << "rosbag path ->" << rosbag_path_ << std::endl;
	try
	{
		rosbag_.open(rosbag_path_, rosbag::bagmode::Read);
	}
	catch (const rosbag::BagIOException &e)
	{
		std::cout << "Please check rosbag_path_! ->" << rosbag_path_ << std::endl;
		return;
	}

	//读取bag帧
	rosbag::View view(rosbag_, rosbag::TopicQuery(topics));

	if (view.size() == 0)
	{
		std::cout << "Can't find topic in " << rosbag_path_ << std::endl;
		return;
	}
	rosbag::View::iterator it = view.begin();

	pub_raw_cloud = nh_.advertise<sensor_msgs::PointCloud2>(lidar_topic_, 1);
	pub_matrix_cloud = nh_.advertise<sensor_msgs::PointCloud2>(matrix_topic_, 1);
	pub_box2D = nh_.advertise<jsk_recognition_msgs::PolygonArray>(box2d_topic_, 1);
	
	//继续读入bag
	while (it != view.end() and ros::ok())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		
		if (it->getTopic() == lidar_topic_.c_str())
		{
			sensor_msgs::PointCloud2ConstPtr cloud_msg = it->instantiate<sensor_msgs::PointCloud2>();
			
			if (cloud_msg != NULL)
			{

				if (pub_raw_cloud.getNumSubscribers() != 0)
				{
					pub_raw_cloud.publish(cloud_msg);
				}
				lidarCloudReceived(cloud_msg);
			}
			
		}
		if (it->getTopic() == matrix_topic_.c_str() and matrix_bag_ == true)
		{
			sensor_msgs::PointCloud2ConstPtr matrix_msg = it->instantiate<sensor_msgs::PointCloud2>();
			if (matrix_msg != NULL)
			{
				if (pub_matrix_cloud.getNumSubscribers() != 0)
				{
					pub_matrix_cloud.publish(matrix_msg);
				}
			}
		} 
		if (it->getTopic() == box2d_topic_.c_str() and matrix_bag_ == true)
		{
			jsk_recognition_msgs::PolygonArrayPtr Pbox2DArrays_msg = it->instantiate<jsk_recognition_msgs::PolygonArray>();
			if (Pbox2DArrays_msg != NULL)
			{
				if (pub_box2D.getNumSubscribers() != 0)
				{
					pub_box2D.publish(Pbox2DArrays_msg);
				}
			}
		} 			
		++it;
	}

	rosbag_.close();
}

getLidarClound::getLidarClound()
{
 pause_ = false;

}
getLidarClound::~getLidarClound()
{}
void sigintHandler(int sig)
{
	printf("get shutdown signal \n");
	ros::shutdown();
}
void getLidarClound::car_extrinsic()
{
	//const float point;
	std::cout<<"PATH:  "<<camera_yaml_path_<<std::endl;
	YAML::Node config = YAML::LoadFile(camera_yaml_path_);
	camera_points[0].x = config["Fount"]["x"].as<float>();
	camera_points[0].y = config["Fount"]["y"].as<float>();
	camera_points[1].x = config["FtoR"]["x"].as<float>();
	camera_points[1].y = config["FtoR"]["y"].as<float>();
	camera_points[2].x = config["Right"]["x"].as<float>();
	camera_points[2].y = config["Right"]["y"].as<float>();
	camera_points[3].x = config["RtoB"]["x"].as<float>();
	camera_points[3].y = config["RtoB"]["y"].as<float>();
	camera_points[4].x = config["Back"]["x"].as<float>();
	camera_points[4].y = config["Back"]["y"].as<float>();
	camera_points[5].x = config["BtoL"]["x"].as<float>();
	camera_points[5].y = config["BtoL"]["y"].as<float>();
	camera_points[6].x = config["Left"]["x"].as<float>();
	camera_points[6].y = config["Left"]["y"].as<float>();
	camera_points[7].x = config["LtoF"]["x"].as<float>();
	camera_points[7].y = config["LtoF"]["y"].as<float>();
}
//boost::shared_ptr<getLidarClound> getLidar;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pub_lidartp");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	//Readbag readbag ;
	//getLidar.reset(new getLidarClound());
	getLidarClound *getLidar = new getLidarClound();
	// Override default sigint handler
	signal(SIGINT, sigintHandler);
	if (getLidar->start(nh, private_nh))
	{
		// run using ROS input
		ros::spin();
	}
	//lidar_thread_->interrupt();
	return (0);
}

