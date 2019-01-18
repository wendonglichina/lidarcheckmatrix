#include "getMatrixObstacle.h"
#include <signal.h>
#define PI 3.1415926

void *getMatrixObs::ThreadRecieve() {	

	bool bRecvFail = false;
	int nRecvFailCount = 0;
	int64_t last_ts = 0;
	while (!g_bStopReceive and ros::ok())
	{
		current_time = ros::Time::now();
		if (!RecvFrame()) {
			std::cout << "Recv Frame Timeout" << std::endl;
			nRecvFailCount++;
			if (nRecvFailCount > 20) {
				g_bStopReceive = true;
				break;
			}
		}
		else
		{
			nRecvFailCount = 0;
		}
	} // end of while
	return 0;
}
getMatrixObs::getMatrixObs()
{
}
void getMatrixObs::close()
{
	zmq_msg_close(&m_recv_msg);
	zmq_close(m_requester);
	zmq_ctx_destroy(m_context);
}

getMatrixObs::~getMatrixObs()
{
	zmq_msg_close(&m_recv_msg);
	zmq_close(m_requester);
	zmq_ctx_destroy(m_context);
}
void getMatrixObs::ReConnect() {
  close();
  init();
}

// get meta data from matrix
int getMatrixObs::RecvFrame() {

	pcl::PointXYZI point1;
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	geometry_msgs::PolygonStamped box2d;
	//geometry_msgs::Polygon box2dpoint;
	jsk_recognition_msgs::PolygonArray Pbox2DArrays;

	zmq_msg_t msg_proto;
	int rc = zmq_msg_init(&msg_proto);
	assert(rc == 0);
	rc = zmq_msg_recv(&msg_proto, m_requester, 0);
	if (rc == -1) {
		recv_faile_count_++;
		std::cout << rc <<std::endl;
		return 0;
	}

	int protolen = zmq_msg_size(&msg_proto);
	//frame = new FrameProtocol::Frame();
	FrameProtocol::FrameShuffle* frame = new FrameProtocol::FrameShuffle(); 
	int rec = frame->ParsePartialFromArray(zmq_msg_data(&msg_proto), protolen);
	//std::cout << rec <<std::endl;
	if (frame->frame_id() > 0)
	{
		const FrameV1Proto::FrameShuffle &fproto = frame->frame_v1();
		const FrameV1Proto::StructurePerception &sperception = fproto.structure_perception();
		if (sperception.obstacles().size()>camera_id_)
		{
			int camera_num = 0;
			int camera_read = 0;
			if (camera_id_ == 0)
			{
				camera_num = sperception.obstacles().size();
				std::cout << "size:"<<camera_num<<std::endl;
			}
			else 
			{
				camera_num = camera_id_;
				camera_read = camera_id_ -1;
			}
			for(int num = camera_read;num < camera_num;num ++)
			{
				std::cout << "Com:"<<num<<std::endl;
				const CommonProto::Obstacles &Sobstacles = sperception.obstacles(num);
		
				for (int i = 0; i < Sobstacles.obstacle().size(); i++)
				{
					const CommonProto::WorldSpaceInfo &sworld = Sobstacles.obstacle(i).world_info();
					//type 0:VehicleRear 1:VehicleFull 2:Pedestrian 3:TrafficSign 4:TrafficLight
					std::cout << "type:" << Sobstacles.obstacle(i).type()<<
						"  ID: " << Sobstacles.obstacle(i).id() <<
						"  life_time: " << Sobstacles.obstacle(i).life_time() <<
						"  X: " << sworld.position().x() <<
						"  Y: " << sworld.position().y() <<
						"  Z: " << sworld.position().z() << std::endl;
					
					point1.x = sworld.position().x();
					point1.y = sworld.position().y();
					point1.z = sworld.position().z();
					point1.intensity = 0;
					cloud->push_back(point1);
					if (sworld.has_length() or sworld.has_width() or sworld.has_yaw()) {
						std::cout<<"width:"<<sworld.width()<<"   height:"<<sworld.height()<<"   yaw:"<<sworld.yaw()<<std::endl;
						box2d = getBox2D(sworld.position(),sworld.width(),sworld.height(),sworld.yaw(),i);
						Pbox2DArrays.polygons.push_back(box2d);
					}
				}
			}
			pcl::toROSMsg(*cloud,output);
			output.header.stamp = current_time;
			output.header.frame_id = "base";
  			pub_matrix.publish (output);
			Pbox2DArrays.header.stamp = current_time;
			Pbox2DArrays.header.frame_id = "base";
			box2D_pub.publish(Pbox2DArrays);
		}
		else
		{
			std::cout << "no camera id: " <<camera_id_<<"     camera number is:"<<sperception.obstacles().size()<< std::endl;
			zmq_msg_close(&msg_proto);
			return 0;
		}
		
	}
	// else
	// {
	// 	std::cout << "no frame id" << std::endl;
	// 	zmq_msg_close(&msg_proto);
	// 	return 0;
	// }
	zmq_msg_close(&msg_proto);
	return 1;
}

geometry_msgs::PolygonStamped getMatrixObs::getBox2D(CommonProto::Point point,float width,float height,float yaw ,int num)
{
	
	geometry_msgs::Point32 Points[4];
	geometry_msgs::Polygon box2dPoint;
	geometry_msgs::PolygonStamped box2d;
	float min[2];
    min[0] = width/2*std::cos(yaw + PI / 2);
    min[1] = width/2*std::sin(yaw + PI / 2);

	Points[0].x = point.x() + min[0];
	Points[0].y = point.y() + min[1];
	Points[0].z = point.z();
	Points[1].x = point.x() - min[0];
	Points[1].y = point.y() - min[1];
	Points[1].z = point.z();
	Points[2].x = point.x() - min[0];
	Points[2].y = point.y() - min[1];
	Points[2].z = point.z() + height;
	Points[3].x = point.x() + min[0];
	Points[3].y = point.y() + min[1];;
	Points[3].z = point.z() + height;;
	
    box2d.polygon.points.push_back(Points[0]);
    box2d.polygon.points.push_back(Points[1]);
    box2d.polygon.points.push_back(Points[2]);
    box2d.polygon.points.push_back(Points[3]);

	box2d.header.frame_id = "base";
	box2d.header.stamp = current_time;
	return box2d;

}
// zmq initial
void getMatrixObs::init()
{
	std::string server = server_IP_;
	// std::cout << "connect to " << server << std::endl;

	std::string end_point;
	if (server.substr(0, 3) != "tcp") {
		end_point = "tcp://";
		end_point += server;
	}
	else {
		end_point = server;
	}

	nRecvFailCount = 0;
	if (m_endpoint != end_point.c_str())
	{
		snprintf(m_endpoint, sizeof(m_endpoint), end_point.c_str());
	}
	
	m_context = zmq_ctx_new();
	// Socket to talk to server
	m_requester = zmq_socket(m_context, ZMQ_SUB);
	int rc = zmq_connect(m_requester, m_endpoint);

	int recvhwm = 5;  // recv buffer size
	int recv_to_ = 500;
	zmq_setsockopt(m_requester, ZMQ_SUBSCRIBE, "", 0);
	zmq_setsockopt(m_requester, ZMQ_RCVHWM, &recvhwm, sizeof(int));
	zmq_setsockopt(m_requester, ZMQ_RCVTIMEO, &recv_to_, sizeof(int));
	zmq_msg_init(&m_recv_msg);

	recv_faile_count_ = 0;
}
bool getMatrixObs::start(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
	nh_ = nh;
	private_nh_ = private_nh;
	private_nh_.param("matrix_topic", matrix_topic_, std::string("/matrix_points"));
	private_nh_.param("box2d_topic", box2d_topic_, std::string("/matrix_box2d"));
	private_nh_.param("server_IP", server_IP_, std::string("192.168.1.11:5560"));
	private_nh_.param("camera_id", camera_id_, 1);
	if (camera_id_ > 4)
	{
		std::cout << "camera id mast less then 4" << std::endl;
		return false;
	}
	pub_matrix = nh.advertise<sensor_msgs::PointCloud2> (matrix_topic_, 1);
	box2D_pub = nh.advertise<jsk_recognition_msgs::PolygonArray> (box2d_topic_, 1);

	init();
	//Recieve_thread_ = new boost::thread(boost::bind(&getMatrixObs::ThreadRecieve, this));
	ThreadRecieve();

	return true;
}

void sigintHandler(int sig)
{
	printf("get shutdown signal \n");
	ros::shutdown();
}
//boost::shared_ptr<getMatrixObs> getmatrixobs;
int main(int argc, char **argv) {
	ros::init(argc, argv, "pub_matrixobs");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");


	//Readbag readbag ;
	getMatrixObs *getmatrixobs = new getMatrixObs();
	//getmatrixobs.reset(new getMatrixObs());
	// Override default sigint handler
	signal(SIGINT, sigintHandler);
	//std::thread th_recv(ThreadRecieve, server);
	if(getmatrixobs->start(nh, private_nh))
	{
		//ros::spin();
	}
	getmatrixobs->close();
	return (0);
}