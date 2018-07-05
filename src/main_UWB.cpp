#include <example_header.h>

static im_num = 0;

// Image encoding in the case of the FLUENCE car and the camera ...
static std::string enc = sensor_msgs::image_encodings::BGR8;

static std::ofstream myfile0;
static std::ofstream myfile1;
static std::ofstream myfile2;
static std::ofstream myfile3;
static std::ofstream myfile4;
static std::ofstream myfile5;
static std::ifstream readfile;

void Run() { }


void callback(const sensor_msgs::Image::ConstPtr& image_msg, const frame_extractor::GlobalPose::ConstPtr& gps_msg, const nav_msgs::Odometry::ConstPtr& odom_msg) {

	cv_bridge::CvImagePtr cv_ptr;
	// cv_bridge::CvImageConstPtr cv_ptr;

	// String used to name the images
	std::stringstream sstring;

	std::string images_path = writePath + "/Images";
	mkdir( images_path.c_str(), 0777 );

	sstring << images_path << "/" << "Image" << std::setfill('0') << std::setw(5) << im_num << ".png";
	im_num++;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(image_msg, enc);
		// cv_ptr = cv_bridge::toCvShare(image_msg, enc);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	double im_secs = image_msg->header.stamp.toSec();

	double gt_secs = gps_msg->header.stamp.toSec();
	double lat = gps_msg->latitude;
	double lon = gps_msg->longitude;


	double odom_secs = odom_msg->header.stamp.toSec();

	double x = odom_msg->pose.pose.position.x;
	double y = odom_msg->pose.pose.position.y;

	double a = odom_msg->pose.pose.orientation.x;
	double b = odom_msg->pose.pose.orientation.y;
	double c = odom_msg->pose.pose.orientation.z;
	double d = odom_msg->pose.pose.orientation.w;

}


int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);
	::ros::init(argc, argv, "UWB_wrapper");

	::ros::NodeHandle nh_("~");

	geometry_msgs::PoseStamped support1;
	geometry_msgs::PoseStamped support2;
	geometry_msgs::PoseStamped support3;
	geometry_msgs::PoseStamped support4;
	geometry_msgs::PoseStamped tag;

	struct sockaddr_in serv_addr, cli_addr;

	char flag = 1;
  int packet_size = 19;
  int portno = 6000;
  int sockfd, newsockfd;
  socklen_t clilen;
  int read_size;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd < 0)
		perror("ERROR opening socket");

	bzero((char *) &serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY; // set the current IP address
	serv_addr.sin_port = htons(portno);

	if ( bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		perror("ERROR on binding");

	listen(sockfd,5);
	clilen = sizeof(cli_addr);

	std::cout << "Waiting for incoming connection" << std::endl;
	newsockfd = accept(sockfd,
										(struct sockaddr *) &cli_addr,
										&clilen);

	if (newsockfd < 0)
		perror("ERROR on accept");

	std::cout << "Connection established. Start receiving images." << std::endl;

	// retrieve the size of the data that we are going to receive
	vector<char> packetArray(packet_size);
	// reading loop
	read_size = 0;

	if ((read_size = read(newsockfd, &packetArray[i], 1)) == -1)
 	 printf("read failed");

	if ((read_size = read(newsockfd, &packetArray[i], 1)) == -1)
 	 printf("read failed");

	if ((read_size = read(newsockfd, &packetArray[i], 8)) == -1)
		printf("read failed");

	int pos_x;
	if ((read_size = read(newsockfd, &pos_x, 4)) == -1)
		printf("read failed");

	int pos_y;
	if ((read_size = read(newsockfd, &packetArray[i], 4)) == -1)
		printf("read failed");

	if ((read_size = read(newsockfd, &packetArray[i], 1)) == -1)
	 printf("read failed");

	vector<T>::const_iterator first = myVec.begin() + 100000;
	vector<T>::const_iterator last = myVec.begin() + 101000;
	vector<T> newVec(first, last);
	packetArray.begin() + 10]


	LOG(INFO) << "Example usage of glogging: info";
	LOG(WARNING) << "Example usage of glogging: warning";
	LOG(ERROR) << "Example usage of glogging: error";

	CHECK_EQ(string("abc")[1], 'b') << "Example of CHECK macro";

	std::string inputImuTopic;
	if (!nh.getParam("imu_topic_input", inputImuTopic)) {
		ROS_ERROR("- Unable to find the input imu topic. Exit...");
		return -1;
	}

	/* Read if we want TF to publish the new transform */
  nh_.getParam("publish_tf", publishTF);

	// declaring a default parameter
	nh_.param<std::string>("default_param", default_param, "default_value");

	// set a parameter resolved relative to the nh_ namespace
	nh_.setParam("/global_param", 5);
	nh_.setParam("relative_param", "my_string");
	nh_.setParam("bool_param", false);

	// resolved relative to the node's namespace
	ros::param::set("/global_param", 5);
	ros::param::set("relative_param", "my_string");
	ros::param::set("bool_param", false);

	// Publishers & Subscribers
	tf::TransformBroadcaster br;

	ros::Rate r(30);
	while (nh_.ok())
	{
		ros::spinOnce();	// check for incoming messages
		br.sendTransform(tf::StampedTransform( tag_pose_4x4_95, ros::Time::now(), "/world", "/marker_95" ));
		r.sleep();
	}

	//::Run();

	return 0;
}
