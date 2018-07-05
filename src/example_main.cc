#include <example_header.h>

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace example_package {
namespace {

static im_num = 0;

// Image encoding
static std::string enc = sensor_msgs::image_encodings::BGR8;

static std::ofstream myfile0;
static std::ofstream myfile1;
static std::ofstream myfile2;
static std::ofstream myfile3;
static std::ofstream myfile4;
static std::ofstream myfile5;
static std::ifstream readfile;

void Run() {
	constexpr double kTfBufferCacheTimeInSeconds = 1e6;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  ExampleClass node(options, &tf_buffer);
  ExampleClass.Initialize();

	// For 2D SLAM, subscribe to exactly one horizontal laser.
  ::ros::Subscriber laser_scan_subscriber;
  if (options.use_laser_scan) {
    laser_scan_subscriber = node.node_handle()->subscribe(
        kLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
            [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
              node.map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleLaserScanMessage(kLaserScanTopic, msg);
            }));
    expected_sensor_ids.insert(kLaserScanTopic);
  }

	::ros::spin();
}


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

void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
	ROS_INFO("Received a %d X %d map @ %.3f m/pix",
	map->info.width,
	map->info.height,
	map->info.resolution);

	cv::Mat1b occupancy_grid = cv::Mat1u::zeros(map->info.width, map->info.height);

	for(unsigned int y = 0; y < map->info.height; y++) {
		for(unsigned int x = 0; x < map->info.width; x++) {
			unsigned int i = x + (map->info.height - y - 1) * map->info.width;
			if (map->data[i] == 0) { //occ [0,0.1)
				occupancy_grid[i] = 254;
			} else if (map->data[i] == +100) { //occ (0.65,1]
				occupancy_grid[i] = 0;
			} else { //occ [0.1,0.65]
				occupancy_grid[i] = 205;
			}
		}
	}

	geometry_msgs::Quaternion orientation = map->info.origin.orientation;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	double yaw, pitch, roll;
	mat.getEulerYPR(yaw, pitch, roll);

	cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  cv::imshow( "Display window", occupancy_grid );// Show our image inside it.

  cv::waitKey(10);

}

} // namespace
} // namespace example_package


int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

	::ros::init(argc, argv, "example_package");
	::ros::start();

	LOG(INFO) << "Example usage of glogging: info";
	LOG(WARNING) << "Example usage of glogging: warning";
	LOG(ERROR) << "Example usage of glogging: error";

	CHECK_EQ(string("abc")[1], 'b') << "Example of CHECK macro";

	::ros::NodeHandle nh("~");

	std::string inputImuTopic;
	if (!nh.getParam("imu_topic_input", inputImuTopic)) {
		ROS_ERROR("- Unable to find the input imu topic. Exit...");
		return -1;
	}

	/* Read if we want TF to publish the new transform */
  nh.getParam("publish_tf", publishTF);

	// declaring a default parameter
	nh.param<std::string>("default_param", default_param, "default_value");

	// set a parameter resolved relative to the nh_ namespace
	nh.setParam("/global_param", 5);
	nh.setParam("relative_param", "my_string");
	nh.setParam("bool_param", false);

	// resolved relative to the node's namespace
	ros::param::set("/global_param", 5);
	ros::param::set("relative_param", "my_string");
	ros::param::set("bool_param", false);

	// Publishers & Subscribers
	tf::TransformBroadcaster br;

	ros::Rate r(30);
	while (nh.ok())
	{
		::ros::spinOnce();	// check for incoming messages
		br.sendTransform(tf::StampedTransform( tag_pose_4x4_95, ros::Time::now(), "/world", "/marker_95" ));
		r.sleep();
	}

	example_package::Run();

	::ros::shutdown();
}
