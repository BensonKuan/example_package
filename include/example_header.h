#ifndef EXAMPLE_PROJECT_H
#define EXAMPLE_PROJECT_H

// C++ headers I/O
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iosfwd>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <new>

#include <numeric>
#include <cstdint>
#include <cassert>

// Linux system generics and networking
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/serialization.h>

#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/Marker.h>

//ROS topics synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf_conversions/tf_eigen.h>

//TF 2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


//Eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//GOOGLE
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "gtest/gtest.h"

//
#include <python2.7/Python.h>

using namespace std;
using namespace cv;
using namespace message_filters;
using namespace sensor_msgs;

#endif // EXAMPLE_PROJECT_H
