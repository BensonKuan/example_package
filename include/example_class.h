#include <example_header.h>

//namespace {}

namespace example_package{

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kSubmapQueryServiceName[] = "submap_query";

class ExampleClass {
 public:
  ExampleClass();
  ~ExampleClass();

  void Initialize();

  ::ros::NodeHandle* node_handle();

 protected:

 private:
  void SpinVisualizationThreadForever();

  ::ros::NodeHandle node_handle_;

  tf2_ros::Buffer* const tf_buffer_;

  std::mutex mutex_;
  bool terminating_ = false;
  std::thread visualization_thread_;


} // class ExampleClass

} // namespace example_package
