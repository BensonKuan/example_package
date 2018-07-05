#include <example_class.h>

namespace example_package {

ExampleClass::ExampleClass() {}

ExampleClass::~ExampleClass() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    terminating_ = true;
  }
  if (visualization_thread_.joinable()) {
    visualization_thread_.join();
  }
}

void ExampleClass::Initialize() {
  cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
}

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

void ExampleClass::SpinVisualizationThreadForever() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (terminating_) {
        return;
      }
    }
    cv::imshow( "Display window", occupancy_grid );// Show our image inside it.
    cv::waitKey(30);
  }
}

} // namespace example_package
