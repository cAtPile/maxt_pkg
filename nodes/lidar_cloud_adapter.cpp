#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

class LidarCloudAdapter {
 public:
  LidarCloudAdapter() {
    publisher_ = node_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
    subscriber_ = node_.subscribe("points", 1, &LidarCloudAdapter::convert, this);
  }

 private:
  void convert(const sensor_msgs::PointCloud::ConstPtr& input) {
    sensor_msgs::PointCloud2 output;
    if (sensor_msgs::convertPointCloudToPointCloud2(*input, output))
      publisher_.publish(output);
  }

  ros::NodeHandle node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_cloud_adapter");
  LidarCloudAdapter adapter;
  ros::spin();
  return 0;
}
