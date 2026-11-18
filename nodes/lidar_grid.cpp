#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

namespace {
struct Coordinate {
  uint32_t offset;
  uint8_t type;
  size_t size;

  double read(const uint8_t* point, bool swap) const {
    uint8_t bytes[8];
    std::memcpy(bytes, point + offset, size);
    if (swap) std::reverse(bytes, bytes + size);
    using Field = sensor_msgs::PointField;
    switch (type) {
      case Field::INT8: return value<int8_t>(bytes);
      case Field::UINT8: return value<uint8_t>(bytes);
      case Field::INT16: return value<int16_t>(bytes);
      case Field::UINT16: return value<uint16_t>(bytes);
      case Field::INT32: return value<int32_t>(bytes);
      case Field::UINT32: return value<uint32_t>(bytes);
      case Field::FLOAT32: return value<float>(bytes);
      default: return value<double>(bytes);  // FLOAT64, validated below.
    }
  }

  template<typename T>
  static T value(const uint8_t* bytes) {
    T result;
    std::memcpy(&result, bytes, sizeof(result));
    return result;
  }
};

Coordinate coordinate(const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  constexpr size_t sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
  for (const auto& field : cloud.fields) {
    if (field.name == name && field.count == 1 && field.datatype >= 1 &&
        field.datatype <= 8 && field.offset + sizes[field.datatype] <= cloud.point_step)
      return {field.offset, field.datatype, sizes[field.datatype]};
  }
  throw std::invalid_argument("Invalid PointCloud2 coordinate: " + name);
}

class LidarGrid {
 public:
  LidarGrid() : listener_(buffer_) {
    ros::NodeHandle params("~");
    auto& info = grid_.info;
    const int width = params.param("width", 600);
    const int height = params.param("height", 600);
    info.resolution = params.param("resolution", 0.1);
    min_z_ = params.param("min_z", 0.2);
    max_z_ = params.param("max_z", 3.0);
    if (width <= 0 || height <= 0 || !std::isfinite(info.resolution) ||
        info.resolution <= 0 || !(min_z_ <= max_z_))
      throw std::invalid_argument("Invalid grid dimensions, resolution or height limits");
    info.width = width;
    info.height = height;
    info.origin.position.x = params.param("origin_x", -30.0);
    info.origin.position.y = params.param("origin_y", -30.0);
    info.origin.orientation.w = 1;
    grid_.header.frame_id = params.param<std::string>("map_frame", "map");
    grid_.data.assign(static_cast<size_t>(width) * height, -1);

    publisher_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    const auto type = params.param<std::string>("cloud_type", "PointCloud");
    if (type == "PointCloud")
      subscriber_ = node_.subscribe("points", 1, &LidarGrid::update<sensor_msgs::PointCloud>, this);
    else if (type == "PointCloud2")
      subscriber_ = node_.subscribe("points", 1, &LidarGrid::update<sensor_msgs::PointCloud2>, this);
    else
      throw std::invalid_argument("Unsupported cloud_type: " + type);
  }

 private:
  template<typename Cloud>
  void update(const boost::shared_ptr<const Cloud>& cloud) {
    try {
      const auto pose = buffer_.lookupTransform(grid_.header.frame_id, cloud->header.frame_id,
          cloud->header.stamp, ros::Duration(0.1)).transform;
      const auto& q = pose.rotation;
      const auto& t = pose.translation;
      const tf2::Transform transform(tf2::Quaternion(q.x, q.y, q.z, q.w), {t.x, t.y, t.z});
      accumulate(*cloud, transform);
      grid_.header.stamp = cloud->header.stamp;
      publisher_.publish(grid_);
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(5, "Skipping lidar scan: %s", error.what());
    } catch (const std::invalid_argument& error) {
      ROS_WARN_THROTTLE(5, "Skipping invalid cloud: %s", error.what());
    }
  }

  void mark(const tf2::Vector3& point) {
    const auto& info = grid_.info;
    const double x = (point.x() - info.origin.position.x) / info.resolution;
    const double y = (point.y() - info.origin.position.y) / info.resolution;
    if (point.z() >= min_z_ && point.z() <= max_z_ &&
        x >= 0 && x < info.width && y >= 0 && y < info.height)
      grid_.data[static_cast<size_t>(y) * info.width + static_cast<size_t>(x)] = 100;
  }

  void accumulate(const sensor_msgs::PointCloud& cloud, const tf2::Transform& transform) {
    for (const auto& p : cloud.points) mark(transform * tf2::Vector3(p.x, p.y, p.z));
  }

  void accumulate(const sensor_msgs::PointCloud2& cloud, const tf2::Transform& transform) {
    if (!cloud.width || !cloud.height) return;
    if (uint64_t(cloud.width) * cloud.point_step > cloud.row_step ||
        uint64_t(cloud.height) * cloud.row_step > cloud.data.size())
      throw std::invalid_argument("Invalid PointCloud2 row layout");
    const auto x = coordinate(cloud, "x");
    const auto y = coordinate(cloud, "y");
    const auto z = coordinate(cloud, "z");
    const uint16_t one = 1;
    const bool host_big_endian = *reinterpret_cast<const uint8_t*>(&one) == 0;
    const bool swap = host_big_endian != bool(cloud.is_bigendian);
    for (size_t row = 0; row < cloud.height; ++row) {
      const auto* point = cloud.data.data() + row * cloud.row_step;
      for (size_t col = 0; col < cloud.width; ++col, point += cloud.point_step)
        mark(transform * tf2::Vector3(x.read(point, swap), y.read(point, swap), z.read(point, swap)));
    }
  }

  ros::NodeHandle node_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  nav_msgs::OccupancyGrid grid_;
  double min_z_, max_z_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_grid");
  try {
    LidarGrid grid;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
  return 0;
}
