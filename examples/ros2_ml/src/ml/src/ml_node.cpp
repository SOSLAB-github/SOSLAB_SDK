#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cstring>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "Lidar.h"

using namespace std::chrono_literals;

static const char* DEFAULT_IP_ADDR_DEVICE          = "192.168.1.10";
static const int   DEFAULT_IP_PORT_DEVICE          = 2000;

static const char* DEFAULT_IP_ADDR_PC              = "0.0.0.0";
static const int   DEFAULT_IP_PORT_PC              = 0;

static const char*   DEFAULT_LIDAR_TYPE              = "MLX";

static const char* DEFAULT_FRAME_ID                = "map";

const std::vector<float> DEFAULT_TRANSFORM         = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};

unsigned char soslab_r[] = {3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 34, 34, 35, 35, 36, 37, 37, 38, 38, 39, 40, 40, 41, 41, 42, 42, 43, 43, 43, 44, 44, 45, 45, 46, 46, 46, 47, 47, 48, 48, 49, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 54, 54, 54, 55, 55, 56, 56, 57, 57, 57, 58, 58, 59, 59, 60, 60, 60, 61, 61, 62, 62, 62, 63, 63, 64, 64, 65, 65, 66, 66, 66, 67, 67, 68, 68, 70, 71, 73, 75, 77, 79, 80, 82, 84, 86, 88, 90, 92, 93, 95, 97, 99, 101, 102, 104, 106, 108, 110, 111, 113, 115, 117, 119, 120, 122, 124, 126, 128, 129, 131, 133, 135, 137, 138, 140, 142, 144, 145, 147, 149, 151, 153, 154, 156, 158, 160, 162, 163, 165, 167, 169, 171, 172, 174, 176, 178, 180, 181, 183, 184, 185, 187, 188, 189, 190, 191, 192, 194, 195, 196, 197, 198, 199, 200, 201, 203, 204, 205, 206, 207, 208, 209, 210, 212, 213, 214, 215, 216, 217, 218, 219, 221, 222, 223, 224, 225, 226, 227, 228, 230, 231, 232, 233, 234, 235, 236, 237, 239, 240, 241, 242, 243, 244, 245, 246, 248, 249, 250, 251, 252, 253, 254, 254};
unsigned char soslab_g[] = {18, 19, 21, 23, 24, 26, 27, 29, 30, 32, 33, 35, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 54, 55, 57, 58, 60, 61, 63, 64, 66, 67, 69, 71, 72, 74, 75, 77, 78, 80, 81, 83, 84, 86, 88, 89, 91, 92, 94, 95, 97, 98, 100, 101, 103, 105, 106, 108, 109, 111, 112, 114, 116, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 171, 172, 173, 174, 175, 176, 177, 178, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199, 200, 201, 202, 203, 204, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 247, 248, 249, 250, 251, 252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
unsigned char soslab_b[] = {107, 108, 109, 110, 111, 112, 113, 114, 114, 115, 116, 117, 118, 119, 120, 120, 121, 122, 123, 124, 125, 126, 126, 127, 128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 137, 138, 138, 139, 140, 141, 142, 143, 144, 144, 145, 146, 147, 148, 149, 150, 151, 151, 152, 153, 154, 155, 156, 157, 157, 158, 159, 160, 161, 162, 162, 163, 163, 163, 164, 164, 164, 164, 165, 165, 165, 166, 166, 166, 167, 167, 167, 168, 168, 168, 169, 169, 169, 169, 170, 170, 170, 171, 171, 171, 172, 172, 172, 173, 173, 173, 174, 174, 174, 174, 175, 175, 175, 176, 176, 176, 177, 177, 177, 178, 178, 178, 179, 179, 179, 179, 180, 180, 180, 181, 181, 181, 182, 182, 181, 180, 179, 178, 177, 175, 175, 174, 172, 171, 170, 169, 168, 167, 166, 165, 164, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 123, 121, 120, 119, 118, 117, 116, 115, 114, 113, 111, 109, 107, 105, 104, 102, 100, 98, 96, 95, 93, 91, 89, 88, 86, 84, 82, 81, 79, 77, 75, 73, 72, 70, 68, 66, 65, 63, 61, 59, 58, 56, 54, 52, 51, 49, 47, 45, 43, 42, 40, 38, 36, 35, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 15, 13, 12, 10, 8, 6, 5, 2, 1, 1};

static inline uint32_t intensity_to_rgb(uint32_t inten, int max_intensity)
{
  if (max_intensity <= 0) max_intensity = 3000;
  int v = static_cast<int>( (static_cast<double>(inten) * 255.0) / static_cast<double>(max_intensity) );
  v = std::clamp(v, 0, 255);

  const uint32_t r = static_cast<uint32_t>(soslab_r[v]);
  const uint32_t g = static_cast<uint32_t>(soslab_g[v]);
  const uint32_t b = static_cast<uint32_t>(soslab_b[v]);

  return (r << 16) | (g << 8) | b;
}

class ML : public rclcpp::Node
{
public:
  ML() : Node("mlx_ros2_node")
  {
    declare_parameter("ip_address_device", std::string(DEFAULT_IP_ADDR_DEVICE));
    declare_parameter("ip_port_device", DEFAULT_IP_PORT_DEVICE);
    declare_parameter("ip_address_pc", std::string(DEFAULT_IP_ADDR_PC));
    declare_parameter("ip_port_pc", DEFAULT_IP_PORT_PC);
    declare_parameter("lidarType", DEFAULT_LIDAR_TYPE);

    declare_parameter("pc_topic_lidar0", std::string("lidar0/pointcloud"));
    declare_parameter("pc_topic_lidar1", std::string("lidar1/pointcloud"));

    declare_parameter("max_intensity", 3000);

    std::string lidarType;

    lidarType = get_parameter("lidarType").as_string();

    if(lidarType == "MLX")
    {
        params_.lidarTypeValue = soslab::lidarType::MLX;
    }
    else if(lidarType == "MLA")
    {
        params_.lidarTypeValue = soslab::lidarType::MLA;
    }
    else if(lidarType == "MLU")
    {
        params_.lidarTypeValue = soslab::lidarType::MLU;
    }
    else if(lidarType == "GL5")
    {
        params_.lidarTypeValue = soslab::lidarType::GL5;
    }
    else if(lidarType == "GL3")
    {
        params_.lidarTypeValue = soslab::lidarType::GL3;
    }
    else if(lidarType == "SLU")
    {
        params_.lidarTypeValue = soslab::lidarType::SLU;
    }
    
    params_.lidarIP   = get_parameter("ip_address_device").as_string();
    params_.lidarPort = get_parameter("ip_port_device").as_int();
    params_.pcIP      = get_parameter("ip_address_pc").as_string();
    params_.pcPort    = get_parameter("ip_port_pc").as_int();
    
    pc_topic_.resize(2);
    pc_topic_[0] = get_parameter("pc_topic_lidar0").as_string();
    pc_topic_[1] = get_parameter("pc_topic_lidar1").as_string();
    max_intensity_ = get_parameter("max_intensity").as_int();

    RCLCPP_INFO(get_logger(), "Device=%s:%d, PC=%s:%d",
    params_.lidarIP.c_str(), params_.lidarPort, params_.pcIP.c_str(), params_.pcPort);
    RCLCPP_INFO(get_logger(), "PointCloud topics: [0]=%s, [1]=%s",
    pc_topic_[0].c_str(), pc_topic_[1].c_str());

    std::cerr << "Parameter loaded\n";

    pc_pub_[0] = create_publisher<sensor_msgs::msg::PointCloud2>(pc_topic_[0], 1);
    pc_pub_[1] = create_publisher<sensor_msgs::msg::PointCloud2>(pc_topic_[1], 1);

    std::cerr << "Publisher created\n";
  }

  ~ML()
  {
    if (lidar_) {
      lidar_->streamStop();
      lidar_->disconnectLidar();
    }
  }

  bool connect()
  {
    lidar_ = std::make_shared<soslab::Lidar>();
    lidar_->setParameters(params_);

    if (!lidar_->connectLidar()) {
      RCLCPP_ERROR(get_logger(), "LiDAR connection failed.");
      return false;
    }

    std::cerr << "Connect done\n";

    lidar_->registerGetDataCallBack(std::bind(&ML::scene_data_callback, this, std::placeholders::_1));
    std::cerr << "Callback registered\n";

    lidar_->streamStop();

    std::cerr << "Connect STOP\n";
    if (!lidar_->streamStart()) {
      RCLCPP_ERROR(get_logger(), "LiDAR streamStart failed.");
      return false;
    }

    std::cerr << "Connect Start\n";

    RCLCPP_INFO(get_logger(), "Streaming started.");
    return true;
  }

private:
  void scene_data_callback(std::shared_ptr<const soslab::FrameData> scene)
  {
    if (!scene) {
      return;
    }

    const int id = 0;


    auto msg = build_pointcloud(scene);
    msg->header.stamp = this->get_clock()->now();
    pc_pub_[id]->publish(std::move(msg));
  }

  std::unique_ptr<sensor_msgs::msg::PointCloud2> build_pointcloud(const std::shared_ptr<const soslab::FrameData>& scene)
  {
    const std::size_t height = static_cast<std::size_t>(scene->rows);
    const std::size_t width  = static_cast<std::size_t>(scene->cols);
    const std::size_t npts   = height * width;

    const bool has_intensity = !scene->intensity.empty() && !scene->intensity[0].empty();
    const std::vector<uint32_t>* intensity_ptr = nullptr;
    if (has_intensity) {
      intensity_ptr = &scene->intensity[0];
    }

    const std::vector<soslab::Points>& pts = scene->points[0];

    auto pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc->header.frame_id = DEFAULT_FRAME_ID;
    pc->height = static_cast<uint32_t>(height);
    pc->width  = static_cast<uint32_t>(width);
    pc->is_bigendian = false;
    pc->is_dense = true;

    pc->fields.resize(4);

    pc->fields[0].name = "x";
    pc->fields[0].offset = 0;
    pc->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[0].count = 1;

    pc->fields[1].name = "y";
    pc->fields[1].offset = 4;
    pc->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[1].count = 1;

    pc->fields[2].name = "z";
    pc->fields[2].offset = 8;
    pc->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[2].count = 1;

    pc->fields[3].name = "rgb";
    pc->fields[3].offset = 12;
    pc->fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    pc->fields[3].count = 1;

    pc->point_step = 16;
    pc->row_step = pc->point_step * pc->width;
    pc->data.resize(pc->row_step * pc->height);

    for (std::size_t idx = 0; idx < npts; ++idx) {
      float x = static_cast<float>(pts[idx].x) / 1000.0f;
      float y = static_cast<float>(pts[idx].y) / 1000.0f;
      float z = static_cast<float>(pts[idx].z) / 1000.0f;

      uint32_t rgb = 0;
      if (has_intensity && intensity_ptr && idx < intensity_ptr->size()) {
        rgb = intensity_to_rgb((*intensity_ptr)[idx], max_intensity_);
      }

      std::memcpy(&pc->data[idx * pc->point_step + 0],  &x,   sizeof(float));
      std::memcpy(&pc->data[idx * pc->point_step + 4],  &y,   sizeof(float));
      std::memcpy(&pc->data[idx * pc->point_step + 8],  &z,   sizeof(float));
      std::memcpy(&pc->data[idx * pc->point_step + 12], &rgb, sizeof(uint32_t));

    }

    return pc;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_[2];

  std::shared_ptr<soslab::Lidar> lidar_;
  soslab::lidarParameters params_;

  std::vector<std::string> pc_topic_;
  int max_intensity_ = 3000;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ML>();
  std::cerr << "node created\n";
  if (!node->connect()) {
    return 0;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
