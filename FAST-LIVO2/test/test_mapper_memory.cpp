#include "LIVMapper.h"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace
{
builtin_interfaces::msg::Time Stamp(double seconds) { return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9))); }

class MapperMemoryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    cv::setNumThreads(1);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    char path[] = "/tmp/fast_livo_test_XXXXXX";
    const char *directory = mkdtemp(path);
    ASSERT_NE(directory, nullptr);
    output_ = directory;
  }
  void TearDown() override { std::filesystem::remove_all(output_); }

  std::unique_ptr<LIVMapper> MakeMapper(std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> parameters{{"extrin_calib.extrinsic_T", std::vector<double>{0, 0, 0}},
                                              {"extrin_calib.extrinsic_R", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1}},
                                              {"extrin_calib.Pcl", std::vector<double>{0, 0, 0}},
                                              {"extrin_calib.Rcl", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1}},
                                              {"camera.cam_model", std::string("Pinhole")},
                                              {"camera.cam_width", 64},
                                              {"camera.cam_height", 64},
                                              {"camera.scale", 1.0},
                                              {"camera.cam_fx", 50.0},
                                              {"camera.cam_fy", 50.0},
                                              {"camera.cam_cx", 32.0},
                                              {"camera.cam_cy", 32.0},
                                              {"vio.grid_n_height", 4},
                                              {"vio.patch_pyrimid_level", 1},
                                              {"memory.max_lidar_buffer", 3},
                                              {"memory.max_image_buffer", 3},
                                              {"memory.max_imu_buffer", 8},
                                              {"memory.max_buffer_duration", 1.0},
                                              {"memory.max_path_poses", 5},
                                              {"pcd_save.output_directory", output_.string()},
                                              {"preprocess.point_filter_num", 1},
                                              {"preprocess.blind", 0.1}};
    for (const auto &parameter : extra)
    {
      parameters.erase(
          std::remove_if(parameters.begin(), parameters.end(), [&](const auto &old) { return old.get_name() == parameter.get_name(); }),
          parameters.end());
      parameters.push_back(parameter);
    }
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides(parameters);
    rclcpp::Node::SharedPtr node;
    return std::make_unique<LIVMapper>(node, "laserMapping", options);
  }

  livox_ros_driver2::msg::CustomMsg::SharedPtr Lidar(double time)
  {
    auto message = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    message->header.stamp = Stamp(time);
    for (int i = 0; i < 12; ++i)
    {
      livox_ros_driver2::msg::CustomPoint point;
      point.x = 2.0f + i * 0.1f;
      point.y = 0.1f;
      point.z = 1.0f;
      point.line = i % 6;
      point.tag = 0x10;
      point.offset_time = i * 1000000;
      message->points.push_back(point);
    }
    message->point_num = message->points.size();
    return message;
  }

  std::filesystem::path output_;
};

TEST_F(MapperMemoryTest, SensorQueuesStayBoundedWhenSynchronizationStalls)
{
  auto mapper = MakeMapper({{"uav.imu_rate_odom", true}});
  mapper->p_imu->imu_need_init = false;
  for (int frame = 0; frame < 30; ++frame)
    mapper->livox_pcl_cbk(Lidar(100 + frame * 0.1));
  EXPECT_LE(mapper->lid_raw_data_buffer.size(), 3u);
  EXPECT_EQ(mapper->lid_raw_data_buffer.size(), mapper->lid_header_time_buffer.size());
  for (int i = 0; i < 100; ++i)
  {
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->header.stamp = Stamp(103 + i * 0.01);
    mapper->imu_cbk(imu);
  }
  EXPECT_LE(mapper->imu_buffer.size(), 8u);
  EXPECT_LE(mapper->prop_imu_buffer.size(), 8u);
  for (int i = 0; i < 20; ++i)
  {
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->header.stamp = Stamp(103 + i * 0.1);
    image->height = image->width = 64;
    image->encoding = "bgr8";
    image->step = 64 * 3;
    image->data.assign(64 * 64 * 3, 42);
    mapper->img_cbk(image);
  }
  EXPECT_LE(mapper->img_buffer.size(), 3u);
  EXPECT_EQ(mapper->img_buffer.size(), mapper->img_time_buffer.size());
  ASSERT_FALSE(mapper->img_buffer.empty());
  EXPECT_EQ(mapper->img_buffer.back().at<cv::Vec3b>(0, 0)[0], 42);
}

TEST_F(MapperMemoryTest, TimestampRollbackKeepsLidarAndTimestampQueuesPaired)
{
  auto mapper = MakeMapper();
  mapper->livox_pcl_cbk(Lidar(100));
  mapper->livox_pcl_cbk(Lidar(101));
  mapper->livox_pcl_cbk(Lidar(99));
  ASSERT_EQ(mapper->lid_raw_data_buffer.size(), 1u);
  ASSERT_EQ(mapper->lid_header_time_buffer.size(), 1u);
  EXPECT_DOUBLE_EQ(mapper->lid_header_time_buffer.front(), 99);
}

TEST_F(MapperMemoryTest, QueueAgeLimitAppliesBeforeCountLimit)
{
  auto mapper = MakeMapper();
  mapper->livox_pcl_cbk(Lidar(100));
  mapper->livox_pcl_cbk(Lidar(102));
  EXPECT_EQ(mapper->lid_raw_data_buffer.size(), 1u);
  EXPECT_DOUBLE_EQ(mapper->lid_header_time_buffer.front(), 102);
}

TEST_F(MapperMemoryTest, SubscriptionsUseFiniteSensorDataQos)
{
  auto mapper = MakeMapper();
  rclcpp::Node::SharedPtr node;
  image_transport::ImageTransport transport(mapper->node);
  mapper->initializeSubscribersAndPublishers(node, transport);
  const auto qos = mapper->sub_img->get_actual_qos().get_rmw_qos_profile();
  EXPECT_EQ(qos.depth, 3u);
  EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(MapperMemoryTest, PureLioSynchronizationDoesNotRetainHistoricalImuGroups)
{
  auto mapper = MakeMapper({{"common.img_en", 0}});
  std::weak_ptr<sensor_msgs::msg::Imu> previous_imu;
  for (int i = 0; i < 100; ++i)
  {
    auto cloud = std::make_shared<PointCloudXYZI>();
    cloud->resize(2);
    cloud->back().curvature = 50;
    const double time = 100 + i * 0.1;
    mapper->lid_raw_data_buffer.push_back(cloud);
    mapper->lid_header_time_buffer.push_back(time);
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->header.stamp = Stamp(time + 0.025);
    mapper->imu_buffer.push_back(imu);
    mapper->last_timestamp_imu = time + 0.1;
    ASSERT_TRUE(mapper->sync_packages(mapper->LidarMeasures));
    EXPECT_EQ(mapper->LidarMeasures.measures.size(), 1u);
    EXPECT_EQ(mapper->LidarMeasures.measures.back().imu.size(), 1u);
    EXPECT_TRUE(previous_imu.expired());
    previous_imu = imu;
  }
}

TEST_F(MapperMemoryTest, PcdChunksHonorPointLimitAndSaveTheFinalPartialChunk)
{
  auto mapper = MakeMapper({{"common.img_en", 0}, {"pcd_save.pcd_save_en", true}, {"pcd_save.interval", 100}, {"pcd_save.max_points", 5}});
  mapper->pcl_w_wait_pub->resize(8);
  auto publisher = mapper->node->create_publisher<sensor_msgs::msg::PointCloud2>("/test_cloud", 1);
  mapper->publish_frame_world(publisher, mapper->vio_manager);
  EXPECT_EQ(mapper->pcd_index, 1);
  EXPECT_EQ(mapper->pcl_wait_save_intensity->size(), 3u);
  mapper->savePCD();
  EXPECT_EQ(mapper->pcd_index, 2);
  EXPECT_TRUE(mapper->pcl_wait_save_intensity->empty());
  PointCloudXYZI first, second;
  ASSERT_EQ(pcl::io::loadPCDFile((output_ / "1.pcd").string(), first), 0);
  ASSERT_EQ(pcl::io::loadPCDFile((output_ / "2.pcd").string(), second), 0);
  EXPECT_EQ(first.size(), 5u);
  EXPECT_EQ(second.size(), 3u);
}

TEST_F(MapperMemoryTest, FailedPcdWritesRetainThePendingPoints)
{
  auto mapper = MakeMapper({{"common.img_en", 0}, {"pcd_save.pcd_save_en", true}, {"pcd_save.interval", 1}, {"pcd_save.max_points", 5}});
  std::filesystem::create_directory(output_ / "1.pcd");
  mapper->pcl_wait_save_intensity->resize(3);
  EXPECT_THROW(mapper->savePCD(), std::exception);
  EXPECT_EQ(mapper->pcl_wait_save_intensity->size(), 3u);
  EXPECT_EQ(mapper->pcd_index, 0);
}

TEST_F(MapperMemoryTest, EmptyRgbProjectionsDoNotAccumulateUnpublishedClouds)
{
  auto mapper = MakeMapper({{"pcd_save.pcd_save_en", true}, {"pcd_save.interval", 2}});
  mapper->vio_manager->new_frame_ = std::make_unique<Frame>(mapper->vio_manager->cam, cv::Mat(64, 64, CV_8UC1, cv::Scalar(0)));
  mapper->vio_manager->new_frame_->T_f_w_ = SE3();
  mapper->vio_manager->img_rgb = cv::Mat(64, 64, CV_8UC3, cv::Scalar(0));
  auto publisher = mapper->node->create_publisher<sensor_msgs::msg::PointCloud2>("/test_cloud", 1);
  for (int frame = 0; frame < 20; ++frame)
  {
    mapper->pcl_w_wait_pub->resize(10);
    for (auto &point : mapper->pcl_w_wait_pub->points)
      point.z = -1;
    mapper->publish_frame_world(publisher, mapper->vio_manager);
    EXPECT_TRUE(mapper->pcl_wait_pub->empty());
  }
  mapper->pcl_w_wait_pub->resize(1);
  auto &point = mapper->pcl_w_wait_pub->front();
  point.x = 0;
  point.y = 0;
  point.z = 5;
  mapper->publish_frame_world(publisher, mapper->vio_manager);
  EXPECT_EQ(mapper->pcd_index, 0);
  EXPECT_EQ(mapper->pcl_wait_save->size(), 1u);
}

TEST_F(MapperMemoryTest, PublishedPathsHaveABoundedHistory)
{
  auto mapper = MakeMapper();
  auto publisher = mapper->node->create_publisher<nav_msgs::msg::Path>("/test_path", 1);
  for (int i = 0; i < 20; ++i)
  {
    mapper->_state.pos_end[0] = i;
    mapper->publish_path(publisher);
  }
  ASSERT_EQ(mapper->path.poses.size(), 5u);
  EXPECT_DOUBLE_EQ(mapper->path.poses.front().pose.position.x, 15);
  EXPECT_DOUBLE_EQ(mapper->path.poses.back().pose.position.x, 19);
}

TEST_F(MapperMemoryTest, RejectsInvalidMemoryAndSaveLimits)
{
  EXPECT_THROW(MakeMapper({{"memory.max_image_buffer", 0}}), std::invalid_argument);
  EXPECT_THROW(MakeMapper({{"pcd_save.pcd_save_en", true}, {"pcd_save.interval", 0}}), std::invalid_argument);
  EXPECT_THROW(MakeMapper({{"memory.max_visual_points", -1}}), std::invalid_argument);
}
}  // namespace
