#include "vio.h"

#include <gtest/gtest.h>

#include <memory>
#include <utility>

namespace
{
class VisualMemoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    cv::setNumThreads(1);
    image_ = cv::Mat(64, 64, CV_8UC1, cv::Scalar(128));
    manager_.setCamera(std::make_unique<vk::PinholeCamera>(64, 64, 1.0, 50, 50, 32, 32));
    manager_.width = manager_.height = 64;
    manager_.patch_size = 8;
    manager_.patch_size_half = 4;
    manager_.patch_size_total = 64;
    manager_.total_points = 1;
    manager_.update_flag.resize(1, 0);
    manager_.state = &state_;
    manager_.visual_submap->search_levels.push_back(0);
    manager_.new_frame_ = std::make_unique<Frame>(manager_.cam, image_);
    manager_.new_frame_->T_f_w_ = SE3();
    manager_.new_frame_->id_ = 100;
  }

  std::unique_ptr<VisualPoint> MakePoint(const V3D &position, int frame_id = 100)
  {
    auto point = std::make_unique<VisualPoint>(position);
    auto feature = std::make_unique<Feature>(point.get(), std::make_unique<float[]>(64), V2D(32, 32), V3D(0, 0, 1), SE3(), 0);
    feature->img_ = image_;
    feature->id_ = frame_id;
    feature->inv_expo_time_ = 1.0;
    feature->score_ = 10.0;
    point->ref_patch = feature.get();
    point->has_ref_patch_ = true;
    point->addFrameRef(feature.release());
    return point;
  }

  VIOManager manager_;
  cv::Mat image_;
  StatesGroup state_;
};

TEST_F(VisualMemoryTest, RepeatedRejectedObservationsDoNotAllocateUnownedPatches)
{
  auto owner = MakePoint(V3D(0, 0, 5));
  auto *point = owner.get();
  manager_.insertPointIntoVoxelMap(std::move(owner));
  manager_.visual_submap->voxel_points.push_back(point);
  for (int i = 0; i < 1000; ++i)
    manager_.updateVisualMapPoints(image_);
  EXPECT_EQ(point->obs_.size(), 1u);
}

TEST_F(VisualMemoryTest, AcceptedObservationsRetainTheirOwnedPatch)
{
  auto owner = MakePoint(V3D(0, 0, 5));
  auto *point = owner.get();
  manager_.insertPointIntoVoxelMap(std::move(owner));
  manager_.visual_submap->voxel_points.push_back(point);
  manager_.new_frame_->T_f_w_ = SE3(Sophus::SO3(), V3D(0.6, 0, 0));
  for (int i = 0; i < 60; ++i)
    manager_.updateVisualMapPoints(image_);
  EXPECT_EQ(point->obs_.size(), 30u);
  EXPECT_NE(point->obs_.front()->patch_.get(), nullptr);
}

TEST_F(VisualMemoryTest, PerVoxelLimitReleasesRejectedPointsAndImages)
{
  manager_.max_points_per_voxel = 2;
  const int baseline = image_.u->refcount;
  for (int i = 0; i < 10; ++i)
    manager_.insertPointIntoVoxelMap(MakePoint(V3D(0, 0, 5)));
  ASSERT_EQ(manager_.feat_map.size(), 1u);
  EXPECT_EQ(manager_.feat_map.begin()->second->voxel_points.size(), 2u);
  EXPECT_EQ(image_.u->refcount, baseline + 2);
}

TEST_F(VisualMemoryTest, EvictionClearsAllBorrowedAliasesBeforeDeletingPoints)
{
  manager_.max_visual_points = 2;
  for (int i = 0; i < 5; ++i)
  {
    auto point = MakePoint(V3D(i, 0, 5));
    manager_.retrieve_voxel_points.push_back(point.get());
    manager_.visual_submap->voxel_points.push_back(point.get());
    manager_.insertPointIntoVoxelMap(std::move(point));
  }
  manager_.pruneVisualMap();
  EXPECT_TRUE(manager_.visual_submap->voxel_points.empty());
  for (auto *point : manager_.retrieve_voxel_points)
    EXPECT_EQ(point, nullptr);
  std::size_t count = 0;
  for (const auto &voxel : manager_.feat_map)
    count += voxel.second->voxel_points.size();
  EXPECT_LE(count, 2u);
}

TEST_F(VisualMemoryTest, ExpiredReferenceImagesAreReleasedAndConvergedPointsCanRecover)
{
  manager_.max_reference_frames = 3;
  auto point = MakePoint(V3D(0, 0, 5), 90);
  auto *raw_point = point.get();
  auto fresh = std::make_unique<Feature>(point.get(), std::make_unique<float[]>(64), V2D(32, 32), V3D(0, 0, 1), SE3(), 0);
  fresh->id_ = 100;
  fresh->img_ = image_;
  point->addFrameRef(fresh.release());
  point->is_converged_ = true;
  manager_.insertPointIntoVoxelMap(std::move(point));
  manager_.pruneVisualMap();
  ASSERT_EQ(manager_.feat_map.size(), 1u);
  EXPECT_EQ(raw_point->obs_.size(), 1u);
  EXPECT_EQ(raw_point->ref_patch, nullptr);
  EXPECT_FALSE(raw_point->is_converged_);
  manager_.new_frame_->id_ = 104;
  manager_.pruneVisualMap();
  EXPECT_TRUE(manager_.feat_map.empty());
}

TEST_F(VisualMemoryTest, RadiusAndNegativeVoxelCoordinatesAreConsistent)
{
  manager_.visual_map_radius = 6;
  manager_.insertPointIntoVoxelMap(MakePoint(V3D(-0.5, 0, 5)));
  manager_.insertPointIntoVoxelMap(MakePoint(V3D(20, 0, 5)));
  EXPECT_NE(manager_.feat_map.find(VOXEL_LOCATION(-1, 0, 10)), manager_.feat_map.end());
  manager_.pruneVisualMap();
  EXPECT_EQ(manager_.feat_map.size(), 1u);
}

TEST_F(VisualMemoryTest, RepeatedWarpCacheClearsReleaseValues)
{
  for (int frame = 0; frame < 1000; ++frame)
  {
    manager_.warp_map.emplace(frame, Warp(0, Matrix2d::Identity()));
    manager_.warp_map.clear();
  }
  EXPECT_TRUE(manager_.warp_map.empty());
}

TEST_F(VisualMemoryTest, SustainedMapGrowthStaysWithinConfiguredLimits)
{
  manager_.max_visual_points = 100;
  manager_.max_reference_frames = 10;
  manager_.visual_map_radius = 100;
  for (int frame = 0; frame < 400; ++frame)
  {
    manager_.new_frame_->id_ = frame + 100;
    manager_.pruneVisualMap();
    image_ = cv::Mat(64, 64, CV_8UC1, cv::Scalar(frame % 255));
    for (int point = 0; point < 20; ++point)
      manager_.insertPointIntoVoxelMap(MakePoint(V3D(frame * 0.1, point * 0.5, 5), frame + 100));
    manager_.pruneVisualMap();
    std::size_t count = 0;
    for (const auto &voxel : manager_.feat_map)
      count += voxel.second->voxel_points.size();
    ASSERT_LE(count, 100u);
  }
}

TEST(VoxelMemoryTest, DestructionReleasesNestedOctrees)
{
  VoxelMapManager manager;
  auto *tree = new VoxelOctoTree(1, 0, 5, 50, 0.01);
  tree->leaves_[0] = new VoxelOctoTree(1, 1, 5, 50, 0.01);
  manager.voxel_map_.emplace(VOXEL_LOCATION(0, 0, 0), tree);
}
}  // namespace
