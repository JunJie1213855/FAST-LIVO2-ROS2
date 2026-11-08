/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <memory>
#include <cstddef>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

// 
struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch; // 覆盖的图像子块
  vector<int> search_levels;        // 金字塔层级
  vector<VisualPoint *> voxel_points; //
  vector<double> inv_expo_list;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class Warp
{
public:
  Matrix2d A_cur_ref; // 参考帧到当前帧的仿射变换矩阵 A
  int search_level;   // 查找的层级
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  int last_used_frame = 0;
  explicit VOXEL_POINTS(int num) : count(num) {}
  VOXEL_POINTS(const VOXEL_POINTS &) = delete;
  VOXEL_POINTS &operator=(const VOXEL_POINTS &) = delete;
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};


/// @brief VIO 管理类，包括图像块、状态更新
class VIOManager
{
public:
  int grid_size;
  vk::AbstractCamera *cam = nullptr;  // Non-owning alias of camera_.
  vk::PinholeCamera *pinhole_cam = nullptr;
  StatesGroup *state;
  StatesGroup *state_propagat;
  // lidar2imu、cam2imu、cam2lidar、cam2world 的旋转 以及对应的雅可比
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  // lidar2imu、cam2imu、cam2lidar、cam2world 的平移
  V3D Pli, Pci, Pcl, Pcw;

  // 网格数量
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false, colmap_output_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  // 相机内参，包括焦距 fx/fy 和相机中心 cx/cy
  double fx, fy, cx, cy;
  // 金字塔层级、图像块大小、图像块数量
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;

  // 状态更新最大迭代次数、全部点云数量
  int max_iterations, total_points;

  // 图像点方差、外点阈值、ncc 阈值
  double img_point_cov, outlier_threshold, ncc_thre;

  // 视觉子图
  // 本帧筛选出来、用于计算光度误差和更新状态的一组视觉观测数据
  std::unique_ptr<SubSparseMap> visual_submap;

  // 图像网格中心对应的一条视线，以及沿这条视线预先采样的三维点。 它用于补充搜索地图。
  std::vector<std::vector<V3D>> rays_with_sample_points;

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  Eigen::MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  // 特征点哈希表
  // 跨帧累积、持续维护的视觉三维点体素地图。
  // 体素里存的是 VisualPoint，每个点关联历史图像观测 Feature。
  // 它不会永久保留所有历史数据，pruneVisualMap() 会清理旧数据。
  unordered_map<VOXEL_LOCATION, std::unique_ptr<VOXEL_POINTS>> feat_map; 
  
  
  // 子图特征点哈希表
  // 本帧的候选体素索引集合。
  // 它主要根据本次激光点 pg 建立，用于去 feat_map 中查找历史视觉点，本身不保存地图点或图像。
  unordered_map<VOXEL_LOCATION, int> sub_feat_map;

  // 图像块变形参数的缓存，用来避免重复计算仿射矩阵和采样层级。
  unordered_map<int, Warp> warp_map;

  // 检索到的体素三维点
  vector<VisualPoint *> retrieve_voxel_points;

  // 插入的体素三维点
  vector<pointWithVar> append_voxel_points;

  // 新帧
  FramePtr new_frame_;
  // 图像
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  // Zero disables the corresponding map retention limit.
  std::size_t max_visual_points = 20000;
  std::size_t max_points_per_voxel = 32;
  int max_reference_frames = 200;
  double visual_map_radius = 200.0;

  VIOManager();
  VIOManager(const VIOManager &) = delete;
  VIOManager &operator=(const VIOManager &) = delete;
  void setCamera(std::unique_ptr<vk::AbstractCamera> camera);
  // Called only between frames, after invalidating all borrowed map-point pointers.
  void pruneVisualMap();
  ~VIOManager();
  void updateStateInverse(cv::Mat img, int level);
  void updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  void initializeVIO();
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  void computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3 &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  void insertPointIntoVoxelMap(std::unique_ptr<VisualPoint> point);
  void plotTrackedPoints();
  void updateFrameState(StatesGroup state);
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);

private:
  // 相机，用于映射
  std::unique_ptr<vk::AbstractCamera> camera_;

  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_
