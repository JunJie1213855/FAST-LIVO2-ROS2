/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_POINT_H_
#define LIVO_POINT_H_

#include <boost/noncopyable.hpp>
#include "common_lib.h"
#include "frame.h"

class Feature;

/// A visual map point on the surface of the scene.
class VisualPoint : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector3d pos_;                // 视觉点对应在世界坐标系下的三维坐标
  Vector3d normal_;             // 在三维点的表面法向量
  Matrix3d normal_information_; // 法向量的信息矩阵，参考 voxelmap 的推理
  Vector3d previous_normal_;    // 上一个更新的法向量
  list<Feature *> obs_;         // 不同帧对应该点的观测
  Eigen::Matrix3d covariance_;  // 该三维点的协方差
  bool is_converged_;           // 是否收敛
  bool is_normal_initialized_;  // 法向量是否初始化
  bool has_ref_patch_;          // 是否存在一个参考图像块
  Feature *ref_patch = nullptr; // 该点对应的图像块

  VisualPoint(const Vector3d &pos);
  ~VisualPoint();
  void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;
  void deleteNonRefPatchFeatures();
  void deleteFeatureRef(Feature *ftr);
  void addFrameRef(Feature *ftr);
  bool getCloseViewObs(const Vector3d &pos, Feature *&obs, const Vector2d &cur_px) const;
};

#endif // LIVO_POINT_H_
