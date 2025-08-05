#include "map_manager_travblox.hpp"

#if (COL_CHECK_METHOD == 0)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  // Get the center of the bounding box as a global index.
  voxblox::LongIndex center_voxel_index =
      voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
          center.cast<voxblox::FloatingPoint>(), voxel_size_inv);

  voxblox::AnyIndex box_voxels(std::floor(size.x() * voxel_size_inv + 0.5),
                               std::floor(size.y() * voxel_size_inv + 0.5),
                               std::floor(size.z() * voxel_size_inv + 0.5));

  // Iterate over all voxels in the bounding box.
  return getBoxStatusInVoxels(center_voxel_index, box_voxels,
                              stop_at_unknown_voxel);
}
#elif (COL_CHECK_METHOD == 1)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  // Get the center of the bounding box as a global index.
  double dist = getPointDistance(center);
  if (dist < robot_radius_) {
    if (dist < 0.0) {
      if (stop_at_unknown_voxel) {
        return VoxelStatus::kUnknown;
      }
    } else {
      return VoxelStatus::kOccupied;
    }
  }
  return VoxelStatus::kFree;
}
#elif (COL_CHECK_METHOD == 2)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;
  float check_size = voxel_size * 2.0;
  const float distance_thres = check_size / 2.0 + 1e-6;

  Eigen::Vector3d reduced_size =
      size - Eigen::Vector3d(check_size, check_size, check_size);
  for (int i = 0; i < 3; ++i) {
    if (reduced_size(i) < 0.0) reduced_size(i) = 0.0;
  }

  for (double dx = 0.0; dx < (reduced_size(0) + check_size / 2.0 + 1e-2);
       dx += check_size) {
    for (double dy = 0.0; dy < (reduced_size(1) + check_size / 2.0 + 1e-2);
         dy += check_size) {
      for (double dz = 0.0; dz < (reduced_size(2) + check_size / 2.0 + 1e-2);
           dz += check_size) {
        Eigen::Vector3d point =
            center + Eigen::Vector3d(dx, dy, dz) - reduced_size / 2.0;
        double dist = getPointDistance(point);
        if (dist < distance_thres) {
          if (dist < 0.0) {
            if (stop_at_unknown_voxel) {
              return VoxelStatus::kUnknown;
            }
          } else {
            return VoxelStatus::kOccupied;
          }
        }
      }
    }
  }

  return VoxelStatus::kFree;
}
#endif

/* Line checking: */

#if (EDGE_CHECK_METHOD == 0)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  // Cast ray along the center to make sure we don't miss anything.
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  Eigen::Vector3d start_e;
  start_e = start;
  int check_count = std::ceil(edge_vec.norm() * voxel_size_inv);
  for (int i = 0; i < check_count; i++) {
    Eigen::Vector3d vox_center = start_e + i * edge_vec / edge_vec.norm();
    VoxelStatus vs = getBoxStatus(vox_center, box_size, stop_at_unknown_voxel);
    if (vs != VoxelStatus::kFree) {
      return vs;
    }
  }
  return VoxelStatus::kFree;
}
#elif (EDGE_CHECK_METHOD == 1)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;
  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  edge_vec(2) = 0.0;
  double edge_len = edge_vec.norm();
  edge_vec /= edge_len;

  Eigen::Vector3d rot_axis;
  rot_axis = edge_vec.cross(Eigen::Vector3d::UnitX());
  rot_axis = rot_axis / rot_axis.norm();

  double cos_angle = edge_vec.dot(Eigen::Vector3d::UnitX());
  if (cos_angle >= 1.0)
    cos_angle = 1.0;
  else if (cos_angle <= -1.0)
    cos_angle = -1.0;

  double angle = acos(cos_angle);

  Eigen::Matrix3d unitx2Edge;
  unitx2Edge = Eigen::AngleAxisd(angle, -rot_axis);

  // Top bottom
  double z_up = box_size[2] / 2.0;
  double z_down = -box_size[2] / 2.0;
  int horiz_voz_count = std::ceil(box_size[1] * voxel_size_inv);

  for (double i = -box_size[1] / 2; i < box_size[1] / 2; i += voxel_size) {
    Eigen::Vector3d top_vox_center_start(0.0, i, z_up);
    Eigen::Vector3d bottom_vox_center_start(0.0, i, z_down);

    Eigen::Vector3d top_vox_center_end(0.0, 0.0 + i, 0.0 + z_up);
    Eigen::Vector3d bottom_vox_center_end(0.0, 0.0 + i, 0.0 + z_down);

    Eigen::Vector3d top_vox_center_start_tf, top_vox_center_end_tf;
    Eigen::Vector3d bottom_vox_center_start_tf, bottom_vox_center_end_tf;

    top_vox_center_start_tf = start + unitx2Edge * top_vox_center_start;
    top_vox_center_end_tf = end + unitx2Edge * top_vox_center_end;
    bottom_vox_center_start_tf = start + unitx2Edge * bottom_vox_center_start;
    bottom_vox_center_end_tf = end + unitx2Edge * bottom_vox_center_end;

    // top ray
    const voxblox::Point top_start_scaled =
        top_vox_center_start_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;
    const voxblox::Point top_end_scaled =
        top_vox_center_end_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector global_voxel_indices;
    voxblox::castRay(top_start_scaled, top_end_scaled, &global_voxel_indices);
    for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      if (checkUnknownStatus(voxel)) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else if (voxel->distance <= distance_thres) {
        return VoxelStatus::kOccupied;
      }
    }

    // bottom ray
    const voxblox::Point bottom_start_scaled =
        bottom_vox_center_start_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;
    const voxblox::Point bottom_end_scaled =
        bottom_vox_center_end_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;

    voxblox::LongIndexVector bottom_global_voxel_indices;
    voxblox::castRay(bottom_start_scaled, bottom_end_scaled,
                     &bottom_global_voxel_indices);
    for (const voxblox::GlobalIndex& global_index :
         bottom_global_voxel_indices) {
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      if (checkUnknownStatus(voxel)) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else if (voxel->distance <= distance_thres) {
        return VoxelStatus::kOccupied;
      }
    }
  }

  // Left Right
  double w_left = box_size[1] / 2.0;
  double w_right = -box_size[1] / 2.0;

  for (double i = -box_size[2] / 2; i < box_size[2] / 2; i += voxel_size) {
    Eigen::Vector3d left_vox_center_start(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_start(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_end(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_end(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_start_tf, left_vox_center_end_tf;
    Eigen::Vector3d right_vox_center_start_tf, right_vox_center_end_tf;

    left_vox_center_start_tf = start + unitx2Edge * left_vox_center_start;
    left_vox_center_end_tf = end + unitx2Edge * left_vox_center_end;
    right_vox_center_start_tf = start + unitx2Edge * right_vox_center_start;
    right_vox_center_end_tf = end + unitx2Edge * right_vox_center_end;

    // left ray
    const voxblox::Point left_start_scaled =
        left_vox_center_start_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;
    const voxblox::Point left_end_scaled =
        left_vox_center_end_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector left_global_voxel_indices;
    voxblox::castRay(left_start_scaled, left_end_scaled,
                     &left_global_voxel_indices);
    for (const voxblox::GlobalIndex& global_index : left_global_voxel_indices) {
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      if (checkUnknownStatus(voxel)) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else if (voxel->distance <= distance_thres) {
        return VoxelStatus::kOccupied;
      }
    }

    // bottom ray
    const voxblox::Point right_start_scaled =
        right_vox_center_start_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;
    const voxblox::Point right_end_scaled =
        right_vox_center_end_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector right_global_voxel_indices;
    voxblox::castRay(right_start_scaled, right_end_scaled,
                     &right_global_voxel_indices);
    for (const voxblox::GlobalIndex& global_index :
         right_global_voxel_indices) {
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      if (checkUnknownStatus(voxel)) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else if (voxel->distance <= distance_thres) {
        return VoxelStatus::kOccupied;
      }
    }
  }
  return VoxelStatus::kFree;
}
#elif (EDGE_CHECK_METHOD == 2)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  Eigen::Vector3d start_e;
  start_e = start;
  int check_count = std::ceil(edge_vec.norm() * voxel_size_inv);
  for (int i = 0; i < check_count; i++) {
    Eigen::Vector3d vox_center = start_e + i * edge_vec / edge_vec.norm();
    double dist = getPointDistance(vox_center);
    if (dist < robot_radius_) {
      if (dist < 0.0) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else {
        return VoxelStatus::kOccupied;
      }
    }
  }
  return VoxelStatus::kFree;
}
#elif (EDGE_CHECK_METHOD == 3)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;
  float check_size = voxel_size * 1.0;
  const float distance_thres = voxel_size / 2.0 + 1e-6;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  edge_vec(2) = 0.0;
  double edge_len = edge_vec.norm();
  edge_vec /= edge_len;

  Eigen::Vector3d rot_axis;
  rot_axis = edge_vec.cross(Eigen::Vector3d::UnitX());
  rot_axis = rot_axis / rot_axis.norm();

  double cos_angle = edge_vec.dot(Eigen::Vector3d::UnitX());
  if (cos_angle >= 1.0)
    cos_angle = 1.0;
  else if (cos_angle <= -1.0)
    cos_angle = -1.0;

  double angle = acos(cos_angle);

  Eigen::Matrix3d unitx2Edge;
  unitx2Edge = Eigen::AngleAxisd(angle, -rot_axis);

  // Top bottom
  double z_up = box_size[2] / 2.0;
  double z_down = -box_size[2] / 2.0;
  int horiz_voz_count = std::ceil(box_size[1] * voxel_size_inv);

  for (double i = -box_size[1] / 2; i < box_size[1] / 2; i += voxel_size) {
    Eigen::Vector3d top_vox_center_start(0.0, i, z_up);
    Eigen::Vector3d bottom_vox_center_start(0.0, i, z_down);

    Eigen::Vector3d top_vox_center_end(0.0, 0.0 + i, 0.0 + z_up);
    Eigen::Vector3d bottom_vox_center_end(0.0, 0.0 + i, 0.0 + z_down);

    Eigen::Vector3d top_vox_center_start_tf, top_vox_center_end_tf;
    Eigen::Vector3d bottom_vox_center_start_tf, bottom_vox_center_end_tf;

    top_vox_center_start_tf = start + unitx2Edge * top_vox_center_start;
    top_vox_center_end_tf = end + unitx2Edge * top_vox_center_end;
    bottom_vox_center_start_tf = start + unitx2Edge * bottom_vox_center_start;
    bottom_vox_center_end_tf = end + unitx2Edge * bottom_vox_center_end;

    Eigen::Vector3d ray = top_vox_center_end_tf - top_vox_center_start_tf;
    Eigen::Vector3d ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = top_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }

    ray = bottom_vox_center_end_tf - bottom_vox_center_start_tf;
    ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = bottom_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }
  }

  // Left Right
  double w_left = box_size[1] / 2.0;
  double w_right = -box_size[1] / 2.0;
  // int horiz_voz_count = std::ceil(box_size[1]*voxel_size_inv);
  for (double i = -box_size[2] / 2; i < box_size[2] / 2; i += voxel_size) {
    Eigen::Vector3d left_vox_center_start(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_start(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_end(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_end(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_start_tf, left_vox_center_end_tf;
    Eigen::Vector3d right_vox_center_start_tf, right_vox_center_end_tf;

    left_vox_center_start_tf = start + unitx2Edge * left_vox_center_start;
    left_vox_center_end_tf = end + unitx2Edge * left_vox_center_end;
    right_vox_center_start_tf = start + unitx2Edge * right_vox_center_start;
    right_vox_center_end_tf = end + unitx2Edge * right_vox_center_end;

    Eigen::Vector3d ray = left_vox_center_end_tf - left_vox_center_start_tf;
    Eigen::Vector3d ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = left_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }

    ray = right_vox_center_end_tf - right_vox_center_start_tf;
    ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = right_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }
  }
  return VoxelStatus::kFree;
}
#endif

