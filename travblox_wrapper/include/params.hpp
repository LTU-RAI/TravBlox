#ifndef PARAMS_H_
#define PARAMS_H_

#include <unordered_map>
#include <vector>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <pcl/common/distances.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"

// Enums
enum ProjectedEdgeStatus {
  kAdmissible = 0,
  kSteep,
  kOccipied,
  kUnknown,
  kHanging
};

enum Verbosity {
  SILENT = 0,
  PLANNER_STATUS = 1,
  ERROR = 2,
  WARN = 3,
  INFO = 4,
  DEBUG = 5
};

#define global_verbosity Verbosity::WARN

enum SensorType { kCamera = 0, kLidar = 1 };
enum CameraType { kFixed = 0, kRotating, kZoom, kRotatingZoom };

enum struct PlannerTriggerModeType { kManual = 0, kAuto = 1 };

enum BoundModeType {
  kExtendedBound = 0,
  kRelaxedBound = 1,
  kMinBound = 2,
  kExactBound = 3,
  kNoBound = 4
};

enum RobotType { kAerialRobot = 0, kGroundRobot };

enum PlannerMode { kSim = 0, kReal };

enum BoundedSpaceType { kCuboid = 0, kSphere };

enum PlanningModeType {
  kBasicExploration = 0,
  kNarrowEnvExploration = 1,
  kAdaptiveExploration = 2
};

enum RRModeType { kGraph = 0, kTree };

// Structs
struct SensorParamsBase {
  SensorType type;
  CameraType camera_type;
  double min_range;
  double max_range;
  Eigen::Vector3d center_offset;
  Eigen::Vector3d rotations;
  Eigen::Vector2d fov;
  Eigen::Vector2d resolution;
  std::string frame_id;
  std::string callback_topic;
  std::string focal_lenght_topic;
  int height;
  int width;
  int heightremoval;
  int widthRemoval;
  bool loadParams(std::string ns, rclcpp::Node::SharedPtr node);

  bool isInsideFOV(Eigen::Vector4d& state, Eigen::Vector3d& pos);
  void getFrustumEndpoints(Eigen::Vector4d& state, std::vector<Eigen::Vector3d>& ep);
  void getFrustumEndpoints(Eigen::Vector4d& state, std::vector<Eigen::Vector3d>& ep,
                           float darkness_range);

  void updateFrustumEndpoints();
  void getFrustumEdges(Eigen::Vector4d& state, std::vector<Eigen::Vector3d>& edges);
  bool isFrontier(double num_unknown_voxels_normalized,
                  double& unknown_percentage);
  void getOccupiedPercentage(double num_occupied_voxels_normalized,
                             double& occupied_percentage);
  void convertBodyToSensor(pcl::PointCloud<pcl::PointXYZ>::Ptr ep,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr ep_s);

 private:
  Eigen::Matrix3d rot_B2S;
  Eigen::Matrix3d rot_S2B;

  Eigen::Matrix<double, 3, 4> edge_points;
  Eigen::Matrix<double, 3, 4> edge_points_B;
  Eigen::Matrix<double, 3, 4> normal_vectors;

  std::vector<Eigen::Vector3d> frustum_endpoints;
  std::vector<Eigen::Vector3d> frustum_endpoints_B;

  double num_voxels_full_fov;
  double frontier_percentage_threshold;
};

class BoundedSpaceParams {
 public:
  BoundedSpaceParams(std::string ns, rclcpp::Node::SharedPtr node);
  BoundedSpaceType type;
  Eigen::Vector3d min_val;
  Eigen::Vector3d max_val;
  Eigen::Vector3d min_extension;
  Eigen::Vector3d max_extension;
  Eigen::Vector3d rotations;
  double radius;
  double radius_extension;
  bool loadParams(std::string ns);

  void setCenter(Eigen::Vector4d& state, bool use_extension);
  void setCenter(Eigen::Vector3d& root, bool use_extension);
  Eigen::Vector3d getCenter() { return root_pos; }
  Eigen::Matrix3d getRotationMatrix() { return rot_B2W; }
  void setBound(Eigen::Vector3d& min_val_in, Eigen::Vector3d& max_val_in);
  void setRotation(Eigen::Vector3d& rotations_in);
  bool isInsideSpace(Eigen::Vector3d& pos);

 private:
  rclcpp::Node::SharedPtr node_;
  Eigen::Vector3d root_pos;
  Eigen::Matrix3d rot_B2W;
  Eigen::Vector3d min_val_total;
  Eigen::Vector3d max_val_total;
  double radius_total;
};

// Classes
class SensorParams {
 public:
  SensorParams(rclcpp::Node::SharedPtr node);
  std::vector<std::string> sensor_list;
  std::unordered_map<std::string, SensorParamsBase> sensor;
  bool loadParams();

 private:
  rclcpp::Node::SharedPtr node_;
};

class RobotParams {
 public:
  RobotParams(rclcpp::Node::SharedPtr node);

  RobotType type;
  Eigen::Vector3d size;
  Eigen::Vector3d size_extension_min;
  Eigen::Vector3d size_extension;
  Eigen::Vector3d center_offset;
  double relax_ratio;
  BoundModeType bound_mode;
  Eigen::Vector3d safety_extension;

  void setBoundMode(BoundModeType bmode);
  void getPlanningSize(Eigen::Vector3d& psize);

  bool loadParams();

 private:
  rclcpp::Node::SharedPtr node_;
};

class PlanningParams {
 public:
  PlanningParams(rclcpp::Node::SharedPtr node);
  PlannerMode planner_mode;
  std::string global_frame_id;
  bool freespace_cloud_enable;
  double v_max;
  double dt;
  double pose_graph_update_dist;
  double v_homing_max;
  double yaw_rate_max;
  bool yaw_tangent_correction;
  PlanningModeType type;
  RRModeType rr_mode;
  double edge_length_min;
  double edge_length_max;
  double distance_to_new_path;
  double distance_to_new_wp;
  double edge_overshoot;
  double num_vertices_max;
  double num_edges_max;
  double num_connections_max;
  double num_loops_cutoff;
  double num_loops_max;
  double nearest_range;
  double nearest_range_min;
  double nearest_range_max;
  double local_space;
  double graph_density_scalar;
  bool use_current_state;
  bool geofence_checking_enable;
  double max_ground_height;
  double robot_height;
  double max_inclination;
  bool interpolate_projection_distance;
  double augment_free_voxels_time;
  bool augment_free_frustum_en;
  bool free_frustum_before_planning;
  std::vector<std::string> exp_sensor_list;
  std::vector<std::string> no_gain_zones_list;
  double exp_gain_voxel_size;
  bool use_ray_model_for_volumetric_gain;
  double free_voxel_gain;
  double occupied_voxel_gain;
  double unknown_voxel_gain;
  double path_length_penalty;
  double path_direction_penalty;
  double hanging_vertex_penalty;
  bool leafs_only_for_volumetric_gain;
  bool cluster_vertices_for_gain;
  double clustering_radius;
  double ray_cast_step_size_multiplier;
  bool nonuniform_ray_cast;
  double traverse_length_max;
  double traverse_time_max;
  bool planning_backward;
  bool path_safety_enhance_enable;
  double path_interpolation_distance;
  double relaxed_corridor_multiplier;
  bool auto_global_planner_enable;
  bool go_home_if_fully_explored;
  bool auto_homing_enable;
  bool homing_backward;
  double exploration_time_budget;
  bool auto_landing_enable;
  double time_budget_before_landing;
  double max_negative_inclination;

  bool loadParams();
  void setPlanningMode(PlanningModeType pmode);

 private:
  rclcpp::Node::SharedPtr node_;
};

#endif
