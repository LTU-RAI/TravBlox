#include "params.hpp"

PlanningParams::PlanningParams(rclcpp::Node::SharedPtr node) : node_(node) {
  // loadParams();
}

SensorParams::SensorParams(rclcpp::Node::SharedPtr node) : node_(node) {
  // loadParams();
}

RobotParams::RobotParams(rclcpp::Node::SharedPtr node) : node_(node) {
  // loadParams();
}

BoundedSpaceParams::BoundedSpaceParams(std::string ns, rclcpp::Node::SharedPtr node) : node_(node) {
  // loadParams(ns);
}

bool SensorParamsBase::loadParams(std::string ns, rclcpp::Node::SharedPtr node) {

  std::string parse_str = node->declare_parameter(ns + ".type", "kLidar");
  if (!parse_str.compare("kCamera"))
    type = SensorType::kCamera;
  else if (!parse_str.compare("kLidar"))
    type = SensorType::kLidar;
  else {
    return false;
  }

  parse_str.clear();
  if (type == SensorType::kCamera) {
    parse_str = node->declare_parameter(ns + ".CameraType", "kFixed");
    if (!parse_str.compare("kFixed"))
      camera_type = CameraType::kFixed;
    else if (!parse_str.compare("kRotating"))
      camera_type = CameraType::kRotating;
    else if (!parse_str.compare("kZoom"))
      camera_type = CameraType::kZoom;
    else if (!parse_str.compare("kRotatingZoom"))
      camera_type = CameraType::kRotatingZoom;
    else {
      return false;
    }
  } else {
    camera_type = CameraType::kFixed;
  }

  callback_topic = node->declare_parameter(ns + ".callback_topic", "callback_topic");
  
  focal_lenght_topic = node->declare_parameter(ns + ".focal_lenght_topic", "focal_lenght_topic");

  min_range = node->declare_parameter(ns + ".min_range", 0.0);
  
  max_range = node->declare_parameter(ns + ".max_range", 10.0);
  
  std::vector<double> param_val;
  std::vector<double> default_param_val;
  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(3);
  default_param_val[0] = 0.0;
  default_param_val[1] = 0.0;
  default_param_val[2] = 0.0;

  param_val = node->declare_parameter(ns + ".center_offset", default_param_val);
  center_offset << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(3);
  default_param_val[0] = 0.0;
  default_param_val[1] = 0.0;
  default_param_val[2] = 0.0;
  
  param_val = node->declare_parameter(ns + ".rotations", default_param_val);

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(2);
  default_param_val[0] = 1.57;
  default_param_val[1] = 1.04719;
  
  param_val = node->declare_parameter(ns + ".fov", default_param_val);
  fov << param_val[0], param_val[1];

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(2);
  default_param_val[0] = M_PI / 180.0;
  default_param_val[1] = M_PI / 180.0;
  
  param_val = node->declare_parameter(ns + ".resolution", default_param_val);
  resolution << param_val[0], param_val[1];
  
  frontier_percentage_threshold = node->declare_parameter(ns + ".frontier_percentage_threshold", 0.1);
  
  frame_id = node->declare_parameter(ns + ".frame_id", "os_sensor");

  width = node->declare_parameter(ns + ".width", 0);

  height = node->declare_parameter(ns + ".height", 0);
  
  widthRemoval = node->declare_parameter(ns + ".width_removal", 0);

  heightremoval = node->declare_parameter(ns + ".height_removal", 0);
  
  // Precompute some const parameters to be used later.
  // Rotation from B to S.
  rot_B2S = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_S2B = rot_B2S.inverse();

  double v_res, h_res;
  v_res = (resolution[1] > 0) ? resolution[1] : (1.0 * M_PI / 180.0);
  h_res = (resolution[0] > 0) ? resolution[0] : (1.0 * M_PI / 180.0);
  // Compute the normal vectors enclosed the FOV.
  if (type == SensorType::kCamera) {
    if (width == 0 && height == 0) {
      // Compute 4 normal vectors for left, right, top, down planes.
      // First, compute 4 coner points.
      // Assume the range is along the hypotenuse of the right angle.
      double h_2 = fov[0] / 2;
      double v_2 = fov[1] / 2;
      Eigen::Vector3d pTL(cos(h_2), sin(h_2), sin(v_2));
      Eigen::Vector3d pTR(cos(h_2), -sin(h_2), sin(v_2));
      Eigen::Vector3d pBR(cos(h_2), -sin(h_2), -sin(v_2));
      Eigen::Vector3d pBL(cos(h_2), sin(h_2), -sin(v_2));
      edge_points.col(0) = pTL;
      edge_points.col(1) = pTR;
      edge_points.col(2) = pBR;
      edge_points.col(3) = pBL;
      // Compute normal vectors for 4 planes. (normalized)
      normal_vectors.col(0) = edge_points.col(0).cross(edge_points.col(1));
      normal_vectors.col(1) = edge_points.col(1).cross(edge_points.col(2));
      normal_vectors.col(2) = edge_points.col(2).cross(edge_points.col(3));
      normal_vectors.col(3) = edge_points.col(3).cross(edge_points.col(0));
      // Compute correct points based on the sensor range.
      edge_points = max_range * edge_points;
      edge_points_B = rot_B2S * edge_points;
      // Frustum endpoints in (S) for gain calculation.
      frustum_endpoints.clear();
      frustum_endpoints_B.clear();
      int w = 0, h = 0;
      height = 0;
      width = 0;
      double h_lim_2 = fov[0] / 2;
      double v_lim_2 = fov[1] / 2;
      for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
        ++h;
        for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
          if (width == 0) {
            ++w;
          }
          double x = max_range * cos(dh);
          double y = max_range * sin(dh);
          double z = max_range * sin(dv);
          Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
          frustum_endpoints.push_back(ep);
          Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
          frustum_endpoints_B.push_back(ep_B);
        }
        if (width == 0) {
          width = w;
        }
      }
      height = h;
      RCLCPP_INFO_EXPRESSION(
          node->get_logger(), global_verbosity >= Verbosity::INFO,
          "Computed multiray_endpoints for volumetric gain [kCamera]: [%d] "
          "points.",
          frustum_endpoints_B.size());
    }
  } else if (type == SensorType::kLidar) {
    // Frustum endpoints in (S) for gain calculation.
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    int w = 0, h = 0;
    height = 0;
    width = 0;
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
      ++h;
      for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
        if (width == 0) {
          ++w;
        }
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
      if (width == 0) {
        width = w;
      }
    }
    height = h;
    RCLCPP_INFO_EXPRESSION(
        node->get_logger(), global_verbosity >= Verbosity::INFO,
        "Computed multiray_endpoints for volumetric gain [kLidar]: [%d] "
        "points.",
        frustum_endpoints_B.size());
  }

  // Compute a number to compare to check for frontier.
  num_voxels_full_fov = (fov[0] / h_res) * (fov[1] / v_res) * max_range;

  return true;
}

void SensorParamsBase::getFrustumEndpoints(Eigen::Vector4d& state,
                                           std::vector<Eigen::Vector3d>& ep) {
  // Convert rays from B to W.
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  ep.clear();
  for (auto& p : frustum_endpoints_B) {
    Eigen::Vector3d p_tf = origin + rot_W2B * p;
    ep.push_back(p_tf);
  }
}

void SensorParamsBase::getFrustumEndpoints(Eigen::Vector4d& state,
                                           std::vector<Eigen::Vector3d>& ep,
                                           float darkness_range) {
  // Convert rays from B to W.
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  ep.clear();
  for (auto& p : frustum_endpoints_B) {
    Eigen::Vector3d p_tf = origin + rot_W2B * p * darkness_range;
    ep.push_back(p_tf);
  }
}

void SensorParamsBase::updateFrustumEndpoints() {
  rclcpp::Logger logger = rclcpp::get_logger("params_logger");
  RCLCPP_INFO_EXPRESSION(logger, global_verbosity >= Verbosity::INFO,
                         "[PARAMS]: fov: h: %f v: %f", fov[0], fov[1]);
  double v_res, h_res;
  v_res = (resolution[1] > 0) ? resolution[1] : (1.0 * M_PI / 180.0);
  h_res = (resolution[0] > 0) ? resolution[0] : (1.0 * M_PI / 180.0);
  if (type == SensorType::kCamera) {
    // Compute 4 normal vectors for left, right, top, down planes.
    // First, compute 4 coner points.
    // Assume the range is along the hypotenuse of the right angle.
    double h_2 = fov[0] / 2;
    double v_2 = fov[1] / 2;
    Eigen::Vector3d pTL(cos(h_2), sin(h_2), sin(v_2));
    Eigen::Vector3d pTR(cos(h_2), -sin(h_2), sin(v_2));
    Eigen::Vector3d pBR(cos(h_2), -sin(h_2), -sin(v_2));
    Eigen::Vector3d pBL(cos(h_2), sin(h_2), -sin(v_2));
    edge_points.col(0) = pTL;
    edge_points.col(1) = pTR;
    edge_points.col(2) = pBR;
    edge_points.col(3) = pBL;
    // Compute normal vectors for 4 planes. (normalized)
    normal_vectors.col(0) = edge_points.col(0).cross(edge_points.col(1));
    normal_vectors.col(1) = edge_points.col(1).cross(edge_points.col(2));
    normal_vectors.col(2) = edge_points.col(2).cross(edge_points.col(3));
    normal_vectors.col(3) = edge_points.col(3).cross(edge_points.col(0));
    // Compute correct points based on the sensor range.
    edge_points = max_range * edge_points;
    edge_points_B = rot_B2S * edge_points;
    // Frustum endpoints in (S) for gain calculation.
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
      for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
    }
    RCLCPP_INFO_EXPRESSION(
        logger, global_verbosity >= Verbosity::INFO,
        "Computed multiray_endpoints for volumetric gain [kCamera]: [%d] "
        "points.",
        frustum_endpoints_B.size());
  } else if (type == SensorType::kLidar) {
    // Frustum endpoints in (S) for gain calculation.
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
      for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
    }
    RCLCPP_INFO_EXPRESSION(
        logger, global_verbosity >= Verbosity::INFO,
        "Computed multiray_endpoints for volumetric gain [kLidar]: [%d] "
        "points.",
        (int)frustum_endpoints_B.size());
  }
}

void SensorParamsBase::convertBodyToSensor(
    pcl::PointCloud<pcl::PointXYZ>::Ptr ep,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ep_s) {
  ep_s->points.clear();

  // Sensor in ROS coordinate, if want to transform to real camera coordinate,
  // apply this TF
  Eigen::Matrix3d rot_S2Cam;
  rot_S2Cam = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

  for (auto p : ep->points) {
    Eigen::Vector3d ip_p(p.x, p.y, p.z);
    Eigen::Vector3d op_p = rot_S2B * (ip_p - center_offset);
    if (type == SensorType::kCamera) op_p = rot_S2Cam * op_p;
    pcl::PointXYZ data;
    data.x = op_p(0);
    data.y = op_p(1);
    data.z = op_p(2);
    ep_s->points.push_back(data);
  }
}

void SensorParamsBase::getFrustumEdges(Eigen::Vector4d& state,
                                       std::vector<Eigen::Vector3d>& edges) {
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  edges.clear();
  for (int i = 0; i < 4; ++i) {
    edges.push_back(origin + rot_W2B * edge_points_B.col(i));
  }
}

bool SensorParamsBase::isInsideFOV(Eigen::Vector4d& state, Eigen::Vector3d& pos) {
  // Method:
  // a) Convert a point into Sensor coordinate.
  //    Usually from World (W) -> Body (B) -> Sensor (S).
  // b) Check if it is inside FOV of sensor.
  //    Check distance to sensor first, then angle.
  //    For Camera: use normal vectors in (S) to identify.
  //    For LiDAR: compute horizontal and vertical angle in (S) to identify.

  // Transform to sensor coordinate.
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d pos_S =
      rot_S2B * Eigen::AngleAxisd(-state[3], Eigen::Vector3d::UnitZ()) *
          (pos - origin) -
      center_offset;
  float pos_S_norm = pos_S.norm();

  // Check range.
  if (pos_S_norm > max_range) return false;

  // Check FOV angles.
  if (type == SensorType::kCamera) {
    for (int i = 0; i < 4; ++i) {
      double res = pos_S.dot(normal_vectors.col(i));
      if (res <= 0) return false;
    }
  } else if (type == SensorType::kLidar) {
    // @TODO: this might be very costly.
    float h_angle = std::atan2((float)pos_S.y(), (float)pos_S.x());
    float v_angle = std::asin((float)pos_S.z() / pos_S_norm);
    if ((std::abs(h_angle) > (fov[0] / 2)) ||
        (std::abs(v_angle) > (fov[1] / 2)))
      return false;
  } else {
    // Unsupported sensor.
    return false;
  }
  return true;
}

bool SensorParamsBase::isFrontier(double num_unknown_voxels_normalized,
                                  double& unknown_percentage) {
  unknown_percentage = num_unknown_voxels_normalized / num_voxels_full_fov;
  if (unknown_percentage >= frontier_percentage_threshold)
    return true;
  else
    return false;
}


void SensorParamsBase::getOccupiedPercentage(double num_occupied_voxels_normalized,
                                  double& occupied_percentage) {
  occupied_percentage = num_occupied_voxels_normalized / num_voxels_full_fov;
}

bool SensorParams::loadParams() {
  std::string param_name;
  std::vector<double> param_val;

  std::vector<std::string> parse_str_list;
  std::vector<std::string> default_str_list;
  default_str_list.clear();
  default_str_list.push_back("fake_sensor");

  // param_name = "sensor_list";
  sensor_list = node_->declare_parameter("SensorParams.sensor_list", default_str_list);
  if (sensor_list.size() <= 0) {
    std::cout << "SensorParams : sensor list is empty" << std::endl;
    return false;
  }
  // sensor_list = parse_str_list;
  for (auto it = sensor_list.begin(); it != sensor_list.end(); ++it) {
    SensorParamsBase spb;
    std::string sensor_ns = "SensorParams." + *it;
    if (spb.loadParams(sensor_ns, node_)) { 
      sensor.emplace(std::make_pair(*it, spb));
    } else {
      return false;
    }
  }

  return true;
}

bool RobotParams::loadParams() {
  std::string param_name;
  std::vector<double> param_val;
  std::vector<double> default_param_val;

  std::string parse_str = node_->declare_parameter("RobotParams.type", "kAerialRobot");
  if (!parse_str.compare("kAerialRobot"))
    type = RobotType::kAerialRobot;
  else if (!parse_str.compare("kGroundRobot"))
    type = RobotType::kGroundRobot;
  else {
    type = RobotType::kAerialRobot;
  }

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(3);
  default_param_val[0] = 0.5;
  default_param_val[1] = 0.5;
  default_param_val[2] = 0.5;
  param_val = node_->declare_parameter("RobotParams.size", default_param_val);
  size << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(3);
  default_param_val[0] = 0.5;
  default_param_val[1] = 0.5;
  default_param_val[2] = 0.5;
  param_val = node_->declare_parameter("RobotParams.size_extension_min", default_param_val);
  size_extension_min << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(3);
  default_param_val[0] = 0.5;
  default_param_val[1] = 0.5;
  default_param_val[2] = 0.5;
  param_val = node_->declare_parameter("RobotParams.size_extension", default_param_val);
  size_extension << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(3);
  default_param_val[0] = 0.5;
  default_param_val[1] = 0.5;
  default_param_val[2] = 0.5;
  param_val = node_->declare_parameter("RobotParams.center_offset", default_param_val);
  center_offset << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  default_param_val.clear();
  default_param_val.resize(3);
  default_param_val[0] = 0.5;
  default_param_val[1] = 0.5;
  default_param_val[2] = 0.5;
  param_val = node_->declare_parameter("RobotParams.safety_extension", default_param_val);
  safety_extension << param_val[0], param_val[1], param_val[2];
  
  relax_ratio = node_->declare_parameter("RobotParams.relax_ratio", 0.5);

  parse_str = node_->declare_parameter("RobotParams.bound_mode", "kExtendedBound");
  if (!parse_str.compare("kExtendedBound"))
    bound_mode = BoundModeType::kExtendedBound;
  else if (!parse_str.compare("kRelaxedBound"))
    bound_mode = BoundModeType::kRelaxedBound;
  else if (!parse_str.compare("kMinBound"))
    bound_mode = BoundModeType::kMinBound;
  else if (!parse_str.compare("kExactBound"))
    bound_mode = BoundModeType::kExactBound;
  else if (!parse_str.compare("kNoBound"))
    bound_mode = BoundModeType::kNoBound;
  else {
    bound_mode = BoundModeType::kExtendedBound;
  }

  return true;
}

void RobotParams::setBoundMode(BoundModeType bmode) {
  rclcpp::Logger logger = rclcpp::get_logger("params_logger");
  bound_mode = bmode;
  switch (bound_mode) {
    case BoundModeType::kExtendedBound:
      RCLCPP_INFO_EXPRESSION(logger, global_verbosity >= Verbosity::INFO,
                             "Set bound mode: kExtendedBound");
      break;
    case BoundModeType::kRelaxedBound:
      RCLCPP_INFO_EXPRESSION(logger, global_verbosity >= Verbosity::INFO,
                             "Set bound mode: kRelaxedBound");
      break;
    case BoundModeType::kMinBound:
      RCLCPP_INFO_EXPRESSION(logger, global_verbosity >= Verbosity::INFO,
                             "Set bound mode: kMinBound");
      break;
    case BoundModeType::kExactBound:
      RCLCPP_INFO_EXPRESSION(logger, global_verbosity >= Verbosity::INFO,
                             "Set bound mode: kExactBound");
      break;
    case BoundModeType::kNoBound:
      RCLCPP_INFO_EXPRESSION(logger, global_verbosity >= Verbosity::INFO,
                             "Set bound mode: kNoBound");
      break;
  }
}

void RobotParams::getPlanningSize(Eigen::Vector3d& psize) {
  psize << 0.0, 0.0, 0.0;
  switch (bound_mode) {
    case BoundModeType::kExtendedBound:
      psize = size + size_extension;
      break;
    case BoundModeType::kRelaxedBound:
      psize = size + relax_ratio * size_extension_min +
              (1 - relax_ratio) * size_extension;
      break;
    case BoundModeType::kMinBound:
      psize = size + size_extension_min;
      break;
    case BoundModeType::kExactBound:
      psize = size;
      break;
    case BoundModeType::kNoBound:
      psize << 0.0, 0.0, 0.0;
      break;
  }
}

bool BoundedSpaceParams::loadParams(std::string ns) {
  std::string param_name;
  std::vector<double> param_val;
  std::vector<double> default_param_val;

  std::string parse_str = node_->declare_parameter(ns + ".type", "kFake");
  if (!parse_str.compare("kCuboid"))
    type = BoundedSpaceType::kCuboid;
  else if (!parse_str.compare("kSphere"))
    type = BoundedSpaceType::kSphere;
  else {
    return false;
  }

  if (type == BoundedSpaceType::kCuboid) {
    param_val.clear();
    default_param_val.clear();
    default_param_val.resize(3);
    default_param_val[0] = 0;
    default_param_val[1] = 0;
    default_param_val[2] = 0;
    param_val = node_->declare_parameter(ns + ".min_val", default_param_val);
    min_val << param_val[0], param_val[1], param_val[2];

    param_val.clear();
    default_param_val.clear();
    default_param_val.resize(3);
    default_param_val[0] = 0;
    default_param_val[1] = 0;
    default_param_val[2] = 0;
    param_val = node_->declare_parameter(ns + ".max_val", default_param_val);
    max_val << param_val[0], param_val[1], param_val[2];

    root_pos = Eigen::Vector3d::Zero();

    param_val.clear();
    default_param_val.clear();
    default_param_val.resize(3);
    default_param_val[0] = 0;
    default_param_val[1] = 0;
    default_param_val[2] = 0;
    param_val = node_->declare_parameter(ns + ".min_extension", default_param_val);
    min_extension << param_val[0], param_val[1], param_val[2];

    param_val.clear();
    default_param_val.clear();
    default_param_val.resize(3);
    default_param_val[0] = 0;
    default_param_val[1] = 0;
    default_param_val[2] = 0;
    param_val = node_->declare_parameter(ns + ".max_extension", default_param_val);
    max_extension << param_val[0], param_val[1], param_val[2];
    
    min_val_total = min_val + min_extension;
    max_val_total = max_val + max_extension;

    param_val.clear();
    default_param_val.clear();
    default_param_val.resize(3);
    default_param_val[0] = 0;
    default_param_val[1] = 0;
    default_param_val[2] = 0;
    param_val = node_->declare_parameter(ns + ".rotations", default_param_val);
    rotations << param_val[0], param_val[1], param_val[2];

    this->setRotation(rotations);

  } else if (type == BoundedSpaceType::kSphere) {
    radius = node_->declare_parameter(ns + ".radius", 10.0);
    radius_extension = node_->declare_parameter(ns + ".radius_extension", 0.0);
  }
  
  return true;
}

void BoundedSpaceParams::setCenter(Eigen::Vector4d& state, bool use_extension) {
  root_pos << state[0], state[1], state[2];
  if (use_extension) {
    min_val_total = min_val + min_extension;
    max_val_total = max_val + max_extension;
    radius_total = radius + radius_extension;
  } else {
    min_val_total = min_val;
    max_val_total = max_val;
    radius_total = radius;
  }
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.transpose();
}

void BoundedSpaceParams::setCenter(Eigen::Vector3d& root, bool use_extension) {
  root_pos = root;
  if (use_extension) {
    min_val_total = min_val + min_extension;
    max_val_total = max_val + max_extension;
    radius_total = radius + radius_extension;
  } else {
    min_val_total = min_val;
    max_val_total = max_val;
    radius_total = radius;
  }
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.transpose();
}

void BoundedSpaceParams::setBound(Eigen::Vector3d& min_val_in,
                                  Eigen::Vector3d& max_val_in) {
  min_val = min_val_in;
  max_val = max_val_in;
}

void BoundedSpaceParams::setRotation(Eigen::Vector3d& rotations_in) {
  rotations = rotations_in;

  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.transpose();
}

bool BoundedSpaceParams::isInsideSpace(Eigen::Vector3d& pos) {
  bool res = true;
  if (type == BoundedSpaceType::kSphere) {
    // No need to check orientation.
    Eigen::Vector3d dist = pos - root_pos;
    double dist_norm = dist.norm();
    if (dist_norm > radius_total) res = false;
  } else if (type == BoundedSpaceType::kCuboid) {
    // Have to check orientation.
    Eigen::Vector3d pos_B = rot_B2W * (pos - root_pos);
    for (int i = 0; i < 3; ++i) {
      if ((pos_B[i] < min_val[i]) || (pos_B[i] > max_val[i])) {
        res = false;
        break;
      }
    }
  }
  return res;
}

bool PlanningParams::loadParams() {
  std::string ns = "ns";

  std::string parse_str =
      node_->declare_parameter("PlanningParams.type", "kBasicExploration");
  if (!parse_str.compare("kBasicExploration"))
    type = PlanningModeType::kBasicExploration;
  else if (!parse_str.compare("kNarrowEnvExploration"))
    type = PlanningModeType::kNarrowEnvExploration;
  else if (!parse_str.compare("kAdaptiveExploration"))
    type = PlanningModeType::kAdaptiveExploration;
  else {
    type = PlanningModeType::kBasicExploration;
  }

  std::string mode_parse_str =
      node_->declare_parameter("PlanningParams.planner_mode", "kSim");
  if (!mode_parse_str.compare("kSim"))
    planner_mode = PlannerMode::kSim;
  else if (!mode_parse_str.compare("kReal"))
    planner_mode = PlannerMode::kReal;
  else {
    planner_mode = PlannerMode::kReal;
  }

  parse_str = node_->declare_parameter("PlanningParams.rr_mode", "kGraph");
  if (!parse_str.compare("kTree"))
    rr_mode = RRModeType::kTree;
  else if (!parse_str.compare("kGraph"))
    rr_mode = RRModeType::kGraph;
  else {
    rr_mode = RRModeType::kGraph;
  }

  std::vector<std::string> default_sensor_list;
  default_sensor_list.clear();
  default_sensor_list.push_back("OS064");
  std::vector<std::string> parse_str_list = node_->declare_parameter(
      "PlanningParams.exp_sensor_list", default_sensor_list);
  if (parse_str_list.size() <= 0) {
    std::cout << "Exploration sensor not loaded " << std::endl;
  } else {
    exp_sensor_list = parse_str_list;
    std::string str_tmp = "Sensors for exploration: ";
    for (int i = 0; i < exp_sensor_list.size(); ++i) {
      str_tmp += exp_sensor_list[i] + ", ";
    }
  }

  pose_graph_update_dist =
      node_->declare_parameter("PlanningParams.pose_graph_update_dist", 0.1111);
  v_max = node_->declare_parameter("PlanningParams.v_max", 0.2);
  v_homing_max = node_->declare_parameter("PlanningParams.v_homing_max", 0.2);
  yaw_rate_max = node_->declare_parameter("PlanningParams.yaw_rate_max", 0.4);
  dt = node_->declare_parameter("PlanningParams.dt", 0.1);
  yaw_tangent_correction =
      node_->declare_parameter("PlanningParams.yaw_tangent_correction", false);
  exp_gain_voxel_size =
      node_->declare_parameter("PlanningParams.exp_gain_voxel_size", 0.4);
  use_ray_model_for_volumetric_gain = node_->declare_parameter(
      "PlanningParams.use_ray_model_for_volumetric_gain", false);
  free_voxel_gain =
      node_->declare_parameter("PlanningParams.free_voxel_gain", 1.0);
  occupied_voxel_gain =
      node_->declare_parameter("PlanningParams.occupied_voxel_gain", 1.0);
  unknown_voxel_gain =
      node_->declare_parameter("PlanningParams.unknown_voxel_gain", 10.0);
  edge_length_min =
      node_->declare_parameter("PlanningParams.edge_length_min", 0.2);
  edge_length_max =
      node_->declare_parameter("PlanningParams.edge_length_max", 0.2);
  distance_to_new_path =
      node_->declare_parameter("PlanningParams.distance_to_new_path", 0.5);
  distance_to_new_wp =
      node_->declare_parameter("PlanningParams.distance_to_new_wp", 0.5);
  num_vertices_max =
      node_->declare_parameter("PlanningParams.num_vertices_max", 500.0);
  num_edges_max =
      node_->declare_parameter("PlanningParams.num_edges_max", 5000.0);
  num_connections_max =
      node_->declare_parameter("PlanningParams.num_connections_max", 10.0);
  edge_overshoot =
      node_->declare_parameter("PlanningParams.edge_overshoot", 0.2);
  num_loops_cutoff =
      node_->declare_parameter("PlanningParams.num_loops_cutoff", 1000.0);
  num_loops_max =
      node_->declare_parameter("PlanningParams.num_loops_max", 10000.0);
  nearest_range = node_->declare_parameter("PlanningParams.nearest_range", 1.0);
  nearest_range_min =
      node_->declare_parameter("PlanningParams.nearest_range_min", 0.5);
  nearest_range_max =
      node_->declare_parameter("PlanningParams.nearest_range_max", 2.0);
  local_space = node_->declare_parameter("PlanningParams.local_space", 10.0);
  graph_density_scalar =
      node_->declare_parameter("PlanningParams.graph_density_scalar", 1.0);
  use_current_state =
      node_->declare_parameter("PlanningParams.use_current_state", true);
  path_length_penalty = node_->declare_parameter(
      "PlanningParams.path_length_penalty", 0.0);  // no penalty
  path_direction_penalty = node_->declare_parameter(
      "PlanningParams.path_direction_penalty", 0.0);  // no penalty
  hanging_vertex_penalty = node_->declare_parameter(
      "PlanningParams.hanging_vertex_penalty", 0.0);  // no penalty
  traverse_length_max =
      node_->declare_parameter("PlanningParams.traverse_length_max",
                               0.2);  // Assuming edge_length_max is 0.2
  traverse_time_max = node_->declare_parameter(
      "PlanningParams.traverse_time_max",
      0.2 / 0.2);  // Assuming edge_length_max is 0.2 and v_max is 0.2
  augment_free_voxels_time =
      node_->declare_parameter("PlanningParams.augment_free_voxels_time", 5.0);
  free_frustum_before_planning = node_->declare_parameter(
      "PlanningParams.free_frustum_before_planning", false);
  auto_homing_enable =
      node_->declare_parameter("PlanningParams.auto_homing_enable", false);
  geofence_checking_enable = node_->declare_parameter(
      "PlanningParams.geofence_checking_enable", false);
  exploration_time_budget = node_->declare_parameter(
      "PlanningParams.exploration_time_budget", std::numeric_limits<double>::max());
  homing_backward =
      node_->declare_parameter("PlanningParams.homing_backward", false);
  planning_backward =
      node_->declare_parameter("PlanningParams.planning_backward", false);
  path_safety_enhance_enable = node_->declare_parameter(
      "PlanningParams.path_safety_enhance_enable", false);
  global_frame_id =
      node_->declare_parameter("PlanningParams.global_frame_id", "world");
  freespace_cloud_enable =
      node_->declare_parameter("PlanningParams.freespace_cloud_enable", false);
  leafs_only_for_volumetric_gain = node_->declare_parameter(
      "PlanningParams.leafs_only_for_volumetric_gain", false);
  max_ground_height =
      node_->declare_parameter("PlanningParams.max_ground_height", 1.2);
  robot_height = node_->declare_parameter("PlanningParams.robot_height", 1.0);
  max_inclination =
      node_->declare_parameter("PlanningParams.max_inclination", 0.52);
  cluster_vertices_for_gain = node_->declare_parameter(
      "PlanningParams.cluster_vertices_for_gain", false);
  clustering_radius =
      node_->declare_parameter("PlanningParams.clustering_radius", 2.0);
  path_interpolation_distance = node_->declare_parameter(
      "PlanningParams.path_interpolation_distance", 0.5);
  auto_global_planner_enable = node_->declare_parameter(
      "PlanningParams.auto_global_planner_enable", true);
  relaxed_corridor_multiplier = node_->declare_parameter(
      "PlanningParams.relaxed_corridor_multiplier", 1.0);
  interpolate_projection_distance = node_->declare_parameter(
      "PlanningParams.interpolate_projection_distance", false);
  go_home_if_fully_explored = node_->declare_parameter(
      "PlanningParams.go_home_if_fully_explored", false);
  ray_cast_step_size_multiplier = node_->declare_parameter(
      "PlanningParams.ray_cast_step_size_multiplier", 1.0);
  nonuniform_ray_cast =
      node_->declare_parameter("PlanningParams.nonuniform_ray_cast", true);
  time_budget_before_landing = node_->declare_parameter(
      "PlanningParams.time_budget_before_landing",
      std::numeric_limits<double>::max());  // Assuming time_budget_limit is max
  auto_landing_enable =
      node_->declare_parameter("PlanningParams.auto_landing_enable", false);
  max_negative_inclination =
      node_->declare_parameter("PlanningParams.max_negative_inclination", 0.37);

  return true;
}

void PlanningParams::setPlanningMode(PlanningModeType pmode) {
  rclcpp::Logger logger = rclcpp::get_logger("params_logger");
  switch (pmode) {
    case PlanningModeType::kBasicExploration:
      type = PlanningModeType::kBasicExploration;
      RCLCPP_WARN_EXPRESSION(logger, global_verbosity >= Verbosity::WARN,
                             "Exploration mode is set to kBasicExploration");
      break;
    default:
      type = PlanningModeType::kBasicExploration;
      RCLCPP_WARN_EXPRESSION(logger, global_verbosity >= Verbosity::WARN,
                             "Exploration mode is set to kBasicExploration");
      break;
  }
}

// }  // namespace explorer
