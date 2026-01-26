#include "apriltag_detector/detector.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace vision_tools
{
  AprilTagDetector::AprilTagDetector(const rclcpp::NodeOptions &options)
      : Node("apriltag_detector", options)
  {
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    this->declare_parameter("max_hamming_distance", 1);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", 10,
        std::bind(&AprilTagDetector::cameraInfoCallback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10, std::bind(&AprilTagDetector::imageCallback, this, std::placeholders::_1));

    tag_publisher_ =
        this->create_publisher<extender_msgs::msg::SharedControlGoalArray>("/tag_detections", 10);

    this->get_parameter("max_hamming_distance", max_hamming_distance_);

    std::string prefix = "tag_sizes";
    auto overrides = this->get_node_parameters_interface()->get_parameter_overrides();

    for (auto const &[name, rcl_value] : overrides)
    {
      if (name.compare(0, prefix.size(), prefix) == 0)
      {
        // Extract the ID (the part after "tag_sizes.")
        std::string id_str = name.substr(prefix.size());

        try
        {
          if (!this->has_parameter(name))
          {
            this->declare_parameter(name, rcl_value.get_type());
          }

          double size_value = this->get_parameter(name).as_double();
          int tag_id = std::stoi(id_str.substr(1)); // Skip the dot

          tag_sizes_[tag_id] = size_value;
          RCLCPP_INFO(this->get_logger(), "Successfully loaded Tag ID: %d, Size: %f meters", tag_id,
                      size_value);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse tag parameter '%s': %s", name.c_str(),
                       e.what());
        }
      }
    }

    // Check if we actually loaded anything
    if (tag_sizes_.empty())
    {
      RCLCPP_WARN(this->get_logger(),
                  "No tag_sizes were loaded from YAML. Check the node name and YAML indentation.");
    }
  }

  AprilTagDetector::~AprilTagDetector()
  {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

  void AprilTagDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Extract intrinsics from the K matrix (3x3 row-major)
    // K = [fx 0 cx; 0 fy cy; 0 0 1]
    fx_ = msg->k[0];
    cx_ = msg->k[2];
    fy_ = msg->k[4];
    cy_ = msg->k[5];

    has_camera_info_ = true;

    RCLCPP_INFO(this->get_logger(), "Camera info received. Intrinsics: fx=%f, fy=%f, cx=%f, cy=%f",
                fx_, fy_, cx_, cy_);

    info.fx = fx_;
    info.fy = fy_;
    info.cx = cx_;
    info.cy = cy_;

    camera_info_sub_.reset();
  }

  void AprilTagDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    current_frame_ = cv_bridge::toCvShare(msg, "mono8")->image;
    image_u8_t current_frame_apriltag_ = {
        current_frame_.cols,
        current_frame_.rows,
        current_frame_.cols,
        current_frame_.data,
    };

    detections_ = apriltag_detector_detect(td_, &current_frame_apriltag_);

    extender_msgs::msg::SharedControlGoalArray detection_array_msg;
    detection_array_msg.header = msg->header;

    double err;
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      zarray_get(detections_, i, &det);
      if (det->hamming > max_hamming_distance_)
        continue;

      info.det = det;
      info.tagsize = tag_sizes_[det->id];

      apriltag_pose_t pose;
      err = estimate_tag_pose(&info, &pose);

      Eigen::Map<Eigen::Vector3d> translation(pose.t->data);
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(pose.R->data);
      Eigen::Quaterniond q(rotation);
      q.normalize();

      extender_msgs::msg::SharedControlGoal detection_msg;
      detection_msg.id = det->id;
      detection_msg.goal_pose.position.x = translation.x();
      detection_msg.goal_pose.position.y = translation.y();
      detection_msg.goal_pose.position.z = translation.z();

      detection_msg.goal_pose.orientation.x = q.x();
      detection_msg.goal_pose.orientation.y = q.y();
      detection_msg.goal_pose.orientation.z = q.z();
      detection_msg.goal_pose.orientation.w = q.w();

      detection_array_msg.goal_array.push_back(detection_msg);
      matd_destroy(pose.R);
      matd_destroy(pose.t);
    }
    apriltag_detections_destroy(detections_);
    tag_publisher_->publish(detection_array_msg);
  }

} // namespace vision_tools
RCLCPP_COMPONENTS_REGISTER_NODE(vision_tools::AprilTagDetector)