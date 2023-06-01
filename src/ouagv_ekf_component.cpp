// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Headers in this package
#include "ouagv_ekf/ouagv_ekf_component.hpp"

// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component

namespace ouagv_ekf
{
  EkfComponent::EkfComponent(const rclcpp::NodeOptions &options)
      : Node("ouagv_ekf_node", options),
        first_odom_subscribed(false)
  {
    declare_parameter("reference_frame_id", "map");
    get_parameter("reference_frame_id", reference_frame_id);
    declare_parameter("base_frame_id", "base_link");
    get_parameter("base_frame_id", base_frame_id);
    declare_parameter("odom_frame_id", "odom");
    get_parameter("odom_frame_id", odom_frame_id);
    declare_parameter<double>("sigma_odom", 1e-1);
    get_parameter("sigma_odom", sigma_odom);
    declare_parameter<double>("sigma_ndt_pose", 1e-5);
    get_parameter("sigma_ndt_pose", sigma_ndt_pose);
    declare_parameter<double>("sigma_imu", 1e-6);
    get_parameter("sigma_imu", sigma_imu);
    declare_parameter("use_imu", false);
    get_parameter("use_imu", use_imu);
    declare_parameter<double>("dt", 0.01);
    get_parameter("dt", dt);

    // 最初のodomのsubscribeの時はposeとtfもpubしておく
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        { 
          RCLCPP_INFO(get_logger(), "odom subscribed");
          if(!first_odom_subscribed)
        {
          RCLCPP_INFO(get_logger(), "first odom subscribed");
          first_odom_subscribed = true;
          current_pose = *msg;

          X_predicted(0) = msg->pose.pose.position.x;
          X_predicted(1) = msg->pose.pose.position.y;
          // geometry_msgs::msg::Quaternionをyawに変換
          const double yaw = tf2::getYaw(msg->pose.pose.orientation);
          X_predicted(2) = yaw;
          X_predicted(3) = msg->twist.twist.angular.z;
          X_updated = X_predicted;

          publishPose();
          geometry_msgs::msg::PoseStamped pose;
          pose.header = current_pose.header;
          pose.pose = current_pose.pose.pose;
          publishTF(pose);
        }
        predict(msg); });
    sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "ndt_pose", 1, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        { 
          RCLCPP_INFO(get_logger(), "ndt_pose subscribed");
          if(!first_odom_subscribed)
          {
            RCLCPP_WARN(get_logger(), "odom is not subscribed yet");
            return;
          }
          observe(msg); });

    if (use_imu)
    {
      sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
          "imu", 1, [this](const sensor_msgs::msg::Imu::SharedPtr msg)
          { 
          RCLCPP_INFO(get_logger(), "imu subscribed");
          if(!first_odom_subscribed)
          {
            RCLCPP_WARN(get_logger(), "odom is not subscribed yet");
            return;
          }
          // X_predictedとしてyaw, angular_velocityのみimuの値を使用
          const double yaw = tf2::getYaw(msg->orientation);
          X_predicted(2) = yaw;
          X_predicted(3) = msg->angular_velocity.z; });
    }

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("current_pose_twist", 1);
    A = Eigen::MatrixXd::Identity(4, 4);
    C = Eigen::MatrixXd::Identity(3, 4);
    Q = Eigen::MatrixXd::Identity(4, 4) * sigma_odom;
    // imuを使用する場合はyaw, angular_velocityのみimuの値を使用
    if (use_imu)
    {
      Q(2, 2) = sigma_imu;
      Q(3, 3) = sigma_imu;
    }
    R = Eigen::MatrixXd::Identity(3, 3) * sigma_ndt_pose;
    P_predicted = Eigen::MatrixXd::Zero(4, 4);
    P_updated = Eigen::MatrixXd::Zero(4, 4);
    X_predicted = Eigen::VectorXd::Zero(4);
    X_updated = Eigen::VectorXd::Zero(4);
    Y = Eigen::VectorXd::Zero(3);

    RCLCPP_INFO(get_logger(), "ouagv_ekf_node has been initialized.");
  }

  /**
   * @brief  EKFの推定ステップ
   * imu未使用時はodomの情報を元に推定（XとしてOdomを使うだけ）
   * P_predicted = Qで固定
   * imu使用時はxとyのみ更新
   * Pもちゃんと更新
   * @param msg
   */
  void EkfComponent::predict(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (use_imu)
    {
      const double V = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2));
      X_predicted(0) = V * std::cos(X_predicted(2)) * dt + X_updated(0);
      X_predicted(1) = V * std::sin(X_predicted(2)) * dt + X_updated(1);
      A(0, 2) = V * std::cos(X_predicted(2)) * dt;
      A(1, 2) = -V * std::sin(X_predicted(2)) * dt;
      P_predicted = A * P_updated * A.transpose() + Q;
    }
    else
    {
      const double yaw = tf2::getYaw(msg->pose.pose.orientation);
      X_predicted << msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          yaw,
          msg->twist.twist.angular.z;
      P_predicted = Q;
    }
  }

  /**
   * @brief EKFの更新ステップ
   * ndtの情報をodomの情報を元に更新する
   * ここでcurrent_pose_twistとtfをpublishする
   * @param msg
   */
  void EkfComponent::observe(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // geometry_msgs::msg::Quaternionをyawに変換
    const double yaw = tf2::getYaw(msg->pose.orientation);
    Y << msg->pose.position.x,
        msg->pose.position.y,
        yaw;
    G = P_predicted * C.transpose() * (C * P_predicted * C.transpose() + R).inverse();
    X_updated = X_predicted + G * (Y - C * X_predicted);
    P_updated = (Eigen::MatrixXd::Identity(4, 4) - G * C) * P_predicted;

    // current_poseに情報を入れる
    current_pose.header = msg->header;
    current_pose.pose.pose.position.x = X_updated(0);
    current_pose.pose.pose.position.y = X_updated(1);
    current_pose.pose.pose.position.z = 0.0;
    // yaw角の値からgeometry_msgs::msg::Quaternionを作成
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, X_updated(2));
    current_pose.pose.pose.orientation.x = q.x();
    current_pose.pose.pose.orientation.y = q.y();
    current_pose.pose.pose.orientation.z = q.z();
    current_pose.pose.pose.orientation.w = q.w();
    current_pose.twist.twist.angular.z = X_updated(3);
    publishPose();

    geometry_msgs::msg::PoseStamped pose;
    pose.header = current_pose.header;
    pose.pose = current_pose.pose.pose;
    publishTF(pose);
  }

  void EkfComponent::publishPose()
  {
    pub_odom->publish(current_pose);
  }

  void EkfComponent::publishTF(const geometry_msgs::msg::PoseStamped pose)
  {
    geometry_msgs::msg::TransformStamped map_to_baselink_tf;
    map_to_baselink_tf.header.frame_id = reference_frame_id;
    map_to_baselink_tf.header.stamp = pose.header.stamp;
    map_to_baselink_tf.child_frame_id = base_frame_id;
    map_to_baselink_tf.transform.translation.x = pose.pose.position.x;
    map_to_baselink_tf.transform.translation.y = pose.pose.position.y;
    map_to_baselink_tf.transform.translation.z = pose.pose.position.z;
    map_to_baselink_tf.transform.rotation.w = pose.pose.orientation.w;
    map_to_baselink_tf.transform.rotation.x = pose.pose.orientation.x;
    map_to_baselink_tf.transform.rotation.y = pose.pose.orientation.y;
    map_to_baselink_tf.transform.rotation.z = pose.pose.orientation.z;

    try
    {
      geometry_msgs::msg::TransformStamped odom_to_base_link_transform;
      odom_to_base_link_transform = tf_buffer_ptr_->lookupTransform(odom_frame_id, base_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));
      geometry_msgs::msg::TransformStamped map_to_odom_transform;
      map_to_odom_transform.header.frame_id = reference_frame_id;
      map_to_odom_transform.child_frame_id = odom_frame_id;
      map_to_odom_transform.header.stamp = pose.header.stamp;
      // mapからbase_linkとodomからbase_linkの情報を元にmapからodomの情報を計算
      map_to_odom_transform.transform = tf2::toMsg(
          tf2::Transform(tf2::Quaternion(map_to_baselink_tf.transform.rotation.x,
                                         map_to_baselink_tf.transform.rotation.y,
                                         map_to_baselink_tf.transform.rotation.z,
                                         map_to_baselink_tf.transform.rotation.w),
                         tf2::Vector3(map_to_baselink_tf.transform.translation.x,
                                      map_to_baselink_tf.transform.translation.y,
                                      map_to_baselink_tf.transform.translation.z)) *
          tf2::Transform(tf2::Quaternion(odom_to_base_link_transform.transform.rotation.x,
                                         odom_to_base_link_transform.transform.rotation.y,
                                         odom_to_base_link_transform.transform.rotation.z,
                                         odom_to_base_link_transform.transform.rotation.w),
                         tf2::Vector3(odom_to_base_link_transform.transform.translation.x,
                                      odom_to_base_link_transform.transform.translation.y,
                                      odom_to_base_link_transform.transform.translation.z))
              .inverse());
      broadcaster_->sendTransform(map_to_odom_transform);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return;
    }
  }
} // namespace ouagv_ekf
RCLCPP_COMPONENTS_REGISTER_NODE(ouagv_ekf::EkfComponent)
