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
        isFirstUpdate(true),
        use_imu_acc(false),
        use_odom_yaw(false)
  {
    Odomsubscription_ = std::shared_ptr<OdomSubscriber>(new OdomSubscriber(this, "odom", rmw_qos_profile_sensor_data));
    Imusubscription_ = std::shared_ptr<ImuSubscriber>(new ImuSubscriber(this, "imu/data", rmw_qos_profile_sensor_data));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    OdomImuSync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),
                                                                               *Odomsubscription_, *Imusubscription_);
    OdomImuSync_->registerCallback(
        std::bind(&EkfComponent::update, this, std::placeholders::_1, std::placeholders::_2));

    EstimatedPosepublisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);

    using namespace std::chrono_literals;
    // 10msごとにPoseをpublishする
    timer_ = this->create_wall_timer(10ms, std::bind(&EkfComponent::publishPose, this));

    XhatMinus = Eigen::VectorXf::Zero(5);
    Xhat = Eigen::VectorXf::Zero(5);
    Phat = Eigen::MatrixXf::Zero(5, 5);
    Pminus = Eigen::MatrixXf::Zero(5, 5);
    A = Eigen::MatrixXf::Zero(5, 5);
    B = Eigen::MatrixXf::Zero(5, 3);
    C = Eigen::MatrixXf::Zero(3, 5);
    Mt = Eigen::MatrixXf::Zero(3, 3);
    G = Eigen::MatrixXf::Zero(5, 5);
    R = Eigen::MatrixXf::Zero(3, 3);
    Y = Eigen::VectorXf::Zero(3);

    C << 1.f, 0, 0, 0, 0,
        0, 1.f, 0, 0, 0,
        0, 0, 0, 0, 1.f;
  }

  void EkfComponent::update(const nav_msgs::msg::Odometry::ConstSharedPtr in1, const sensor_msgs::msg::Imu::ConstSharedPtr in2)
  {

    if (isFirstUpdate)
    {
      update_timestamp = in1->header.stamp;
      isFirstUpdate = false;
      return;
    }
    // 状態更新
    const double dt = (rclcpp::Time(in1->header.stamp) - update_timestamp).seconds();
    const double omega = in2->angular_velocity.z;
    const double ax = in2->linear_acceleration.x;
    const double ay = in2->linear_acceleration.y;

    const double cov_acc_x = 1e-1;
    const double cov_acc_y = 1e-1;
    const double cov_acc_omega = 1e-5;
    // Mt
    // cov(ax),0,0
    // 0,cov(ay),0
    // 0,0,cov(omega)
    // Mt(0, 0) = in2->linear_acceleration_covariance.at(0) * dt * dt;
    // Mt(1, 1) = in2->linear_acceleration_covariance.at(4) * dt * dt;
    // Mt(2, 2) = in2->angular_velocity_covariance.at(8) * dt * dt;
    Mt(0, 0) = cov_acc_x;
    Mt(1, 1) = cov_acc_y;
    Mt(2, 2) = cov_acc_omega;

    // 予測ステップ
    // x-(k+1) = f(x(k))
    XhatMinus(2) = Xhat(2) + ax * dt;
    XhatMinus(3) = Xhat(3) + ay * dt;
    XhatMinus(0) = Xhat(0) + XhatMinus(2) * dt + 0.5f * ax * dt * dt;
    XhatMinus(1) = Xhat(1) + XhatMinus(3) * dt + 0.5f * ay * dt * dt;
    XhatMinus(4) = Xhat(4) + omega * dt;

    // 行列Aを更新
    A << 1, 0, dt, 0, 0,
        0, 1, 0, dt, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    B << 1.5f * dt * dt, 0, 0,
        0, 1.5f * dt * dt, 0,
        1.f * dt, 0, 0,
        0, 1.f * dt, 0,
        0, 0, 1.f * dt;

    // 事前誤差共分散行列を更新
    Pminus = A * Phat * A.transpose() + B * Mt * B.transpose();

    // 観測

    Y(0) = in1->pose.pose.position.x;
    Y(1) = in1->pose.pose.position.y;
    Y(2) = tf2::getYaw(in1->pose.pose.orientation);
    // xの分散
    R(0, 0) = in1->pose.covariance.at(0);
    // yの分散
    R(1, 1) = in1->pose.covariance.at(7);
    // yawの分散
    R(2, 2) = in1->pose.covariance.at(35);

    // カルマンゲインを計算
    G = Pminus * C.transpose() * (C * Pminus * C.transpose() + R).inverse();
    G(0, 0) = (use_imu_acc) ? G(0, 0) : 1.f;
    G(1, 1) = (use_imu_acc) ? G(1, 1) : 1.f;
    G(4, 2) = (use_odom_yaw) ? G(4, 2) : 0.f;
    // フィルタリングステップ
    Xhat = XhatMinus + G * (Y - C * XhatMinus);
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(5, 5);
    Phat = (I - G * C) * Pminus;
    publish_stamp = in1->header.stamp;
    update_timestamp = in1->header.stamp;
  }

  void EkfComponent::publishPose()
  {
    if (!isFirstUpdate)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.frame_id = "odom";
      pose.header.stamp = publish_stamp;
      pose.pose.pose.position.x = Xhat(0);
      pose.pose.pose.position.y = Xhat(1);
      tf2::Quaternion quat;
      quat.setRPY(0, 0, Xhat(4));
      pose.pose.pose.orientation.x = quat.getX();
      pose.pose.pose.orientation.y = quat.getY();
      pose.pose.pose.orientation.z = quat.getZ();
      pose.pose.pose.orientation.w = quat.getW();
      pose.pose.covariance.at(0) = Phat(0, 0);
      pose.pose.covariance.at(7) = Phat(1, 1);
      pose.pose.covariance.at(35) = Phat(4, 4);

      EstimatedPosepublisher_->publish(pose);

      geometry_msgs::msg::TransformStamped pose_transform_stamped;
      pose_transform_stamped.header.stamp = publish_stamp;
      pose_transform_stamped.header.frame_id = "odom";
      pose_transform_stamped.child_frame_id = "base_link";
      pose_transform_stamped.transform.translation.x = Xhat(0);
      pose_transform_stamped.transform.translation.y = Xhat(1);
      pose_transform_stamped.transform.translation.z = 0;
      pose_transform_stamped.transform.rotation.x = quat.getX();
      pose_transform_stamped.transform.rotation.y = quat.getY();
      pose_transform_stamped.transform.rotation.z = quat.getZ();
      pose_transform_stamped.transform.rotation.w = quat.getW();
      tf_broadcaster_->sendTransform(pose_transform_stamped);
    }
  }
} // namespace ouagv_ekf

RCLCPP_COMPONENTS_REGISTER_NODE(ouagv_ekf::EkfComponent)
