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
  EkfComponent::EkfComponent(const rclcpp::NodeOptions &options) : Node("ouagv_ekf_node", options)
  {
    Odomsubscription_ = std::shared_ptr<OdomSubscriber>(new OdomSubscriber(this, "odom", rmw_qos_profile_sensor_data));
    Imusubscription_ = std::shared_ptr<ImuSubscriber>(new ImuSubscriber(this, "imu/data", rmw_qos_profile_sensor_data));
    OdomImuSync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),
                                                                               *Odomsubscription_, *Imusubscription_);
    OdomImuSync_->registerCallback(
        std::bind(&EkfComponent::prediction, this, std::placeholders::_1, std::placeholders::_2));

    ScanMatchedPosesubscription_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/icp_matching/pose", 10,
            std::bind(&EkfComponent::Observation, this, std::placeholders::_1));
    EstimatedPosepublisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);

    using namespace std::chrono_literals;
    // 10msごとにPoseをpublishする
    timer_ = this->create_wall_timer(10ms, std::bind(&EkfComponent::publishPose, this));

    XhatMinus = Eigen::VectorXd::Zero(3);
    Xhat = Eigen::VectorXd::Zero(3);
    Phat = Eigen::MatrixXd::Zero(3, 3);
    Pminus = Eigen::MatrixXd::Zero(3, 3);
    A = Eigen::MatrixXd::Zero(3, 3);
    B = Eigen::MatrixXd::Zero(2, 3);
    C = Eigen::MatrixXd::Zero(3, 3);
    Mt = Eigen::MatrixXd::Zero(2, 2);
    G = Eigen::MatrixXd::Zero(3, 3);
    R = Eigen::MatrixXd::Zero(3, 3);
    C << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
  }

  void EkfComponent::prediction(const nav_msgs::msg::Odometry::ConstSharedPtr in1, const sensor_msgs::msg::Imu::ConstSharedPtr in2)
  {
    if (isFirstPrediction || isFirstObservation)
    {
      prediction_timestamp = in1->header.stamp;
      isFirstPrediction = false;
      return;
    }
    const double dt = (rclcpp::Time(in1->header.stamp) - prediction_timestamp).seconds();
    const double v = in1->twist.twist.linear.x;
    const double omega = in2->angular_velocity.z;
    const double pre_theta = XhatMinus(2);

    // Mt(0,0)にはOdomのVxの分散を入れる
    Mt(0, 0) = in1->twist.covariance.at(21);
    Mt(0, 1) = 0;
    Mt(1, 0) = 0;
    // Mt(1,1)にはImuのomega_xの分散を入れる
    Mt(1, 1) = in2->angular_velocity_covariance.at(0);

    // 予測ステップ
    // x-(k+1) = f(x-(k))
    XhatMinus(0) = XhatMinus(0) + v / omega * (sin(pre_theta + omega * dt) - sin(pre_theta));
    XhatMinus(1) = XhatMinus(1) + v / omega * (-cos(pre_theta + omega * dt) + cos(pre_theta));
    XhatMinus(2) = pre_theta + dt * omega;

    // 行列Bを更新
    B(0, 0) = (sin(pre_theta + omega * dt) - sin(pre_theta)) / omega;
    B(0, 1) = -v / pow(omega, 2) * (sin(pre_theta + omega * dt) - sin(pre_theta)) + v / omega * dt * cos(pre_theta + omega * dt);
    B(1, 0) = (-cos(pre_theta + omega * dt) + cos(pre_theta)) / omega;
    B(1, 1) = -v / pow(omega, 2) * (-cos(pre_theta + omega * dt) + cos(pre_theta)) + v / omega * dt * sin(pre_theta + omega * dt);
    B(2, 0) = 0;
    B(2, 1) = dt;

    // 行列Aを更新
    A(0, 0) = 1;
    A(0, 1) = 0;
    A(0, 2) = v / omega * (cos(Xhat(2) + omega * dt) - cos(Xhat(2)));
    A(1, 0) = 0;
    A(1, 1) = 1;
    A(1, 2) = v / omega * (sin(Xhat(2) + omega * dt) - sin(Xhat(2)));
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    // 事前誤差共分散行列を更新
    Pminus = A * Phat * A.transpose() + B * Mt * B.transpose();

    // std::cout << "A:" << A << std::endl;

    prediction_timestamp = in1->header.stamp;
    publish_stamp = in1->header.stamp;
  }

  void EkfComponent::Observation(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    Eigen::VectorXd Y(3);
    Y(0) = msg->pose.pose.position.x;
    Y(1) = msg->pose.pose.position.y;
    Y(2) = tf2::getYaw(msg->pose.pose.orientation);
    if (isFirstObservation || isFirstPrediction)
    {
      Xhat = Y;
      isFirstObservation = false;
      return;
    }

    // xの分散
    R(0, 0) = msg->pose.covariance.at(0);
    // yの分散
    R(1, 1) = msg->pose.covariance.at(7);
    // yawの分散
    R(2, 2) = msg->pose.covariance.at(35);

    // カルマンゲインを計算
    G = Pminus * C * (C.transpose() * Pminus * C + R).transpose();

    // フィルタリングステップ
    Xhat = XhatMinus + G * (Y - C * XhatMinus);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    Phat = (I - G * C.transpose()) * Pminus;
    publish_stamp = msg->header.stamp;
  }

  void EkfComponent::publishPose()
  {
    if (!isFirstObservation && !isFirstPrediction)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.frame_id = "odom";
      pose.header.stamp = publish_stamp;
      pose.pose.pose.position.x = Xhat(0);
      pose.pose.pose.position.y = Xhat(1);
      tf2::Quaternion quat;
      quat.setRPY(0, 0, Xhat(2));
      pose.pose.pose.orientation.x = quat.getX();
      pose.pose.pose.orientation.x = quat.getY();
      pose.pose.pose.orientation.x = quat.getZ();
      pose.pose.pose.orientation.x = quat.getW();
      pose.pose.covariance.at(0) = Phat(0);
      pose.pose.covariance.at(7) = Phat(4);
      pose.pose.covariance.at(35) = Phat(8);

      EstimatedPosepublisher_->publish(pose);
    }
  }
} // namespace ouagv_ekf

RCLCPP_COMPONENTS_REGISTER_NODE(ouagv_ekf::EkfComponent)
