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

#pragma once

#include "ouagv_ekf/visibility_control.h"

// Headers in ROS2
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/buffer_core.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Headers needed in pub/sub, exposed types
#include <memory> // shared_ptr in pub_
//#include <perception_msgs/msg/tracking2_d.hpp> // Tracking2D in pub_

namespace ouagv_ekf
{
  typedef message_filters::Subscriber<nav_msgs::msg::Odometry> OdomSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::msg::Imu> ImuSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu> SyncPolicy;

  class EkfComponent : public rclcpp::Node
  {
  public:
    OUAGV_EKF_PUBLIC
    explicit EkfComponent(const rclcpp::NodeOptions &options);

  private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        ScanMatchedPosesubscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        EstimatedPosepublisher_;

    std::shared_ptr<OdomSubscriber> Odomsubscription_;
    std::shared_ptr<ImuSubscriber> Imusubscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> OdomImuSync_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // (x,y,theta)^T 状態ベクトルの予測
    Eigen::VectorXd XhatMinus;
    // (x,y,theta)^T 状態ベクトルの事後推定値
    Eigen::VectorXd Xhat;
    // 誤差共分散行列の予測
    Eigen::MatrixXd Pminus;
    // 誤差共分散行列の事後推定値
    Eigen::MatrixXd Phat;
    // 状態遷移モデルの関数fを状態ベクトルxで偏微分したヤコビ行列
    Eigen::MatrixXd A;
    // 状態遷移モデルの関数fを入力ベクトルu(v,omega)^Tで偏微分したヤコビ行列
    Eigen::MatrixXd B;
    // 観測モデルの観測行列
    Eigen::MatrixXd C;
    // 入力uの共分散行列
    Eigen::MatrixXd Mt;
    // 観測モデルの共分散行列
    Eigen::MatrixXd R;
    // カルマンゲイン
    Eigen::MatrixXd G;

    rclcpp::Time prediction_timestamp;
    rclcpp::Time publish_stamp;

    bool isFirstPrediction;
    bool isFirstObservation;

    const bool no_observation;

    void prediction(
        const nav_msgs::msg::Odometry::ConstSharedPtr in1, const sensor_msgs::msg::Imu::ConstSharedPtr in2);
    void Observation(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void publishPose();
  };
} // namespace ouagv_ekf
