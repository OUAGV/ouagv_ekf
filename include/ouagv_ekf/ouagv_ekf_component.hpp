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
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/LU>

// Headers needed in pub/sub, exposed types
#include <memory> // shared_ptr in pub_

namespace ouagv_ekf
{
  class EkfComponent : public rclcpp::Node
  {
  public:
    OUAGV_EKF_PUBLIC
    explicit EkfComponent(const rclcpp::NodeOptions &options);
    void publishTF(const geometry_msgs::msg::PoseStamped pose);
    void publish(rclcpp::Time time, Eigen::VectorXd X);
    void predict(const nav_msgs::msg::Odometry::SharedPtr msg);
    void observe(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    std::mutex mutex;

    bool first_odom_subscribed;
    nav_msgs::msg::Odometry current_pose;
    rclcpp::Time last_odom_time;

    // parameters
    std::string reference_frame_id;
    std::string base_frame_id;
    std::string odom_frame_id;
    // odomの共分散行列の対角成分
    double sigma_odom;
    // imuの共分散行列の対角のうち、yaw角とyaw_rateの共分散
    double sigma_imu;
    // ndt_poseの共分散行列の対角成分
    double sigma_ndt_pose;
    bool use_imu;
    // 10Hzでimu, odomをsubscribe
    double dt;

    // EKFに使う行列
    // 状態推定ベクトル（x,y,yaw,yaw_rate）
    Eigen::VectorXd X;

    // 観測状態ベクトル (x,y,yaw)
    Eigen::VectorXd Y;
    // 状態方程式ヤコビ行列 4x4
    Eigen::MatrixXd A;
    // 観測方程式ヤコビ行列 3x4
    Eigen::MatrixXd C;
    // 誤差共分散行列 4x4
    Eigen::MatrixXd P;
    // 推定誤差行列 4x4 対角成分がsigma_odomで他は0
    Eigen::MatrixXd Q;
    // 観測誤差行列 3x3 対角成分がsigma_ndt_poseで他は0
    Eigen::MatrixXd R;
    // カルマンゲイン 4x3
    Eigen::MatrixXd G;
  };
} // namespace ouagv_ekf
