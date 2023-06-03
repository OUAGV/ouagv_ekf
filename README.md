# ouagv_ekf
拡張カルマンフィルタを行うノード

# environment
- ROS2 Humble
- Ubuntu 22.04
# overview
## parameter
| パラメータ名 | 説明 | 初期値 |
| --- | --- | --- |
| reference_frame_id | マップの参照フレームのID | "map" |
| base_frame_id | ベースリンクのフレームのID | "base_link" |
| odom_frame_id | オドメトリのフレームのID | "odom" |
| sigma_odom | オドメトリのノイズの標準偏差 | 0.1 |
| sigma_ndt_pose | NDTポーズのノイズの標準偏差 | 0.00001 |
| sigma_imu | IMUのノイズの標準偏差 | 0.000001 |
| use_imu | IMUを使用するかどうかのフラグ | false |

### parameter tuning
sigma_odomを小さくしてsigma_ndt_poseを大きくするとodomを信用するようになる、逆も然り

launchファイルではodomを強めに信頼している

## subscribe
- /odom (nav_msgs::msg::Odometry) 
  - デッドレコニングで得られた機体の姿勢と速度
- /imu (sensor_msgs::msg::Imu)  
  - IMUから得られる機体の角度、角速度
  - use_imuがFalseならsubscribeしない   
- /ndt_pose (geometry_msgs::msg::PoseStamped)
  - NDTマッチングで得られた機体の姿勢

## publish
- /current_pose_twist (nav_msgs::msg::Odometry)
  - EKFで推定した機体の姿勢と速度
- /tf 
  - odom->base_link

# usage
```
ros2 launch ouagv_ekf ouagv_ekf.launch.xml
```



# calculation
状態ベクトルは以下で表される。ただし、
$x$は位置のx座標, $y$は位置のy座標, $\theta$は機体のヨー角の角度, $\omega$は機体のヨー角の角速度

$$
x=
\begin{pmatrix}
x \\
y \\ 
\theta \\
\omega \\
\end{pmatrix}
$$

入力 $u$ は、

$$
u=
\begin{pmatrix}
v \\
\theta \\
\omega
\end{pmatrix}
$$

となる。ここで、
$v$ は```/odom```から得られる機体の並進速度、 $\theta$ と $\omega$は```/imu```から得られる

観測ベクトルは

$$
y=
\begin{pmatrix}
x \\
y \\ 
\theta \\
\end{pmatrix}
$$

であり、NDTから取得できる。

具体的な計算は
https://qiita.com/Crafty_as_a_Fox/items/55448e2ed9ce0f340814
を参照。
