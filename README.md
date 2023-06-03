# ouagv_ekf
拡張カルマンフィルタを行うノード

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
