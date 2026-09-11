# KF 与 EKF 传感器融合示例

这份笔记用两个小例子说明：融合底盘里程计、GPS、IMU 时，在程序里应该定义哪些模型、矩阵和函数。

## 0. KF 和 EKF 的总区别

先记住一句话：

- `KF` 适合线性系统，模型可以直接写成固定矩阵。
- `EKF` 适合非线性系统，模型先用函数计算，再在当前状态附近线性化。

设状态为 `x`，控制输入或 IMU 输入为 `u`，观测为 `z`。

### 0.0 符号说明

这一章里常见符号含义如下：

- `x_k`：第 `k` 时刻状态
- `x_k^-`：第 `k` 时刻先验状态，也就是预测值
- `x_k^+`：第 `k` 时刻后验状态，也就是更新后的值
- `P_k`：状态协方差矩阵
- `F_k`：状态转移矩阵，或者非线性状态函数的雅可比
- `H_k`：观测矩阵，或者非线性观测函数的雅可比
- `B_k`：控制矩阵
- `u_k`：控制输入或外部输入
- `z_k`：传感器实际观测值
- `z_k^-`：根据当前状态预测出来的理论观测值
- `y_k`：残差，也叫创新
- `K_k`：卡尔曼增益
- `S_k`：残差协方差
- `Q_k`：过程噪声协方差
- `R_k`：观测噪声协方差
- `w_k`：过程噪声
- `v_k`：观测噪声
- `I`：单位矩阵

### 0.1 KF 的完整公式

KF 的系统模型和观测模型都要求是线性的：

```text
x_k = F_k x_{k-1} + B_k u_k + w_k
z_k = H_k x_k + v_k
```

其中：

- `F_k`：状态转移矩阵
- `B_k`：控制矩阵
- `H_k`：观测矩阵
- `w_k ~ N(0, Q_k)`：过程噪声
- `v_k ~ N(0, R_k)`：观测噪声

KF 预测步骤：

```text
x_k^- = F_k x_{k-1}^+ + B_k u_k
P_k^- = F_k P_{k-1}^+ F_k^T + Q_k
```

KF 更新步骤：

```text
y_k = z_k - H_k x_k^-
S_k = H_k P_k^- H_k^T + R_k
K_k = P_k^- H_k^T S_k^-1
x_k^+ = x_k^- + K_k y_k
P_k^+ = (I - K_k H_k) P_k^-
```

KF 里，`F_k` 和 `H_k` 都是固定矩阵，不需要对非线性函数求导。

这些式子里的变量对应关系是：

- `x_k`：第 `k` 时刻的状态
- `x_{k-1}`：第 `k-1` 时刻的状态
- `F_k`：状态转移矩阵
- `B_k`：控制矩阵
- `u_k`：控制输入
- `w_k`：过程噪声
- `z_k`：第 `k` 时刻实际观测
- `H_k`：观测矩阵
- `v_k`：观测噪声
- `x_k^-`：预测得到的先验状态
- `x_k^+`：更新后的后验状态
- `P_k^-`：先验协方差
- `P_k^+`：后验协方差
- `Q_k`：过程噪声协方差
- `R_k`：观测噪声协方差
- `K_k`：卡尔曼增益
- `y_k`：观测残差
- `S_k`：残差协方差

### 0.2 EKF 的完整公式

EKF 允许系统模型或观测模型是非线性的：

```text
x_k = f(x_{k-1}^+, u_k) + w_k
z_k = h(x_k^-) + v_k
```

这里要注意：

- `f( )` 是非线性状态传播函数
- `h( )` 是非线性观测函数
- `w_k ~ N(0, Q_k)`
- `v_k ~ N(0, R_k)`

EKF 预测步骤分两部分：

1. **均值预测，直接用非线性状态传播函数**

```text
x_k^- = f(x_{k-1}^+, u_k)
```

2. **协方差预测，在上一时刻后验状态 `x_{k-1}^+` 处线性化**

```text
F_k = ∂f / ∂x |_(x_{k-1}^+, u_k)
P_k^- = F_k P_{k-1}^+ F_k^T + Q_k
```

EKF 更新步骤也分两部分：

1. **先用非线性观测函数算理论观测值**

```text
z_k^- = h(x_k^-)
y_k = z_k - z_k^-
```

2. **在先验状态 `x_k^-` 处线性化观测函数，得到观测雅可比**

```text
H_k = ∂h / ∂x |_(x_k^-)
S_k = H_k P_k^- H_k^T + R_k
K_k = P_k^- H_k^T S_k^-1
x_k^+ = x_k^- + K_k y_k
P_k^+ = (I - K_k H_k) P_k^-
```

### 0.3 EKF 里哪里直接用非线性模型

EKF 里直接用非线性模型的地方有两个：

- 预测均值：`x_k^- = f(x_{k-1}^+, u_k)`
- 观测预测：`z_k^- = h(x_k^-)`

EKF 里必须用雅可比矩阵的地方有两个：

- 预测协方差：`F_k = ∂f/∂x`
- 更新协方差和增益：`H_k = ∂h/∂x`

这些式子里的变量对应关系是：

- `f( )`：非线性状态传播函数
- `h( )`：非线性观测函数
- `x_{k-1}^+`：上一时刻后验状态，作为预测输入
- `x_k^-`：当前时刻先验状态，作为更新输入
- `u_k`：外部输入，例如 IMU 测量
- `w_k`：过程噪声
- `v_k`：观测噪声
- `F_k`：在 `x_{k-1}^+` 处对 `f( )` 求导得到的雅可比
- `H_k`：在 `x_k^-` 处对 `h( )` 求导得到的雅可比
- `z_k^-`：用 `h(x_k^-)` 算出来的理论观测
- `y_k`：`z_k - z_k^-`
- `K_k`：卡尔曼增益
- `P_k^-`：先验协方差
- `P_k^+`：后验协方差
- `Q_k`：过程噪声协方差
- `R_k`：观测噪声协方差

### 0.4 雅可比矩阵在什么状态下计算

雅可比不是随便算的，通常是在“当前估计点”附近算：

- 预测雅可比 `F_k`：在上一时刻后验状态 `x_{k-1}^+` 处，围绕预测模型 `f()` 求导
- 观测雅可比 `H_k`：在当前先验状态 `x_k^-` 处，围绕观测模型 `h()` 求导

也就是：

```text
F_k = ∂f/∂x |_(x_{k-1}^+)
H_k = ∂h/∂x |_(x_k^-)
```

这一步的意义是：

- `f( )` 和 `h( )` 负责“真实地按非线性模型算值”
- `F_k` 和 `H_k` 负责“在当前点附近把非线性模型近似成线性”

### 0.5 误差状态卡尔曼滤波和 IESKF 的整体流程

LIGO 里更接近的是误差状态滤波，也就是 ESKF / IESKF，不是最朴素的 EKF。

这里先用一个通用写法说明：

- `x̂_k`：名义状态，也叫主状态
- `δx_k`：误差状态，也叫小偏差

真实状态和名义状态的关系可以写成：

```text
x_k = x̂_k ⊞ δx_k
```

如果是普通欧式变量，可以近似成：

```text
x_k ≈ x̂_k + δx_k
```

其中：

- `x̂_k` 负责保存滤波器当前相信的“主答案”
- `δx_k` 只描述“真实值和主答案之间差了多少”
- 协方差 `P_k` 是围绕 `δx_k` 建的，不是围绕 `x̂_k` 本身建的

#### 0.5.1 预测步骤

ESKF / IESKF 的预测也分两层：

1. **名义状态传播**

```text
x̂_k^- = f(x̂_{k-1}^+, u_k)
```

这一步直接用非线性模型传播名义状态。

2. **误差状态协方差传播**

```text
δx_k ≈ F_k δx_{k-1} + G_k w_k
P_k^- = F_k P_{k-1}^+ F_k^T + G_k Q_k G_k^T
```

这里：

- `F_k` 是误差状态传播雅可比
- `G_k` 是噪声注入矩阵
- `P_k` 是误差状态协方差

也就是说，**预测时不是拿误差状态去“生成真实状态”，而是先更新名义状态，再用线性化的误差模型更新协方差。**

#### 0.5.2 更新步骤

更新时先做观测预测：

```text
z_k^- = h(x̂_k^-)
y_k = z_k - z_k^-
```

然后在当前名义状态 `x̂_k^-` 处线性化观测模型：

```text
H_k = ∂h / ∂x |_(x̂_k^-)
S_k = H_k P_k^- H_k^T + R_k
K_k = P_k^- H_k^T S_k^-1
```

得到的是**误差状态修正量**：

```text
δx̂_k = K_k y_k
```

随后把这个修正量注入名义状态：

```text
x̂_k^+ = x̂_k^- ⊞ δx̂_k
```

如果是普通欧式状态，也可以看成：

```text
x̂_k^+ = x̂_k^- + δx̂_k
```

最后通常还要做一步“误差清零”：

```text
δx_k ← 0
```

因为误差已经被吸收到名义状态里了。

#### 0.5.3 IESKF 为什么要迭代

IESKF 的“迭代”发生在更新步骤中。

因为观测模型可能很非线性，一次线性化不够准，所以会反复做：

1. 用当前名义状态算 `z_k^-`
2. 算残差 `y_k`
3. 重新线性化得到 `H_k`
4. 算 `K_k`
5. 注入修正量更新名义状态
6. 直到收敛或达到迭代次数上限

所以可以把 IESKF 理解成：

- 预测阶段：普通 ESKF 风格
- 更新阶段：在同一帧观测上多次重线性化、多次修正

#### 0.5.4 面试时可以怎么说

你可以这样总结：

**误差状态滤波不是直接拿误差状态当主状态，而是用名义状态保存当前估计，用误差状态表示小偏差，协方差围绕误差状态传播。预测时名义状态按非线性模型走，更新时先用观测模型算残差，再把 Kalman 增益求出的误差修正量注入名义状态。IESKF 则是在更新步骤里反复重线性化、反复注入，直到收敛。**

第一个 KF 例子的传感器假设：

- 底盘里程计提供速度：`z_odom = [vx, vy, vz]^T`
- GPS 提供位置：`z_gps = [px, py, pz]^T`
- 状态为 6 维：`x = [px, py, pz, vx, vy, vz]^T`

为了让例子保持简单，默认底盘里程计速度已经被转换到世界坐标系。如果底盘里程计给的是车体坐标系速度，实际工程里通常还需要姿态，或者至少需要 yaw 角，这时模型会变成更典型的 EKF。

## 1. KF 示例：线性位置速度模型

KF 适用于系统模型和观测模型都是线性的情况。

这里使用最常见的匀速模型：

```text
p(k+1) = p(k) + v(k) * dt
v(k+1) = v(k)
```

写成矩阵形式：

```text
x(k+1) = F * x(k) + w
```

其中：

```text
x = [px, py, pz, vx, vy, vz]^T
```

状态转移矩阵：

```text
F =
[1 0 0 dt 0  0 ]
[0 1 0 0  dt 0 ]
[0 0 1 0  0  dt]
[0 0 0 1  0  0 ]
[0 0 0 0  1  0 ]
[0 0 0 0  0  1 ]
```

这个矩阵的含义是：

- 位置由上一时刻位置和速度预测得到。
- 速度假设保持不变。
- `dt` 是两次滤波预测之间的时间间隔。

### 1.1 过程噪声 Q

过程噪声描述的是“预测模型本身有多不可靠”。

即使模型写的是匀速运动，真实小车也可能加速、减速、打滑、震动，所以不能完全相信：

```text
p(k+1) = p(k) + v(k) * dt
v(k+1) = v(k)
```

可以简单定义：

```text
Q =
[q_p 0   0   0   0   0  ]
[0   q_p 0   0   0   0  ]
[0   0   q_p 0   0   0  ]
[0   0   0   q_v 0   0  ]
[0   0   0   0   q_v 0  ]
[0   0   0   0   0   q_v]
```

其中：

- `q_p`：位置预测噪声
- `q_v`：速度预测噪声

如果你认为里程计速度比较可靠，可以让 `q_v` 小一些。如果小车运动变化大，匀速模型不可靠，就让 `q_v` 大一些。

### 1.2 底盘里程计速度观测模型

底盘里程计观测的是速度：

```text
z_odom = [vx, vy, vz]^T
```

观测模型：

```text
z_odom = H_odom * x + n_odom
```

观测矩阵：

```text
H_odom =
[0 0 0 1 0 0]
[0 0 0 0 1 0]
[0 0 0 0 0 1]
```

含义是：从状态 `x` 里面取出速度部分。

观测噪声：

```text
R_odom =
[r_vx 0    0   ]
[0    r_vy 0   ]
[0    0    r_vz]
```

`R_odom` 越小，表示越相信底盘里程计速度。

### 1.3 GPS 位置观测模型

GPS 观测的是位置：

```text
z_gps = [px, py, pz]^T
```

观测模型：

```text
z_gps = H_gps * x + n_gps
```

观测矩阵：

```text
H_gps =
[1 0 0 0 0 0]
[0 1 0 0 0 0]
[0 0 1 0 0 0]
```

含义是：从状态 `x` 里面取出位置部分。

观测噪声：

```text
R_gps =
[r_px 0    0   ]
[0    r_py 0   ]
[0    0    r_pz]
```

`R_gps` 越小，表示越相信 GPS 位置。

### 1.4 KF 程序中通常要定义什么

如果用 Eigen 写，核心对象大概是：

```cpp
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat36 = Eigen::Matrix<double, 3, 6>;

Vec6 x;       // 状态: [px, py, pz, vx, vy, vz]
Mat6 P;       // 状态协方差
Mat6 F;       // 状态转移矩阵
Mat6 Q;       // 过程噪声协方差
Mat36 H_gps;  // GPS 观测矩阵
Mat36 H_odom; // 底盘里程计速度观测矩阵
Mat3 R_gps;   // GPS 观测噪声
Mat3 R_odom;  // 里程计观测噪声
```

预测步骤：

```cpp
void predictKF(double dt)
{
    F.setIdentity();
    F.block<3, 3>(0, 3) = dt * Mat3::Identity();

    x = F * x;
    P = F * P * F.transpose() + Q;
}
```

通用更新步骤：

```cpp
void updateKF(const Vec3& z, const Mat36& H, const Mat3& R)
{
    Vec3 y = z - H * x;
    Mat3 S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (Mat6::Identity() - K * H) * P;
}
```

收到 GPS 时：

```cpp
updateKF(gps_position, H_gps, R_gps);
```

收到底盘里程计速度时：

```cpp
updateKF(odom_velocity, H_odom, R_odom);
```

KF 的特点是：预测和观测都可以直接写成矩阵乘法，所以程序里主要定义 `F`、`Q`、`H`、`R`。

## 2. EKF 示例：融合 IMU、底盘里程计、GPS

EKF 适用于系统模型或者观测模型是非线性的情况。

这里把状态扩展到 15 维，比前面的 KF 更接近真实机器人定位：

```text
x = [p, v, theta, bg, ba]^T

p     = [px, py, pz]^T          位置，世界坐标系
v     = [vx, vy, vz]^T          速度，世界坐标系
theta = [roll, pitch, yaw]^T    姿态，这里为了教学先用欧拉角近似
bg    = [bgx, bgy, bgz]^T       陀螺仪零偏
ba    = [bax, bay, baz]^T       加速度计零偏
```

传感器作用：

- IMU 提供角速度 `omega_m` 和加速度 `acc_m`，主要用于预测。
- GPS 提供位置 `z_gps = [px, py, pz]^T`，用于更新位置。
- 底盘里程计提供车体系速度 `z_odom = [vx_body, vy_body, vz_body]^T`，用于更新速度。

注意：实际系统中姿态最好用四元数或 SO(3) 流形表示。这里为了让矩阵更直观，先把姿态写成 3 维小角度 `theta`。LIGO 里使用的是旋转流形，所以比这个例子更严谨。

### 2.1 IMU 预测模型 f(x, u)

IMU 测量值：

```text
u = [acc_m, omega_m]
```

去掉零偏后：

```text
acc_unbias   = acc_m   - ba
omega_unbias = omega_m - bg
```

把加速度从 IMU 坐标系转到世界坐标系：

```text
acc_world = R(theta) * acc_unbias + g
```

其中：

- `R(theta)` 是由姿态得到的旋转矩阵。
- `g = [0, 0, -9.81]^T` 是重力。

预测模型：

```text
p(k+1)     = p(k) + v(k) * dt + 0.5 * acc_world * dt^2
v(k+1)     = v(k) + acc_world * dt
theta(k+1) = theta(k) + omega_unbias * dt
bg(k+1)    = bg(k)
ba(k+1)    = ba(k)
```

这个模型是非线性的，原因主要有两个：

- `R(theta) * acc_unbias` 里面有三角函数。
- 底盘里程计如果给的是车体系速度，也要用 `R(theta)^T * v` 做坐标变换。

所以 EKF 程序里一般要写：

```text
x_pred = f(x, imu, dt)
F      = df / dx
```

`f(x, imu, dt)` 用来传播状态均值，`F` 用来传播协方差。

### 2.2 EKF 预测函数的程序写法

程序里可以先定义类型：

```cpp
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Mat315 = Eigen::Matrix<double, 3, 15>;

struct ImuData
{
    Vec3 acc;   // 加速度计测量值
    Vec3 gyro;  // 陀螺仪测量值
};
```

状态下标可以约定为：

```cpp
constexpr int IDX_P = 0;      // p:     0, 1, 2
constexpr int IDX_V = 3;      // v:     3, 4, 5
constexpr int IDX_THETA = 6;  // theta: 6, 7, 8
constexpr int IDX_BG = 9;     // bg:    9, 10, 11
constexpr int IDX_BA = 12;    // ba:    12, 13, 14
```

预测函数：

```cpp
Vec15 predictStateEKF(const Vec15& x, const ImuData& imu, double dt)
{
    Vec3 p = x.segment<3>(0);
    Vec3 v = x.segment<3>(3);
    Vec3 theta = x.segment<3>(6);
    Vec3 bg = x.segment<3>(9);
    Vec3 ba = x.segment<3>(12);

    Mat3 R = eulerToRotation(theta);
    Vec3 g(0.0, 0.0, -9.81);

    Vec3 acc_unbias = imu.acc - ba;
    Vec3 gyro_unbias = imu.gyro - bg;
    Vec3 acc_world = R * acc_unbias + g;

    Vec15 x_pred = x;
    x_pred.segment<3>(0) = p + v * dt;
    x_pred.segment<3>(0) += 0.5 * acc_world * dt * dt;
    x_pred.segment<3>(3) = v + acc_world * dt;
    x_pred.segment<3>(6) = theta + gyro_unbias * dt;
    x_pred.segment<3>(9) = bg;
    x_pred.segment<3>(12) = ba;

    return x_pred;
}
```

这里的 `eulerToRotation(theta)` 是把欧拉角转成旋转矩阵的函数。真实工程里更推荐用四元数或者 SO(3) 的 `Exp()` 更新姿态。

### 2.3 EKF 预测雅可比 F

EKF 的协方差传播仍然需要矩阵：

```text
P(k+1) = F * P(k) * F^T + Q
```

但这里的 `F` 不再是固定状态转移矩阵，而是：

```text
F = df / dx
```

为了教学，可以先记住它的主要块结构：

```text
F =
[I   I*dt  F_p_theta   0       F_p_ba]
[0   I     F_v_theta   0       F_v_ba]
[0   0     I           F_th_bg 0     ]
[0   0     0           I       0     ]
[0   0     0           0       I     ]
```

各块含义：

- `F_p_v = I * dt`：速度不确定性会传播到位置。
- `F_p_theta`：姿态误差会影响加速度转到世界系的结果，从而影响位置。
- `F_v_theta`：姿态误差会影响速度预测。
- `F_v_ba`：加速度计零偏误差会影响速度预测。
- `F_th_bg`：陀螺零偏误差会影响姿态预测。

常用近似写法：

```text
F_p_v     = I * dt
F_p_theta = -0.5 * R * skew(acc_unbias) * dt^2
F_p_ba    = -0.5 * R * dt^2
F_v_theta = -R * skew(acc_unbias) * dt
F_v_ba    = -R * dt
F_th_bg   = -I * dt
```

`skew(a)` 是反对称矩阵：

```text
skew(a) =
[ 0   -az   ay]
[ az   0   -ax]
[-ay   ax   0 ]
```

程序写法：

```cpp
Mat3 skew(const Vec3& a)
{
    Mat3 A;
    A << 0.0, -a.z(), a.y(),
         a.z(), 0.0, -a.x(),
        -a.y(), a.x(), 0.0;
    return A;
}

Mat15 jacobianFEKF(const Vec15& x, const ImuData& imu, double dt)
{
    Mat15 F = Mat15::Identity();

    Vec3 theta = x.segment<3>(IDX_THETA);
    Vec3 ba = x.segment<3>(IDX_BA);
    Mat3 R = eulerToRotation(theta);
    Vec3 acc_unbias = imu.acc - ba;

    F.block<3, 3>(0, 3) = dt * Mat3::Identity();
    F.block<3, 3>(0, 6) = -0.5 * R * skew(acc_unbias) * dt * dt;
    F.block<3, 3>(0, 12) = -0.5 * R * dt * dt;

    F.block<3, 3>(3, 6) = -R * skew(acc_unbias) * dt;
    F.block<3, 3>(3, 12) = -R * dt;

    F.block<3, 3>(6, 9) = -Mat3::Identity() * dt;

    return F;
}
```

这个雅可比的目的不是更新状态本身，而是更新不确定性：

```text
P = F * P * F^T + Q
```

### 2.4 EKF 过程噪声 Q

EKF 里也需要过程噪声：

```text
Q =
diag(
  q_p     * I3,
  q_v     * I3,
  q_theta * I3,
  q_bg    * I3,
  q_ba    * I3
)
```

含义：

- `q_p`：位置传播额外不确定性。
- `q_v`：速度传播额外不确定性，通常和加速度噪声有关。
- `q_theta`：姿态传播额外不确定性，通常和陀螺噪声有关。
- `q_bg`：陀螺零偏随机游走噪声。
- `q_ba`：加速度计零偏随机游走噪声。

简单程序写法：

```cpp
Mat15 buildProcessNoise(double q_p,
                        double q_v,
                        double q_theta,
                        double q_bg,
                        double q_ba)
{
    Mat15 Q = Mat15::Zero();
    Q.block<3, 3>(0, 0) = q_p * Mat3::Identity();
    Q.block<3, 3>(3, 3) = q_v * Mat3::Identity();
    Q.block<3, 3>(6, 6) = q_theta * Mat3::Identity();
    Q.block<3, 3>(9, 9) = q_bg * Mat3::Identity();
    Q.block<3, 3>(12, 12) = q_ba * Mat3::Identity();
    return Q;
}
```

在更严谨的 IMU 预积分或者 ESKF 里，经常不是直接写一个 15x15 的 `Q`，而是先定义 IMU 噪声矩阵 `Qn`，再通过噪声传递矩阵 `G` 得到：

```text
Q = G * Qn * G^T
```

但刚开始学习时，可以先用上面的对角块理解每一类状态的不确定性。

### 2.5 GPS 观测模型

GPS 观测位置：

```text
z_gps = [px, py, pz]^T
```

观测函数：

```text
h_gps(x) = p
```

雅可比：

```text
H_gps =
[I3  0  0  0  0]
```

程序写法：

```cpp
Vec3 hGps(const Vec15& x)
{
    return x.segment<3>(IDX_P);
}

Mat315 jacobianHGps(const Vec15&)
{
    Mat315 H = Mat315::Zero();
    H.block<3, 3>(0, IDX_P) = Mat3::Identity();
    return H;
}
```

### 2.6 底盘里程计观测模型

底盘里程计通常给的是车体坐标系速度。假设它提供：

```text
z_odom = [vx_body, vy_body, vz_body]^T
```

状态里的速度 `v` 是世界坐标系速度，所以要转换到车体系：

```text
h_odom(x) = R(theta)^T * v
```

这个观测模型也是非线性的，因为它依赖姿态 `theta`。

雅可比主要块：

```text
H_odom =
[0  R^T  H_theta  0  0]
```

其中：

- 对位置 `p` 的导数是 0。
- 对速度 `v` 的导数是 `R^T`。
- 对姿态 `theta` 的导数和扰动定义有关，常用近似可以写成 `skew(R^T * v)` 或相反符号。
- 对 `bg`、`ba` 的导数是 0。

为了学习主线，先记住：底盘里程计如果观测车体系速度，`H_odom` 不再是固定矩阵，而是要根据当前姿态实时计算。

程序写法：

```cpp
Vec3 hOdom(const Vec15& x)
{
    Vec3 v = x.segment<3>(IDX_V);
    Vec3 theta = x.segment<3>(IDX_THETA);
    Mat3 R = eulerToRotation(theta);
    return R.transpose() * v;
}

Mat315 jacobianHOdom(const Vec15& x)
{
    Mat315 H = Mat315::Zero();

    Vec3 v = x.segment<3>(IDX_V);
    Vec3 theta = x.segment<3>(IDX_THETA);
    Mat3 R = eulerToRotation(theta);
    Vec3 v_body = R.transpose() * v;

    H.block<3, 3>(0, IDX_V) = R.transpose();
    H.block<3, 3>(0, IDX_THETA) = skew(v_body);

    return H;
}
```

如果你的底盘里程计已经输出世界坐标系速度，那么观测模型会退化为线性：

```text
h_odom(x) = v
H_odom = [0  I3  0  0  0]
```

### 2.7 EKF 程序中通常要定义什么

EKF 中不只定义矩阵，还要定义函数和函数的雅可比：

```cpp
Vec15 x;     // 状态: [p, v, theta, bg, ba]
Mat15 P;     // 状态协方差
Mat15 Q;     // 过程噪声协方差
Mat3 R_gps;  // GPS 观测噪声
Mat3 R_odom; // 底盘里程计观测噪声

Vec15 predictStateEKF(const Vec15& x, const ImuData& imu, double dt);
Mat15 jacobianFEKF(const Vec15& x, const ImuData& imu, double dt);

Vec3 hGps(const Vec15& x);
Mat315 jacobianHGps(const Vec15& x);

Vec3 hOdom(const Vec15& x);
Mat315 jacobianHOdom(const Vec15& x);
```

预测步骤：

```cpp
void predictEKF(const ImuData& imu, double dt)
{
    Mat15 F = jacobianFEKF(x, imu, dt);

    x = predictStateEKF(x, imu, dt);
    P = F * P * F.transpose() + Q;
}
```

通用 EKF 更新：

```cpp
void updateEKF(const Vec3& z,
               const Vec3& z_pred,
               const Mat315& H,
               const Mat3& R)
{
    Vec3 y = z - z_pred;
    Mat3 S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 15, 3> K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (Mat15::Identity() - K * H) * P;
}
```

收到 IMU 时：

```cpp
predictEKF(imu_data, dt);
```

收到 GPS 时：

```cpp
updateEKF(gps_position, hGps(x), jacobianHGps(x), R_gps);
```

收到底盘里程计时：

```cpp
updateEKF(odom_body_velocity, hOdom(x), jacobianHOdom(x), R_odom);
```

### 2.8 这个 EKF 例子的完整流程

可以按时间顺序理解：

1. IMU 高频到来，用 `predictEKF()` 做状态预测。
2. 底盘里程计到来，用 `hOdom()` 做速度约束更新。
3. GPS 到来，用 `hGps()` 做位置约束更新。
4. 下一帧 IMU 继续预测，循环往复。

这和 LIGO 的思路很像：

- IMU 负责让状态在两个观测之间连续传播。
- LiDAR、GNSS、NMEA、里程计等传感器负责把预测状态拉回真实观测。
- 预测需要 `f(x)` 和 `F`。
- 每一种观测都需要 `h(x)`、`H`、`R`。

## 3. KF 和 EKF 在程序设计上的核心区别

KF 写程序时，核心是定义固定矩阵：

```text
F, Q, H_gps, R_gps, H_odom, R_odom
```

KF 预测：

```text
x = F * x
P = F * P * F^T + Q
```

KF 更新：

```text
y = z - H * x
K = P * H^T * (H * P * H^T + R)^-1
x = x + K * y
P = (I - K * H) * P
```

EKF 写程序时，核心是定义非线性函数和对应雅可比：

```text
f(x), F = df/dx
h_gps(x), H_gps = dh_gps/dx
h_odom(x), H_odom = dh_odom/dx
Q, R_gps, R_odom
```

EKF 预测：

```text
x = f(x)
P = F * P * F^T + Q
```

EKF 更新：

```text
y = z - h(x)
K = P * H^T * (H * P * H^T + R)^-1
x = x + K * y
P = (I - K * H) * P
```

所以可以这样记：

- KF：模型本身就是矩阵，直接用矩阵乘状态。
- EKF：模型是函数，状态用函数更新，协方差用函数的雅可比更新。
- `Q` 描述预测模型的不确定性。
- `R` 描述传感器观测的不确定性。
- `P` 描述当前状态估计本身的不确定性。

## 4. 和 LIGO 中 Estimator.cpp 的对应关系

这个小例子和 LIGO 的 `Estimator.cpp` 可以这样对应：

```text
小例子的 predictStateEKF()  <->  LIGO 的 get_f_output()
小例子的 jacobianFEKF()     <->  LIGO 的 df_dx_output()
小例子的 Q                 <->  LIGO 的 process_noise_cov_output()
小例子的 hGps()/hOdom()     <->  LIGO 的 h_model_GNSS_output()/h_model_NMEA_output()
小例子的 H                 <->  LIGO 各个 h_model_* 中构造的 H 矩阵
```

区别是：

- 前面的 KF 例子是 6 维普通欧式状态。
- 前面的 EKF 例子是 15 维惯导融合状态。
- LIGO 是 24 维误差状态，并且包含旋转流形。
- 这个教学 EKF 例子的滤波更新可以直接 `x = x + K * y`。
- LIGO 的状态更新涉及流形上的误差注入，所以由 `esekfom.hpp` 里的通用 ESKF 框架完成。

学习时可以先把这两个小例子看懂，再回头看 LIGO：

```text
先看状态是什么
再看预测函数 f(x)
再看预测雅可比 F
再看过程噪声 Q
再看每个传感器的 h(x)
再看每个传感器的 H 和 R
```
