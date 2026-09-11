/*
 * BSD 3-Clause License

 *  Copyright (c) 2025, Dongjiao He
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// 这个文件在当前主流程中主要承担三件事：
// 1. IMU 初始化阶段：用 sync_packages() 打包进 Measures.imu 的 IMU 数据估计平均加速度和平均角速度。
// 2. IMU 初始化完成后：把当前 LiDAR 帧转交给 feats_undistort，供 laserMapping.cpp 后续下采样和地图匹配。
// 3. 未启用 IMU 时：跳过 IMU 初始化逻辑，直接把当前 LiDAR 帧交给后续流程。

#include "IMU_Processing.h"

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

void ImuProcess::pointBodyToWorld_li_init(PointType const * const pi, PointType * const po)
{    
    V3D p_body(pi->x, pi->y, pi->z);
    
    V3D p_global;

    {
        p_global = state_LI_Init.rot * p_body + state_LI_Init.pos; // .normalized()
    }

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_vel_scale = scaler;
}

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true)
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  after_imu_init_ = false;
  state_cov.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  imu_need_init_    = true;
  init_iter_num     = 1;
  after_imu_init_   = false;
  time_last_scan = 0.0;
}

void ImuProcess::Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1),
              -gravity_(2), 0.0, gravity_(0),
              gravity_(1), -gravity_(0), 0.0;
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
  if (align_norm < 1e-6)
  {
    if (align_cos > 1e-6)
    {
      rot = Eye3d;
    }
    else
    {
      rot = -Eye3d;
    }
  }
  else
  {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos); 
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
    rot = Eigen::Quaterniond(rot).normalized().toRotationMatrix();
  }
}

void ImuProcess::IMU_init(const MeasureGroup &meas, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    // 第一帧参与 IMU 初始化的数据到来时，先重置 IMU 初始化状态。
    // mean_acc / mean_gyr 用第一条 IMU 数据作为初值，后面再用递推平均不断更新。
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto &imu : meas.imu)
  {
    // Measures.imu 是 sync_packages() 按当前 LiDAR 时间窗口同步出来的一段 IMU 数据。
    // 初始化阶段不做复杂积分，只统计加速度和角速度均值，用于后续估计重力方向和初始姿态。
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // 递推平均：不用保存所有历史 IMU，就可以随着 N 增加更新均值。
    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    N ++;
  }
}

void ImuProcess::Forward_propagation_without_imu(const MeasureGroup &meas, PointCloudXYZI &pcl_out) {
    pcl_out = *(meas.lidar);
    /*** sort point clouds by offset time ***/
    const double &pcl_beg_time = meas.lidar_beg_time;
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

    MD(12, 12) F_x, cov_w;
    double dt = 0.0;

    if (b_first_frame_) {
        dt = 0.1;
        b_first_frame_ = false;
    } else {
        dt = pcl_beg_time - time_last_scan;
        time_last_scan = pcl_beg_time;
    }

    /* covariance propagation */
    F_x.setIdentity();
    cov_w.setZero();
    /** In CV model, bg represents angular velocity **/
    /** In CV model，ba represents linear acceleration **/
    V3D dR = state_LI_Init.bg * dt;
    // M3D Exp_f = Exp(state_LI_Init.bg, dt);
    F_x.block<3, 3>(3, 3) = Exp(state_LI_Init.bg, -dt);
    F_x.block<3, 3>(3, 9) = Eye3d * dt;
    F_x.block<3, 3>(0, 6) = Eye3d * dt;


    cov_w.block<3, 3>(9, 9).diagonal() = cov_gyr_scale * dt * dt;
    cov_w.block<3, 3>(6, 6).diagonal() = cov_vel_scale * dt * dt;

    /** Forward propagation of covariance**/
    state_cov = F_x * state_cov * F_x.transpose() + cov_w;

    /** Forward propagation of attitude **/
    state_LI_Init.rot.boxplus(dR); // = state_LI_Init.rot * Exp_f;
                                                                                                            
    /** Position Propagation **/   
    V3D dp = state_LI_Init.vel * dt;              
    state_LI_Init.pos.boxplus(dp);                                                                                                   
                 
    /**CV model： un-distort pcl using linear interpolation **/        
    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;
    for(; it_pcl != pcl_out.points.begin(); it_pcl --)
    {
      dt_j= pcl_end_offset_time - it_pcl->curvature/double(1000);
      M3D R_jk(Exp(state_LI_Init.bg, - dt_j));
      V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
      // Using rotation and translation to un-distort points
      V3D p_jk;
      p_jk = - state_LI_Init.rot.transpose() * state_LI_Init.vel * dt_j; // .normalized().toRotationMatrix()

      V3D P_compensate =  R_jk * P_j + p_jk;

      /// save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);
    }
}

//这个在lasermapping中有p_imu->Process(Measures, feats_undistort);
void ImuProcess::Process(const MeasureGroup &meas, PointCloudXYZI::Ptr cur_pcl_un_)
{  
  if (imu_en)
  {
    // 启用 IMU 时，当前 LiDAR 帧必须有同步到的 IMU 数据。
    // 如果为空，说明这一帧还不能完成 IMU 相关初始化/处理，直接返回等待下一轮主循环。
    if(meas.imu.empty())  return;

    if (imu_need_init_)
    {
      {
        /// The very first lidar frame
        // IMU 尚未初始化：先用当前 LiDAR 时间窗口内的 IMU 数据更新平均加速度和平均角速度。
        // 多帧累计到 MAX_INI_COUNT 之后，认为 IMU 初始化数据足够。
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;

        if (init_iter_num > MAX_INI_COUNT)
        {
          ROS_INFO("IMU Initializing: %.1f %%", 100.0);
          // 初始化完成后关闭 imu_need_init_，并把当前 LiDAR 点云交给后续前端。
          // 这里的 cur_pcl_un_ 在 laserMapping.cpp 中对应 feats_undistort。
          imu_need_init_ = false;
          *cur_pcl_un_ = *(meas.lidar);
        }
        // *cur_pcl_un_ = *(meas.lidar);
      }
      // 初始化期间不继续执行后面的建图流程，laserMapping.cpp 会等 imu_need_init_ 变为 false。
      return;
    }
    // IMU 已经初始化完成。当前实现里这里不再做完整的逐 IMU 积分去畸变，
    // 而是把当前 LiDAR 帧复制给 cur_pcl_un_，后续点云排序、状态推进和地图匹配在 laserMapping.cpp 中继续完成。
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
  else
  {
    // 未启用 IMU 时，没有 IMU 初始化和 IMU 辅助处理，直接把 LiDAR 点云传给后续模块。
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}
