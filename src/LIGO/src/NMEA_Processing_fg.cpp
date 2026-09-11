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

#include "NMEA_Processing_fg.h"

NMEAProcess::NMEAProcess()
{
  Reset();
  // initNoises();
}

NMEAProcess::~NMEAProcess() {}

void NMEAProcess::Reset() 
{
  ROS_WARN("Reset NMEAProcess");
  p_assign->change_ext = 1;
  p_assign->gtSAMgraph.resize(0); 
  p_assign->initialEstimate.clear();
  p_assign->isamCurrentEstimate.clear();
  frame_delete = 0;
  nmea_meas_.resize(WINDOW_SIZE+1);
  p_assign->factor_id_frame.clear();
  id_accumulate = 0;
  frame_num = 0;
  last_nmea_time = 0.0;
  frame_count = 0;
  invalid_lidar = false;
  Rot_nmea_init.setIdentity();
  init_svd_last_accept_pos.setZero();
  init_svd_last_accept_time = 0.0;
  init_svd_has_last = false;
  p_assign->process_feat_num = 0;
  nmea_ready = false;
  // if (nolidar)
  {
    pre_integration->repropagate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 5; // may matter? improtant!
  p_assign->isam = gtsam::ISAM2(parameters);
}

Eigen::Vector3d NMEAProcess::local2enu(Eigen::Matrix3d R_enu_local_, Eigen::Vector3d anc, Eigen::Vector3d &pos)
{
  Eigen::Vector3d enu_pos;
  if (!nolidar)
  {
    enu_pos = R_enu_local_ * (pos - anc_local) + anc; // 
    // enu_pos = ecef2enu(first_lla_pvt, ecef_pos_ - first_xyz_ecef_pvt);
  }
  else
  {
    Eigen::Vector3d pos_r = pos;
    enu_pos = pos_r;
    // Eigen::Vector3d lla_pos = ecef2geo(first_xyz_enu_pvt);
    // enu_pos = ecef2enu(first_lla_pvt, pos_r - first_xyz_ecef_pvt);
  }
  return enu_pos;
}


void NMEAProcess::processNMEA(const nav_msgs::OdometryPtr &nmea_meas, state_output &state)
{
  double nmea_ts = nmea_meas->header.stamp.toSec();
  ROS_INFO("[DEBUG] processNMEA called! frame_count=%d, wind_size=%d, nmea_ts=%.3f, nmea_ready=%d",
           frame_count, wind_size, nmea_ts, nmea_ready);
  if (!nmea_ready)
  {

    //新增: 打印协方差 + 当前窗口累积进度
    ROS_INFO("[DIAG] processNMEA cov[0]=%.2f cov[7]=%.2f cov[14]=%.2f thres=%.2f | frame_count=%d/%d nmea_ts=%.3f",
             nmea_meas->pose.covariance[0], nmea_meas->pose.covariance[7],
             nmea_meas->pose.covariance[14], p_assign->ppp_std_threshold,
             frame_count, wind_size + 1, nmea_ts);

    if (nmea_meas->pose.covariance[0] > p_assign->ppp_std_threshold || nmea_meas->pose.covariance[7] > p_assign->ppp_std_threshold || nmea_meas->pose.covariance[14] > p_assign->ppp_std_threshold)
    {
      ROS_WARN("[DIAG] NMEA 协方差过大被拦截!");   // ← 加这行
      return;
    }
    if (!shouldAcceptInitKeyframe(nmea_meas, state))
    {
      return;
    }
    {
      rot_window[frame_count] = state.rot; //.normalized().toRotationMatrix();
      pos_window[frame_count] = state.pos + state.rot * Tex_imu_r; // .normalized()
      Eigen::Matrix3d omg_skew;
      omg_skew << SKEW_SYM_MATRX(state.omg);
      vel_window[frame_count] = state.vel + state.rot * omg_skew * Tex_imu_r; // .normalized().toRotationMatrix()
    }
    nmea_meas_[frame_count] = nmea_meas;
    frame_count ++; 
    ROS_INFO("[DEBUG] About to call NMEALIAlign with frame_count=%d", frame_count);
    nmea_ready = NMEALIAlign();
    if (nmea_ready)
    {
      ROS_INFO("NMEA Initialization is done");
      state_const_ = state;
    }
    else
    {
      ROS_INFO("[DEBUG] NMEALIAlign returned false");
      if (frame_count > wind_size)
      {
        frame_count = wind_size;
      }
    }
  }
  else
  {
    ROS_INFO_THROTTLE(5.0, "[NMEA Factor] ✅ SVD已成功! nmea_ready=true, 正在将NMEA因子加入图优化. "
                      "当前帧 ts=%.3f frame_num=%d", nmea_meas->header.stamp.toSec(), frame_num);
    nmea_meas_[0] = nmea_meas;
  }
}

bool NMEAProcess::shouldAcceptInitKeyframe(const nav_msgs::OdometryPtr &nmea_meas, const state_output &state)
{
  Eigen::Vector3d current_pos;
  if (!nolidar)
  {
    current_pos = state.pos + state.rot * Tex_imu_r;
  }
  else
  {
    current_pos = state.pos;
  }

  if (!init_svd_has_last)
  {
    init_svd_has_last = true;
    init_svd_last_accept_pos = current_pos;
    init_svd_last_accept_time = nmea_meas->header.stamp.toSec();
    ROS_INFO("[NMEA Init] accept first keyframe ts=%.3f", init_svd_last_accept_time);
    return true;
  }

  double move_dist = (current_pos - init_svd_last_accept_pos).norm();
  if (move_dist < init_svd_min_distance)
  {
    ROS_INFO_THROTTLE(1.0,
                      "[NMEA Init] Skip non-keyframe ts=%.3f move=%.3fm < threshold=%.3fm",
                      nmea_meas->header.stamp.toSec(), move_dist, init_svd_min_distance);
    return false;
  }

  init_svd_last_accept_pos = current_pos;
  init_svd_last_accept_time = nmea_meas->header.stamp.toSec();
  ROS_INFO("[NMEA Init] accept keyframe ts=%.3f move=%.3fm threshold=%.3fm",
           init_svd_last_accept_time, move_dist, init_svd_min_distance);
  return true;
}

void NMEAProcess::runISAM2opt(void) //
{
  gtsam::FactorIndices delete_factor;
  gtsam::FactorIndices().swap(delete_factor);

  if (nmea_ready)
  {
    bool delete_happen = false;
    if (frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 4000)
    {
      delete_happen = true;
    while (frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 3000)
    { 
      if (!p_assign->factor_id_frame.empty())       
      {
        // if (frame_delete > 0)
        {
        for (size_t i = 0; i < p_assign->factor_id_frame[0].size(); i++)
        {
          // if (p_assign->factor_id_frame[0][i] != 0 && p_assign->factor_id_frame[0][i] != 1 || nolidar)
          {
            delete_factor.push_back(p_assign->factor_id_frame[0][i]);
          }
        }
        // index_delete += p_assign->factor_id_frame[0].size();
        }
      
        p_assign->factor_id_frame.pop_front();
        frame_delete ++;
      }
      if (p_assign->factor_id_frame.empty()) break;
    }
    }

    if (delete_happen)
    {
      p_assign->delete_variables(nolidar, frame_delete, frame_num, id_accumulate, delete_factor);
    }
    else
    {
      p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
      p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
      p_assign->initialEstimate.clear();
      p_assign->isam.update();
    }
  }
  else
  {
    p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
    p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
    p_assign->initialEstimate.clear();
    p_assign->isam.update();
  }
  p_assign->isamCurrentEstimate = p_assign->isam.calculateEstimate();
  
  if (nolidar) // || invalid_lidar)
  {
    pre_integration->repropagate(p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(6),
                                p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(9));
  }
}

void NMEAProcess::TrajAlign(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&local_traj, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&enu_traj, Eigen::Vector3d &pos, Eigen::Matrix3d &rot)
{
// Eigen::Matrix<Type, 4,4> ComputeSim3<Type>::GetSim3() {
    // Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> trajPoints_1;
    // Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> trajPoints_2;

    // const int n = _vSyncedTraj_1.size();
    // trajPoints_1.resize(3, n);
    // trajPoints_2.resize(3, n);

    // for (int i = 0; i < _vSyncedTraj_1.size(); ++i) {
    //     trajPoints_1.block(0, i, 3, 1) = _vSyncedTraj_1[i].translation;
    //     trajPoints_2.block(0, i, 3, 1) = _vSyncedTraj_2[i].translation;
    // }
    const int n = local_traj.cols();

    Eigen::Matrix<double, 3, 1> means_1;
    Eigen::Matrix<double, 3, 1> means_2;

    double one_over_n = 1 / static_cast<double>(n);
    means_1 = local_traj.rowwise().sum() * one_over_n;
    means_2 = enu_traj.rowwise().sum() * one_over_n;

    Eigen::Matrix<double, 3, Eigen::Dynamic> demeans_1;
    Eigen::Matrix<double, 3, Eigen::Dynamic> demeans_2;
    demeans_1 = local_traj.colwise() - means_1;
    demeans_2 = enu_traj.colwise() - means_2;

    double var_1 = demeans_1.rowwise().squaredNorm().sum() * one_over_n;
    // std::cout << demeans_1.rowwise().squaredNorm().sum() << ";" << demeans_1.squaredNorm() << ";" << var_1 << std::endl;

    Eigen::Matrix<double, 3, 3> sigma;
    sigma = demeans_2 * demeans_1.transpose() * one_over_n;

    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<double, 3, 1> S;
    S = Eigen::Matrix<double, 3, 1>::Ones();

    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
        S(2) = -1;

    // Eigen::Matrix<double, 4,4> sim3 = Eigen::Matrix<Type, 4,4>::Identity();

    // sim3.block(0,0,3,3).noalias() = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    rot = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

    double c = 1 / var_1 * svd.singularValues().dot(S);

    // sim3.block(0,3,3,1) = means_2 - c * sim3.block(0,0,3,3)*means_1;
    pos = means_2 - c * rot * means_1;
    // sim3.block(0,0,3,3) = sim3.block(0,0,3,3)*c;
    rot = rot * c;
    rot.normalized();
    // return sim3;
    return;
}

// bool NMEAProcess::NMEALIAlign()
// {
//   if (frame_count < wind_size + 1) return false;

//   // --- 1. 计算 2点 SVD 的 Yaw 角 (作为对比变量) ---
//   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> local_traj_2(3, 2), enu_traj_2(3, 2);
//   for (int i = 0; i < 2; ++i) {
//       local_traj_2.block(0, i, 3, 1) = pos_window[i];
//       enu_traj_2.block(0, i, 3, 1) = Eigen::Vector3d(nmea_meas_[i]->pose.pose.position.x, nmea_meas_[i]->pose.pose.position.y, nmea_meas_[i]->pose.pose.position.z);
//   }
//   Eigen::Vector3d pos_trans_2; Eigen::Matrix3d rot_trans_2;
//   TrajAlign(local_traj_2, enu_traj_2, pos_trans_2, rot_trans_2);
//   yaw_2pt_saved = atan2(rot_trans_2(1, 0), rot_trans_2(0, 0)); // 存下2点Yaw

//   // --- 2. 计算 6点 SVD 的 Yaw 角 (本文前端方法) ---
//   const int n = wind_size + 1; // n = 6
//   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> local_traj_6(3, n), enu_traj_6(3, n);
//   for (int i = 0; i < n; ++i) {
//       local_traj_6.block(0, i, 3, 1) = pos_window[i];
//       enu_traj_6.block(0, i, 3, 1) = Eigen::Vector3d(nmea_meas_[i]->pose.pose.position.x, nmea_meas_[i]->pose.pose.position.y, nmea_meas_[i]->pose.pose.position.z);
//   }
//   Eigen::Vector3d pos_trans_6; Eigen::Matrix3d rot_trans_6;
//   TrajAlign(local_traj_6, enu_traj_6, pos_trans_6, rot_trans_6);
//   yaw_6pt_saved = atan2(rot_trans_6(1, 0), rot_trans_6(0, 0)); // 存下6点Yaw

//   // --- 3. 正式初始化系统 ---
//   yaw_enu_local = yaw_6pt_saved; // 使用6点作为系统初始优化的起点
//   anc_enu = pos_trans_6;
//   anc_local = pos_window[WINDOW_SIZE]; 
//   Rot_nmea_init = Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  
//   ROS_INFO("\033[1;32m------> SVD Init: 2-pts Yaw = %.2f deg, 6-pts Yaw = %.2f deg <------\033[0m", yaw_2pt_saved * 180.0 / M_PI, yaw_6pt_saved * 180.0 / M_PI);
//   std::cout << "★★★ [NMEA Init] 局部地图与全局 NMEA 坐标系对齐成功！开始将 GPS 引入图优化 ★★★" << std::endl;
//   SetInit();
//   frame_num = 1; 
//   last_nmea_time = nmea_meas_[wind_size]->header.stamp.toSec();
//   runISAM2opt();
//   return true;
// }

bool NMEAProcess::NMEALIAlign()
{
  // 【诊断1】打印入口状态，确认函数是否被调用及计数是否正确
  ROS_INFO_THROTTLE(3.0, "[NMEA Align Debug] Waiting for SVD init: frame_count=%d / required=%d",
                     frame_count, wind_size + 1);

  if (frame_count < wind_size + 1) return false;

  const int n = wind_size + 1; // n = 6 (wind_size=5) or 11 (wind_size=10)

  // 【诊断2】打印用于SVD对齐的数据时间范围
  {
    double t_first_local = nmea_meas_[0]->header.stamp.toSec();
    double t_last_local  = nmea_meas_[wind_size]->header.stamp.toSec();
    ROS_INFO("[SVD Diag] 对齐窗口: NMEA数据时间范围 [%.3f, %.3f] (跨度=%.2fs), 共%d帧",
             t_first_local, t_last_local, t_last_local - t_first_local, n);
  }

  // 【诊断2】在构造轨迹后、调用SVD前，检查数据有效性
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> local_traj_6(3, n), enu_traj_6(3, n);
  for (int i = 0; i < n; ++i) {

      local_traj_6.block(0, i, 3, 1) = pos_window[i];
      
      enu_traj_6.block(0, i, 3, 1) = Eigen::Vector3d(
          nmea_meas_[i]->pose.pose.position.x, 
          nmea_meas_[i]->pose.pose.position.y, 
          nmea_meas_[i]->pose.pose.position.z);
  }

  // 计算局部轨迹和ENU轨迹的空间跨度
  double local_dx = (local_traj_6.row(0).maxCoeff() - local_traj_6.row(0).minCoeff());
  double local_dy = (local_traj_6.row(1).maxCoeff() - local_traj_6.row(1).minCoeff());
  double enu_dx   = (enu_traj_6.row(0).maxCoeff() - enu_traj_6.row(0).minCoeff());
  double enu_dy   = (enu_traj_6.row(1).maxCoeff() - enu_traj_6.row(1).minCoeff());
  double local_span = std::sqrt(local_dx * local_dx + local_dy * local_dy);
  double enu_span   = std::sqrt(enu_dx * enu_dx + enu_dy * enu_dy);

  ROS_INFO("[SVD Diag] Trajectory span -> Local: %.3fm (dx=%.3f dy=%.3f) | ENU: %.3fm (dx=%.3f dy=%.3f)",
           local_span, local_dx, local_dy, enu_span, enu_dx, enu_dy);

  // 【安全守卫】几何退化检测：位移不足时SVD结果无意义
  constexpr double MIN_SPAN_THRESHOLD = 0.5; // 最小有效位移阈值(m)，可根据实际场景调整
  if (local_span < MIN_SPAN_THRESHOLD || enu_span < MIN_SPAN_THRESHOLD) {
      ROS_WARN("⚠️ [SVD Abort] 轨迹几何跨度不足! Local=%.3fm ENU=%.3fm (阈值=%.2fm). "
               "请检查前端里程计输出质量或启动时运动激励是否足够。", 
               local_span, enu_span, MIN_SPAN_THRESHOLD);
      return false; // 拒绝使用退化数据初始化，继续等待更好的窗口
  }

  // --- 1. 计算 2点 SVD 的 Yaw 角 (作为对比变量) ---
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> local_traj_2(3, 2), enu_traj_2(3, 2);
  for (int i = 0; i < 2; ++i) {
      local_traj_2.block(0, i, 3, 1) = pos_window[i];
      enu_traj_2.block(0, i, 3, 1) = Eigen::Vector3d(
          nmea_meas_[i]->pose.pose.position.x, 
          nmea_meas_[i]->pose.pose.position.y, 
          nmea_meas_[i]->pose.pose.position.z);
  }
  Eigen::Vector3d pos_trans_2; Eigen::Matrix3d rot_trans_2;
  TrajAlign(local_traj_2, enu_traj_2, pos_trans_2, rot_trans_2);
  yaw_2pt_saved = atan2(rot_trans_2(1, 0), rot_trans_2(0, 0));

  // --- 2. 计算 6点 SVD 的 Yaw 角 (本文前端方法) ---
  Eigen::Vector3d pos_trans_6; Eigen::Matrix3d rot_trans_6;
  TrajAlign(local_traj_6, enu_traj_6, pos_trans_6, rot_trans_6);
  yaw_6pt_saved = atan2(rot_trans_6(1, 0), rot_trans_6(0, 0));

  // 【诊断3】检查SVD输出的旋转矩阵是否合法
  if (std::isnan(yaw_6pt_saved) || std::isinf(yaw_6pt_saved)) {
      ROS_ERROR("❌ [SVD Failed] 6-pt Yaw 计算结果为 NaN/Inf! 协方差矩阵可能秩亏。");
      return false;
  }

  // --- 3. 正式初始化系统 ---
  yaw_enu_local = yaw_6pt_saved;
  anc_enu = pos_trans_6;
  anc_local = pos_window[WINDOW_SIZE]; 
  Rot_nmea_init = Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  
  ROS_INFO("\033[1;32m------> SVD Init: 2-pts Yaw = %.2f deg, 6-pts Yaw = %.2f deg <------\033[0m", 
           yaw_2pt_saved * 180.0 / M_PI, yaw_6pt_saved * 180.0 / M_PI);
  std::cout << "★★★ [NMEA Init] 局部地图与全局 NMEA 坐标系对齐成功！开始将 GPS 引入图优化 ★★★" << std::endl;
  
  SetInit();
  frame_num = 1; 
  last_nmea_time = nmea_meas_[wind_size]->header.stamp.toSec();
  runISAM2opt();
  return true;
}

bool NMEAProcess::Evaluate(state_output &state)
{
  if (nmea_meas_[0]->pose.covariance[0] > p_assign->ppp_std_threshold || nmea_meas_[0]->pose.covariance[7] > p_assign->ppp_std_threshold || nmea_meas_[0]->pose.covariance[14] > p_assign->ppp_std_threshold)
  {
    return false;
  }
  double time_current = nmea_meas_[0]->header.stamp.toSec();
  double delta_t = time_current - last_nmea_time;

  gtsam::Rot3 rel_rot; // = gtsam::Rot3(pre_integration->delta_q);
  gtsam::Point3 rel_pos, pos, acc, omg;
  gtsam::Vector3 rel_vel, vel, ba, bg; 
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  if (!nolidar) // && !invalid_lidar)
  {
    // Eigen::Matrix3d last_rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(0)).matrix(); // state_const_.rot; // 
    // // cout << "check time period" << pre_integration->sum_dt << ";" << time_current - last_gnss_time <<  endl;
    // Eigen::Vector3d last_pos = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(0)).segment<3>(0); // state_.pos; // 
    // Eigen::Vector3d last_vel = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(0)).segment<3>(3); // state_.vel; //
    rot = state.rot; //.normalized().toRotationMatrix(); last_rot.transpose() *
    pos = state.pos; // - last_pos; // last_rot.transpose() * (state.pos - last_pos - last_vel * delta_t - 0.5 * state.gravity * delta_t * delta_t); 
    vel = state.vel; // - last_vel; // last_rot.transpose() * (state.vel - last_vel - state.gravity * delta_t); // (state.vel - last_vel);
    ba = state.ba;
    bg = state.bg;
    acc = state.acc;
    omg = state.omg;
    // Eigen::Matrix3d rot1 = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix().transpose();
    // Eigen::Vector3d pos1 = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(0);
    // Eigen::Vector3d vel1 = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(3);
    // rel_pos = rot1 * (state.pos - pos1 - vel1 * delta_t - 0.5 * state.gravity * delta_t * delta_t);
    // rel_vel = rot1 * (state.vel - vel1 - state.gravity * delta_t);
    // rel_rot = gtsam::Rot3(rot1 * state.rot);
  }
  else
  {
    ba = state.ba;
    bg = state.bg;
    rel_rot = gtsam::Rot3(pre_integration->delta_q);
    rel_pos = pre_integration->delta_p;
    rel_vel = pre_integration->delta_v; 
  }
  
  if (!nolidar) // && invalid_lidar)
  {
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector_imu;
    Eigen::Matrix<double, 12, 1> init_others_vector_imu;
    init_vel_bias_vector_imu.block<3,1>(0,0) = state.pos;
    init_vel_bias_vector_imu.block<3,1>(3,0) = state.vel;
    init_others_vector_imu.block<3,1>(0,0) = state.omg;
    init_others_vector_imu.block<3,1>(3,0) = state.acc;
    init_others_vector_imu.block<3,1>(6,0) = state.bg;
    init_others_vector_imu.block<3,1>(9,0) = state.ba;
    p_assign->initialEstimate.insert(A(frame_num), gtsam::Vector6(init_vel_bias_vector_imu));
    p_assign->initialEstimate.insert(O(frame_num), gtsam::Vector12(init_others_vector_imu));
    p_assign->initialEstimate.insert(G(frame_num), gtsam::Vector3(state.gravity));
    p_assign->initialEstimate.insert(R(frame_num), gtsam::Rot3(state.rot));  // .normalized().toRotationMatrix()
  }
  else
  {
    Eigen::Matrix<double, 12, 1> init_vel_bias_vector;
    init_vel_bias_vector.block<3,1>(0,0) = state.pos;
    init_vel_bias_vector.block<3,1>(3,0) = state.vel;
    init_vel_bias_vector.block<3,1>(6,0) = state.ba;
    init_vel_bias_vector.block<3,1>(9,0) = state.bg;
    p_assign->initialEstimate.insert(F(frame_num), gtsam::Vector12(init_vel_bias_vector));
    p_assign->initialEstimate.insert(R(frame_num), gtsam::Rot3(state.rot)); // .normalized().toRotationMatrix()
  }              
  // rot_pos = state.rot; //.normalized().toRotationMatrix();
  if (AddFactor(rel_rot, rel_pos, rel_vel, state.gravity, delta_t, time_current, ba, bg, pos, vel, acc, omg, rot))
  {
    frame_num ++;
    runISAM2opt();
  }
  else
  {
    return false;
  }
  
  if (nolidar)
  {
    state.rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix();
    state.pos = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(0);
    state.vel = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(3);
    state.ba = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(6);
    state.bg = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(9);
    state.gravity = gravity_init;
  }
  else
  {
    state_const_.rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix();
    state_const_.pos = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(0);
    state_const_.vel = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(3);
    state.gravity = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix().transpose() * gravity_init;
  }
  last_nmea_time = time_current;
  return true;
}

bool NMEAProcess::AddFactor(gtsam::Rot3 rel_rot, gtsam::Point3 rel_pos, gtsam::Vector3 rel_vel, Eigen::Vector3d state_gravity, double delta_t, double time_current,
                Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d omg, Eigen::Matrix3d rot)
{
  invalid_lidar = false;
  if (!nolidar)
  {
    invalid_lidar = nolidar_cur;
    double weight_lid = 1;
    if (p_assign->process_feat_num < 10) 
    {
      weight_lid = 0;
    }
    else
    {
      weight_lid = 2 * double(norm_vec_num) / double(p_assign->process_feat_num);
    }
    norm_vec_num = 0;
    p_assign->process_feat_num = 0;
    double weight_check = (sqrt_lidar(0, 0) + sqrt_lidar(1, 1) + sqrt_lidar(2, 2) 
                          + sqrt_lidar(6, 6) + sqrt_lidar(7, 7) + sqrt_lidar(8, 8)) / 6; // + sqrt_lidar(3, 3) + sqrt_lidar(4, 4) + sqrt_lidar(5, 5)
    sqrt_lidar *= weight_lid / weight_check;
    // invalid_lidar = nolidar_cur;
    // size_t num_norm = norm_vec_holder.size();
    // if (num_norm > 2) // 10)
    for (size_t j = 0; j < 9; j++)
    {
      if (sqrt_lidar(j, j) < 0.50)
      {
        sqrt_lidar(j, j) = 0.5;
        invalid_lidar = true;
      }
    }
  }
  if (nolidar_cur && !nolidar) nolidar_cur = false;

  std::vector<size_t> factor_id_cur;
  M3D omg_skew;
  omg_skew << SKEW_SYM_MATRX(omg);
  Eigen::Vector3d hat_omg_T = omg_skew * Tex_imu_r;
  if (!nolidar)
  {
    bool no_weight = false;
    // if (invalid_lidar)
    // {
    //   p_assign->gtSAMgraph.add(ligo::NmeaLioGravRelFactor(P(0), R(0), A(0), R(frame_num), A(frame_num), gravity_init, state_gravity, ba, bg, rot, sqrt_lidar, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
    //   factor_id_cur.push_back(id_accumulate);
    //   id_accumulate += 1;
    // }
    // else
    {
      p_assign->gtSAMgraph.add(ligo::NmeaLioGravRelFactor(P(0), R(frame_num), A(frame_num), O(frame_num), G(frame_num), gravity_init, state_gravity, pos, vel, rot, ba, bg, acc, omg, sqrt_lidar, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
      factor_id_cur.push_back(id_accumulate);
      id_accumulate += 1;
    }
    // p_assign->gtSAMgraph.add(ligo::NmeaLioFactor(R(frame_num-1), A(frame_num-1), R(frame_num), A(frame_num), rel_rot, rel_pos, rel_vel, state_gravity, delta_t, p_assign->relatNoise));
    // factor_id_cur.push_back(id_accumulate);
    // id_accumulate += 1;
    // if (frame_num < 200)
    // {
    //   odo_weight1 = 2*sqrt_lidar(0, 0); // odo_weight4 = sqrt_lidar(3, 3);
    //   odo_weight2 = 2*sqrt_lidar(1, 1); // odo_weight5 = sqrt_lidar(4, 4);
    //   odo_weight3 = 2*sqrt_lidar(2, 2) / 3; // odo_weight6 = sqrt_lidar(5, 5);
    //   // odo_weight4 = sqrt_lidar(3, 3) / 3;
    //   // odo_weight5 = sqrt_lidar(4, 4) / 3;
    //   // odo_weight6 = sqrt_lidar(5, 5) / 3;
    // }
    // else
    // {
    //   odo_weight1 = 3*sqrt_lidar(0, 0); // odo_weight4 = sqrt_lidar(3, 3);
    //   odo_weight2 = 3*sqrt_lidar(1, 1); // odo_weight5 = sqrt_lidar(4, 4);
    //   odo_weight3 = sqrt_lidar(2, 2); // odo_weight6 = sqrt_lidar(5, 5);
    //   // odo_weight4 = sqrt_lidar(3, 3) / 2; // odo_weight6 = sqrt_lidar(5, 5);
    //   // odo_weight5 = sqrt_lidar(4, 4) / 2; // odo_weight6 = sqrt_lidar(5, 5);
    //   // odo_weight6 = sqrt_lidar(5, 5) / 2; // odo_weight6 = sqrt_lidar(5, 5);
    // }
  }
  else
  {
    p_assign->gtSAMgraph.add(ligo::GnssLioFactorNolidar(R(frame_num-1), F(frame_num-1), R(frame_num), F(frame_num), rel_rot, rel_pos, rel_vel, 
                  state_gravity, delta_t, ba, bg, pre_integration, p_assign->odomNoiseIMU));
    p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate);
    id_accumulate += 1;
  }
  {
    double values[17];
    values[0] = Tex_imu_r[0]; values[1] = Tex_imu_r[1]; values[2] = Tex_imu_r[2]; values[3] = anc_local[0]; values[4] = anc_local[1]; values[5] = anc_local[2];
    values[6] = nmea_meas_[0]->pose.pose.position.x; values[7] = nmea_meas_[0]->pose.pose.position.y; values[8] = nmea_meas_[0]->pose.pose.position.z; 
    values[9] = nmea_meas_[0]->twist.twist.linear.x; values[10] = nmea_meas_[0]->twist.twist.linear.y; values[11] = nmea_meas_[0]->twist.twist.linear.z;
    values[12] = nmea_meas_[0]->pose.pose.orientation.w; values[13] = nmea_meas_[0]->pose.pose.orientation.x; values[14] = nmea_meas_[0]->pose.pose.orientation.y;
    values[15] = nmea_meas_[0]->pose.pose.orientation.z; 
    values[16] = nmea_weight; 
    if (!nolidar)
    {
      // Eigen::Vector3d RTex1 = rot * Tex_imu_r;
      // values[0] = RTex1[0]; values[1] = RTex1[1]; values[2] = RTex1[2]; 
      // 【添加打印】确认 NMEA Factor 被成功加入到因子图
      std::cout << "[Factor Graph] 正在添加 NMEA 绝对位姿因子 (NMEAFactor), 当前帧: " << frame_num << std::endl;
      if (frame_num < delete_thred)
      {
        p_assign->gtSAMgraph.add(ligo::NMEAFactor(P(0), E(0), A(frame_num), R(frame_num), invalid_lidar, values, hat_omg_T, Rex_imu_r, p_assign->robustnmeaNoise_init));
      }
      else
      {
        p_assign->gtSAMgraph.add(ligo::NMEAFactor(P(0), E(0), A(frame_num), R(frame_num), invalid_lidar, values, hat_omg_T, Rex_imu_r, p_assign->robustnmeaNoise));
      }
    }
    else
    {
      p_assign->gtSAMgraph.add(ligo::NMEAFactorNolidar(R(frame_num), F(frame_num), values, hat_omg_T, Rex_imu_r, p_assign->robustnmeaNoise)); // not work
    }
    factor_id_cur.push_back(id_accumulate);
    id_accumulate += 1;
  }

  {
    p_assign->factor_id_frame.push_back(factor_id_cur);
    std::vector<size_t>().swap(factor_id_cur);
  }
  return true;
}

void NMEAProcess::SetInit()
{
  if (!nolidar)
  {
    Eigen::Matrix3d R_enu_local_;
    R_enu_local_.setIdentity(); // = Rot_nmea_init; // * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()) 
    // prior factor 
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector;
    Eigen::Matrix<double, 12, 1> init_others_vector;
    init_vel_bias_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero(); // (pos_window - rot_window * Tex_imu_r); // Rot_nmea_init.transpose() * 
    init_vel_bias_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero(); // vel_window; // Rot_nmea_init.transpose() * 
    init_others_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero(); // vel_window; // Rot_nmea_init.transpose() * 
    init_others_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero(); // vel_window; // Rot_nmea_init.transpose() * 
    init_others_vector.block<3,1>(6,0) = Eigen::Vector3d::Zero(); // vel_window; // Rot_nmea_init.transpose() * 
    init_others_vector.block<3,1>(9,0) = Eigen::Vector3d::Zero(); // vel_window; // Rot_nmea_init.transpose() * 
    // dt[0] = para_rcv_dt[wind_size*4]; dt[1] = para_rcv_dt[wind_size*4+1], dt[2] = para_rcv_dt[wind_size*4+2], dt[3] = para_rcv_dt[wind_size*4+3];
    // ddt = para_rcv_ddt[wind_size];
    p_assign->initialEstimate.insert(P(0), gtsam::Rot3(Rot_nmea_init)); // rot_window)); // Rot_nmea_init.transpose() * 
    // p_assign->initialEstimate.insert(F(0), gtsam::Vector12(init_vel_bias_vector));
    p_assign->initialEstimate.insert(A(0), gtsam::Vector6(init_vel_bias_vector));
    p_assign->initialEstimate.insert(O(0), gtsam::Vector12(init_others_vector));
    p_assign->initialEstimate.insert(E(0), gtsam::Vector3(anc_enu[0], anc_enu[1], anc_enu[2]));
    p_assign->initialEstimate.insert(R(0), gtsam::Rot3(R_enu_local_));
    p_assign->initialEstimate.insert(G(0), gtsam::Vector3(gravity_init));

    gtsam::PriorFactor<gtsam::Rot3> init_rot_ext(P(0), gtsam::Rot3(gtsam::Rot3(Rot_nmea_init)), p_assign->priorextrotNoise);
    gtsam::PriorFactor<gtsam::Vector3> init_pos_ext(E(0), gtsam::Vector3(anc_enu[0], anc_enu[1], anc_enu[2]), p_assign->priorextposNoise);
    gtsam::PriorFactor<gtsam::Rot3> init_rot_(R(0), gtsam::Rot3(R_enu_local_), p_assign->priorrotNoise); // Rot_nmea_init.transpose() * 
    gtsam::PriorFactor<gtsam::Vector6> init_vel_(A(0), gtsam::Vector6(init_vel_bias_vector), p_assign->priorNoise); // priorposNoise);
    gtsam::PriorFactor<gtsam::Vector12> init_bias_(O(0), gtsam::Vector12(init_others_vector), p_assign->priorBiasNoise); // priorposNoise);
    gtsam::PriorFactor<gtsam::Vector3> init_grav_(G(0), gtsam::Vector3(gravity_init), p_assign->priorGravNoise); // priorposNoise);
    p_assign->gtSAMgraph.add(init_rot_ext);
    p_assign->gtSAMgraph.add(init_pos_ext);
    p_assign->gtSAMgraph.add(init_rot_);
    p_assign->gtSAMgraph.add(init_vel_);
    p_assign->gtSAMgraph.add(init_bias_);
    p_assign->gtSAMgraph.add(init_grav_);
    p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1, 2, 3, 4, 5});
    id_accumulate += 6;
  }
  else
  {
    gtsam::PriorFactor<gtsam::Rot3> init_rot(R(0), gtsam::Rot3(rot_window[wind_size]), p_assign->priorrotNoise); //  * R_enu_local_
    Eigen::Matrix<double, 12, 1> init_vel_bias_vector;
    init_vel_bias_vector.block<3,1>(0,0) = anc_enu + pos_window[wind_size] - rot_window[wind_size] * Tex_imu_r; //  * R_enu_local_
    init_vel_bias_vector.block<3,1>(3,0) = vel_window[wind_size]; // R_enu_local_ * 
    init_vel_bias_vector.block<6,1>(6,0) = Eigen::Matrix<double, 6, 1>::Zero();
    gtsam::PriorFactor<gtsam::Vector12> init_vel_bias(F(0), gtsam::Vector12(init_vel_bias_vector), p_assign->priorposNoise);
    p_assign->gtSAMgraph.add(init_rot);
    p_assign->gtSAMgraph.add(init_vel_bias);
    p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1}); //{i * 4, i * 4 + 1, i * 4  + 2, i * 4 + 3});
    p_assign->initialEstimate.insert(R(0), gtsam::Rot3(rot_window[wind_size])); // R_enu_local_ * 
    p_assign->initialEstimate.insert(F(0), gtsam::Vector12(init_vel_bias_vector));
    id_accumulate += 2;
  }
}
