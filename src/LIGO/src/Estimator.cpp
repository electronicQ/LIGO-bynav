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

// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"

// Estimator.cpp 是当前 LIGO 前端 ESKF 的“模型定义层”：
// 1. 定义全局滤波器对象 kf_output 以及前端匹配中会复用的全局缓存。
// 2. 定义状态传播模型 get_f_output() 和传播雅可比 df_dx_output()。这两个用于对应预测步骤的均值和协方差传播
// 3. 定义 LiDAR / IMU / GNSS / NMEA 的观测残差和观测雅可比。
// 真正调用 predict() / update() 的调度逻辑在 laserMapping.cpp 中；
// 真正执行卡尔曼滤波矩阵运算的通用类在 include/IKFoM/.../esekfom.hpp 中。
// 对应的24维状态在common_lib.h中，MTK_BUILD_MANIFOLD

// normvec: 保存 LiDAR 点到局部地图平面匹配后得到的平面参数。
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
// time_seq: laserMapping.cpp 按点云 curvature 时间压缩后的分段长度。
// h_model_output() 每次处理 time_seq[k] 个相同时间段内的点。
std::vector<int> time_seq;
// feats_down_body: 当前帧下采样后的 LiDAR 点，仍在 LiDAR/body 坐标系。
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));
// feats_down_world: 当前帧点变换到世界坐标系后的结果，用于和局部地图匹配。
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));
// pbody_list / pimu_list / crossmat_list: 为 LiDAR 量测雅可比提前缓存点坐标和反对称矩阵。
std::vector<V3D> pbody_list;
std::vector<V3D> pimu_list;
std::vector<PointVector> Nearest_Points; 
std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox，这个局部地图是用什么存储的？
std::shared_ptr<IVoxType> ivox_last_ = nullptr;                    // localmap in ivox
std::vector<double> knots_t;
std::vector<V3D> cp_pos;
std::vector<V3D> updatedmap;
std::vector<M3D> cp_rot;
std::vector<V3D> lidar_points;
std::deque<std::unordered_set<Eigen::Matrix<int, 3, 1>, faster_lio::hash_vec<3>>> empty_voxels;
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
bool   point_selected_surf[100000] = {0};
std::vector<M3D> crossmat_list;
int effct_feat_num = 0;
int k = 0;
int idx = -1;
// kf_output: 当前 LIGO 前端使用的 ESKF 实例。
// state_output 是状态类型，24 是过程噪声维度，input_ikfom 是输入类型。
// 它的模型函数在 laserMapping.cpp 中通过 init_dyn_share_modified_3h() 注册。
esekfom::esekf<state_output, 24, input_ikfom> kf_output;
double G_m_s2 = 9.81;
input_ikfom input_in;
// angvel_avr / acc_avr: laserMapping.cpp 从当前 IMU 消息取出的角速度和加速度观测。
// h_model_IMU_output() 会用它们构造 IMU 量测残差。
V3D angvel_avr, acc_avr, acc_avr_norm;
int feats_down_size = 0;  
// LiDAR 相对 IMU 的外参，由 parameters.cpp 从配置文件读入，pointBodyToWorld() 会使用。
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
int scan_count = 0;

/**
 * @brief 构造 input 状态形式的过程噪声协方差。
 *
 * 这个函数返回 19x19 的噪声矩阵，对应另一套 input/state 建模方式中的噪声排列。
 * 从当前源码搜索结果看，主运行路径主要使用 process_noise_cov_output()，
 * 这个函数更像保留的旧接口或备用接口。
 *
 * @return Eigen::Matrix<double, 19, 19> 过程噪声协方差矩阵。
 */
Eigen::Matrix<double, 19, 19> process_noise_cov_input()
{
	Eigen::Matrix<double, 19, 19> cov;
	cov.setZero();
	// 陀螺仪白噪声。
	cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
	// 加速度计白噪声。
	cov.block<3, 3>(6, 6).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
	// 陀螺仪 bias 随机游走噪声。
	cov.block<3, 3>(10, 10).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	// 加速度计 bias 随机游走噪声。
	cov.block<3, 3>(13, 13).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	return cov;
}


/**
 * @brief 构造当前主流程 kf_output 使用的 24x24 过程噪声协方差 Q_output。
 *
 * 	这里vel_cov的意思:
 * 	我对速度预测过程不是完全信任。
	即使用 v_new = v_old + a * dt 这个模型传播，
	中间仍可能有没建模的扰动、传感器误差、模型误差，
	所以预测后要给速度状态额外增加不确定性。
 * 
 * 用从 yaml / 默认参数读取的噪声值构造 Q_output，
 * 它描述预测模型本身的不确定性，最终用于传播状态协方差 P
 * 
 * laserMapping.cpp 初始化时调用本函数得到 Q_output，随后在 kf_output.predict()
 * 中用于协方差传播：P = F * P * F^T + Q * dt^2。
 *
 * 当前 state_output 的误差状态自由度为 24：
 * pos / rot / vel / omg / acc / gravity / bg / ba 各 3 维。
 *
 * @return Eigen::Matrix<double, 24, 24> ESKF 过程噪声协方差矩阵。
 */
Eigen::Matrix<double, 24, 24> process_noise_cov_output()
{
	Eigen::Matrix<double, 24, 24> cov;
	cov.setZero();
	// 第 6~8 维：速度过程噪声。
	cov.block<3, 3>(6, 6).diagonal() << vel_cov, vel_cov, vel_cov;
	// 第 9~11 维：角速度过程噪声。
	cov.block<3, 3>(9, 9).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
	// 第 12~14 维：加速度过程噪声。
	cov.block<3, 3>(12, 12).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
	// 第 18~20 维：陀螺仪 bias 随机游走噪声。
	cov.block<3, 3>(18, 18).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	// 第 21~23 维：加速度计 bias 随机游走噪声。
	cov.block<3, 3>(21, 21).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	return cov;
}


/**
 * @brief ESKF 状态传播函数 f(x, u)。
 *
 * esekfom::esekf::predict() 会调用这个函数，得到状态在当前时刻的导数。
 * 当前实现没有显式使用 input_ikfom in，而是使用 state_output 内部的
 * s.vel / s.omg / s.acc / s.gravity 作为连续时间模型。
 *
 * 状态排列来自 common_lib.h 的 state_output：
 * pos, rot, vel, omg, acc, gravity, bg, ba。
 *
 * 当前非零导数：
 * - pos_dot = vel   				等价于 pos = pos0 + vel*dt 
 * - rot_dot = omg	 				等价于 rot = rot0 + omg*dt
 * - vel_dot = rot * acc + gravity 	等价于 vel = vel0 + (R * acc + gravity) * dt
 * 除了位置、姿态、线速度，其他状态传播都认为是不变。
 *
 * @param s 当前滤波状态。
 * @param in 当前滤波输入，当前主模型中未直接使用。
 * @return Eigen::Matrix<double, 24, 1> 状态导数向量。
 */
Eigen::Matrix<double, 24, 1> get_f_output(state_output &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	// s.acc 是机体系下估计的加速度；乘以姿态 s.rot 转到世界系。
	vect3 a_inertial = s.rot * s.acc; // .normalized()
	for(int i = 0; i < 3; i++ ){
		// 位置导数 = 当前速度。
		res(i) = s.vel[i];
		// 姿态导数 = 当前角速度。SO3 的具体积分由 MTK/esekfom 的 oplus 完成。
		res(i + 3) = s.omg[i]; 
		// 速度导数 = 世界系加速度 + 重力。
		res(i + 6) = a_inertial[i] + s.gravity[i]; 
	}
	return res;
}

/**
 * @brief 状态传播模型对误差状态的雅可比 F。
 *
 * esekfom::esekf::predict() 在传播协方差时调用这个函数。
 * 返回矩阵用于近似：
 * P_k+1 = F * P_k * F^T + Q * dt^2
 *
 * @param s 当前滤波状态。
 * @param in 当前滤波输入，当前函数中未直接使用。
 * @param delta_t 当前预测步长。
 * @return Eigen::Matrix<double, 24, 24> 离散传播雅可比矩阵。
 */
Eigen::Matrix<double, 24, 24> df_dx_output(state_output &s, const input_ikfom &in, double delta_t)
{
	//刚开始就初始化为单位阵，24维状态，刚开始都初始为单位矩阵，
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Identity();

	SO3 delta_R;
	Eigen::MatrixXd Jacob;
	// 角速度在 delta_t 内形成小旋转增量。
	Eigen::Vector3d delta_theta = s.omg * delta_t;
	Eigen::VectorXd delta_theta_ = delta_theta;
	delta_R = SO3::exp(delta_theta);
	// SO3 右雅可比，用于姿态误差传播。
	delta_R.Jacob_right(delta_theta_, Jacob);

	// pos 对 vel 的导数：p_new = p + v * dt。
	cov.template block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * delta_t;
	// vel 对 rot 的导数：世界系加速度 rot * acc 对姿态扰动敏感。
	cov.template block<3, 3>(6, 3) = -s.rot * MTK::hat(s.acc) * delta_t; // .normalized().toRotationMatrix()
	// vel 对 acc 的导数：a_world = rot * acc。
	cov.template block<3, 3>(6, 12) = s.rot * delta_t; //.normalized().toRotationMatrix();
	// rot 对 omg 的导数。
	cov.template block<3, 3>(3, 9) = Jacob * delta_t;
	// rot 对上一时刻 rot 误差的传播。
	cov.template block<3, 3>(3, 3) = delta_R.transpose();
	// Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	// Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	// s.S2_Mx(grav_matrix, vec, 21);
	// vel 对 gravity 的导数：v_new = v + gravity * dt。
	cov.template block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * delta_t; // grav_matrix; 
	return cov;
}

/**
 * @brief LiDAR 点到局部地图平面的观测模型。
 *
 * 这个函数由 esekfom::esekf::update_iterated_dyn_share_modified() 调用。
 * 它负责从当前时间段的 LiDAR 点中选择有效匹配点，构造：
 * - ekfom_data.z：点到平面的残差；
 * - ekfom_data.h_x：LiDAR 观测对位置/姿态误差的雅可比；
 * - ekfom_data.M_Noise：LiDAR 量测噪声。
 *
 * 注意：这里的 LiDAR 观测 h_x 只有 6 列，直接约束 pos 和 rot。
 * 其他状态通过协方差相关性被间接修正。
 *
 * @param s 当前滤波状态。
 * @param cov_p 当前状态位置协方差块，当前函数中没有直接使用。
 * @param cov_R 当前状态姿态协方差块，当前函数中没有直接使用。
 * @param ekfom_data 输出给 ESKF 更新步骤的共享量测数据结构。
 */
void h_model_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	bool match_in_map = false;
	VF(4) pabcd;
	pabcd.setZero();
	// time_seq[k] 表示当前这次 LiDAR 更新要处理的点数。
	normvec->resize(time_seq[k]);
	int effect_num_k = 0;
	// 第一遍：把当前的一帧点云根据每个点对应的状态逐个的变换到世界系，在 iVox 局部地图中找邻域点，并拟合局部平面。（一次循环处理body系下的一个点）
	for (int j = 0; j < time_seq[k]; j++)
	{
		//已知的是feats_down_body指向的一帧lidar系下的点云，取出这帧点云的一个点，利用这个点对应的时间状态转换到世界系
		PointType &point_body_j  = feats_down_body->points[idx+j+1];
		PointType &point_world_j = feats_down_world->points[idx+j+1];
		// 使用当前状态估计，把 LiDAR 点从机体系变换到世界系。
		pointBodyToWorld(&point_body_j, &point_world_j); 
		V3D p_body = pbody_list[idx+j+1];
		double p_norm = p_body.norm();
		V3D p_world;
		p_world << point_world_j.x, point_world_j.y, point_world_j.z;
		{
			auto &points_near = Nearest_Points[idx+j+1];
			// 在局部地图 ivox_ 中寻找当前点附近的 NUM_MATCH_POINTS 个近邻。
            ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS); // 
			if ((points_near.size() < NUM_MATCH_POINTS)) // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
			{
				// 近邻数量不够，不能稳定拟合平面，该点不参与 LiDAR 更新。
				point_selected_surf[idx+j+1] = false;
			}
			else
			{
				point_selected_surf[idx+j+1] = false;
				// 用近邻点拟合平面，pabcd 表示平面参数 Ax + By + Cz + D = 0。
				if (esti_plane(pabcd, points_near, plane_thr)) //(planeValid)
				{
					// pd2 是当前点到拟合平面的绝对距离。
					float pd2 = fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
					
					if (effect_num_k > 0) continue;
					// 用距离和点深度做一次筛选，过滤不可靠的平面约束。
					if (p_norm > match_s * pd2 * pd2)
					{
						point_selected_surf[idx+j+1] = true;
						// normvec 临时保存平面法向量和 D，后面构造残差/雅可比时使用。
						normvec->points[j].x = pabcd(0);
						normvec->points[j].y = pabcd(1);
						normvec->points[j].z = pabcd(2);
						normvec->points[j].intensity = pabcd(3);
						effect_num_k ++;
					}
				}  
			}
		}
	}
	if (effect_num_k == 0) 
	{
		// 没有有效 LiDAR 平面约束，本次 ESKF LiDAR 更新无效。
		ekfom_data.valid = false;
		return;
	}
	ekfom_data.M_Noise = laser_point_cov;
	// double sqrt_laser_noise = sqrt(laser_point_cov);
	// LiDAR 点到平面残差只直接约束 6 维：位置误差 3 维 + 姿态误差 3 维。
	ekfom_data.h_x.resize(effect_num_k, 6);
	ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 6); // 12);
	ekfom_data.z.resize(effect_num_k);
	// ekfom_data.z_R.resize(effect_num_k);
	int m = 0;
	// V3D last_norm_vec = V3D::Zero();
	// if (!p_gnss->norm_vec_holder.empty()) 
	// {
	// 	last_norm_vec = p_gnss->norm_vec_holder.back();
	// }
	for (int j = 0; j < time_seq[k]; j++)
	{
		// ekfom_data.converge = false;
		if(point_selected_surf[idx+j+1])
		{
			// 取出当前点匹配到的平面法向量。
			V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
			
			// {   
				M3D point_crossmat = crossmat_list[idx+j+1];
				// 将世界系平面法向量转到当前机体系相关的扰动表达中，用于姿态雅可比。
				V3D C(s.rot.transpose() * norm_vec); // conjugate().normalized()
				V3D A(point_crossmat * C);

				if (std::fabs(norm_vec(2)) > 0.9) A = A.normalized(); // if (A.norm() > 1.0 && std::fabs(norm_vec(2)) > 0.9) 
				// h_x = [对位置的偏导, 对姿态的偏导]。
				ekfom_data.h_x.block<1, 6>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A); //, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			// }
			// 点到平面的有符号残差，目标是让 n^T * p + D 接近 0。
			ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx+j+1].x -norm_vec(1) * feats_down_world->points[idx+j+1].y -norm_vec(2) * feats_down_world->points[idx+j+1].z-normvec->points[j].intensity;
			
			m++;
		}
	}
	effct_feat_num += effect_num_k;
	if (GNSS_ENABLE) p_gnss->norm_vec_num += effect_num_k;
	if (NMEA_ENABLE) p_nmea->norm_vec_num += effect_num_k;
}

/**
 * @brief IMU 观测模型。
 *
 * 这个函数由 esekfom::esekf::update_iterated_dyn_share_IMU() 调用。
 * 它把当前 IMU 角速度/加速度观测与状态中的 omg/acc/bg/ba 作比较，
 * 构造 6 维 IMU 残差：
 * - 前 3 维：角速度残差；
 * - 后 3 维：加速度残差。
 *
 * @param s 当前滤波状态。
 * @param ekfom_data 输出 IMU 残差、IMU 噪声和饱和标志。
 */
void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data)
{
    // satu_check 用来标记 IMU 某一轴是否接近量程饱和，饱和轴会在更新中被弱化/跳过。
    std::memset(ekfom_data.satu_check, false, 6);
	// 角速度残差：测量角速度 - 状态角速度 - 陀螺仪 bias。
	ekfom_data.z_IMU.block<3,1>(0, 0) = angvel_avr - s.omg - s.bg;
	// 加速度残差：归一化到重力尺度后的测量加速度 - 状态加速度 - 加速度计 bias。
	ekfom_data.z_IMU.block<3,1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
	// IMU 量测噪声，前三维对应 gyro，后三维对应 acc。
    ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov, imu_meas_acc_cov, imu_meas_acc_cov;
	if(check_satu)
	{
		// 如果某个轴接近传感器量程上限，就认为该轴可能饱和，避免用它强行修正状态。
		if(fabs(angvel_avr(0)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[0] = true; 
			ekfom_data.z_IMU(0) = 0.0;
		}
		
		if(fabs(angvel_avr(1)) >= 0.99 * satu_gyro) 
		{
			ekfom_data.satu_check[1] = true;
			ekfom_data.z_IMU(1) = 0.0;
		}
		
		if(fabs(angvel_avr(2)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[2] = true;
			ekfom_data.z_IMU(2) = 0.0;
		}
		
		if(fabs(acc_avr(0)) >= 0.99 * satu_acc)
		{
			ekfom_data.satu_check[3] = true;
			ekfom_data.z_IMU(3) = 0.0;
		}

		if(fabs(acc_avr(1)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[4] = true;
			ekfom_data.z_IMU(4) = 0.0;
		}

		if(fabs(acc_avr(2)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[5] = true;
			ekfom_data.z_IMU(5) = 0.0;
		}
	}
}

/**
 * @brief GNSS 观测模型。
 *
 * 当 GNSS_ENABLE 为 true 时，laserMapping.cpp 会把本函数注册为第三类观测模型。
 * GNSSProcess::processGNSS() 会先根据原始 GNSS 后端结果更新 p_gnss->state_const_，
 * 本函数再把该 GNSS 约束转换成 ESKF 可用的残差和雅可比。
 *
 * 当前残差主要包含：
 * - 位置残差：p_gnss->state_const_.pos - s.pos
 * - 姿态残差：Log(s.rot^T * p_gnss->state_const_.rot)
 *
 * @param s 当前滤波状态。
 * @param cov_p 当前位置协方差块，保留接口，当前主要逻辑中未直接使用。
 * @param cov_R 当前姿态协方差块，保留接口，当前主要逻辑中未直接使用。
 * @param ekfom_data 输出 GNSS 残差、雅可比和噪声。
 */
void h_model_GNSS_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	// 姿态残差：当前 ESKF 姿态到 GNSS 后端约束姿态之间的相对旋转。
	Eigen::Matrix3d res_R = s.rot.transpose() * p_gnss->state_const_.rot;
	Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
	// h_GNSS 是 6x6，默认位置/姿态直接观测自身。
	ekfom_data.h_GNSS.setIdentity();
	// ekfom_data.h_GNSS *= p_gnss->odo_weight;
	// ekfom_data.h_GNSS(0, 0) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(3, 3) = p_gnss->odo_weight4;
	// ekfom_data.h_GNSS(1, 1) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(4, 4) = p_gnss->odo_weight5;
	// ekfom_data.h_GNSS(0, 0) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(1, 1) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(2, 2) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// 姿态误差在 SO3 上，需要用右雅可比逆把旋转残差线性化。
	ekfom_data.h_GNSS.block<3, 3>(3, 3) = 0.1 * Jacob_right_inv<double>(res_r); // 0.1 *
	// ekfom_data.h_GNSS.block<1, 3>(3, 3) *= p_gnss->odo_weight4;
	// ekfom_data.h_GNSS.block<1, 3>(4, 3) *= p_gnss->odo_weight5;
	// ekfom_data.h_GNSS.block<1, 3>(5, 3) *= p_gnss->odo_weight6;
	ekfom_data.z_GNSS.setZero();
	// ekfom_data.h_GNSS.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.h_GNSS.block<3, 3>(6, 6) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.z_GNSS.block<3, 1>(3, 0) = res_r;
	// ekfom_data.z_GNSS.block<3, 1>(0, 0) = p_gnss->odo_weight * (p_gnss->state_const_.pos - s.pos); // 
	// 位置残差。
	ekfom_data.z_GNSS.block<3, 1>(0, 0) = p_gnss->state_const_.pos - s.pos;
	// ekfom_data.z_GNSS(0) *= 1.5;
	// ekfom_data.z_GNSS(1) *= 1.5;
	// ekfom_data.z_GNSS(2) *= 1.5;
	// ekfom_data.z_GNSS(0) = p_gnss->odo_weight1 * (p_gnss->state_const_.pos(0) - s.pos(0)); // 
	// ekfom_data.z_GNSS(1) = p_gnss->odo_weight2 * (p_gnss->state_const_.pos(1) - s.pos(1)); // 
	// ekfom_data.z_GNSS(2) = p_gnss->odo_weight3 * (p_gnss->state_const_.pos(2) - s.pos(2)); // 
	// 姿态残差，这里乘 0.1 表示对 GNSS 姿态约束做了缩放。
	ekfom_data.z_GNSS.block<3, 1>(3, 0) = 0.1 * res_r; // s.rot.transpose() * p_gnss->state_.rot; // 0.1 *
	// ekfom_data.z_GNSS(3) *= p_gnss->odo_weight4; // s.rot.transpose() * p_gnss->state_.rot; //  
	// ekfom_data.z_GNSS(4) *= p_gnss->odo_weight5; // s.rot.transpose() * p_gnss->state_.rot; //  
	// ekfom_data.z_GNSS(5) *= p_gnss->odo_weight6; // s.rot.transpose() * p_gnss->state_.rot; //  
	// ekfom_data.z_GNSS(3) = p_gnss->odo_weight1 * (p_gnss->state_const_.vel(0) - s.vel(0)); // 
	// ekfom_data.z_GNSS(4) = p_gnss->odo_weight2 * (p_gnss->state_const_.vel(1) - s.vel(1)); // 
	// ekfom_data.z_GNSS(5) = p_gnss->odo_weight3 * (p_gnss->state_const_.vel(2) - s.vel(2)); // 
	// ekfom_data.z_GNSS.block<3, 1>(6, 0) = p_gnss->state_const_.vel - s.vel;
	// double error_1 = abs(ekfom_data.z_GNSS(0)) - sqrt(abs(cov_p(0, 0)));
	// // error_1 *= error_1;
	// double error_2 = abs(ekfom_data.z_GNSS(1)) - sqrt(abs(cov_p(1, 1)));
	// // error_2 *= error_2;
	// double error_3 = abs(ekfom_data.z_GNSS(2)) - sqrt(abs(cov_p(2, 2)));
	// // error_3 *= error_3;
	// double error_4 = abs(ekfom_data.z_GNSS(3)) - sqrt(abs(cov_R(0, 0)));
	// // error_4 *= error_4;
	// double error_5 = abs(ekfom_data.z_GNSS(4)) - sqrt(abs(cov_R(1, 1)));
	// // error_5 *= error_5;
	// double error_6 = abs(ekfom_data.z_GNSS(5)) - sqrt(abs(cov_R(2, 2)));
	// error_6 *= error_6;
	// double gnss_noise_sqrt = sqrt(gnss_ekf_noise);
	// ekfom_data.R_GNSS(0) = gnss_noise_sqrt > error_1? gnss_ekf_noise : error_1 * error_1;
	// ekfom_data.R_GNSS(1) = gnss_noise_sqrt > error_2? gnss_ekf_noise : error_2 * error_2;
	// ekfom_data.R_GNSS(2) = gnss_noise_sqrt > error_3? gnss_ekf_noise : error_3 * error_3;
	// ekfom_data.R_GNSS(3) = gnss_noise_sqrt > error_4? gnss_ekf_noise : error_4 * error_4;
	// ekfom_data.R_GNSS(4) = gnss_noise_sqrt > error_5? gnss_ekf_noise : error_5 * error_5;
	// ekfom_data.R_GNSS(5) = gnss_noise_sqrt > error_6? gnss_ekf_noise : error_6 * error_6;
	// ekfom_data.R_GNSS(0) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(1) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(2) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(3) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(4) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(5) = gnss_ekf_noise;
	// double max_err = error_1 > error_2? error_1 : error_2;
	// max_err = max_err > error_3? max_err : error_3;
	// GNSS 量测噪声，当前使用统一标量 gnss_ekf_noise。
	ekfom_data.M_Noise = gnss_ekf_noise; // > max_err? gnss_ekf_noise : max_err;
}

/**
 * @brief NMEA / RTK Odom 观测模型。
 *
 * 当前 avia_handsfree.yaml 中 gnss_enable=false、nmea_enable=true，
 * 因此实际运行主要走这个观测模型。
 *
 * NMEAProcess::processNMEA() 会先把 /rtk/gps_odom 等松耦合里程计观测
 * 转成 p_nmea->state_const_，本函数再构造 9 维残差：
 * - 位置残差 3 维；
 * - 姿态残差 3 维；
 * - 速度残差 3 维。
 *
 * @param s 当前滤波状态。
 * @param cov_p 当前位置协方差块，保留接口，当前主要逻辑中未直接使用。
 * @param cov_R 当前姿态协方差块，保留接口，当前主要逻辑中未直接使用。
 * @param ekfom_data 输出 NMEA 残差、雅可比和噪声。
 */
void h_model_NMEA_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	// 姿态残差：当前 ESKF 姿态到 NMEA/GNSS-Odom 约束姿态之间的相对旋转。
	Eigen::Matrix3d res_R = s.rot.transpose() * p_nmea->state_const_.rot;
	Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
	// h_NMEA 是 9x9，默认直接观测 pos / rot / vel。
	ekfom_data.h_NMEA.setIdentity();
	// ekfom_data.h_GNSS *= p_gnss->odo_weight;
	// ekfom_data.h_GNSS(0, 0) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(3, 3) = p_gnss->odo_weight4;
	// ekfom_data.h_GNSS(1, 1) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(4, 4) = p_gnss->odo_weight5;
	// ekfom_data.h_GNSS(0, 0) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(1, 1) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(2, 2) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// 姿态部分在 SO3 上线性化。
	ekfom_data.h_NMEA.block<3, 3>(3, 3) = Jacob_right_inv<double>(res_r); // 0.1 *
	// ekfom_data.h_GNSS.block<1, 3>(3, 3) *= p_gnss->odo_weight4;
	// ekfom_data.h_GNSS.block<1, 3>(4, 3) *= p_gnss->odo_weight5;
	// ekfom_data.h_GNSS.block<1, 3>(5, 3) *= p_gnss->odo_weight6;
	ekfom_data.z_NMEA.setZero();
	// ekfom_data.h_GNSS.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.h_GNSS.block<3, 3>(6, 6) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.z_GNSS.block<3, 1>(3, 0) = res_r;
	// ekfom_data.z_GNSS.block<3, 1>(0, 0) = p_gnss->odo_weight * (p_gnss->state_const_.pos - s.pos); // 
	// 位置残差。
	ekfom_data.z_NMEA.block<3, 1>(0, 0) = p_nmea->state_const_.pos - s.pos;
	// 速度残差。
	ekfom_data.z_NMEA.block<3, 1>(6, 0) = p_nmea->state_const_.vel - s.vel;
	// ekfom_data.z_GNSS(0) *= 1.5;
	// ekfom_data.z_GNSS(1) *= 1.5;
	// ekfom_data.z_GNSS(2) *= 1.5;
	// ekfom_data.z_GNSS(0) = p_gnss->odo_weight1 * (p_gnss->state_const_.pos(0) - s.pos(0)); // 
	// ekfom_data.z_GNSS(1) = p_gnss->odo_weight2 * (p_gnss->state_const_.pos(1) - s.pos(1)); // 
	// ekfom_data.z_GNSS(2) = p_gnss->odo_weight3 * (p_gnss->state_const_.pos(2) - s.pos(2)); // 
	// 姿态残差。
	ekfom_data.z_NMEA.block<3, 1>(3, 0) = res_r;
	// vect3 so3_deg = s.omg * s.time_diff(0);
	// Eigen::Matrix3d rot_const = MTK::SO3<double>::exp(so3_deg);
	// Eigen::Matrix3d res_R = rot_const.transpose() * s.rot.transpose() * p_nmea->state_const_.rot;
	// Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
	// ekfom_data.h_NMEA.setIdentity();
	// ekfom_data.h_NMEA.block<3, 3>(3, 3) = Jacob_right_inv<double>(res_r) * rot_const.transpose();
	// ekfom_data.h_NMEA.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 1>(0, 9) = s.vel + s.acc * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 3>(0, 13) = 0.5 * Eigen::Matrix3d::Identity() * s.time_diff(0) * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 1>(3, 9) = Jacob_right_inv<double>(res_r) * s.omg;
	// ekfom_data.h_NMEA.block<3, 3>(3, 10) = Jacob_right_inv<double>(res_r) * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 1>(6, 9) = s.acc;
	// ekfom_data.h_NMEA.block<3, 3>(6, 13) = Eigen::Matrix3d::Identity() * s.time_diff(0);
	// // ekfom_data.h_NMEA.block<3, 3>(6, 6) = Eigen::Matrix3d::Zero(); Jacob_right_inv<double>(res_r);
	// ekfom_data.z_NMEA.setZero();
	// ekfom_data.z_NMEA.block<3, 1>(0, 0) = p_nmea->state_const_.pos - s.pos - s.vel * s.time_diff - 0.5 * s.acc * s.time_diff * s.time_diff; // 
	// ekfom_data.z_NMEA.block<3, 1>(6, 0) = p_nmea->state_const_.vel - s.vel - s.acc * s.time_diff; // 
	// ekfom_data.z_NMEA.block<3, 1>(3, 0) = res_r; // s.rot.transpose() * p_gnss->state_.rot; //  
	// NMEA 量测噪声，当前复用 gnss_ekf_noise 这个参数。
	ekfom_data.M_Noise = gnss_ekf_noise;
}

/**
 * @brief 把 LiDAR/body 坐标系下的点变换到世界坐标系。
 *
 * h_model_output() 在构造 LiDAR 点到地图残差时会调用这个函数。
 * 变换链路：
 * 1. LiDAR 点先通过 LiDAR-IMU 外参变到 IMU/body 坐标系；
 * 2. 再通过当前滤波状态 kf_output.x_ 的 rot/pos 变到世界坐标系。
 *
 * 数学形式：
 * p_world = R_world_imu * (R_imu_lidar * p_lidar + t_imu_lidar) + t_world_imu
 *
 * @param pi 输入点，通常来自 feats_down_body。
 * @param po 输出点，写入 feats_down_world。
 */
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{    
    V3D p_body(pi->x, pi->y, pi->z);
    
    V3D p_global;
	{
		// if (!use_imu_as_input)
		{
			// 当前状态 kf_output.x_ 提供世界系下的 IMU/body 位姿。
			p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_output.x_.pos; // .normalized()
		}
	}

	// 只更新几何坐标和 intensity；curvature 在这里不再使用。
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
