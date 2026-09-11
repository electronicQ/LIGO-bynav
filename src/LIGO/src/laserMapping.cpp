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

// #include <so3_math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
// 🌟【新增这一行】引入PCL点云变换库
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "li_initialization.h"
#include <malloc.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "chi-square.h"
// #include <ros/console.h>


#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0; 

bool init_map = false, flg_first_scan = true;
std::vector<ObsPtr> gnss_cur;
nav_msgs::OdometryPtr nmea_cur;
Eigen::Vector3d first_pvt_anc, first_lla_anc;
Eigen::Vector3d first_pvt_used, first_lla_used;

bool  flg_reset = false, flg_exit = false;

// =========================
// 全局点云缓存与地图维护变量
// =========================

//保存当前 LiDAR 帧经过 p_imu->Process() 处理后的点云，后续用于下采样
//当前代码中名字叫“去畸变点云”，但 IMU_Processing.cpp 当前主要是复制点云，并没有完成完整运动去畸变
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());

//（遗留变量），实际使用的是feats_down_body
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());

//保存地图初始化阶段积累的点云，当前帧点云先通过当前位姿转换到世界坐标系，再不断追加到 init_feats_world
//达到 init_map_size 后，加入 ivox_ 地图
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

//（遗留变量）
std::deque<PointCloudXYZI::Ptr> depth_feats_world;

//这是 PCL 的体素降采样器。
pcl::VoxelGrid<PointType> downSizeFilterSurf;

V3D euler_cur;

//ros输出：
//1、保存整条轨迹 2、内部包含多个 geometry_msgs::PoseStamped 3、发布到 ROS 话题 /path
nav_msgs::Path path;
//保存当前 LIO 估计的位姿，发布到/aft_mapped_to_init，还会发布camera_init -> aft_mapped的TF
nav_msgs::Odometry odomAftMapped;
//保存当前时刻的一个位姿，它是 path 中单个轨迹点的临时消息，每次调用 publish_path() 时，先把当前滤波位姿写入它，再追加到 path
geometry_msgs::PoseStamped msg_body_pose;



// 收到 SIGINT 后只置退出标志，主循环在安全位置退出。
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

// 将 LiDAR 坐标系下的点通过外参转换到 IMU / body 坐标系。
void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

// 向增量式体素地图中加入当前帧的新点，更新局部地图。
void MapIncremental() {
    //一个存点云的vector容器，存储当前帧点云
    PointVector points_to_add;

    //当前帧点云在feats_down_world变量中，提取其数量
    int cur_pts = feats_down_world->size(); 

    //这是容器自带的函数，因为容器也是一个动态数组，这个函数用处的分配points_to_add容器cur_pts个元素的存储空间，但不创建存储空间内的对象
    points_to_add.reserve(cur_pts);
    
    //把
    for (size_t i = 0; i < cur_pts; ++i) {
        /* decide if need add to map */
        PointType &point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }
    ivox_->AddPoints(points_to_add);   
}

// 发布初始化阶段积累出来的第一批地图点云。
void publish_init_map(const ros::Publisher & pubLaserCloudFullRes)
{
    int size_init_map = init_feats_world->size();

    sensor_msgs::PointCloud2 laserCloudmsg;
                
    pcl::toROSMsg(*init_feats_world, laserCloudmsg);
        
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

// 发布当前帧在世界系下的点云，并可选写入 PCD 文件。
void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    // 当前帧点云已被估计位姿转换到世界坐标系。
    PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body); // (points_num); 
    int size = laserCloudFullRes->points.size();

    // 发布到 RViz 的世界系点云。
    if (scan_pub_en)
    {
   
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        
        for (int i = 0; i < size; i++)
        {
            // if (i % 3 == 0)
            {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x; // updatedmap[i / 3](0); // 
            laserCloudWorld->points[i].y = feats_down_world->points[i].y; // updatedmap[i / 3](1); // 
            laserCloudWorld->points[i].z = feats_down_world->points[i].z; // updatedmap[i / 3](2); // 
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; // updatedmap[i / 3](2); //feats_down_world->points[i].z; // 
            }
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time); // (map_time); 

        //告诉这帧sensor_msgs::PointCloud2格式的点云的坐标系为camera_init坐标系，也就是其实局部坐标系
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        // publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** save map ****************/
    /* 1. 确保内存足够
     * 2. 点云写盘会影响实时性，因此这里按间隔批量保存
     */
    if (pcd_save_en)
    {
        //修改，注释掉了
        //int size = points_num; // feats_down_world->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        //将一帧点云存储到laserCloudWorld这个地址下
        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x; // updatedmap[i](0); //
            laserCloudWorld->points[i].y = feats_down_world->points[i].y; // updatedmap[i](1); //
            laserCloudWorld->points[i].z = feats_down_world->points[i].z; // updatedmap[i](2); //
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // updatedmap[i](2); //
        }

        //把这一帧点云加入到pcl_wait_save这个存储点云指针的地址下。在建图过程中，地图是不断叠加的，之前地图的点云就不会
        //再改变了，因为这个程序也没有对之前的建图点进行优化，每次定位都是优化后的，在将定位处的点加入，不会出现对之前点云进行拉动的情况
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

// 发布当前帧在 body / IMU 坐标系下的点云，便于调试坐标变换。
void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
}

// 把当前状态写入 ROS 位姿消息。
template<typename T>
void set_posestamp(T & out)
{
    {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        Eigen::Quaterniond q(kf_output.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

// 发布当前估计位姿，并广播 camera_init -> aft_mapped 的 TF。
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(time_current);
    }
    else
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    
    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped" ) );
}

// 发布轨迹路径，供 RViz 查看整条运动轨迹。
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath.publish(path);
    }
}        

// main 入口：
// 1. 初始化 ROS 与后台回调线程
// 2. 读取参数、建立地图、注册滤波模型
// 3. 进入主循环，按时间顺序处理 LiDAR / IMU / GNSS / NMEA
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    setlocale(LC_ALL, ""); // 🌟 新增这行，让 C++ 终端支持中文输出
    ros::NodeHandle nh("~");

    // 启动后台回调线程池，负责消费 subscriber 回调。
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // 读取 launch / yaml 中的运行参数。
    readParameters(nh);
    cout<<"lidar_type: "<<lidar_type<<endl;

    // 初始化局部体素地图容器。
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    ivox_last_ = std::make_shared<IVoxType>(ivox_options_); //(*ivox_);
    
    path.header.stamp    = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id ="camera_init";

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    {
        // 读取 LiDAR 到 IMU 的外参。
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    }

    // 将参数同步到各处理模块。
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;
    if (GNSS_ENABLE)
    {
        std::copy(default_gnss_iono_params.begin(), default_gnss_iono_params.end(), 
            std::back_inserter(p_gnss->p_assign->latest_gnss_iono_params));
        p_gnss->Tex_imu_r << VEC_FROM_ARRAY(extrinT_gnss);
        p_gnss->gnss_ready = false; // gnss_quick_init; // edit
        p_gnss->nolidar = nolidar; // edit
        p_gnss->pre_integration->setnoise();

        if (p_gnss->p_assign->ephem_from_rinex)
        {
            p_gnss->p_assign->Ephemfromrinex(LOCAL_FILE_DIR(ephem_fname));
        }
    }
    else if (NMEA_ENABLE)
    {
        p_nmea->Tex_imu_r << VEC_FROM_ARRAY(extrinT_gnss);
        p_nmea->Rex_imu_r << MAT_FROM_ARRAY(extrinR_gnss);
        p_nmea->nmea_ready = false; // gnss_quick_init; // edit
        p_nmea->nolidar = nolidar; // edit
        p_nmea->pre_integration->setnoise();
    }
    if (NMEA_ENABLE)
    {
        kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output, h_model_NMEA_output);
    }
    else
    {
        kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output, h_model_GNSS_output);
    }
    Eigen::Matrix<double, 24, 24> P_init_output; // = MD(24, 24)::Identity() * 0.01;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_output = process_noise_cov_output();
    open_file();

    /*** ROS 订阅初始化：LiDAR / IMU / GNSS / NMEA ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    ros::Subscriber sub_ephem, sub_glo_ephem, sub_gnss_meas, sub_gnss_iono_params, sub_nmea_meas;
    ros::Subscriber sub_gnss_time_pluse_info, sub_local_trigger_info;
    ros::Subscriber sub_rtk_pvt_info, sub_rtk_lla_info;
    if (GNSS_ENABLE)
    {
        sub_ephem = nh.subscribe(gnss_ephem_topic, 10000, gnss_ephem_callback);
        sub_glo_ephem = nh.subscribe(gnss_glo_ephem_topic, 10000, gnss_glo_ephem_callback);
        if (p_gnss->p_assign->obs_from_rinex)
        {
            sub_gnss_meas = nh.subscribe("/gnss_preprocessor_node/GNSSPsrCarRov1", 200, gnss_meas_callback_urbannav);
            sub_rtk_pvt_info = nh.subscribe("/gnss_preprocessor_node/ECEFSolutionRTK", 500, rtklibOdomHandler); 
        }
        else
        {
            sub_gnss_meas = nh.subscribe(gnss_meas_topic, 10000, gnss_meas_callback);
            
            sub_rtk_lla_info = nh.subscribe(rtk_lla_topic, 1000, rtk_lla_callback);
        }

        if (p_gnss->p_assign->pvt_is_gt)
        {
            sub_rtk_pvt_info = nh.subscribe(rtk_pvt_topic, 1000, rtk_pvt_callback);
        }
        else
        {
            std::vector<Eigen::Vector4d> gt_holder;
            if (gt_file_type == LIVOX)
            {
                GtfromTXT_LIVOX(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            else if (gt_file_type == URBAN)
            {
                GtfromTXT_URBAN(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            else if (gt_file_type == M2DGR)
            {
                GtfromTXT_M2DGR(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            std::cout << "check gt size:" << gt_holder.size() << std::endl;
            if (gt_file_type == M2DGR)
            {
                for (size_t i = 0; i < gt_holder.size(); i++)
                {
                    inputpvt_ecef(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                            p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
                }
            }
            else
            {
                for (size_t i = 0; i < gt_holder.size(); i++)
                {
                    inputpvt_lla(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                            p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
                }
            }
        }
        sub_gnss_iono_params = nh.subscribe(gnss_iono_params_topic, 10000, gnss_iono_params_callback);

        if (gnss_local_online_sync)
        {
            sub_gnss_time_pluse_info = nh.subscribe(gnss_tp_info_topic, 100, 
                gnss_tp_info_callback);
            sub_local_trigger_info = nh.subscribe(local_trigger_info_topic, 100, 
                local_trigger_info_callback);
        }
        else
        {
            time_diff_gnss_local = gnss_local_time_diff; // 18.0
            p_gnss->inputGNSSTimeDiff(time_diff_gnss_local);
            time_diff_valid = true;
        }
    }
    else
    {
        if (!NMEA_ENABLE)
        {
            sub_rtk_pvt_info = nh.subscribe(rtk_pvt_topic, 100, rtk_pvt_callback);
        }
    }
    // #ifdef process_ppp
    // if (NMEA_ENABLE)
    // {
    //     std::vector<Eigen::Vector4d> gt_holder;
    //     // GtfromTXT_DJI(string("/home/joannahe/NewDisk/self-collected/gt_deg2.txt"), gt_urbannav_holder);
    //     // std::vector<Eigen::Vector4d>().swap(gt_urbannav_holder);
    //     // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_TST_GT_raw.txt"), gt_urbannav_holder);
    //     // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_whampoa_raw.txt"), gt_urbannav_holder);
    //     // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_mongkok_GT_part_raw.txt"), gt_urbannav_holder);
    //     // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_tunnel_GT_raw.txt"), gt_urbannav_holder);
    //     // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/tree3x.txt"), gt_urbannav_holder);
    //     // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/switch2_rawcut.txt"), gt_urbannav_holder);
    //     if (gt_file_type == LIVOX)
    //     {
    //         GtfromTXT_LIVOX(LOCAL_FILE_DIR(gt_fname), gt_holder);
    //     }
    //     else if (gt_file_type == URBAN)
    //     {
    //         GtfromTXT_URBAN(LOCAL_FILE_DIR(gt_fname), gt_holder);
    //     }
    //     else if (gt_file_type == M2DGR)
    //     {
    //         GtfromTXT_M2DGR(LOCAL_FILE_DIR(gt_fname), gt_holder);
    //     }
    //     // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/parking2.txt"), gt_urbannav_holder);
    //     // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/bridge2.txt"), gt_urbannav_holder);
    //     if (gt_file_type == M2DGR)
    //     {
    //         for (size_t i = 0; i < gt_holder.size(); i++)
    //         {
    //             inputpvt_ecef(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
    //                     p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
    //         }
    //     }
    //     else
    //     {
    //         for (size_t i = 0; i < gt_holder.size(); i++)
    //         {
    //             inputpvt_lla(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
    //                     p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
    //         }
    //     }
    // }
    
    // PPPfromTXT(LOCAL_FILE_DIR(ppp_fname), ppp_sol, ppp_ecef);
    // if (NMEA_ENABLE)
    // {

    //     first_pvt_anc = p_gnss->first_xyz_ecef_pvt;
    //     first_lla_anc = p_gnss->first_lla_pvt;
    //     if (p_gnss->p_assign->pvt_is_gt)
    //     {
    //         first_pvt_anc << VEC_FROM_ARRAY(ppp_anc);
    //         first_lla_anc = ecef2geo(first_pvt_anc);
    //     }
    //     // first_pvt_anc << 3959058.559396,-87615.730649,4983325.235812; //  -2152900.934855,4380649.467661,4091851.462459; // bri 
    //     // -2414309.951157,5388624.811131,2403467.959772; // main2 // -2169505.899002,4385241.855870,4078231.236204; // out // 
    //     first_pvt_used = ppp_ecef[0].segment<3>(1);
    //     first_lla_used = ecef2geo(first_pvt_used);
    //     for (int i = 0; i < ppp_sol.size(); i++)
    //     {   
    //         // Eigen::Vector3d ppp_enu = ecef2enu(p_gnss->first_lla_pvt, ppp_ecef[i].segment<3>(1) - p_gnss->first_xyz_ecef_pvt);
    //         // Eigen::Vector3d ppp_enu = ecef2enu(first_lla, ppp_ecef[i].segment<3>(1) - first_pvt);
    //         nav_msgs::Odometry gps_odom;
    //         gps_odom.header.stamp = ros::Time().fromSec(ppp_sol[i][0]);
    //         // gps_odom->header.frame_id = "map";
    //         gps_odom.pose.pose.position.x = ppp_sol[i][1]; //[1];
    //         gps_odom.pose.pose.position.y = ppp_sol[i][2]; //[2];
    //         gps_odom.pose.pose.position.z = ppp_sol[i][3]; //[3];
    //         gps_odom.pose.covariance[0] = ppp_sol[i][4]; //[4];
    //         gps_odom.pose.covariance[1] = ppp_sol[i][5]; //[4];
    //         gps_odom.pose.covariance[2] = ppp_sol[i][6]; //[4];
    //         nmea_meas_buf.push(nav_msgs::OdometryPtr(new nav_msgs::Odometry(gps_odom)));
    //     }
    // }
    // #endif

    //如果使用松耦合，使用NMEA数据，应该读取nmea_meas_topic这个话题的数据。nmea_meas_callback回调函数对应的直接接受Odometry数据，而gpsHandler接受的是navsatFix原始数据。
    if (NMEA_ENABLE)
    {
        sub_nmea_meas = nh.subscribe(nmea_meas_topic, 10000, nmea_meas_callback);
        // sub_nmea_meas = nh.subscribe(nmea_meas_topic, 10000, gpsHandler);
    }

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 1000);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 1000);
    ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 1000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 1000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/aft_mapped_to_init", 1000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 1000);
    ros::Publisher plane_pub = nh.advertise<visualization_msgs::Marker>
            ("/planner_normal", 1000);
    // ros::Publisher pub_gnss_lla = nh.advertise<sensor_msgs::NavSatFix>("gnss_fused_lla", 1000);
    
    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate loop_rate(500);
    bool status = ros::ok();

    // =========================
    // 主循环：按同步后的 LiDAR 帧驱动整套前端流程
    // =========================
    while (status)
    {
        if (flg_exit) break;
        // 可省略，因为回调已由后台线程池处理，这里只是顺手刷新一次队列。
        ros::spinOnce();

        // 从缓存中取出一组按时间对齐的数据包。
        if(sync_packages(Measures, p_gnss->gnss_msg, p_nmea->nmea_msg)) 
        {
            // =========================
            // 1. 异常复位：清空地图、滤波器和轨迹管理器
            // =========================
            if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                {
                    state_out = state_output();
                    kf_output.change_P(P_init_output);
                }
                is_first_gnss = true;
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                init_map = false;
                
                {
                    ivox_.reset(new IVoxType(ivox_options_));
                    ivox_last_.reset(new IVoxType(ivox_options_)); // = std::make_shared<IVoxType>(*ivox_);
                    traj_manager.reset(new curvefitter::TrajectoryManager<4>());
                    // while (!empty_voxels.empty())
                    // {
                        // std::unordered_set<Eigen::Matrix<int, 3, 1>, faster_lio::hash_vec<3>>().swap(empty_voxels[0]);
                        // empty_voxels.pop_front();
                    // }
                }
            }

            // =========================
            // 2. 第一帧处理：确定时间基准、重力方向、初始姿态
            // =========================
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1)
                {
                    first_imu_time = imu_next.header.stamp.toSec();
                    // printf("first imu time: %f acceleration: %f%f%f\n", first_imu_time, imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z);
                }
                time_current = 0.0;
                if(imu_en)
                {
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc *= -1; 

                    if (!nolidar && !imu_deque.empty())
                    {
                        while (Measures.lidar_beg_time > imu_next.header.stamp.toSec()) // if it is needed for the new map?
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty())
                            {
                                break;
                            }
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_deque.pop();
                        }
                    }
                }
                else
                {
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc *= -1; 
                    p_imu->imu_need_init_ = false;
                    // p_imu->after_imu_init_ = true;
                }  
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
                // if (GNSS_ENABLE)
                // {   
                //     // p_gnss->gnss_ready = true;
                //     // p_gnss->gtSAMgraphMade = true;
                //     set_gnss_offline_init(false);
                // }         
            }

            double t0, t5;
            t0 = omp_get_wtime();
            
            // =========================
            // 3. 当前帧预处理：LiDAR 去畸变后下采样并排序
            // =========================
            p_imu->Process(Measures, feats_undistort);
            if(space_down_sample)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            else
            {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            if (!nolidar)
            {
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }
            else
            {
                time_seq.clear();
            }
         
            if (!p_imu->after_imu_init_)
            {
                if (!p_imu->imu_need_init_)
                { 
                    V3D tmp_gravity;
                    if (init_with_imu && imu_en)
                    {
                        tmp_gravity = - p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    }
                    else
                    {   tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                        p_imu->after_imu_init_ = true;
                    }
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    // p_gnss->Rot_gnss_init = rot_init;  
                    kf_output.x_.rot = rot_init;
                    // kf_output.x_.rot; //.normalize();
                    kf_output.x_.acc = - rot_init.transpose() * kf_output.x_.gravity;
                }
                else{
                continue;}
            }
            // =========================
            // 4. 地图初始化：先积累一批点云，再开始正式匹配
            // =========================
            if(!init_map && !nolidar && !lose_lid)
            {
                feats_down_world->resize(feats_undistort->size());
                for(int i = 0; i < feats_undistort->size(); i++)
                {
                    {
                        pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
                    }
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) 
                {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if(init_feats_world->size() < init_map_size) 
                {init_map = false;}
                else
                {   
                    ivox_->AddPoints(init_feats_world->points);
                    // 
                    publish_init_map(pubLaserCloudMap); //(pubLaserCloudFullRes);
                    
                    init_feats_world.reset(new PointCloudXYZI());
                    init_map = true;
                    if (GNSS_ENABLE || NMEA_ENABLE) traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                }
                continue;
            }

            // =========================
            // 5. 正式前端匹配：局部地图配准 + 迭代状态估计
            // =========================
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);
            // t2 = omp_get_wtime();
            
            // 为后续残差和雅可比计算预先缓存点坐标。
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);
            pimu_list.reserve(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);
                          
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    pimu_list[i] = point_this;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i]=point_crossmat;
            }
            {     
                effct_feat_num = 0;
                // =========================
                // 5.1 点级迭代：按点时间戳边预测边更新
                // =========================
                if (time_seq.size() > 0) // || (!GNSS_ENABLE && !NMEA_ENABLE) )
                {
                    if (GNSS_ENABLE)  
                    {p_gnss->p_assign->process_feat_num += time_seq.size();
                    p_gnss->nolidar_cur = false;}
                    if (NMEA_ENABLE)  
                    {p_nmea->p_assign->process_feat_num += time_seq.size();
                    p_nmea->nolidar_cur = false;}
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;

                // 遍历当前帧中的每个时间段，逐点推进状态。
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (time_current < time_predict_last_const)
                    {
                        continue;
                    }

                    if (is_first_frame)
                    {
                        if(imu_en && !imu_deque.empty())
                        {
                            while (time_current > imu_next.header.stamp.toSec())
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            if (imu_deque.empty()) break;
                        }
                        if (GNSS_ENABLE)
                        {
                            // std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            p_gnss->p_assign->process_feat_num = 0;
                            p_gnss->norm_vec_num = 0;
                            // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                            // p_gnss->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            // p_gnss->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr);
                        }
                        if (NMEA_ENABLE)
                        {
                            p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;
                        }
                        is_first_frame = false;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if(imu_en && !imu_deque.empty())
                    {
                        // 5.1.1 先用 IMU 把状态推进到当前点时刻附近。
                        bool last_imu = imu_next.header.stamp.toSec() == imu_deque.front()->header.stamp.toSec();
                        while (imu_next.header.stamp.toSec() < time_predict_last_const && !imu_deque.empty())
                        {
                            if (!last_imu)
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                break;
                            }
                            else
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            if (imu_deque.empty()) break;
                        }
                        bool imu_comes = time_current >= imu_next.header.stamp.toSec();
                        while (imu_comes) 
                        {
                            // 5.1.2 若中间穿插 GNSS 观测，先推进到 GNSS 时刻，再做 GNSS 更新。
                            if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                            {   
                                gnss_cur = p_gnss->gnss_msg.front();
                                // printf("%f, %f, %f\n", time2sec(gnss_cur[0]->time), time_diff_gnss_local, time_predict_last_const);
                                while (time2sec(gnss_cur[0]->time) - time_diff_gnss_local < time_predict_last_const)
                                {
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                if (p_gnss->gnss_msg.empty()) break;
                                while ((imu_next.header.stamp.toSec() >= time2sec(gnss_cur[0]->time) - time_diff_gnss_local) && (time2sec(gnss_cur[0]->time) - time_diff_gnss_local >= time_predict_last_const))
                                {
                                    double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                                    double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;

                                    if (p_gnss->gnss_ready)
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        kf_output.predict(dt, Q_output, input_in, true, false);
                                        // p_gnss->pre_integration->push_back(dt, kf_output.x_.acc + kf_output.x_.ba, kf_output.x_.omg + kf_output.x_.bg); // acc_avr, angvel_avr); 
                                        // p_gnss->processIMUOutput(dt, kf_output.x_.acc, kf_output.x_.omg);
                                        time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        time_update_last = time_predict_last_const;
                                        p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                        p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                        // p_gnss->sqrt_lidar *= 0.002;
                                        update_gnss = p_gnss->Evaluate(kf_output.x_);
                                        if (!p_gnss->gnss_ready)
                                        {
                                            flg_reset = true;
                                            p_gnss->gnss_msg.pop();
                                            if(!p_gnss->gnss_msg.empty())
                                            {
                                                gnss_cur = p_gnss->gnss_msg.front();
                                            }
                                            break; // ?
                                        }

                                        if (update_gnss)
                                        {
                                            state_output out_state = kf_output.x_;
                                            kf_output.update_iterated_dyn_share_GNSS();
                                            Eigen::Vector3d pos_enu;
                                            if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                            // sensor_msgs::NavSatFix gnss_lla_msg;
                                            // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                            // gnss_lla_msg.header.frame_id = "camera_init";
                                            // gnss_lla_msg.latitude = pos_enu(0);
                                            // gnss_lla_msg.longitude = pos_enu(1);
                                            // gnss_lla_msg.altitude = pos_enu(2);
                                            // pub_gnss_lla.publish(gnss_lla_msg);
                                            if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                            {                                                
                                                curvefitter::PoseData pose_data;
                                                pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                                map_time = pose_data.timestamp;
                                                pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                                pose_data.position = kf_output.x_.pos;
                                                if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                                {
                                                    pose_time_vector.push_back(pose_data.timestamp);
                                                    pose_graph_key_pose.emplace_back(pose_data);
                                                }
                                                else
                                                // else if (map_time == pose_time_vector.back())
                                                {
                                                    pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                                    pose_graph_key_pose.back() = pose_data;
                                                }
                                                // curvefitter::Trajectory<4> traj(0.1);
                                                // std::shared_ptr<curvefitter::Trajectory<4> > Traj_ptr = std::make_shared<curvefitter::Trajectory<4> >(traj);  
                                                traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                                traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                                updatedmap.resize(points_num);
                                                updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                                ivox_last_->AddPoints(updatedmap);
                                                ivox_->grids_map_ = ivox_last_->grids_map_;
                                                // for (auto &t : ivox_last_->grids_map_)
                                                // {
                                                    // ivox_->grids_map_[t.first] = (t.second);
                                                // }
                                                // ivox_ = std::make_shared<IVoxType>(*ivox_last_);
                                            }
                                            else
                                            {
                                                ivox_last_->grids_map_ = ivox_->grids_map_;
                                            }
                                            // reset_cov_output(kf_output.P_);
                                            traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                        }
                                    }
                                    else
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        
                                        kf_output.predict(dt, Q_output, input_in, true, false);

                                        time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        time_update_last = time_predict_last_const;
                                        state_out = kf_output.x_;
                                        // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                        // state_out.rot.normalize();
                                        // state_out.pos = state_out.pos;
                                        // state_out.vel = state_out.vel;
                                        p_gnss->processGNSS(gnss_cur, state_out);
                                        if (p_gnss->gnss_ready)
                                        {
                                            // printf("time gnss ready: %f \n", time_predict_last_const);
                                            Eigen::Vector3d pos_enu;
                                            if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                            // sensor_msgs::NavSatFix gnss_lla_msg;
                                            // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                            // gnss_lla_msg.header.frame_id = "camera_init";
                                            // gnss_lla_msg.latitude = pos_enu(0);
                                            // gnss_lla_msg.longitude = pos_enu(1);
                                            // gnss_lla_msg.altitude = pos_enu(2);
                                            // pub_gnss_lla.publish(gnss_lla_msg);
                                        }
                                    }
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                            }
                            // 5.1.3 若中间穿插 NMEA 观测，同样先推进再更新。
                            if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                            {
                                //新增
                                ROS_INFO("[DIAG] 进入 NMEA 处理, nmea_msg.size=%zu, nmea_ready=%d, frame_count=%d",
                                        p_nmea->nmea_msg.size(), p_nmea->nmea_ready, p_nmea->frame_count);
                                nmea_cur = p_nmea->nmea_msg.front();

                                //新增
                                ROS_INFO("[DIAG] 第1关: nmea_ts=%.3f time_predict_last=%.3f imu_next_ts=%.3f nmea_msg.size=%zu",
                                nmea_cur->header.stamp.toSec(), time_predict_last_const, 
                                imu_next.header.stamp.toSec(), p_nmea->nmea_msg.size());

                                while (nmea_cur->header.stamp.toSec() - time_diff_nmea_local < time_predict_last_const)
                                {
                                    //新增
                                    ROS_WARN("[DIAG] 第1关拦截! NMEA丢弃 ts=%.3f < predict_last=%.3f", 
                                    nmea_cur->header.stamp.toSec(), time_predict_last_const);

                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        // 新增
                                        ROS_WARN("[DIAG] nmea_msg 在第1关全部被丢弃!"); 
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                if (p_nmea->nmea_msg.empty()) break;
                               
                                // 新增
                                ROS_INFO("[DIAG] 第2关检查: imu_next=%.3f >= nmea_ts=%.3f ? %d, nmea_ts=%.3f >= predict_last=%.3f ? %d",
                                imu_next.header.stamp.toSec(), 
                                nmea_cur->header.stamp.toSec() - time_diff_nmea_local,
                                (imu_next.header.stamp.toSec() >= nmea_cur->header.stamp.toSec() - time_diff_nmea_local),
                                nmea_cur->header.stamp.toSec() - time_diff_nmea_local,
                                time_predict_last_const,
                                (nmea_cur->header.stamp.toSec() - time_diff_nmea_local >= time_predict_last_const));
                                
                                
                                while ((imu_next.header.stamp.toSec() >= nmea_cur->header.stamp.toSec() - time_diff_nmea_local) && (nmea_cur->header.stamp.toSec() - time_diff_nmea_local >= time_predict_last_const))
                                {
                                    double dt = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_predict_last_const;
                                    double dt_cov = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_update_last;

                                    if (p_nmea->nmea_ready)
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        kf_output.predict(dt, Q_output, input_in, true, false);
                                        time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                        time_update_last = time_predict_last_const;
                                        p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                        p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                        // p_gnss->sqrt_lidar *= 0.002;
                                        update_nmea = p_nmea->Evaluate(kf_output.x_);
                                        if (!p_nmea->nmea_ready)
                                        {
                                            flg_reset = true;
                                            p_nmea->nmea_msg.pop();
                                            if(!p_nmea->nmea_msg.empty())
                                            {
                                                nmea_cur = p_nmea->nmea_msg.front();
                                            }
                                            break; // ?
                                        }

                                        if (update_nmea)
                                        {
                                            // 【添加打印】对比融合 NMEA 前后的位置变化
                                            // 【添加打印】对比融合 NMEA 前后的位置变化
                                            Eigen::Vector3d pos_before = kf_output.x_.pos;
                                            
                                            kf_output.update_iterated_dyn_share_NMEA(); // 执行更新
                                            
                                            Eigen::Vector3d pos_after = kf_output.x_.pos;
                                            double pos_diff = (pos_after - pos_before).norm();
                                            
                                            if (pos_diff > 0.01) {
                                                std::cout << "[ESKF Update] NMEA 数据修正了当前位姿! 修正量: " << pos_diff << " 米." << std::endl;
                                            }
                                            if (!runtime_pos_log) cout_state_to_file_nmea();
                                        }
                                    }
                                    else
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        
                                        kf_output.predict(dt, Q_output, input_in, true, false);

                                        time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                        time_update_last = time_predict_last_const;
                                        state_out = kf_output.x_;
                                        // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                        // state_out.pos = state_out.pos;
                                        // state_out.vel = state_out.vel;
                                        p_nmea->processNMEA(nmea_cur, state_out);
                                    }
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                            }
                            if (flg_reset)
                            {
                                break;
                            }
                            // 5.1.4 到达当前 IMU 时刻后，执行一次 IMU 预测 + IMU 更新。
                            angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            double dt = imu_next.header.stamp.toSec() - time_predict_last_const;
                            time_predict_last_const = imu_next.header.stamp.toSec(); 
                            double dt_cov = imu_next.header.stamp.toSec() - time_update_last; 

                            if (dt_cov > 0.0)
                            {
                                time_update_last = imu_next.header.stamp.toSec();

                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            }
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            kf_output.update_iterated_dyn_share_IMU();
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_comes = time_current >= imu_next.header.stamp.toSec();
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }
                    // 5.1.5 当前点处理完后，再检查是否还有 GNSS 观测需要补入。
                    if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                    {
                        gnss_cur = p_gnss->gnss_msg.front();
                        // printf("%f, %f, %f\n", time2sec(gnss_cur[0]->time), time_diff_gnss_local, time_predict_last_const);
                        while ( time2sec(gnss_cur[0]->time) - time_diff_gnss_local < time_predict_last_const)
                        {
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (p_gnss->gnss_msg.empty()) break;
                        while (time_current >= time2sec(gnss_cur[0]->time) - time_diff_gnss_local && time2sec(gnss_cur[0]->time) - time_diff_gnss_local >= time_predict_last_const)
                        {
                            double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                            double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;
                            // cout << "check gnss ready:" << p_gnss->gnss_ready << endl;
                            if (p_gnss->gnss_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);

                                // p_gnss->pre_integration->push_back(dt, kf_output.x_.acc + kf_output.x_.ba, kf_output.x_.omg + kf_output.x_.bg); // acc_avr, angvel_avr); 
                                // p_gnss->processIMUOutput(dt, kf_output.x_.acc, kf_output.x_.omg);

                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                time_update_last = time_predict_last_const;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                // p_gnss->sqrt_lidar *= 0.002;
                                update_gnss = p_gnss->Evaluate(kf_output.x_);
                                if (!p_gnss->gnss_ready)
                                {
                                    flg_reset = true;
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    break; // ?
                                }

                                if (update_gnss)
                                {
                                    state_output out_state = kf_output.x_;
                                    kf_output.update_iterated_dyn_share_GNSS();
                                    // reset_cov_output(kf_output.P_);
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                    if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                    {                                         
                                        curvefitter::PoseData pose_data;
                                        pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        map_time = pose_data.timestamp;
                                        // pose_time_vector.push_back(pose_data.timestamp);
                                        pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                        pose_data.position = kf_output.x_.pos;
                                        if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                        {
                                            pose_time_vector.push_back(pose_data.timestamp);
                                            pose_graph_key_pose.emplace_back(pose_data);
                                        }
                                        else
                                        // else if (map_time == pose_time_vector.back())
                                        {
                                            pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                            pose_graph_key_pose.back() = pose_data;
                                        }
                                        // pose_graph_key_pose.emplace_back(pose_data);
                                        traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                        traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                        updatedmap.resize(points_num);
                                        updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                        ivox_last_->AddPoints(updatedmap);
                                        ivox_->grids_map_ = ivox_last_->grids_map_;
                                    }
                                    else
                                    {
                                        ivox_last_->grids_map_ = ivox_->grids_map_;
                                    }
                                    traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                time_update_last = time_predict_last_const;
                                state_out = kf_output.x_;
                                // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                // state_out.rot.normalize();
                                // state_out.pos = state_out.pos;
                                // state_out.vel = state_out.vel;
                                p_gnss->processGNSS(gnss_cur, state_out);
                                if (p_gnss->gnss_ready)
                                {
                                    // printf("time gnss ready: %f \n", time_predict_last_const);
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                }
                            }
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    // 5.1.6 当前点处理完后，再检查是否还有 NMEA 观测需要补入。
                    if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                    {
                        nmea_cur = p_nmea->nmea_msg.front();
                        while ( nmea_cur->header.stamp.toSec() - time_diff_nmea_local < time_predict_last_const)
                        {
                            p_nmea->nmea_msg.pop();
                            if(!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (p_nmea->nmea_msg.empty()) break;
                        while (time_current >= nmea_cur->header.stamp.toSec() - time_diff_nmea_local && nmea_cur->header.stamp.toSec() - time_diff_nmea_local >= time_predict_last_const)
                        {
                            double dt = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_predict_last_const;
                            double dt_cov = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_update_last;
                            if (p_nmea->nmea_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);

                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                time_update_last = time_predict_last_const;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                // p_gnss->sqrt_lidar *= 0.002;
                                update_nmea = p_nmea->Evaluate(kf_output.x_);
                                if (!p_nmea->nmea_ready)
                                {
                                    flg_reset = true;
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    break; // ?
                                }

                                if (update_nmea)
                                {
                                    // 【添加打印】对比融合 NMEA 前后的位置变化
                                    Eigen::Vector3d pos_before = kf_output.x_.pos;
                                    
                                    kf_output.update_iterated_dyn_share_NMEA(); // 执行更新
                                    
                                    Eigen::Vector3d pos_after = kf_output.x_.pos;
                                    double pos_diff = (pos_after - pos_before).norm();
                                    
                                    if (pos_diff > 0.01) {
                                        std::cout << "[ESKF Update] NMEA 数据修正了当前位姿! 修正量: " << pos_diff << " 米." << std::endl;
                                    }
                                    if (!runtime_pos_log) cout_state_to_file_nmea();
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                time_update_last = time_predict_last_const;
                                state_out = kf_output.x_;
                                // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                // state_out.pos = state_out.pos;
                                // state_out.vel = state_out.vel;
                                p_nmea->processNMEA(nmea_cur, state_out);
                            }
                            p_nmea->nmea_msg.pop();
                            if(!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }
                    // 5.1.7 用当前点时刻做一次 LiDAR 匹配更新。
                    double dt = time_current - time_predict_last_const;
                    // double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;   
                        }
                    }
                    // if (dt > 0.0)
                    {
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    time_predict_last_const = time_current;
                    if (feats_down_size < 1)
                    {
                        ROS_WARN("No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) 
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    }
                    // else
                    // {
                    //     idx = idx+time_seq[k];
                    //     continue;
                    // }
                    
                    // solve_start = omp_get_wtime();
                        
                    if (publish_odometry_without_downsample)
                    {
                        // 6. 发布当前估计位姿，给 RViz 和外部模块看。

                        publish_odometry(pubOdomAftMapped);
                        if (runtime_pos_log)
                        {
                            euler_cur = SO3ToEuler(kf_output.x_.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << kf_output.x_.pos.transpose() << " " << euler_cur.transpose() << " " << kf_output.x_.vel.transpose() \
                            <<" "<<kf_output.x_.omg.transpose()<<" "<<kf_output.x_.acc.transpose()<<" "<<kf_output.x_.gravity.transpose()<<" "<<kf_output.x_.bg.transpose()<<" "<<kf_output.x_.ba.transpose() << endl;
                        }
                    }
                    std::vector<Eigen::Vector3d> lidarpoints;
                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    if (GNSS_ENABLE || NMEA_ENABLE)
                        lidarpoints.push_back(pimu_list[idx+j+1]); // (Eigen::Vector3d(point_body_j.x, point_body_j.y, point_body_j.z));
                    }
                    if (GNSS_ENABLE || NMEA_ENABLE)
                    {
                        if (pose_graph_key_pose.empty()){
                            traj_manager->AddGraphPose(Eigen::Quaterniond(kf_output.x_.rot).normalized(), kf_output.x_.pos, lidarpoints, time_current, pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                        }else
                        {
                            if (time_current > pose_graph_key_pose.back().timestamp && lidarpoints.size() > 0)
                                traj_manager->AddGraphPose(Eigen::Quaterniond(kf_output.x_.rot).normalized(), kf_output.x_.pos, lidarpoints, time_current, pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                        }
                    }
                    idx += time_seq[k];
                }
                }
                else
                {
                    if (GNSS_ENABLE)  p_gnss->nolidar_cur = true;
                    if (NMEA_ENABLE)  p_nmea->nolidar_cur = true;
                    if (!imu_deque.empty())
                    { 
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                    while (imu_next.header.stamp.toSec() > time_current && ((imu_next.header.stamp.toSec() < imu_first_time + lidar_time_inte && nolidar) || (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte && !nolidar)))
                    { // >= ?
                        if (is_first_frame)
                        {
                            if (!nolidar && GNSS_ENABLE) //std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            {p_gnss->p_assign->process_feat_num = 0;
                            p_gnss->norm_vec_num = 0;}
                            if (!nolidar && NMEA_ENABLE) //std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            {p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;}

                            if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                                double front_gnss_ts = time2sec(gnss_cur[0]->time); // take time
                                time_current = front_gnss_ts - time_diff_gnss_local;
                                while (imu_next.header.stamp.toSec() < time_current) // 0.05
                                {
                                    ROS_WARN("throw IMU, only should happen at the beginning 2510");
                                    imu_deque.pop_front();
                                    if (imu_deque.empty()) break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front()); // could be used to initialize
                                }
                                if (imu_deque.empty()) break;
                            }
                            else if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                                double front_nmea_ts = nmea_cur->header.stamp.toSec(); // take time
                                time_current = front_nmea_ts - time_diff_nmea_local;
                                while (imu_next.header.stamp.toSec() < time_current) // 0.05
                                {
                                    ROS_WARN("throw IMU, only should happen at the beginning 2510");
                                    imu_deque.pop_front();
                                    if (imu_deque.empty()) break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front()); // could be used to initialize
                                }
                                if (imu_deque.empty()) break;
                            }
                            else
                            {
                                if (nolidar)
                                {
                                    while (imu_next.header.stamp.toSec() < imu_first_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front()); // could be used to initialize
                                    }
                                    // if (imu_deque.empty()) break;
                                }
                                else
                                {
                                    while (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front());
                                    }
                                }
                                break;
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            if (nolidar) kf_output.x_.omg = angvel_avr;
                                            
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            time_current = imu_next.header.stamp.toSec();

                            time_update_last = time_current;
                            time_predict_last_const = time_current;
                            acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                            if (GNSS_ENABLE)
                            {
                            p_gnss->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            p_gnss->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr); 
                            }
                            if (NMEA_ENABLE)
                            {
                            p_nmea->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            p_nmea->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr); 
                            }

                            {
                                is_first_frame = false;
                            }
                        }
                        time_current = imu_next.header.stamp.toSec();

                        if (!is_first_frame)
                        {
                        if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                        {
                            gnss_cur = p_gnss->gnss_msg.front();
                            while (time2sec(gnss_cur[0]->time) - time_diff_gnss_local <= time_predict_last_const)
                            {
                                p_gnss->gnss_msg.pop();
                                if(!p_gnss->gnss_msg.empty())
                                {
                                    gnss_cur = p_gnss->gnss_msg.front();
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (p_gnss->gnss_msg.empty()) break;
                        while ((time_current > time2sec(gnss_cur[0]->time) - time_diff_gnss_local) && (time2sec(gnss_cur[0]->time) - time_diff_gnss_local > time_predict_last_const))
                        {
                            double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                            double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;

                            if (p_gnss->gnss_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                p_gnss->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); //acc_avr_norm, angvel_avr); 
                                // change to state_const.omg and state_const.acc? 
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                if (!nolidar)
                                {
                                    p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                }
                                update_gnss = p_gnss->Evaluate(kf_output.x_); 
                                if (!p_gnss->gnss_ready)
                                {
                                    flg_reset = true;
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    break; // ?
                                }
                                if (update_gnss)
                                {
                                    if (!nolidar)
                                    {
                                        state_output out_state = kf_output.x_;
                                        kf_output.update_iterated_dyn_share_GNSS();
                                        // reset_cov_output(kf_output.P_);
                                        if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                        {                                    
                                            curvefitter::PoseData pose_data;
                                            pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                            map_time = pose_data.timestamp;
                                            // pose_time_vector.push_back(pose_data.timestamp);
                                            pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                            pose_data.position = kf_output.x_.pos;
                                            if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                            {
                                                pose_time_vector.push_back(pose_data.timestamp);
                                                pose_graph_key_pose.emplace_back(pose_data);
                                            }
                                            // else if (map_time == pose_time_vector.back())
                                            else
                                            {
                                                pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                                pose_graph_key_pose.back() = pose_data;
                                            }
                                            // pose_graph_key_pose.emplace_back(pose_data);
                                            // curvefitter::Trajectory<4> traj(0.1);
                                            // std::shared_ptr<curvefitter::Trajectory<4> > Traj_ptr = std::make_shared<curvefitter::Trajectory<4> >(traj);  
                                            traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                            traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                            updatedmap.resize(points_num);
                                            updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                            ivox_last_->AddPoints(updatedmap);
                                            ivox_->grids_map_ = ivox_last_->grids_map_;
                                            // for (auto &t : ivox_last_->grids_map_)
                                            // {
                                                // (ivox_->grids_map_[t.first]) = (t.second);
                                            // }
                                            // ivox_ = std::make_shared<IVoxType>(*ivox_last_);
                                        }
                                        else
                                        {
                                            ivox_last_->grids_map_ = ivox_->grids_map_;
                                        }
                                        traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                    }
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                if (p_gnss->gnss_ready)
                                {
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // printf("time gnss ready: %f \n", time_predict_last_const);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                    if (nolidar)
                                    {
                                        // Eigen::Matrix3d R_enu_local_;
                                        // R_enu_local_ = Eigen::AngleAxisd(p_gnss->yaw_enu_local, Eigen::Vector3d::UnitZ());
                                        kf_output.x_.pos = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_gnss->frame_num-1)).segment<3>(0); // p_gnss->anc_ecef - p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end * p_gnss->Tex_imu_r;
                                        kf_output.x_.rot = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_gnss->frame_num-1)).matrix(); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end;
                                        // kf_output.x_.rot.normalize();
                                        kf_output.x_.vel = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_gnss->frame_num-1)).segment<3>(3); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.vel_end; // Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.ba = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.bg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.omg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.gravity = p_gnss->R_ecef_enu * kf_output.x_.gravity; // * R_enu_local_ 
                                        kf_output.x_.acc = kf_output.x_.rot.transpose() * (-kf_output.x_.gravity); // R_ecef_enu * state.vel_end;.conjugate().normalized()
                                        
                                        kf_output.P_ = MD(24,24)::Identity() * INIT_COV;
                                    }
                                }
                            }
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        }
                        if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                        {
                            nmea_cur = p_nmea->nmea_msg.front();
                            while ( nmea_cur->header.stamp.toSec() - time_diff_nmea_local < time_predict_last_const)
                            {
                                p_nmea->nmea_msg.pop();
                                if(!p_nmea->nmea_msg.empty())
                                {
                                    nmea_cur = p_nmea->nmea_msg.front();
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (p_nmea->nmea_msg.empty()) break;
                        while ((time_current > nmea_cur->header.stamp.toSec() - time_diff_nmea_local) && (nmea_cur->header.stamp.toSec() - time_diff_nmea_local >= time_predict_last_const))
                        {
                            double dt = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_predict_last_const;
                            double dt_cov = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_update_last;

                            if (p_nmea->nmea_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                p_nmea->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); //acc_avr_norm, angvel_avr); 
                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                if (!nolidar)
                                {
                                    p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                }
                                update_nmea = p_nmea->Evaluate(kf_output.x_); 
                                if (!p_nmea->nmea_ready)
                                {
                                    flg_reset = true;
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    break; // ?
                                }
                                if (update_nmea)
                                {
                                    if (!nolidar)
                                    {
                                        // 【添加打印】对比融合 NMEA 前后的位置变化
                                        Eigen::Vector3d pos_before = kf_output.x_.pos;
                                        
                                        kf_output.update_iterated_dyn_share_NMEA(); // 执行更新
                                        
                                        Eigen::Vector3d pos_after = kf_output.x_.pos;
                                        double pos_diff = (pos_after - pos_before).norm();
                                        
                                        if (pos_diff > 0.01) {
                                            std::cout << "[ESKF Update] NMEA 数据修正了当前位姿! 修正量: " << pos_diff << " 米." << std::endl;
                                        }
                                            // reset_cov_output(kf_output.P_);
                                        }
                                        
                                        if (!runtime_pos_log) cout_state_to_file_nmea();
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                if (p_nmea->nmea_ready)
                                {
                                    if (nolidar)
                                    {
                                        Eigen::Matrix3d R_enu_local;
                                        R_enu_local = Eigen::AngleAxisd(p_nmea->yaw_enu_local, Eigen::Vector3d::UnitZ()); 
                                        kf_output.x_.pos = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(0); // p_gnss->anc_ecef - p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end * p_gnss->Tex_imu_r;
                                        kf_output.x_.rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_nmea->frame_num-1)).matrix(); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end;
                                        kf_output.x_.vel = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(3); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.vel_end; // Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.ba = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.bg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.omg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.gravity = R_enu_local * kf_output.x_.gravity; // * R_enu_local_ 
                                        kf_output.x_.acc = kf_output.x_.rot.transpose() * (-kf_output.x_.gravity); // R_ecef_enu * state.vel_end;.conjugate().normalized()
                                        
                                        kf_output.P_ = MD(24,24)::Identity() * INIT_COV;
                                    }
                                }
                            }
                            p_nmea->nmea_msg.pop();
                            if(!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        }
                        if (flg_reset)
                        {
                            break;
                        }
                        double dt = time_current - time_predict_last_const;
                        {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0)
                            {
                                // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                            // kf_output.predict(dt, Q_output, input_in, true, false);
                            if (GNSS_ENABLE)   p_gnss->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); // acc_avr_norm, angvel_avr); // 
                            if (NMEA_ENABLE)   p_nmea->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); // acc_avr_norm, angvel_avr); // 
                        }

                        time_predict_last_const = time_current;

                        angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        if (nolidar) kf_output.x_.omg = angvel_avr;
                        acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z; 
                        acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                        kf_output.update_iterated_dyn_share_IMU();
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    else
                    {
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    }
                    }
                }
            }
            
            // 7. 当前帧处理结束后，再统一发布一次里程计结果。
            if (!publish_odometry_without_downsample)
            {
                publish_odometry(pubOdomAftMapped);
            }

            // 把本帧有效特征点加入局部地图。
            if(feats_down_size > 4)
            {
                MapIncremental();
            }

            t5 = omp_get_wtime();
            // 发布轨迹与点云结果，供 RViz / 记录文件使用。
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
            
            // 记录运行耗时与调试日志。
            if (runtime_pos_log)
            {
                frame_num ++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                s_plot[time_log_counter] = t5 - t0;
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter ++;
                if (!publish_odometry_without_downsample)
                {
                    {
                        {
                            Eigen::Matrix3d R_enu_local_;
                            Eigen::Vector3d pos_r = kf_output.x_.rot * p_gnss->Tex_imu_r + kf_output.x_.pos; // .normalized()
                            time_frame.push_back(lidar_end_time); //(time_predict_last_const);
                            est_poses.push_back(pos_r);
                        }
                        euler_cur = SO3ToEuler(kf_output.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << kf_output.x_.pos.transpose() << " " << euler_cur.transpose() << " " << kf_output.x_.vel.transpose() \
                        <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose() << endl;
                    }
                }
            }
        }
        status = ros::ok();
        loop_rate.sleep();
    }
    fout_out.close();



    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
        bool transform_success = false;

        // 核心：尝试从松耦合的因子图中提取 LIO 局部到 全局 ENU 的变换
        if (NMEA_ENABLE && p_nmea->p_assign->isamCurrentEstimate.size() > 0)
        {
            try {
                // LIGO 后端中，P(0) 记录了旋转，E(0) 记录了平移锚点
                Eigen::Matrix3d enu_rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
                Eigen::Vector3d anc_cur = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
                
                final_transform.block<3, 3>(0, 0) = enu_rot;
                final_transform.block<3, 1>(0, 3) = anc_cur;
                transform_success = true;

                cout << "\n=============================================" << endl;
                cout << "[Map Saving] 成功获取全局变换矩阵!" << endl;
                cout << "Rotation:\n" << enu_rot << endl;
                cout << "Translation:\n" << anc_cur.transpose() << endl;
                cout << "=============================================\n" << endl;
            } catch (std::exception& e) {
                ROS_WARN("因子图提取 P(0)/E(0) 失败。可能是 GNSS 数据不足，放弃全局对齐。");
            }
        } else {
            ROS_WARN("NMEA_ENABLE 未开启或因子图为空，放弃全局对齐。");
        }

        string file_name = transform_success ? string("scans_Global_ENU.pcd") : string("scans_Local.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        
        pcl::PCDWriter pcd_writer;

        if (transform_success) {
            // 新建一个点云用于存放转换后的结果
            PointCloudXYZI::Ptr pcl_enu_save(new PointCloudXYZI());
            // 执行矩阵乘法，将局部点云一把挪到你 Python 脚本指定的 Datum 坐标系下
            pcl::transformPointCloud(*pcl_wait_save, *pcl_enu_save, final_transform);
            pcd_writer.writeBinary(all_points_dir, *pcl_enu_save);
            cout << " 全局 ENU 地图已完美保存至: " << all_points_dir << endl;
        } else {
            // 如果没收到卫星信号，降级保存原本的局部地图
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            cout << " 保存了未全局对齐的局部地图至: " << all_points_dir << endl;
        }
    }


    // 这段是用来将原始轨迹转换到规定的ppp_ecef原点处的。
    // // if (GNSS_ENABLE || NMEA_ENABLE)
    // {
    //     Eigen::Matrix3d enu_rot = ecef2rotation(first_pvt_used);
    //     for (int i = 0; i < time_frame.size(); i++)
    //     {
    //         // Eigen::Vector3d euler_ext = SO3ToEuler(local_rots[i]);
    //         if (NMEA_ENABLE)
    //         {
    //             Eigen::Vector3d ecef_r = enu_rot * est_poses[i] + first_pvt_used;
    //             Eigen::Vector3d pos_enu = ecef2enu(first_lla_anc, ecef_r - first_pvt_anc);
    //             fout_global << setw(20) << time_frame[i] - ppp_ecef[0][0] + 18.0 << " " << pos_enu.transpose() << endl; //"\n"; // p_gnss->pvt_time[0] + 18.0
    //         }
    //         else
    //         {
    //             fout_global << setw(20) << time_frame[i] - time_frame[0] << " " << est_poses[i].transpose() << endl; // << " " << local_poses[i].transpose() << " " << euler_ext.transpose() << endl; //"\n"; // p_gnss->pvt_time[0] + 18.0
    //             // fout_global << setw(20) << time_frame[i] - ppp_ecef[0][0] + 18.0 << " " << est_poses[i].transpose() << endl; //"\n"; // p_gnss->pvt_time[0] + 18.0
    //         }
    //         // printf("time: %f, pos: %f %f %f\n", ppp_ecef[0][0] + 18.0, est_poses[i](0), est_poses[i](1), est_poses[i](2));
    //         // Eigen::Vector3d euler_ext = SO3ToEuler(local_rots[i]);
    //     }
    //     fout_global.close();
    // }


    // //--------------------------save trajectory-----------------------------------
    // // 【修改后：安全且对齐全局 ENU 的轨迹保存逻辑】
    // if (fout_global.is_open() && time_frame.size() > 0)
    // {
    //     // 尝试提取因子图中的全局变换（和刚才保存地图的逻辑一模一样）
    //     Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
    //     bool has_global_tf = false;

    //     if (NMEA_ENABLE && p_nmea->p_assign->isamCurrentEstimate.size() > 0)
    //     {
    //         try {
    //             final_transform.block<3, 3>(0, 0) = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
    //             final_transform.block<3, 1>(0, 3) = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
    //             has_global_tf = true;
    //         } catch (...) {
    //             // 如果提取失败，静默降级为保存局部轨迹
    //         }
    //     }

    //     for (int i = 0; i < time_frame.size(); i++)
    //     {
    //         Eigen::Vector3d pos = est_poses[i];
            
    //         // 如果成功获取了全局矩阵，把轨迹点也转换到你指定的 ENU 下！
    //         // if (has_global_tf) {
    //         //     pos = final_transform.block<3, 3>(0, 0) * pos + final_transform.block<3, 1>(0, 3);
    //         // }
            
    //         // 写入时间戳和 X, Y, Z (摒弃了原作者危险的 ppp_ecef)
    //         // fout_global << std::fixed << std::setprecision(6) 
    //         //             << time_frame[i] << " " 
    //         //             << pos(0) << " " 
    //         //             << pos(1) << " " 
    //         //             << pos(2) << std::endl;
    //         fout_global << std::fixed << std::setprecision(6) 
    //         << time_frame[i] << " " 
    //         << pos(0) << " " 
    //         << pos(1) << " " 
    //         << pos(2) << " "
    //         << "0.0 0.0 0.0 1.0" << std::endl; // 补上单位四元数
    //     }
    //     fout_global.close();
    //     std::cout << "轨迹文件(tum)已成功保存!" << std::endl;
    // }
    //--------------------------save trajectory (一鱼三吃版)-----------------------------------
    if (time_frame.size() > 0)
    {
        bool has_global_tf = false;
        double yaw_opt = 0.0;
        Eigen::Vector3d global_anchor = Eigen::Vector3d::Zero();

        if (NMEA_ENABLE && p_nmea->p_assign->isamCurrentEstimate.size() > 0)
        {
            try {
                // 提取最终优化好的 完美Yaw角 和 完美平移原点
                Eigen::Matrix3d opt_rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
                yaw_opt = atan2(opt_rot(1, 0), opt_rot(0, 0));
                global_anchor = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
                has_global_tf = true;
            } catch (...) { }
        }

        if (has_global_tf)
        {
            // 构造三个不同的纯 Yaw 角旋转矩阵
            Eigen::Matrix3d R_2pt = Eigen::AngleAxisd(p_nmea->yaw_2pt_saved, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Matrix3d R_6pt = Eigen::AngleAxisd(p_nmea->yaw_6pt_saved, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Matrix3d R_opt = Eigen::AngleAxisd(yaw_opt, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            // 打开三个文件准备写入
            ofstream f_2pt("/home/cclg/traj_2pts.tum", ios::out);
            ofstream f_6pt("/home/cclg/traj_6pts.tum", ios::out);
            ofstream f_opt("/home/cclg/traj_opt.tum", ios::out);

            f_2pt << std::fixed << std::setprecision(6);
            f_6pt << std::fixed << std::setprecision(6);
            f_opt << std::fixed << std::setprecision(6);

            for (int i = 0; i < time_frame.size(); i++)
            {
                Eigen::Vector3d pos_local = est_poses[i]; // 完美平滑的局部坐标

                // 核心控制变量：相同的局部坐标，相同的原点，只受不同的 Yaw 角影响！
                Eigen::Vector3d pos_2pt = R_2pt * pos_local + global_anchor;
                Eigen::Vector3d pos_6pt = R_6pt * pos_local + global_anchor;
                Eigen::Vector3d pos_opt = R_opt * pos_local + global_anchor;

                f_2pt << time_frame[i] << " " << pos_2pt(0) << " " << pos_2pt(1) << " " << pos_2pt(2) << " 0.0 0.0 0.0 1.0\n";
                f_6pt << time_frame[i] << " " << pos_6pt(0) << " " << pos_6pt(1) << " " << pos_6pt(2) << " 0.0 0.0 0.0 1.0\n";
                f_opt << time_frame[i] << " " << pos_opt(0) << " " << pos_opt(1) << " " << pos_opt(2) << " 0.0 0.0 0.0 1.0\n";
            }

            f_2pt.close(); f_6pt.close(); f_opt.close();
            std::cout << "\n=============================================" << std::endl;
            std::cout << " [消融实验轨迹] 成功保存 3 条轨迹！" << std::endl;
            std::cout << " - 2点SVD Yaw : " << p_nmea->yaw_2pt_saved * 180.0 / M_PI << " deg" << std::endl;
            std::cout << " - 6点SVD Yaw : " << p_nmea->yaw_6pt_saved * 180.0 / M_PI << " deg" << std::endl;
            std::cout << " - 优化后 Yaw : " << yaw_opt * 180.0 / M_PI << " deg" << std::endl;
            std::cout << "=============================================\n" << std::endl;
        }
    }

    // for (int i = 0; i < p_gnss->pvt_time.size(); i++)
    // {
    //     fout_rtk << setw(20) << p_gnss->pvt_time[i] - p_gnss->pvt_time[0] << " " << p_gnss->pvt_holder[i].transpose() << " " << p_gnss->diff_holder[i] << " " << p_gnss->float_holder[i] << endl; // "\n";
    // }
    // fout_rtk.close();


    // #ifdef process_ppp
    // for (int i = 0; i < ppp_ecef.size(); i++)
    // {
    //     Eigen::Vector3d pos_enu = ecef2enu(p_gnss->first_lla_pvt, ppp_ecef[i].segment<3>(1) - p_gnss->first_xyz_ecef_pvt);
    //     fout_ppp << setw(20) << ppp_ecef[i][0] - ppp_ecef[0][0] << " " << pos_enu.transpose() << endl;
    // }
    // fout_ppp.close();
    // #endif


    return 0;
}
