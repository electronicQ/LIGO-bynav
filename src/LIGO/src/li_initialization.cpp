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

#include "li_initialization.h"
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::Imu imu_last, imu_next;
// sensor_msgs::Imu::ConstPtr imu_last_ptr;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());
double s_plot[MAXN], s_plot3[MAXN];

bool first_gps = false;
Eigen::Vector3d first_gps_lla;
Eigen::Vector3d first_gps_ecef;
condition_variable sig_buffer;
int loop_count = 0;
int scan_count_point = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
std::deque<double>               time_buffer;
std::deque<sensor_msgs::Imu::Ptr> imu_deque;
std::queue<std::vector<ObsPtr>> gnss_meas_buf;
std::queue<nav_msgs::OdometryPtr> nmea_meas_buf;

void gnss_ephem_callback(const GnssEphemMsgConstPtr &ephem_msg)
{
    EphemPtr ephem = msg2ephem(ephem_msg);
    p_gnss->p_assign->inputEphem(ephem);
}

void gnss_glo_ephem_callback(const GnssGloEphemMsgConstPtr &glo_ephem_msg)
{
    GloEphemPtr glo_ephem = msg2glo_ephem(glo_ephem_msg);
    p_gnss->p_assign->inputEphem(glo_ephem);
}

void gnss_iono_params_callback(const StampedFloat64ArrayConstPtr &iono_msg)
{
    double ts = iono_msg->header.stamp.toSec();
    std::vector<double> iono_params;
    std::copy(iono_msg->data.begin(), iono_msg->data.end(), std::back_inserter(iono_params));
    assert(iono_params.size() == 8);
    p_gnss->inputIonoParams(ts, iono_params);
}

void rtk_pvt_callback(const GnssPVTSolnMsgConstPtr &groundt_pvt)
{
    double ts = time2sec(gst2time(groundt_pvt->time.week, groundt_pvt->time.tow));
    p_gnss->inputpvt(ts, groundt_pvt->latitude, groundt_pvt->longitude, groundt_pvt->altitude, groundt_pvt->carr_soln, groundt_pvt->diff_soln);
}

void rtk_lla_callback(const sensor_msgs::NavSatFixConstPtr &lla_msg)
{
    double ts = lla_msg->header.stamp.toSec();
    p_gnss->inputlla(ts, lla_msg->latitude, lla_msg->longitude, lla_msg->altitude);
}

void gnss_meas_callback(const GnssMeasMsgConstPtr &meas_msg)
{
    std::vector<ObsPtr> gnss_meas = msg2meas(meas_msg);
    
    latest_gnss_time = time2sec(gnss_meas[0]->time);
    // printf("gnss time: %f\n", latest_gnss_time);

    // cerr << "gnss ts is " << std::setprecision(20) << time2sec(gnss_meas[0]->time) << endl;
    if (!time_diff_valid)   return;

    // mtx_buffer.lock();
    gnss_meas_buf.push(std::move(gnss_meas)); // ?
    // mtx_buffer.unlock();
    // sig_buffer.notify_all(); // notify_one()?
}

void gnss_meas_callback_urbannav(const nlosExclusion::GNSS_Raw_ArrayConstPtr &meas_msg)
{
    rtklib_gnss_meas_callback(meas_msg, gnss_meas_buf);
}


//先将   lio  和   GNSS的enu里程计   都存储到对应的buff队列中，然后后续在队列中取出相同时间戳的数据进行svd匹配以及后续算法的使用
void nmea_meas_callback(const nav_msgs::OdometryConstPtr &meas_msg)
{
    double nmea_stamp = meas_msg->header.stamp.toSec();
    // 【诊断日志】打印 NMEA 数据的时间戳、位置和帧ID，用于与 LiDAR/IMU 时间戳对比
    ROS_INFO_STREAM_THROTTLE(1.0, "[NMEA Source] 收到 NMEA 数据! ts=" << std::fixed << std::setprecision(3)
                             << nmea_stamp << " X=" << meas_msg->pose.pose.position.x
                             << " Y=" << meas_msg->pose.pose.position.y
                             << " frame_id=" << meas_msg->header.frame_id);

    // 【诊断日志】每次第1条、此后每20条打印一次时间戳信息，用于验证 time_diff_nmea_local 是否正确
    static int nmea_count = 0;
    nmea_count++;
    if (nmea_count == 1 || nmea_count % 20 == 0)
    {
        double ros_now = ros::Time::now().toSec();
        ROS_INFO("[NMEA TIME DIAG] #%d | NMEA msg ts=%.3f | ROS now=%.3f | diff(NMEA-ROS)=%.3f sec | "
                 "若此差值远大于 time_diff_nmea_local=%.1f 则 sync_packages 可能丢弃所有 NMEA 数据!",
                 nmea_count, nmea_stamp, ros_now, nmea_stamp - ros_now, time_diff_nmea_local);
    }

    nav_msgs::OdometryPtr nmea_meas(new nav_msgs::Odometry(*meas_msg));
    last_nmea_time = nmea_meas->header.stamp.toSec();
    nmea_meas_buf.push(std::move(nmea_meas)); // ?
    ROS_INFO("[DEBUG] NMEA data pushed to buffer. Buffer size: %zu", nmea_meas_buf.size());
}

void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg)
{
    // if (gpsMsg->status.status != 0)
    //     return;

    Eigen::Vector3d trans_local_;
    if (!first_gps) {
        first_gps = true;
        Eigen::Vector3d geo;
        geo << gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude;
        first_gps_lla = geo;
        first_gps_ecef = geo2ecef(geo);
    }
    Eigen::Vector3d cur_ecef = geo2ecef(Eigen::Vector3d(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude));
    trans_local_ = ecef2enu(first_gps_lla, cur_ecef - first_gps_ecef);

    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = gpsMsg->header.stamp;
    // gps_odom->header.frame_id = "map";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    // gps_odom->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    // pubGpsOdom.publish(gps_odom);
    // gpsQueue.push_back(gps_odom);
    nmea_meas_buf.push(nav_msgs::OdometryPtr(new nav_msgs::Odometry(gps_odom)));
}

void local_trigger_info_callback(const ligo::LocalSensorExternalTriggerConstPtr &trigger_msg) // pps time sync
{
    std::lock_guard<std::mutex> lg(m_time);

    if (next_pulse_time_valid)
    {
        time_diff_gnss_local = next_pulse_time - trigger_msg->header.stamp.toSec();
        p_gnss->inputGNSSTimeDiff(time_diff_gnss_local);
        if (!time_diff_valid)       // just get calibrated
            std::cout << "time difference between GNSS and LI-Sensor got calibrated: "
                << std::setprecision(15) << time_diff_gnss_local << " s\n";
        time_diff_valid = true;
    }
}

void gnss_tp_info_callback(const GnssTimePulseInfoMsgConstPtr &tp_msg) // time stamp of GNSS signal
{
    gtime_t tp_time = gpst2time(tp_msg->time.week, tp_msg->time.tow);
    if (tp_msg->utc_based || tp_msg->time_sys == SYS_GLO)
        tp_time = utc2gpst(tp_time);
    else if (tp_msg->time_sys == SYS_GAL)
        tp_time = gst2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_BDS)
        tp_time = bdt2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_NONE)
    {
        std::cerr << "Unknown time system in GNSSTimePulseInfoMsg.\n";
        return;
    }
    double gnss_ts = time2sec(tp_time);

    std::lock_guard<std::mutex> lg(m_time);
    next_pulse_time = gnss_ts;
    next_pulse_time_valid = true;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    // mtx_buffer.lock();
    scan_count ++;
    // double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        return;
    }

    last_timestamp_lidar = msg->header.stamp.toSec();

    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
    p_pre->process(msg, ptr);
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            // cout << "ptr div num:" << ptr_div->size() << endl;
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            // 多帧合并时，time_con 是合并窗口内第一帧 LiDAR 的起始绝对时间(s)。
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    { 
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            // time_buffer 与 lidar_buffer 一一对应，保存这帧点云的帧起始绝对时间(s)。
            // 点云中每个点的 curvature 字段在 preprocess 中被借用为相对帧起点的时间偏移(ms)。
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    }
    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    // mtx_buffer.lock();
    // double preprocess_start_time = omp_get_wtime();

    //新增
    ROS_INFO_THROTTLE(2.0, "[DIAG] livox_pcl_cbk 收到 LiDAR 数据, ts=%.3f, buffer大小=%zu", 
                      msg->header.stamp.toSec(), lidar_buffer.size());

    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
        // lidar_buffer.shrink_to_fit();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();    

    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
    p_pre->process(msg, ptr); 
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            // 多帧合并时，time_con 是合并窗口内第一帧 LiDAR 的起始绝对时间(s)。
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            // time_buffer 与 lidar_buffer 一一对应，保存这帧点云的帧起始绝对时间(s)。
            // 点云中每个点的 curvature 字段在 preprocess 中被借用为相对帧起点的时间偏移(ms)。
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    }
    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    // mtx_buffer.lock();
    // publish_count ++;

    //新增1
    ROS_INFO_THROTTLE(2.0,"[DIAG] imu_cbk 收到 IMU 数据, ts = %.3f,deque大小 = %zu",
                msg_in->header.stamp.toSec(),imu_deque.size());

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = msg->header.stamp.toSec();
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        // cout << "check time:" << timestamp << ";" << last_timestamp_imu << endl;
        // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);
        
        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }

    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

//以一帧 LiDAR 为主时间窗口，把这一帧 LiDAR、对应时间段内的 IMU、对应时间段内的 GNSS/NMEA 数据同步出来。

bool sync_packages(MeasureGroup &meas, queue<std::vector<ObsPtr>> &gnss_msg, queue<nav_msgs::OdometryPtr> &nmea_msg)
{
    if (nolidar)
    {
        if (is_first_gnss && !NMEA_ENABLE)
        {
            if (gnss_meas_buf.empty())
            {
                // imu_pushed = false;
                return false;
            }
            else
            {
                // double imu_time = imu_deque.front()->header.stamp.toSec();
                // double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                // if (last_timestamp_imu < front_gnss_ts - time_diff_gnss_local)
                // {
                    // return false;
                // }
                // while (front_gnss_ts - imu_time > time_diff_gnss_local) // wrong
                // {
                    // imu_last = *(imu_deque.front());
                    // imu_deque.pop_front();
                    // if(imu_deque.empty()) break;
                    // imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    // imu_next = *(imu_deque.front());
                // }
                // else
                while (!imu_deque.empty())
                {
                    imu_deque.pop_front();
                }
                {
                    is_first_gnss = false;
                }
            }
        }

        if (is_first_nmea && NMEA_ENABLE)
        {
            if (nmea_meas_buf.empty())
            {
                return false;
            }
            else
            {
                while (!imu_deque.empty())
                {
                    imu_deque.pop_front();
                }
                {
                    is_first_nmea = false;
                }
            }
        }


        if (imu_deque.empty())
        {
            return false;
        }
        
        imu_first_time = imu_deque.front()->header.stamp.toSec(); // 

        if ((latest_gnss_time < time_diff_gnss_local + imu_first_time + lidar_time_inte) && GNSS_ENABLE)
        {
            return false;
        }

        if ((last_nmea_time < time_diff_nmea_local + imu_first_time + lidar_time_inte) && NMEA_ENABLE)
        {
            return false;
        }

        if (last_timestamp_imu < imu_first_time + lidar_time_inte)
        {
            return false;
        }

        if (imu_deque.empty())
        {
            cout << "could not be here" << endl;
            return false;
        }

        if (!imu_pushed)
        { 
            double imu_time = imu_deque.front()->header.stamp.toSec();
            // imu_first_time = imu_time;

            double imu_last_time = imu_deque.back()->header.stamp.toSec();
            if (imu_last_time - imu_first_time < lidar_time_inte)
            {
                return false;
            }
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_)
            {
                imu_next = *(imu_deque.front());
                meas.imu.shrink_to_fit();
                while (imu_time - imu_first_time < lidar_time_inte)
                {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if(imu_deque.empty()) break;
                    imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    imu_next = *(imu_deque.front());
                }
                if (!gnss_meas_buf.empty())
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    while (front_gnss_ts < imu_first_time + lidar_time_inte + time_diff_gnss_local)
                    {
                        gnss_meas_buf.pop();
                        if(gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    }
                    if (meas.imu.empty())
                    {
                        return false;
                    }
                }
                if (!nmea_meas_buf.empty())
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    while (front_nmea_ts < imu_first_time + lidar_time_inte + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    }
                    if (meas.imu.empty())
                    {
                        return false;
                    }
                }
            }
            imu_pushed = true;
        }

        if (GNSS_ENABLE)
        {
            if (!gnss_meas_buf.empty()) // or can wait for a short time?
            {
                // double back_gnss_ts = time2sec(gnss_meas_buf.back()[0]->time);
                
                // if (back_gnss_ts - imu_first_time < time_diff_gnss_local + lidar_time_inte)
                // {
                //     return false;
                // }
                double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                while (front_gnss_ts - imu_first_time < time_diff_gnss_local + lidar_time_inte) 
                {
                    gnss_msg.push(gnss_meas_buf.front());
                    gnss_meas_buf.pop();
                    if (gnss_meas_buf.empty()) break;
                    front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                }

                if (!gnss_msg.empty())
                {
                    imu_pushed = false;
                    return true;
                }
            }

            // if (gnss_meas_buf.empty())
            // {
            //     wait_num ++;
            //     if (wait_num > 2) 
            //     {
            //         wait_num = 0;
            //     }
            //     else
            //     {
            //         return false;
            //     }
            // }
        }
        if (NMEA_ENABLE)
        {
            if (!nmea_meas_buf.empty()) // or can wait for a short time?
            {
                double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                while (front_nmea_ts - imu_first_time < time_diff_nmea_local + lidar_time_inte) 
                {
                    nmea_msg.push(nmea_meas_buf.front());
                    nmea_meas_buf.pop();
                    if (nmea_meas_buf.empty()) break;
                    front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                }

                if (!nmea_msg.empty())
                {
                    imu_pushed = false;
                    return true;
                }
            }
        }        
        imu_pushed = false;
        return true;
    }
    else
    {
    if (!imu_en)
    {
        if (is_first_gnss && GNSS_ENABLE)
        {
            if (gnss_meas_buf.empty())
            {
                return false;
            }
            else
            {
                while (!lidar_buffer.empty())
                {
                    lidar_buffer.pop_front();
                }
            }
        }
        if (!lidar_buffer.empty())
        {
            if (!lidar_pushed)
            {
                //meas定义在common_lib.h中
                //从lidar回调函数对应的缓存队列中取出最早的雷达点云存在meas.lidar中
                meas.lidar = lidar_buffer.front();

                //从lidar回调函数对应的缓存队列中取出最早的雷达时间存在meas.lidar中
                meas.lidar_beg_time = time_buffer.front();
                lose_lid = false;
                if(meas.lidar->points.size() < 1) 
                {
                    cout << "lose lidar" << std::endl;
                    // return false;
                    lose_lid = true;
                }
                else
                {
                    double end_time = meas.lidar->points.back().curvature;

                    
                    for (auto pt: meas.lidar->points)
                    {
                        if (pt.curvature > end_time)
                        {
                            end_time = pt.curvature;
                        }
                    }
                    lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                    meas.lidar_last_time = lidar_end_time;
                }
                lidar_pushed = true;
            }
            
            if (GNSS_ENABLE)
            {
                if (!gnss_meas_buf.empty()) // or can wait for a short time?
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    while (front_gnss_ts < meas.lidar_beg_time + time_diff_gnss_local) // 0.05
                    {
                        ROS_WARN("throw gnss, only should happen at the beginning 542");
                        gnss_meas_buf.pop();
                        if (gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    }
                    if (!gnss_meas_buf.empty())
                    {
                    while ((!lose_lid && (front_gnss_ts <= lidar_end_time + time_diff_gnss_local)) || (lose_lid && (front_gnss_ts <= meas.lidar_beg_time + time_diff_gnss_local + lidar_time_inte) ))
                    {
                        gnss_msg.push(gnss_meas_buf.front());
                        gnss_meas_buf.pop();
                        if (gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    }
                    if (!gnss_msg.empty())
                    {  
                        time_buffer.pop_front();
                        lidar_buffer.pop_front();
                        lidar_pushed = false;
                        return true;
                    }
                    }
                }

                // if (gnss_meas_buf.empty())
                // {
                //     wait_num ++;
                //     if (wait_num > 2) 
                //     {
                //         wait_num = 0;
                //     }
                //     else
                //     {
                //         return false;
                //     }
                // }
            }
            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty()) // or can wait for a short time?
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                    while (front_nmea_ts < meas.lidar_beg_time + time_diff_nmea_local) // 0.05
                    {
                        ROS_WARN("throw nmea, only should happen at the beginning 542");
                        nmea_meas_buf.pop();
                        if (nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                    }
                    if (!nmea_meas_buf.empty())
                    {
                    while ((!lose_lid && (front_nmea_ts <= lidar_end_time + time_diff_nmea_local)) || (lose_lid && (front_nmea_ts <= meas.lidar_beg_time + time_diff_nmea_local + lidar_time_inte) ))
                    {
                        nmea_msg.push(nmea_meas_buf.front());
                        nmea_meas_buf.pop();
                        if (nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
                    }
                    if (!nmea_msg.empty())
                    {
                        time_buffer.pop_front();
                        lidar_buffer.pop_front();
                        lidar_pushed = false;
                        return true;
                    }
                    }
                }
            }
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            lidar_pushed = false;
            if (!lose_lid)
            {
                return true;
            }
            else
            {
                return false;
            }
        }        
        return false;
    }

    if (0) // (is_first_gnss && GNSS_ENABLE)
    {
        if (gnss_meas_buf.empty())
        {
            return false;
        }
        else
        {
            while (!lidar_buffer.empty())
            {
                lidar_buffer.pop_front();
            }
            is_first_gnss = false;
            while (!imu_deque.empty())
            {
                imu_deque.pop_front();
            }
        }
    }

    //新增
    if (lidar_buffer.empty() || imu_deque.empty()) {
        ROS_INFO_THROTTLE(1.0, "[DIAG] sync_packages 返回 false, lidar_buf=%zu, imu_deque=%zu, nmea_buf=%zu",
                        lidar_buffer.size(), imu_deque.size(), nmea_meas_buf.size());
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        lose_lid = false;
        meas.lidar = lidar_buffer.front();
        // 当前待处理 LiDAR 帧的起始绝对时间(s)，由 LiDAR 消息 header.stamp 写入 time_buffer。
        meas.lidar_beg_time = time_buffer.front();
        if(meas.lidar->points.size() < 1) 
        {
            cout << "lose lidar" << endl;
            lose_lid = true;
            // lidar_buffer.pop_front();
            // time_buffer.pop_front();
            // return false;
        }
        else
        {
            // curvature 在这里表示点相对当前 LiDAR 帧起点的时间偏移，单位是 ms。
            // 先从最后一个点取一个初值，再遍历整帧点云找最大偏移，避免点云未按时间排序。
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            // 帧结束绝对时间(s) = 帧起始绝对时间(s) + 最大点时间偏移(ms)/1000。
            // 后续同步 IMU / GNSS / NMEA 都围绕 [lidar_beg_time, lidar_end_time] 这个窗口。
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            // cout << "check time lidar:" << end_time << endl;
            meas.lidar_last_time = lidar_end_time;
        }
        lidar_pushed = true;
    }

    if (!lose_lid && (last_timestamp_imu < lidar_end_time + 2))
    {
        // lidar_pushed = false;
        return false;
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte + 2)
    {
        // lidar_pushed = false;
        return false;
    }

    if (!lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();
            imu_next = *(imu_deque.front());
            meas.imu.shrink_to_fit();
            while (imu_time < lidar_end_time)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                imu_next = *(imu_deque.front());
            }
            if (GNSS_ENABLE)
            {
                if (!gnss_meas_buf.empty())
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take timedouble front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
                    while (front_gnss_ts < lidar_end_time + time_diff_gnss_local)
                    {
                        gnss_meas_buf.pop();
                        if(gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    }
                }
            }
            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty())
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); 
                    while (front_nmea_ts < lidar_end_time + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    }
                }
            }
        }
        imu_pushed = true;
    }
    
    if (lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();
            meas.imu.shrink_to_fit();

            imu_next = *(imu_deque.front());
            while (imu_time < meas.lidar_beg_time + lidar_time_inte)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                imu_next = *(imu_deque.front());
            }

            if (GNSS_ENABLE)
            {
                if (!gnss_meas_buf.empty())
                {
                    double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    while (front_gnss_ts < meas.lidar_beg_time + lidar_time_inte + time_diff_gnss_local)
                    {
                        gnss_meas_buf.pop();
                        if(gnss_meas_buf.empty()) break;
                        front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
                    }
                }
            }

            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty())
                {
                    double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    while (front_nmea_ts < meas.lidar_beg_time + lidar_time_inte + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
                    }
                }
            }
        }

        imu_pushed = true;
    }

    if (GNSS_ENABLE)
    {
        if (!gnss_meas_buf.empty()) // or can wait for a short time?
        {
            double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time); // take time
            while ((!lose_lid && (front_gnss_ts < lidar_end_time + time_diff_gnss_local)) || (lose_lid && (front_gnss_ts < meas.lidar_beg_time + time_diff_gnss_local + lidar_time_inte) )) // (front_gnss_ts >= meas.lidar_beg_time + time_diff_gnss_local) && 
            {
                gnss_msg.push(gnss_meas_buf.front());
                gnss_meas_buf.pop();
                if (gnss_meas_buf.empty()) break;
                front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
            }
            if (!gnss_msg.empty())
            {
                time_buffer.pop_front();
                lidar_buffer.pop_front();
                lidar_pushed = false;
                imu_pushed = false;
                return true;
            }
        }
    }
    if (NMEA_ENABLE)
    {
        if (!nmea_meas_buf.empty()) // or can wait for a short time?
        {
            double front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec(); // take time
            double back_nmea_ts  = nmea_meas_buf.back()->header.stamp.toSec();

            // 【诊断日志】打印时间戳对齐关键参数：NMEA范围 vs LiDAR窗口
            double match_window_start = meas.lidar_beg_time + time_diff_nmea_local;
            double match_window_end   = lidar_end_time + time_diff_nmea_local;
            ROS_INFO_THROTTLE(2.0, "[NMEA SYNC DIAG] nmea_buf=%zu | nmea_range=[%.3f, %.3f] | "
                              "lidar_beg=%.3f lidar_end=%.3f | match_window=[%.3f, %.3f] | time_diff=%.1f | lose_lid=%d",
                              nmea_meas_buf.size(), front_nmea_ts, back_nmea_ts,
                              meas.lidar_beg_time, lidar_end_time,
                              match_window_start, match_window_end,
                              time_diff_nmea_local, lose_lid);

            // 【诊断日志】如果 NMEA 数据的时间戳完全落在匹配窗口之外，说明 time_diff 配错了
            if (back_nmea_ts < match_window_start)
            {
                ROS_WARN_THROTTLE(2.0, "[NMEA SYNC WARN] ⚠️ 所有NMEA数据(最新ts=%.3f)都早于匹配窗口起点(%.3f)! "
                                  "time_diff_nmea_local=%.1f 可能太小! 尝试增大到 %.1f 左右",
                                  back_nmea_ts, match_window_start, time_diff_nmea_local,
                                  meas.lidar_beg_time - front_nmea_ts + 0.5);
            }
            if (front_nmea_ts > match_window_end + 1.0)
            {
                ROS_WARN_THROTTLE(2.0, "[NMEA SYNC WARN] ⚠️ 最早NMEA数据(ts=%.3f)远大于匹配窗口终点(%.3f)! "
                                  "time_diff_nmea_local=%.1f 可能太大或NMEA时间源有偏移",
                                  front_nmea_ts, match_window_end, time_diff_nmea_local);
            }

            int nmea_matched = 0;
            while ((!lose_lid && (front_nmea_ts < lidar_end_time + time_diff_nmea_local)) || (lose_lid && (front_nmea_ts < meas.lidar_beg_time + time_diff_nmea_local + lidar_time_inte) ))
            {
                nmea_msg.push(nmea_meas_buf.front());
                nmea_meas_buf.pop();
                nmea_matched++;
                if (nmea_meas_buf.empty()) break;
                front_nmea_ts = nmea_meas_buf.front()->header.stamp.toSec();
            }
            if (nmea_matched > 0)
            {
                ROS_INFO("[NMEA SYNC] ✅ 匹配到 %d 条 NMEA 数据, nmea_msg队列大小=%zu",
                         nmea_matched, nmea_msg.size());
            }
            if (!nmea_msg.empty())
            {
                time_buffer.pop_front();
                lidar_buffer.pop_front();
                lidar_pushed = false;
                imu_pushed = false;
                return true;
            }
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
    }
}
