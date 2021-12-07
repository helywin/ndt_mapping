/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */
#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

#include <time.h>

class ndt_mapping
{
public:
    ndt_mapping();
    ~ndt_mapping();

private:

    ros::NodeHandle nh_;
    // 订阅点云
    ros::Subscriber points_sub_;

    struct pose
    {
        double x, y, z;
        double roll, pitch, yaw;
    };
    struct pose current_pose_, current_pose_imu_;
    struct pose previous_pose_;

    // 点云地图
    pcl::PointCloud<pcl::PointXYZI> map_;
    // 体素网格过滤器(对点云数据下采样,减少点云数据的大小)
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
    // 正态分布配准
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

    // Default values
    int max_iter_;         // 最大迭代次数
    double ndt_res_;       // 分辨率
    double step_size_;     // 步长
    double trans_eps_;     // 变换epsilon

    double voxel_leaf_size_;// 体素网格过滤器的叶子大小

    double scan_rate_;      // 扫描频率
    double min_scan_range_; // 最小扫描范围
    double max_scan_range_; // 最大扫描范围
    bool use_imu_;          // 是否使用imu

    // 地图发布, 地图位置发布
    ros::Publisher ndt_map_pub_, current_pose_pub_;
    // 当前位置数据
    geometry_msgs::PoseStamped current_pose_msg_;

    // tf变换发布
    tf::TransformBroadcaster br_;

    int initial_scan_loaded;
    // 最小扫描增量步长
    double min_add_scan_shift_;

    // 变换的变量
    double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
    // base_link到地图, 地图到base_link
    Eigen::Matrix4f tf_btol_, tf_ltob_;//base_link2localizer等の略?

    // 
    bool _incremental_voxel_update;

    // 首次建图
    bool is_first_map_;

    std::ofstream ofs;
    std::string filename;

    void imu_calc(ros::Time current_time);
    void imu_callback(const sensor_msgs::Imu::Ptr &input);
    void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input);
};
