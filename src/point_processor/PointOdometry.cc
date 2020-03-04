/*
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 3/22/18.
//

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

/*
 * Paper(LOAM)与代码的差别:
 * paper中使用LM方法进行优化,而代码中使用的是高斯牛顿法.
 * paper中使用的是angle-axis进行优化,而代码中使用的是旋转矩阵直接对欧拉角求导,最终优化的是欧拉角
 */

#include "point_processor/PointOdometry.h"

namespace lio {
	
	PointOdometry::PointOdometry(float scan_period, int io_ratio, size_t num_max_iterations)
					: scan_period_(scan_period),
					  time_factor_(1 / scan_period),
					  io_ratio_(io_ratio),
					  system_inited_(false),
					  frame_count_(0),
					  num_max_iterations_(num_max_iterations),
					  delta_r_abort_(0.1),
					  delta_t_abort_(0.1),
					  corner_points_sharp_(new PointCloud()),
					  corner_points_less_sharp_(new PointCloud()),
					  surf_points_flat_(new PointCloud()),
					  surf_points_less_flat_(new PointCloud()),
					  full_cloud_(new PointCloud()),
					  last_corner_cloud_(new PointCloud()),
					  last_surf_cloud_(new PointCloud()),
					  laser_cloud_ori_(new PointCloud()),
					  coeff_sel_(new PointCloud()),
					  kdtree_corner_last_(new pcl::KdTreeFLANN<PointT>()),
					  kdtree_surf_last_(new pcl::KdTreeFLANN<PointT>()) {
		
		// adapted from LOAM
		// initialize odometry and odometry tf messages
		laser_odometry_msg_.header.frame_id = "/camera_init";
		laser_odometry_msg_.child_frame_id = "/laser_odom";
		
		laser_odometry_trans_.frame_id_ = "/camera_init";
		// laser_odometry_trans_.child_frame_id_ = "/laser_odom";
		laser_odometry_trans_.child_frame_id_ = "/camera";

/*  laser_odometry_msg_.header.frame_id = "/map";
  laser_odometry_msg_.child_frame_id = "/laser_odom";

  laser_odometry_trans_.frame_id_ = "/map";
  laser_odometry_trans_.child_frame_id_ = "/laser_odom";*/
	}
	
	void PointOdometry::Spin() {
		ros::Rate rate(200);
		bool status = ros::ok();
		
		// loop until shutdown
		while (status) {
			ros::spinOnce();
			
			// try processing new data
			Process();
			
			status = ros::ok();
			rate.sleep();
		}
	}

// 初始化subscribe和publish
	void PointOdometry::SetupRos(ros::NodeHandle &nh) {
		
		is_ros_setup_ = true;
		
		nh.param("compact_data", compact_data_, true);
		nh.param("no_deskew", no_deskew_, false);
		
		enable_odom_service_ = nh.advertiseService("/enable_odom", &PointOdometry::EnableOdom, this);
		
		if (compact_data_) {
			pub_compact_data_ = nh.advertise<sensor_msgs::PointCloud2>("/compact_data", 2);
		} else {
			// advertise laser odometry topics
			pub_laser_cloud_corner_last_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
			pub_laser_cloud_surf_last_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
			pub_full_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/full_odom_cloud", 2);
		}
		
		pub_laser_odometry_ = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
		
		/// for test
		pub_diff_odometry_ = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_last", 5);
		
		// subscribe to scan registration topics
		sub_corner_points_sharp_ = nh.subscribe<sensor_msgs::PointCloud2>
						("/laser_cloud_sharp", 2, &PointOdometry::LaserCloudSharpHandler, this);
		
		sub_corner_points_less_sharp_ = nh.subscribe<sensor_msgs::PointCloud2>
						("/laser_cloud_less_sharp", 2, &PointOdometry::LaserCloudLessSharpHandler, this);
		
		sub_surf_points_flat_ = nh.subscribe<sensor_msgs::PointCloud2>
						("/laser_cloud_flat", 2, &PointOdometry::LaserCloudFlatHandler, this);
		
		sub_surf_points_less_flat_ = nh.subscribe<sensor_msgs::PointCloud2>
						("/laser_cloud_less_flat", 2, &PointOdometry::LaserCloudLessFlatHandler, this);
		
		sub_full_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>
						("/full_cloud", 2, &PointOdometry::LaserFullCloudHandler, this);

//  sub_imu_trans_ = node.subscribe<sensor_msgs::PointCloud2>
//      ("/imu_trans", 5, &LaserOdometry::ImuTransHandler, this);
	
	}

// 以下是几个话题的回调函数
	void PointOdometry::LaserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_sharp_msg) {
		time_corner_points_sharp_ = corner_points_sharp_msg->header.stamp;
		
		// DLOG(INFO) << "corner_points_sharp_msg at " << time_corner_points_sharp_.toSec();
		
		corner_points_sharp_->clear();
		pcl::fromROSMsg(*corner_points_sharp_msg, *corner_points_sharp_);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*corner_points_sharp_, *corner_points_sharp_, indices);
		new_corner_points_sharp_ = true;
	}
	
	void PointOdometry::LaserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_less_sharp_msg) {
		time_corner_points_less_sharp_ = corner_points_less_sharp_msg->header.stamp;
		
		// DLOG(INFO) << "corner_points_less_sharp_msg at " << time_corner_points_less_sharp_.toSec();
		
		corner_points_less_sharp_->clear();
		pcl::fromROSMsg(*corner_points_less_sharp_msg, *corner_points_less_sharp_);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*corner_points_less_sharp_, *corner_points_less_sharp_, indices);
		new_corner_points_less_sharp_ = true;
	}
	
	void PointOdometry::LaserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_flat_msg) {
		time_surf_points_flat_ = surf_points_flat_msg->header.stamp;
		
		// DLOG(INFO) << "surf_points_flat_msg at " << time_surf_points_flat_.toSec();
		
		surf_points_flat_->clear();
		pcl::fromROSMsg(*surf_points_flat_msg, *surf_points_flat_);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*surf_points_flat_, *surf_points_flat_, indices);
		new_surf_points_flat_ = true;
	}
	
	void PointOdometry::LaserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_less_flat_msg) {
		time_surf_points_less_flat_ = surf_points_less_flat_msg->header.stamp;
		
		// DLOG(INFO) << "surf_points_less_flat_msg at " << time_surf_points_less_flat_.toSec();
		
		surf_points_less_flat_->clear();
		pcl::fromROSMsg(*surf_points_less_flat_msg, *surf_points_less_flat_);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*surf_points_less_flat_, *surf_points_less_flat_, indices);
		new_surf_points_less_flat_ = true;
	}
	
	void PointOdometry::LaserFullCloudHandler(const sensor_msgs::PointCloud2ConstPtr &full_cloud_msg) {
		time_full_cloud_ = full_cloud_msg->header.stamp;
		
		// DLOG(INFO) << "full_cloud_msg at " << time_full_cloud_.toSec();
		
		full_cloud_->clear();
		pcl::fromROSMsg(*full_cloud_msg, *full_cloud_);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*full_cloud_, *full_cloud_, indices);
		new_full_cloud_ = true;
	}
	
	void PointOdometry::ImuTransHandler(const sensor_msgs::PointCloud2ConstPtr &imu_trans_msg) {
		time_imu_trans_ = imu_trans_msg->header.stamp;
		
		pcl::PointCloud<pcl::PointXYZ> imu_trans;
		pcl::fromROSMsg(*imu_trans_msg, imu_trans);
		
		new_imu_trans_ = true;
	}
	
	void PointOdometry::Reset() {
		new_corner_points_sharp_ = false;
		new_corner_points_less_sharp_ = false;
		new_surf_points_flat_ = false;
		new_surf_points_less_flat_ = false;
		new_full_cloud_ = false;
		
		// No imu first
		new_imu_trans_ = true;
	}
	
	bool PointOdometry::HasNewData() {
		return new_corner_points_sharp_ && new_corner_points_less_sharp_ && new_surf_points_flat_ &&
		       new_surf_points_less_flat_ && new_full_cloud_ &&
		       fabs((time_corner_points_less_sharp_ - time_corner_points_sharp_).toSec()) < 0.005 &&
		       fabs((time_surf_points_flat_ - time_corner_points_sharp_).toSec()) < 0.005 &&
		       fabs((time_surf_points_less_flat_ - time_corner_points_sharp_).toSec()) < 0.005 &&
		       fabs((time_full_cloud_ - time_corner_points_sharp_).toSec()) < 0.005;
	}
	
	// 当前点云中的点相对第一个点去除因匀速运动产生的畸变，效果相当于得到在点云扫描开始位置静止扫描得到的点云
	void PointOdometry::TransformToStart(const PointT &pi, PointT &po) {
		float s = time_factor_ * (pi.intensity - int(pi.intensity));
		if (no_deskew_) {
			s = 0;
		}
		if (s < 0 || s > 1.001) {
			po = pi;
			LOG(ERROR) << "time ratio error: " << s;
			return;
		}
		po.x = pi.x - s * transform_es_.pos.x();
		po.y = pi.y - s * transform_es_.pos.y();
		po.z = pi.z - s * transform_es_.pos.z();
		po.intensity = pi.intensity;
		
		Eigen::Quaternionf q_id, q_s, q_e;
		q_e = transform_es_.rot;
		q_id.setIdentity();
		// 插值
		q_s = q_id.slerp(s, q_e);
		
		RotatePoint(q_s.conjugate(), po);
	}
	
	// 将上一帧点云中的点相对结束位置去除因匀速运动产生的畸变，
	// 效果相当于得到在点云扫描结束位置静止扫描得到的点云
	size_t PointOdometry::TransformToEnd(PointCloudPtr &cloud) {
		size_t cloud_size = cloud->points.size();
		
		for (size_t i = 0; i < cloud_size; i++) {
			PointT &point = cloud->points[i];
			// int 向下取整
			float s = time_factor_ * (point.intensity - int(point.intensity));
			
			if (no_deskew_) {
				s = 0;
			}
			
			point.x -= s * transform_es_.pos.x();
			point.y -= s * transform_es_.pos.y();
			point.z -= s * transform_es_.pos.z();
			point.intensity = int(point.intensity);
			
			Eigen::Quaternionf q_id, q_s, q_e;
			q_e = transform_es_.rot;
			q_id.setIdentity();
			// 插值
			q_s = q_id.slerp(s, q_e);
			
			// conjugate共轭
			RotatePoint(q_s.conjugate(), point);
			RotatePoint(q_e, point);
			
			point.x += transform_es_.pos.x();
			point.y += transform_es_.pos.y();
			point.z += transform_es_.pos.z();
		}
		return cloud_size;
	}
	
	void PointOdometry::Process() {
		// 在laserOdometry中配准的source是新一帧的少数特征点的点云 ,target是前一帧的多数特征点的点云.
		if (!HasNewData()) {
			// DLOG(INFO) << "no data received or dropped";
			return;
		}
		
		Reset();
		// 第一次进入，先初始化
		// 将第一个点云数据集发送给laserMapping，从下一个点云数据开始处理
		// =====================留意 laserCloudCornerLast 和 laserCloudSurfLast 在后面有什么作用=========================
		// 答：laserCloudCornerLast和laserCloudSurfLast储存上一个sweep获取的特征点，用于寻找当前sweep的特征点的对应关系，然后根据距离进行非线性的优化。然后再最后，last被发送给建图的节点，建立点云地图
		// =========================================================================================================
		// 系统初始化，将获得的第一帧数据存到两个last容器里面，将两个第一帧特征点云发送给建图节点，先建立第一帧地图
		if (!system_inited_) {
			// 将cornerPointsLessSharp与laserCloudCornerLast交换,目的保存cornerPointsLessSharp的值下轮使用
			corner_points_less_sharp_.swap(last_corner_cloud_);
			// 将surfPointLessFlat与laserCloudSurfLast交换，目的保存surfPointsLessFlat的值下轮使用
			surf_points_less_flat_.swap(last_surf_cloud_);
			// 使用上一帧的特征点构建kd-tree
			kdtree_corner_last_->setInputCloud(last_corner_cloud_);
			kdtree_surf_last_->setInputCloud(last_surf_cloud_);
			
			system_inited_ = true;
			return;
		}
		
		PointT coeff;
		bool is_degenerate = false;
		
		++frame_count_;
		
		size_t last_corner_size = last_corner_cloud_->points.size();
		size_t last_surf_size = last_surf_cloud_->points.size();
		
		tic_toc_.Tic();
		
		if (enable_odom_) {
			// NOTE: fixed number here
			// 特征点足够多再向下做优化
			if (last_corner_size > 10 && last_surf_size > 100) {
				std::vector<int> point_search_idx(1);
				std::vector<float> point_search_sq_dis(1);
				
				// 新一帧点云边缘点数量
				size_t num_curr_corner_points_sharp = corner_points_sharp_->points.size();
				// 新一帧点云平面点数量
				size_t num_curr_surf_points_flat = surf_points_flat_->points.size();
				
				idx_corner1_.resize(num_curr_corner_points_sharp);
				idx_corner2_.resize(num_curr_corner_points_sharp);
				idx_surf1_.resize(num_curr_surf_points_flat);
				idx_surf2_.resize(num_curr_surf_points_flat);
				idx_surf3_.resize(num_curr_surf_points_flat);
				// Levenberg-Marquardt算法(L-M method)，非线性最小二乘算法，最优化算法的一种
				// 最多迭代25次 (基本不会迭代超过5次就会收敛)
				for (size_t iter_count = 0; iter_count < num_max_iterations_; ++iter_count) {
					PointT point_sel, tripod1, tripod2, tripod3;
					//保存匹配距离点对 成功的当前帧点云 的 点 //匹配到的点的个数(即存在多少个约束)
					laser_cloud_ori_->clear();
					coeff_sel_->clear();
					// 处理当前点云中的曲率最大的特征点,从上个点云中曲率比较大的特征点中找两个最近距离点，一个点使用kd-tree查找，另一个根据找到的点在其相邻线找另外一个最近距离的点
					// ===============================开始处理边缘点==============================
					for (int i = 0; i < num_curr_corner_points_sharp; ++i) {
						// 去除运动畸变
						// TODO 查看transform_es是不是在mapping中更改更新的
						// ====答：transform_es代表end2start，会打包传给mapping
						TransformToStart(corner_points_sharp_->points[i], point_sel);
						// 每迭代五次，重新查找最近点，在第一次迭代的时候，对于每个边缘点都会寻找一遍最近点
						if (iter_count % 5 == 0) {
							// 构建KD树(返回两个数组，按照距离近到远排序)
							kdtree_corner_last_->nearestKSearch(point_sel, 1, point_search_idx, point_search_sq_dis);
							
							// 距离目标点距离最小的点index 和 相邻线距离目标点距离最小的点index
							int closest_point_idx = -1, second_closet_point_idx = -1; /// the second one is in another ring
							// 寻找相邻线距离目标点距离最小的点
							// TODO 测试输出kd树查找的距离数组
							if (point_search_sq_dis[0] < 25) {
								// 找到的最近点距离的确很近的话  //搜索到的点平方距离
								//最近点的索引
								closest_point_idx = point_search_idx[0];
								// 提取最近点线号
								int closest_point_scan = int(last_corner_cloud_->points[closest_point_idx].intensity);
								// 初始门槛值5米，可大致过滤掉scanID相邻，但实际线不相邻的值
								float point_sq_dis, second_point_sq_dis = 25;
								// 寻找距离目标点最近距离的平方和最小的点
								// 向scanID增大的方向查找，就查找下一条线(目前是相邻的上下各两条线
								for (int j = closest_point_idx + 1; j < last_corner_size; j++) { // NOTE: find points in upper rings
									if (int(last_corner_cloud_->points[j].intensity) > closest_point_scan + 2.5) {
										// 非相邻线
										break;
									}
									
									// 确保两个点不在同一条scan上（相邻线查找应该可以用scanID == closestPointScan +/- 1 来做）
									point_sq_dis = CalcSquaredDiff(last_corner_cloud_->points[j], point_sel);
									
									if (int(last_corner_cloud_->points[j].intensity) > closest_point_scan) {
										if (point_sq_dis < second_point_sq_dis) {
											// 距离更近，要小于初始值5米
											// 更新最小距离与点序
											second_point_sq_dis = point_sq_dis;
											second_closet_point_idx = j;
										}
									}
								}
								// 同理向scanID减小的方向查找
								for (int j = closest_point_idx - 1; j >= 0; j--) { // NOTE: find points in lower rings
									if (int(last_corner_cloud_->points[j].intensity) < closest_point_scan - 2.5) {
										break;
									}
									// 计算距离
									point_sq_dis = CalcSquaredDiff(last_corner_cloud_->points[j], point_sel);
									
									if (int(last_corner_cloud_->points[j].intensity) < closest_point_scan) {
										if (point_sq_dis < second_point_sq_dis) {
											second_point_sq_dis = point_sq_dis;
											second_closet_point_idx = j;
										}
									}
								}
							}
							// 寻找到的距离最近的两个点
							idx_corner1_[i] = closest_point_idx;
							idx_corner2_[i] = second_closet_point_idx;
						} // NOTE: two points for closest points in a line, update points
						// 对于边缘点来说，如果两个对应点都找到了的话，就计算一下边缘点距离两个对应点构成的直线的距离，同时计算一下权重，距离越小，权重越大。如果距离过大就直接舍弃。同时舍弃距离为0的点
						// ===============================思考：为什么要舍弃距离为0的点，同时，为什么从第五次迭代开始，开始减小权重？=====================================
						// ============================猜想：=================================
						// 前五次所有的边缘点都进入非线性优化， 为了获得更准确的数值，如果迭代五次都没有结束，可能不收敛或者收敛速度太慢，因此去除迭代五次距离仍然很远的点，以加快收敛速度
						
						// 只有第一个点存在，第二个点才会开始找，所以只需要一个判断条件就可以了
						if (idx_corner2_[i] >= 0) {
							// 拿出来找到的上一帧率的距离最近的两个点
							tripod1 = last_corner_cloud_->points[idx_corner1_[i]];
							tripod2 = last_corner_cloud_->points[idx_corner2_[i]];
							
							// 选择的特征点记为O，kd-tree最近距离点记为1，另一个最近距离点记为2
							float x0 = point_sel.x;
							float y0 = point_sel.y;
							float z0 = point_sel.z;
							float x1 = tripod1.x;
							float y1 = tripod1.y;
							float z1 = tripod1.z;
							float x2 = tripod2.x;
							float y2 = tripod2.y;
							float z2 = tripod2.z;
							// 模为：
							float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							                  * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							                  + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
							                    * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
							                  + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))
							                    * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));
							
							// 两个最近距离点之间的距离，即向量AB的模
							float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
							// NOTE: l-abc is the distance direction from the line to the point
							// 向量[la；lb；lc] 为距离ld2分别对[x0 y0 z0](选中的特征点)的偏导，具体推导见IPad：goodnotes：loam笔记
							// ∂(ld2) / ∂(x0)
							float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							            + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;
							
							float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							             - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
							
							float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
							             + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
							
							// 点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|
							float ld2 = a012 / l12;
							// 权重计算，距离越大权重越小，距离越小权重越大，得到的权重范围<=1
							// TODO 这个地方是什么意思？？？
							float s = 1;
							if (iter_count >= 5) {
								// 5次迭代之后开始  减小 权重因素  ???
								s = 1 - 1.8f * fabs(ld2);
							}
							// 考虑权重
							coeff.x = s * la;
							coeff.y = s * lb;
							coeff.z = s * lc;
							coeff.intensity = s * ld2;
							
							// NOTE: ld2 <= 0.5
							if (s > 0.1 && ld2 != 0) {
								// 只保留权重大的，也即距离比较小的点，同时也舍弃距离为零的
								laser_cloud_ori_->push_back(corner_points_sharp_->points[i]);
								coeff_sel_->push_back(coeff);
							}
						}
					} // NOTE: for corner points
					
					// 当前状态：具有了每个点到其对应直线的距离，以及权重，接下来开始对于每个平面点，进行同样的操作
					// 对本次接收到的曲率最小的点,从上次接收到的点云曲率比较小的点中找三点组成平面，一个使用kd-tree查找，另外一个在同一线上查找满足要求的，第三个在不同线上查找满足要求的
					// ===============================开始处理平面点==============================
					for (int i = 0; i < num_curr_surf_points_flat; ++i) {
						TransformToStart(surf_points_flat_->points[i], point_sel);
						// 每隔五次重新寻找最邻近点
						if (iter_count % 5 == 0) {
							// kd-tree最近点查找
							kdtree_surf_last_->nearestKSearch(point_sel, 1, point_search_idx, point_search_sq_dis);
							// 初始化三个要寻找的点的index
							int closest_point_idx = -1, second_closet_point_idx = -1, third_clost_point_idx = -1;
							if (point_search_sq_dis[0] < 25) {
								closest_point_idx = point_search_idx[0];
								// 获取最邻近的点在上一帧所在的scanID
								int closestPointScan = int(last_surf_cloud_->points[closest_point_idx].intensity);
								
								float point_sq_dis, point_sq_dis2 = 25, point_sq_dis3 = 25;
								// ===========向scanID增大的方向寻找===========
								for (int j = closest_point_idx + 1; j < last_surf_size; j++) {
									// 找到最邻近的下一条直线去了，就退出
									if (int(last_surf_cloud_->points[j].intensity) > closestPointScan + 2.5) {
										break;
									}
									
									point_sq_dis = CalcSquaredDiff(last_surf_cloud_->points[j], point_sel);
									// 如果第二个点和上一个最邻近点在同一条scan中：
									// 即论文中的点j和点l
									if (int(last_surf_cloud_->points[j].intensity) <= closestPointScan) {
										// 如果点的线号小于等于最近点的线号(应该最多取等，也即同一线上的点!!!)
										if (point_sq_dis < point_sq_dis2) {
											point_sq_dis2 = point_sq_dis;
											second_closet_point_idx = j;
										}
									}
										// 如果和i不在同一条scan中，即paper中的点m
									else {
										//如果点处在大于该线上
										if (point_sq_dis < point_sq_dis3) {
											point_sq_dis3 = point_sq_dis;
											third_clost_point_idx = j;
										}
									}
								}
								// ===========向scanID减小的方向寻找===========
								for (int j = closest_point_idx - 1; j >= 0; j--) {
									if (int(last_surf_cloud_->points[j].intensity) < closestPointScan - 2.5) {
										break;
									}
									
									point_sq_dis = CalcSquaredDiff(last_surf_cloud_->points[j], point_sel);
									
									if (int(last_surf_cloud_->points[j].intensity) >= closestPointScan) {
										if (point_sq_dis < point_sq_dis2) {
											point_sq_dis2 = point_sq_dis;
											second_closet_point_idx = j;
										}
									} else {
										if (point_sq_dis < point_sq_dis3) {
											point_sq_dis3 = point_sq_dis;
											third_clost_point_idx = j;
										}
									}
								}
							}
							// kd-tree最近距离点,-1表示未找到满足要求的点
							idx_surf1_[i] = closest_point_idx;
							// 同一线号上的距离最近的点，-1表示未找到满足要求的点
							idx_surf2_[i] = second_closet_point_idx;
							// 不同线号上的距离最近的点，-1表示未找到满足要求的点
							idx_surf3_[i] = third_clost_point_idx;
						}
						// 这个地方就是必须2 3同时找到了
						if (idx_surf2_[i] >= 0 && idx_surf3_[i] >= 0) {
							// 找到了三个点
							tripod1 = last_surf_cloud_->points[idx_surf1_[i]];
							tripod2 = last_surf_cloud_->points[idx_surf2_[i]];
							tripod3 = last_surf_cloud_->points[idx_surf3_[i]];
							// 具体推导公式见Ipad goodnotes LOAM笔记
							// 求∂(pd2) / ∂(x0)(少一个模长，下面pa /= ps添加了)
							float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
							           - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
							// 求∂(pd2) / ∂(y0)(少一个模长，下面pb /= ps添加了)
							float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
							           - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
							// 求∂(pd2) / ∂(z0)(少一个模长，下面pc /= ps添加了)
							float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
							           - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
							float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);
							
							// 法向量的模
							float ps = sqrt(pa * pa + pb * pb + pc * pc);
							// pa pb pc为法向量各方向上的单位向量
							pa /= ps;
							pb /= ps;
							pc /= ps;
							pd /= ps;
							
							// 点到面的距离：向量OA与与法向量的点积除以法向量的模
							float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;
							
							// 同理计算权重
							float s = 1;
							// TODO 和上面边缘点一样，这里没明白为什么
							if (iter_count >= 5) {
								// ==又一次迷惑，这个地方为什么要这样更新这个s参数？？？？？？？==
								s = 1 - 1.8f * fabs(pd2) / sqrt(CalcPointDistance(point_sel));
							}
							
							coeff.x = s * pa;
							coeff.y = s * pb;
							coeff.z = s * pc;
							coeff.intensity = s * pd2;
							
							if (s > 0.1 && pd2 != 0) {
								laser_cloud_ori_->push_back(surf_points_flat_->points[i]);
								coeff_sel_->push_back(coeff);
							}
						}
					} // NOTE: for plane points
					
					// 匹配到的点的个数(即存在多少个约束) （包括边缘点和平面点，不包括less的点）
					int num_point_sel = laser_cloud_ori_->points.size();
					// =======如果获取的特征点太少，不足10个，continue没用？？？？？？应该是break吧？？？=========================
					// 答：不是，后面的迭代中，从五次迭代之后s会变化======================
					if (num_point_sel < 10) {
						continue;
					}
					
					Eigen::Matrix<float, Eigen::Dynamic, 6> mat_A(num_point_sel, 6);
					Eigen::Matrix<float, 6, Eigen::Dynamic> mat_At(6, num_point_sel);
					Eigen::Matrix<float, 6, 6> mat_AtA;
					Eigen::VectorXf mat_B(num_point_sel);
					Eigen::Matrix<float, 6, 1> mat_AtB;
					Eigen::Matrix<float, 6, 1> mat_X;
					
					SO3 R_SO3(transform_es_.rot); /// SO3
					
					// 计算matA,matB矩阵 Jacobian矩阵  pointSelNum行 6列
					for (int i = 0; i < num_point_sel; ++i) {
						/* 采用Levenberg-Marquardt计算
						 * 首先建立当前时刻Lidar坐标系下提取到的特征点与点到直线/平面
						 * 的约束方程。而后对约束方程求对坐标变换(3旋转+3平移)的偏导
						 * 公式参见论文(2)-(8)*/
						const PointT &point_ori = laser_cloud_ori_->points[i];
						coeff = coeff_sel_->points[i];
						Eigen::Vector3f p(point_ori.x, point_ori.y, point_ori.z);
						Eigen::Vector3f w(coeff.x, coeff.y, coeff.z);
						Eigen::Vector3f p_minus_t = p - transform_es_.pos;

//        Eigen::Vector3f J_r = w.transpose() * RotationTransposeVectorJacobian(R_SO3, p_minus_t);
						Eigen::Vector3f J_r = w.transpose() * SkewSymmetric(transform_es_.rot.conjugate() * p_minus_t);
						Eigen::Vector3f J_t = -w.transpose() * transform_es_.rot.toRotationMatrix().transpose();
						
						// float s = 1;
						// ？？？？？？？？迷惑行为？？？s = 1
						// 猜测：之前在transToStart部分已经去除了运动畸变了，所以这里就把s当做1了，
						// 因为当前时刻，所有点的坐标都是一个sweep的起始坐标，transform是上一个sweep到这个sweep的转移矩阵，
						// 然后依据允许运动模型，当做和上次的transform是相同的，作为一个优化的起始位姿
						// J = ∂f / ∂T_k+1
						// intensity是 s * 距离
						float d2 = coeff.intensity;
						
						mat_A(i, 0) = J_r.x();
						mat_A(i, 1) = J_r.y();
						mat_A(i, 2) = J_r.z();
						mat_A(i, 3) = J_t.x();
						mat_A(i, 4) = J_t.y();
						mat_A(i, 5) = J_t.z();
//        mat_B(i, 0) = -0.05 * d2;
						// TODO 为什么要乘-0.05
						mat_B(i, 0) = -0.1 * d2;
					}
					
					// 最小二乘计算(QR分解法)
					mat_At = mat_A.transpose();
					mat_AtA = mat_At * mat_A;
					mat_AtB = mat_At * mat_B;
					
					// cout << "mat_At" << endl << mat_At << endl;
					// cout << "mat_B" << endl << mat_B << endl;
					
					// 求解matAtA * matX = matAtB   得到的结果就是△x
					mat_X = mat_AtA.colPivHouseholderQr().solve(mat_AtB);
					
					Eigen::Matrix<float, 6, 6> mat_P;
					
					// 第一次开始迭代
					if (iter_count == 0) {
						//特征值1*6矩阵F
						Eigen::Matrix<float, 1, 6> mat_E;
						Eigen::Matrix<float, 6, 6> mat_V;
						Eigen::Matrix<float, 6, 6> mat_V2;
						//求解特征值/特征向量
						// 此时的matATA就是 海塞矩阵 H
						Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6> > esolver(mat_AtA);
						mat_E = esolver.eigenvalues().real();
						mat_V = esolver.eigenvectors().real();
						
						mat_V2 = mat_V;
						
						is_degenerate = false;
						// TODO 发生退化要怎么处理
						// 特征值取值门槛
						float eign_thre[6] = {10, 10, 10, 10, 10, 10};
						for (int i = 0; i < 6; i++) {
							//从小到大查找
							if (mat_E(0, i) < eign_thre[i]) {
								// 特征值太小，则认为处在兼并环境中，发生了退化
								for (int j = 0; j < 6; j++) {
									// 对应的特征向量置为0
									mat_V2(i, j) = 0;
								}
								cout << mat_E << endl;
								is_degenerate = true;
							} else {
								break;
							}
						}
						// 计算P矩阵
						// TODO p矩阵是干啥的
						mat_P = mat_V2 * mat_V.inverse();
					}
					
					if (is_degenerate) {
						// 如果发生退化，只使用预测矩阵P计算
						Eigen::Matrix<float, 6, 1> mat_X2;
						mat_X2 = mat_X;
						mat_X = mat_P * mat_X2;
					}
					
					// 对数映射获取李代数
					Eigen::Vector3f r_so3 = R_SO3.log();
					
					// 旋转更新量
					r_so3.x() += mat_X(0, 0);
					r_so3.y() += mat_X(1, 0);
					r_so3.z() += mat_X(2, 0);
					
					// DLOG(INFO) << "mat_X: " << mat_X.transpose();
					
					transform_es_.pos.x() += mat_X(3, 0);
					transform_es_.pos.y() += mat_X(4, 0);
					transform_es_.pos.z() += mat_X(5, 0);
					
					if (!isfinite(r_so3.x())) r_so3.x() = 0;
					if (!isfinite(r_so3.y())) r_so3.y() = 0;
					if (!isfinite(r_so3.z())) r_so3.z() = 0;

//      SO3 es_SO3 = SO3::exp(r_so3);
//      transform_es_.rot = es_SO3.unit_quaternion();
					
					// 利用计算出的△x，更新transform，计算下次的误差累加每次迭代的旋转平移量
					// 搞明白这个公式，或者做好自己的想法
					// 解释 一个旋转量的更新，可以利用q * 【1， 1/2w】 （w代表旋转向量） 来更新旋转量
					transform_es_.rot = transform_es_.rot * DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0)));
					
					if (!isfinite(transform_es_.pos.x())) transform_es_.pos.x() = 0.0;
					if (!isfinite(transform_es_.pos.y())) transform_es_.pos.y() = 0.0;
					if (!isfinite(transform_es_.pos.z())) transform_es_.pos.z() = 0.0;
					
					// 计算旋转平移量，如果很小就停止迭代
					float delta_r = RadToDeg(R_SO3.unit_quaternion().angularDistance(transform_es_.rot));
					float delta_t = sqrt(pow(mat_X(3, 0) * 100, 2) +
					                     pow(mat_X(4, 0) * 100, 2) +
					                     pow(mat_X(5, 0) * 100, 2));
					
					if (delta_r < delta_r_abort_ && delta_t < delta_t_abort_) {
						DLOG(INFO) << "iter_count: " << iter_count;
						break;
					}
					
				} /// iteration
			} /// enough points
			Twist<float> transform_se = transform_es_.inverse();
			Twist<float> transform_sum_tmp = transform_sum_ * transform_se;
			transform_sum_ = transform_sum_tmp;
			
			ROS_DEBUG_STREAM(transform_sum_.pos.transpose() << endl << transform_sum_.rot.coeffs().transpose());
			
			TransformToEnd(corner_points_less_sharp_);
			TransformToEnd(surf_points_less_flat_);
			
			// 归一化
			transform_es_.rot.normalize();
			DLOG(INFO) << "transform diff: " << transform_es_;
		} /// enable odom
		
		corner_points_less_sharp_.swap(last_corner_cloud_); // NOTE: move current less feature points into last clouds
		// 将surfPointLessFlat与laserCloudSurfLast交换，目的保存surfPointsLessFlat的值下轮使用
		surf_points_less_flat_.swap(last_surf_cloud_);
		
		last_corner_size = last_corner_cloud_->points.size();
		last_surf_size = last_surf_cloud_->points.size();
		
		if (last_corner_size > 10 && last_surf_size > 100) {
			kdtree_corner_last_->setInputCloud(last_corner_cloud_);
			kdtree_surf_last_->setInputCloud(last_surf_cloud_);
		}
		
		ROS_DEBUG_STREAM("odom: " << tic_toc_.Toc() << " ms");
		/// process ends
		
		PublishResults();
		
	} // PointOdometry::Process
	
	void PointOdometry::PublishResults() {
		
		if (!is_ros_setup_) {
			DLOG(WARNING) << "ros is not set up, and no results will be published";
			return;
		}
		
		// publish odometry transformations
		geometry_msgs::Quaternion geo_quat;
		geo_quat.x = transform_sum_.rot.x();
		geo_quat.y = transform_sum_.rot.y();
		geo_quat.z = transform_sum_.rot.z();
		geo_quat.w = transform_sum_.rot.w();
		
		laser_odometry_msg_.header.stamp = time_corner_points_sharp_;
		laser_odometry_msg_.pose.pose.orientation = geo_quat;
		laser_odometry_msg_.pose.pose.position.x = transform_sum_.pos.x();
		laser_odometry_msg_.pose.pose.position.y = transform_sum_.pos.y();
		laser_odometry_msg_.pose.pose.position.z = transform_sum_.pos.z();
		pub_laser_odometry_.publish(laser_odometry_msg_);
		
		laser_odometry_trans_.stamp_ = time_corner_points_sharp_;
		laser_odometry_trans_.setRotation(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w));
		laser_odometry_trans_.setOrigin(
						tf::Vector3(transform_sum_.pos.x(), transform_sum_.pos.y(), transform_sum_.pos.z()));
		tf_broadcaster_.sendTransform(laser_odometry_trans_);
		
		
		/// for test
		geo_quat.x = transform_es_.rot.x();
		geo_quat.y = transform_es_.rot.y();
		geo_quat.z = transform_es_.rot.z();
		geo_quat.w = transform_es_.rot.w();
		laser_odometry_msg_.pose.pose.orientation = geo_quat;
		laser_odometry_msg_.pose.pose.position.x = transform_es_.pos.x();
		laser_odometry_msg_.pose.pose.position.y = transform_es_.pos.y();
		laser_odometry_msg_.pose.pose.position.z = transform_es_.pos.z();
		pub_diff_odometry_.publish(laser_odometry_msg_);
		
		
		// publish cloud results according to the input output ratio
		if (io_ratio_ < 2 || frame_count_ % io_ratio_ == 1) {
			
			ros::Time sweepTime = time_corner_points_sharp_;
			if (enable_odom_) {
				TransformToEnd(full_cloud_);  // transform full resolution cloud to sweep end before sending it
			}
			
			if (compact_data_) {
				TicToc tic_toc_encoder;
				
				PointCloud compact_data;
				
				PointT compact_point;
				
				{
					// NOTE: push_back odom
					compact_point.x = transform_sum_.pos.x();
					compact_point.y = transform_sum_.pos.y();
					compact_point.z = transform_sum_.pos.z();
					compact_data.push_back(compact_point);
					
					compact_point.x = transform_sum_.rot.x();
					compact_point.y = transform_sum_.rot.y();
					compact_point.z = transform_sum_.rot.z();
					compact_point.intensity = transform_sum_.rot.w();
					compact_data.push_back(compact_point);
				}
				
				{
					compact_point.x = last_corner_cloud_->size();
					compact_point.y = last_surf_cloud_->size();
					compact_point.z = full_cloud_->size();
					compact_data.push_back(compact_point);
					
					compact_data += (*last_corner_cloud_);
					compact_data += (*last_surf_cloud_);
					compact_data += (*full_cloud_);
				}
				
				PublishCloudMsg(pub_compact_data_, compact_data, sweepTime, "/camera");
				
				ROS_DEBUG_STREAM("encode compact data and publish time: " << tic_toc_encoder.Toc() << " ms");
			} else {
				PublishCloudMsg(pub_laser_cloud_corner_last_, *last_corner_cloud_, sweepTime, "/camera");
				PublishCloudMsg(pub_laser_cloud_surf_last_, *last_surf_cloud_, sweepTime, "/camera");
				PublishCloudMsg(pub_full_cloud_, *full_cloud_, sweepTime, "/camera");
			}
		}
	}
}