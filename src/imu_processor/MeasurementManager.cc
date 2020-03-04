/**
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
// Created by hyye on 3/27/18.
//

#include "imu_processor/MeasurementManager.h"

namespace lio {
	
	void MeasurementManager::SetupRos(ros::NodeHandle &nh) {
		is_ros_setup_ = true;
		
		nh_ = nh;
		// imu_topic = "/imu/data"
		sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(mm_config_.imu_topic, 1000,
		                                           &MeasurementManager::ImuHandler, this,
		                                           ros::TransportHints().tcpNoDelay());
		// compact data是由PointOdometry处理完点云数据之后，发布的压缩后的数据，包括点云帧间的估计R和P
		// compact_data_topic = "/compact_data"
		sub_compact_data_ = nh_.subscribe<sensor_msgs::PointCloud2>(mm_config_.compact_data_topic, 10,
		                                                            &MeasurementManager::CompactDataHandler, this);
/*  sub_laser_odom_ = nh_.subscribe<nav_msgs::Odometry>(mm_config_.laser_odom_topic, 10,
  				&MeasurementManager::LaserOdomHandler, this);*/
	}
	
	PairMeasurements MeasurementManager::GetMeasurements() {
		PairMeasurements measurements;
		while (true) {
			// 如果需要IMU
			if (mm_config_.enable_imu) {
				// compact_data是由lio estimator节点发布的压缩点云数据
				// 当任意一个缓存器数据空了，就将当前的measurements返回
				if (imu_buf_.empty() || compact_data_buf_.empty()) {
					ROS_INFO("no compact data Return...........");
					return measurements;
				}
				// imu_buf和compact_data_buf都是队列类型的，front代表队列第一个元素，back代表队列最后一个元素
				// 如果imu缓存器中最后一个（最新的）数据都比compact第一个（最早的）数据早，即：
				// 早=======imu=========晚
				//                           早======compact=====晚
				if (imu_buf_.back()->header.stamp.toSec()
				    <= compact_data_buf_.front()->header.stamp.toSec() + mm_config_.msg_time_delay) {
					ROS_DEBUG("wait for imu, only should happen at the beginning");
					// Count for waiting time
					return measurements;
				}
				// 如果imu的第一个数据比compact的第一个晚，即：
				// 早=======compact======晚
				//               早=======imu=======晚
				// 此时就把compact的第一个数据弹出（因为没有相对应的imu数据和它配对，这个数据就没用了）（一直弹出，直到compact数据和IMU的第一个数据对齐）
				if (imu_buf_.front()->header.stamp.toSec()
				    >= compact_data_buf_.front()->header.stamp.toSec() + mm_config_.msg_time_delay) {
					ROS_INFO("yes compact data Return...........");
					ROS_DEBUG("throw compact_data, only should happen at the beginning");
					compact_data_buf_.pop();
					continue;
				}
				// 经过了循环以上步骤，现在的情况是：
				// 早=======compact======晚
				// 早=======imu=======晚
				// ---------------------或者---------------------
				//   早=======compact======晚
				// 早=======imu=======晚
				// 存储compact点云队列的第一个元素并从队列中弹出
				CompactDataConstPtr compact_data_msg = compact_data_buf_.front();
				compact_data_buf_.pop();
				
				vector<sensor_msgs::ImuConstPtr> imu_measurements;
				// 将上一次的compact点云到这一次的compact点云过程中的所有的imu数据都放在一个imu_measurements里面，
				// 用于后面配成一对，进行处理
				while (imu_buf_.front()->header.stamp.toSec()
				       < compact_data_msg->header.stamp.toSec() + mm_config_.msg_time_delay) {
					imu_measurements.emplace_back(imu_buf_.front());
					imu_buf_.pop();
				}
				
				// 再多加一个compact点云时间点之后的imu数据
				// 注意：这里的imu数据就不要从queue中弹出了
				imu_measurements.emplace_back(imu_buf_.front());
				
				if (imu_measurements.empty()) {
					ROS_DEBUG("no imu between two image");
				}
				// 将一个compact数据和很多的imu数据的集合配成一对，压缩到measurements中
				measurements.emplace_back(imu_measurements, compact_data_msg);
			}
			// 如果不需要IMU
			else {
				// 新建一个空的imu_measurements
				vector<sensor_msgs::ImuConstPtr> imu_measurements;
				if (compact_data_buf_.empty()) {
					return measurements;
				}
				// 按照同样的格式，不过imu_measurements里面是空的数据
				CompactDataConstPtr compact_data_msg = compact_data_buf_.front();
				compact_data_buf_.pop();
				measurements.emplace_back(imu_measurements, compact_data_msg);
			}
		}
	}

// IMU_data话题的回调函数
	void MeasurementManager::ImuHandler(const sensor_msgs::ImuConstPtr &raw_imu_msg) {
		if (raw_imu_msg->header.stamp.toSec() <= imu_last_time_) {
			LOG(ERROR) << ("imu message in disorder!");
			return;
		}
		
		imu_last_time_ = raw_imu_msg->header.stamp.toSec();
		// 存入数据的时候要先锁定，防止其他线程占用
		buf_mutex_.lock();
		imu_buf_.push(raw_imu_msg);
		buf_mutex_.unlock();
		// 随机唤醒一个等待的线程
		con_.notify_one();
		
		{
			lock_guard<mutex> lg(state_mutex_);
/*     TODO: is it necessary to do predict here?
    Predict(raw_imu_msg);

    std_msgs::Header header = imu_msg->header;
    header.frame_id = "world";
    if (flag == INITIALIZED)
      PubLatestOdometry(states, header);*/
		}
	} // ImuHandler
	
	void MeasurementManager::LaserOdomHandler(const nav_msgs::OdometryConstPtr &laser_odom_msg) {
		buf_mutex_.lock();
		laser_odom_buf_.push(laser_odom_msg);
		buf_mutex_.unlock();
		con_.notify_one();
	}
	
	void MeasurementManager::CompactDataHandler(const sensor_msgs::PointCloud2ConstPtr &compact_data_msg) {
		buf_mutex_.lock();
		compact_data_buf_.push(compact_data_msg);
		buf_mutex_.unlock();
		con_.notify_one();
	}
	
}
