

#ifndef LIO_POINTODOMETRY_H_
#define LIO_POINTODOMETRY_H_

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <std_srvs/SetBool.h>

#include "utils/Twist.h"
#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"
#include "3rdparty/sophus/se3.hpp"

namespace lio {
	
	using namespace std;
	using namespace mathutils;
	using namespace geometryutils;
	typedef pcl::PointXYZI PointT;
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
	typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
	typedef Twist<float> Transform;
	typedef Sophus::SO3f SO3;
	
	class PointOdometry {
	
	public:
		PointOdometry(float scan_period = 0.1,
		              int io_ratio = 2,
		              size_t num_max_iterations = 25);
		
		void SetupRos(ros::NodeHandle &nh);
		
		void Reset();
		
		void LaserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_sharp_msg);
		
		void LaserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_less_sharp_msg);
		
		void LaserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_flat_msg);
		
		void LaserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surf_points_less_flat_msg);
		
		void LaserFullCloudHandler(const sensor_msgs::PointCloud2ConstPtr &full_cloud_msg);
		
		bool HasNewData();
		
		void TransformToStart(const PointT &pi,
		                      PointT &po);
		
		size_t TransformToEnd(PointCloudPtr &cloud);
		
		void Process();
		
		void PublishResults();
		
		bool EnableOdom(std_srvs::SetBoolRequest &req,
		                std_srvs::SetBoolResponse &res) {
			enable_odom_ = req.data > 0;
			res.success = true;
			return true;
		}
		
		void Spin();
	
	private:
		TicToc tic_toc_;
		
		float scan_period_;
		float time_factor_;
		int io_ratio_;       ///< ratio of input to output frames
		long frame_count_;        ///< number of processed frames
		
		bool system_inited_ = false;
		size_t num_max_iterations_ = 25;
		Transform transform_es_; ///< transform from the start to the end
		Transform transform_sum_; ///< transform from the current to the init
		
		double delta_r_abort_;
		double delta_t_abort_;
		
		PointCloudPtr corner_points_sharp_;
		PointCloudPtr corner_points_less_sharp_;
		PointCloudPtr surf_points_flat_;
		PointCloudPtr surf_points_less_flat_;
		PointCloudPtr full_cloud_;
		
		PointCloudPtr last_corner_cloud_;
		PointCloudPtr last_surf_cloud_;
		PointCloudPtr laser_cloud_ori_;
		PointCloudPtr coeff_sel_;
		
		pcl::KdTreeFLANN<PointT>::Ptr kdtree_corner_last_;
		pcl::KdTreeFLANN<PointT>::Ptr kdtree_surf_last_;
		
		ros::Time time_corner_points_sharp_;      ///< time of current sharp corner cloud
		ros::Time time_corner_points_less_sharp_;  ///< time of current less sharp corner cloud
		ros::Time time_surf_points_flat_;         ///< time of current flat surface cloud
		ros::Time time_surf_points_less_flat_;     ///< time of current less flat surface cloud
		ros::Time time_full_cloud_;      ///< time of current full resolution cloud
		ros::Time time_imu_trans_;               ///< time of current IMU transformation information
		
		bool new_corner_points_sharp_;       ///< flag if a new sharp corner cloud has been received
		bool new_corner_points_less_sharp_;   ///< flag if a new less sharp corner cloud has been received
		bool new_surf_points_flat_;          ///< flag if a new flat surface cloud has been received
		bool new_surf_points_less_flat_;      ///< flag if a new less flat surface cloud has been received
		bool new_full_cloud_;       ///< flag if a new full resolution cloud has been received
		bool new_imu_trans_;                ///< flag if a new IMU transformation information cloud has been received
		
		vector<int> idx_corner1_;    ///< first corner point search index buffer
		vector<int> idx_corner2_;    ///< second corner point search index buffer
		
		vector<int> idx_surf1_;    ///< first surface point search index buffer
		vector<int> idx_surf2_;    ///< second surface point search index buffer
		vector<int> idx_surf3_;    ///< third surface point search index buffer
		
		nav_msgs::Odometry laser_odometry_msg_;       ///< laser odometry message
		tf::StampedTransform laser_odometry_trans_;   ///< laser odometry transformation
		
		ros::Publisher pub_laser_cloud_corner_last_;  ///< last corner cloud message publisher
		ros::Publisher pub_laser_cloud_surf_last_;    ///< last surface cloud message publisher
		ros::Publisher pub_full_cloud_;     ///< full resolution cloud message publisher
		ros::Publisher pub_diff_odometry_;         ///< laser odometry publisher
		ros::Publisher pub_laser_odometry_;         ///< laser odometry publisher
		ros::Publisher pub_compact_data_;         ///< laser odometry publisher
		tf::TransformBroadcaster tf_broadcaster_;  ///< laser odometry transform broadcaster
		
		ros::Subscriber sub_corner_points_sharp_;      ///< sharp corner cloud message subscriber
		ros::Subscriber sub_corner_points_less_sharp_;  ///< less sharp corner cloud message subscriber
		ros::Subscriber sub_surf_points_flat_;         ///< flat surface cloud message subscriber
		ros::Subscriber sub_surf_points_less_flat_;     ///< less flat surface cloud message subscriber
		ros::Subscriber sub_full_cloud_;      ///< full resolution cloud message subscriber
		
		bool is_ros_setup_ = false;
		bool compact_data_ = false;
		bool enable_odom_ = true;
		bool no_deskew_ = false;
		
		ros::ServiceServer enable_odom_service_;
		
	};
	
}

#endif //LIO_POINTODOMETRY_H_
