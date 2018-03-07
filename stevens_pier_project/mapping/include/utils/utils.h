#ifndef UTILS_H
#define UTILS_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <queue>

namespace utils{

	template<typename PointCloudT>
	void printCloud(PointCloudT a);

	template<typename PointT>
	float Distance(PointT a, PointT b);

	template<typename PointT>
	float Distance(PointT a);
	
	template<typename PointT>
	float squaredDistance(PointT a, PointT b);

	template<typename PointT>
	float squaredDistance(PointT a);

	bool listenForTransform(std::string from, std::string to, tf::StampedTransform transform);


	class poseTracker{
		std::queue<Eigen::Vector3f> pose;
	public:	
		poseTracker(){ };
		bool newPose(tf::StampedTransform transform);
	};


	namespace params{
		template<typename T>
		T loadParam( std::string name, ros::NodeHandle &nh);
	}

}

#include "utils.hpp"
#endif