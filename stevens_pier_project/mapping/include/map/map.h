/**
 *	Maintaines a 2d feature map from a 3d sonar scanþ
 *	Each feature is assumed to be infinetly high
 *  @author Unnar Axelsson
 */

#ifndef SONARMAP_H
#define SONARMAP_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Bool.h>
#include "scanBuffer/scanBuffer.h"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace mapping{

class sonarMap{

PointCloudT::Ptr map;
std::vector<PointCloudT::Ptr> pillarPoints;
std::vector<PointCloudT::Ptr> featurePoints;
std::vector<int> featureCount;
PointCloudT::Ptr aligned_tmp;
PointCloudT::Ptr features;
PointCloudT::Ptr pillars;
scanBuffer sb;
tf::TransformListener listener_;
ros::Publisher *pubTransform_;

public:
	sonarMap(){
		map = PointCloudT::Ptr (new PointCloudT());
		aligned_tmp = PointCloudT::Ptr (new PointCloudT());
		aligned_tmp->header.frame_id="odom";
		features = PointCloudT::Ptr (new PointCloudT());
		pillars = PointCloudT::Ptr (new PointCloudT());
		pillars->header.frame_id="odom";
		features->header.frame_id="odom";
	}

	void setPublisher(ros::Publisher *pubTransform, ros::Publisher *pubTransformSB){
		pubTransform_ = pubTransform;
		sb.setPublisher(pubTransformSB);
	}

	/**
	 * Takes in a new sonar scan, processes the data and calles 
	 * the appropriate functions to update the map.
	 * @param scan PointCloud where each point represents a possible feature. 
	 */
	bool newScan(PointCloudT::Ptr scan);

	// Returns the map in the form of pointCLoud.
	PointCloudT::Ptr returnMap(void){ return map; }
	// Returns the last scan aligned to the current map. 
	PointCloudT::Ptr returnAligned(void){ return aligned_tmp; }
	// Returen the buffer that has the highest number of scans.
	PointCloudT::Ptr returnCurrentBuffer(void){ return features; }

	PointCloudT::Ptr returnPillars(void){ return pillars; }

	std::vector<PointCloudT::Ptr> returnPillarPoints(void);
	std::vector<PointCloudT::Ptr> returnFeaturePoints(void);

	void updateBuffer(void){
		// sb.updateBuffer();
	}
private:
	/**
	 * Function is called to initialize the map or check if it has been initialized.
	 * @param  scan PointCloud where each point is a possible feature, 
	 *              assumes all points have been projected to the xy-plane.
	 * @return      Returns true if map has been initialized, false otherwise.
	 */
	bool intializeMap(const PointCloudT::Ptr scan);

	// Simple passthrough filter, bad implementation, to be TERMINATED.
	void passThroughFilter(const PointCloudT::Ptr scan, PointCloudT::Ptr filtered, float lim_low);
	
	/**
	 * Uses icp from pcl to aligne pointcloud to the map we have.
	 * @param  map     2d feature map to be aligned to. 
	 * @param  scan    Feature scan to align.
	 * @param  aligned The aligned feature scan
	 * @param  trans   4by4 transformation matrix describing the transformation.
	 * @return         Return true if icp was successful, false otherwise.
	 */
	bool icpAlign(const PointCloudT::Ptr map, const PointCloudT::Ptr scan, const pcl::KdTreeFLANN<PointT> &kdtree, PointCloudT &aligned, Eigen::Affine3f &trans);
	
	/**
	 * Takes in feature scan that is aligned to the map and find associations between them and updates the map accordingly.
	 * @param aligned Aligned feature scan in the form of a pointcloud.
	 */
	void dataAssociation(PointCloudT::Ptr aligned);

	// Functions that change the intensity of pointclouds based on intensity criterias.
	void changeIntensity(float limLow, float limHigh, float change);
	void changeIntensity(float limLow, float limHigh, float change, std::vector<int> idx);
	
	// Removes obsolete map features, to be COMBINED WITH PASSTHROUGH FILTER.
	void pruneMap(void);

	/**
	 * Takes in 3d rotation as euler angles and minimizes the angles.
	 * @param euler vector with 3d rotation in form of euler angles.
	 */
	void minimizeEuler(Eigen::Vector3f &euler);
	
	/**
	 * Finds the transformation from a feature scan with two points to the map. 
	 * @param  map    2d feature map to be aligned to.
	 * @param  scan   Feature scan to align, only 2 points.
	 * @param  kdtree KD-tree created from the map.
	 * @param  trans  The resulting transformation
	 * @return        Returns true if alignment was successful, false otherwise.
	 */
	bool matchTwoPoints(PointCloudT::Ptr map, PointCloudT::Ptr scan, const pcl::KdTreeFLANN<PointT> &kdtree, Eigen::Affine3f &trans);

	/**
	 * Takes in two pairs of points and finds the transformation between them.
	 * @param  a1 Point 1 from the map, connected to b1.
	 * @param  a2 Point 2 from the map, connected to b2.
	 * @param  b1 Point 1 from the scan, connected to a1.
	 * @param  b2 Point 2 from the scan, connected to a2.
	 * @return    transformation between the two pairs of points.
	 */
	Eigen::Affine3f transformationBetweenPairsOfPoints(PointT &a1, PointT &a2, PointT &b1, PointT &b2);

	void publishSonarTransform(Eigen::Affine3f translation);

};

}
#endif