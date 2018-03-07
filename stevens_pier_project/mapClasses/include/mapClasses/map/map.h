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

typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZINormal PointTNormal;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointTNormal> PointCloudTNormal;

using namespace std;

struct mapParams{
	string outputTransform; // Output topic for sonar update
	string outputMapWalls;  // Output topic for map of walls
	string outputMapWallsAligned;  // Output topic for map of walls
};


class sonarMap{

ros::NodeHandle* nh_;
mapParams par_;

PointCloudT::Ptr map;
PointCloudTNormal::Ptr mapWalls;
std::vector<PointCloudT::Ptr> pillarPoints;
std::vector<PointCloudT::Ptr> featurePoints;
std::vector<int> featureCount;
PointCloudT::Ptr aligned_tmp;
PointCloudT::Ptr features; //initialize() -> creation , newWallScan() -> clear , newPillarScan() -> clear, dataAssociation(), icpAlign()
PointCloudT::Ptr pillars;
tf::TransformListener listener_;
ros::Publisher pubTransform_;
ros::Publisher pubMapWalls_;
ros::Publisher pubMapWallsAligned_;


public:

	sonarMap(){}

	sonarMap(ros::NodeHandle *nh){
		sonarMap::initialize(nh);
	}

	void initialize(ros::NodeHandle *nh);

	/**
	 * Takes in a new sonar scan, processes the data and calles 
	 * the appropriate functions to update the map.
	 * @param scan PointCloud where each point represents a possible feature. 
	 */
	bool newScan(PointCloudT::Ptr scan,std::vector<int> & TdataAss,std::vector<int> & erasedId,  const int type);

	// Returns the map in the form of pointCLoud.
	PointCloudT::Ptr returnMap(void){ return map; }
	// Returns the last scan aligned to the current map. 
	PointCloudT::Ptr returnAligned(void){ return aligned_tmp; }
	// Returen the buffer that has the highest number of scans.
	// PointCloudT::Ptr returnCurrentBuffer(void){ return features; }

	PointCloudT::Ptr returnPillars(void){ return pillars; }

	std::vector<PointCloudT::Ptr> returnPillarPoints(void);
	std::vector<PointCloudT::Ptr> returnFeaturePoints(void);
	
private:

	bool newPillarScan(PointCloudT::Ptr beam,std::vector<int> & TdataAss, std::vector<int> & erasedId);

	bool newWallScan(PointCloudT::Ptr beam);


	/**
	 * \brief 			Function is called to initialize the map or check if it has been initialized.
	 * \detail 			assume the PointCloud have been projected to the odom frame
	 *
	 * \param[in]  		scan 			PointCloud where each point is a possible feature
	 * \param[in\out]  	mapPtr 		PointCloud where each point is point in the map
	 * \param[in]  		size	 		minimal map size intended  
	 * \param[in]  		size	 		minimal size between two different points in the map 
	 * 
	 * \return      	Returns true if map has been initialized, false otherwise.
	 */
	bool intializeMapPillar(const PointCloudT::Ptr scan, PointCloudT::Ptr mapPtr, const int size, int sqr_distance);


	bool intializeMapWall(const PointCloudTNormal::Ptr scan, PointCloudTNormal::Ptr mapPtr, const int size);


	/**
	 * \brief 			Function is called to filter the input data according to the threshold
	 * \detail 			only the data with an intensity higher than the threshold are return 
	 *
	 * \param[in] 		  input 						PointCloud : input data  
	 * \param[out]   	  filtered_output 		PointCloud : ouput data  
	 * \param[in]  	  lim_low	 				threshold : low limite 
	 */
	void passThroughFilter(const PointCloudT::Ptr input, PointCloudT::Ptr filtered_output, float lim_low);
	
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
	void dataAssociation(PointCloudT::Ptr aligned, std::vector<int>  & TdataAss, std::vector<int> & erasedId );

	// Functions that change the intensity of pointclouds based on intensity criterias.
	void changeIntensity(float limLow, float limHigh, float change);
	void changeIntensity(float limLow, float limHigh, float change, std::vector<int> idx);
	
	// Removes obsolete map features, to be COMBINED WITH PASSTHROUGH FILTER.
	void pruneMap(std::vector<int> & erasedId);

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
	bool publishWallTransform(Eigen::Affine3f translation);

	void setParameters();

};

#endif