/**
 *	Maintaines a buffer of previous scans.
 *  @author Unnar Axelsson
 */

#ifndef SCANBUFFER_H
#define SCANBUFFER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <dlib/rand.h>
#include <dlib/svm_threaded.h>

using namespace std;
using namespace dlib;


 /**  \typedef  PointT: each beam is associate with x,y,z and intensity   */
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// dlib typedefs
typedef matrix<double,20,1> sample_type_short;
typedef matrix<double,40,1> sample_type;
typedef one_vs_one_trainer<any_trainer<sample_type> > ovo_trainer;
typedef polynomial_kernel<sample_type> poly_kernel;


/**
 * \brief     setting og buffer
 * \details 	parameters are finding in mapping/include/parameters/MapBuilder.yaml
 */
struct scanBufferParams{
	int bufferSize;         /**< Number of buffers at any time			*/
	int scanSize;           /**<  Maximum number of beams stored in each buffer			*/
	float ecTolerance;      /**<  maximum to closest point within cluster			*/
	float ecMinClusterSize; /**<  Minimum size of a single cluster			*/
	string outputLines;     /**<  Output topic for line fit over cluster			*/
	string outputBuffUpdate;/**<  Output topic for buffer			*/
	string outputBuff;      /**<  Output topic for buffer			*/
	// int maxIntensity;    /**<  Maximum intensity point that a cluster needs to include			*/
};

struct buff{
	PointCloudT::Ptr buffer;
	std::vector<float> dist;
	int countStart;
	int countLast;

	buff(){
		buffer = PointCloudT::Ptr (new PointCloudT());
		countStart = 0;
		countLast = 0;
	}
};


class scanBuffer{

	ros::NodeHandle *nh_;
	ros::Publisher pubLines_;
	ros::Publisher pubBuff_;
	ros::Publisher pubBuffUpdate_;
	ros::Subscriber _sub_poseKalman; 

	scanBufferParams par_;
	std::vector<buff> buff_;

	std::vector<PointCloudT::Ptr> clouds_;
	tf::TransformListener listener_;
	std::vector<int> cloudsCount_;
	int idxMax;
	int countLastProcessed;

	ovo_trainer trainer;
	svm_nu_trainer<poly_kernel> poly_trainer;
	one_vs_one_decision_function<ovo_trainer, decision_function<poly_kernel> > df;
	vector_normalizer<sample_type_short> normalizer;
	probabilistic_decision_function<poly_kernel> df_1_3;

	bool isFirstPose; 
	double nbrBeamReceived ;

public:

	scanBuffer();

	void initialize(ros::NodeHandle *nh);

	bool newScan(buff bu, PointCloudT::Ptr out, int &type);
	void updateBuffer(Eigen::Affine3f trans);

private:

	void processBuffer(buff bu, PointCloudT::Ptr out, int &type);
	void findWalls(const buff bu, PointCloudT::Ptr out, std::vector<PointCloudT::Ptr> &walls);
	void findPillars(const buff bu, PointCloudT::Ptr out);
	void loadParams(void);

};

#endif
