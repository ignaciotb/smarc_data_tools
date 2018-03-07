/*
MIT License

Copyright (c) 2016 LAMBERT Clarisse

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * \file      graphSlam.h
 * \author    LAMBERT Clarisse
 * \date      June, 2016
 * \brief     Headers for the GraphSlam class. This class manipulates graph
 *				 to implement a graph-based SLAM according to the g2o librairy. 
 *
 * \details   This class uses the GaussNewton Algorithm to solve the optimization.
 *				The slam is performed in unknown environement with observed features. 
 *				These are detected using MSIS. The robot localization is determined 
 *				with USBL and compass sensor. 
 *				Assumption : 
 *					The nodes and the robot position measurements are expressed
 *				in the global coordonate frame, whereas the landmark 
 *				measurements are in robot coodinate frame.
 *					The landmark node (localization) are a cartesian coordonates 
 *					and the landmark measurements in polar coordonates.
 */


#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <Eigen/Geometry>

#include "std_msgs/String.h"
#include <string>
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/Pose.h>

#include <sstream>
#include <fstream> 
#include <math.h>

// g2o include 
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/stuff/misc.h>

#include "simulator.h"
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "types_optim_slam2d.h"

// namespaces
using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

//typedef using PCL librairy 
typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZINormal PointTNormal;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointTNormal> PointCloudTNormal;

//typedef using g2o librairy : the optimizer 
typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver; 




/** \class GraphSlam graphSlam.h <optim/voiture.h> */
class GraphSlam{
public:

 	SparseOptimizer  _optimizer; 
private: 
  	int _globalId ; /*!< Global node id in graph  */

  	std::vector<int> _posesId; /*!< nodes id list */
  	std::vector<SE2> _poses ;
  	std::vector<SE2> _kalmanPose ;
  	std::vector<int> _lkId; /*!< nodes id list */

 	/* allocating the optimizer */
    //SlamLinearSolver* linearSolver;
    //SlamBlockSolver* blockSolver ;
 	OptimizationAlgorithmGaussNewton* solver;

 	/* Initialisation of the measurement pose error */
 	// the less the Covariance matrix, the larger the Information
  	Eigen::Matrix2d _landmarkCovariance; /*!< covariance relative to the landmark measurement  Dim=2x2*/  
  	Eigen::Matrix2d _landmarkInformation; /*!< information computed by the inverse of covariance matrix Dim=2x2*/ 
	Eigen::Matrix3d _robotPoseCovariance; /*!< covariance relative to the current pose Dim3x3*/  
	Eigen::Matrix3d _robotPoseInformation; /*!< information computed by the inverse of covariance matrix Dim=3x3*/ 
 	
 	
  	Simulator simulator;
  	ParameterSE2Offset* sensorOffset = new ParameterSE2Offset;

    
    double _dtUsblComp ; /*!< delta t bewteen Usbl and Compass measurement  */
    std::vector<double> _dtkalman; /*!< vector of deltaT between two usbl measurements : */

  	PointCloudT::Ptr map ;

public:
	GraphSlam(double landmarkCovariance = 0.4, double robotPoseCovariance = 0.4);
	~GraphSlam();

	/**
	* \brief      Add an pose in the optimazer 
 	* \details    
 	* \param[in]    pose         robot pose 
 	* \return    
 	*/
	void add2DPose(Eigen::Vector3d pose);



	/**
	* \brief      Add an pose in the optimazer 
 	* \details    
 	* \param[in]    landmark         landmark localization
  	* \param[in]    TdataAss         map and feature association 
  	* \param[in]    eraseId          map points erased
  	* \param[in]    map        		 actual map during after the ICP 
 	* \return    
 	*/
	void addPointTNodeVector(PointCloudT::Ptr landmarks, std::vector<int> & TdataAss, std::vector<int> & eraseId, PointCloudT::Ptr map);


private: 


	Eigen::Vector2d cartesianToPolar(Eigen::Vector2d cartesian);
	void writePoseinTxt(SE2 pose_computed,double dt);

	/**
	 * \brief      Optimaze the graph with the active nodes and edges   
	 */
	void optimization();

	/**
	 * \brief      Allow to erase the uncertain point of the map before the last optimization
	 * \details    
	 * \param[in]    map    	actual map with the certainty
	 * \param[in]    lim        lim for the erasing
	 */
	void updateOptimization(PointCloudT::Ptr map, int lim);


	/**
	 * \brief      Compute and add the edge between two poses
	 * \details    
	 * \param[in]    prev_pose    previous pose x-1
	 * \param[in]    pose         pose x
	 * \return    
	 */
	void add_pose_edge(int prevPoseId, SE2 prevPose, int poseId, SE2 pose);


	/**
	 * \brief      Compute and add the edge between one pose and one landmark
	 * \details    
	 * \param[in]    pose    	 		pose id where the landmark is seen 
	 * \param[in]    robotPoseXYT       pose where the landmark is seen in odom frame
	 * \param[in]    landmark    	 	landmark id 
	 * \param[in]    landmarkRobotPose  landmark in the robot pose frame 
	 * \return    
	 */
	void add_landmark_edge(int pose, SE2 robotPoseXYT,int landmark, Eigen::Vector2d landmarkRobotPose);




};

