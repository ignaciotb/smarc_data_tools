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
 * \file      graphSlam.cpp
 * \author    LAMBERT Clarisse
 * \date      June, 2016
 * \brief     Sources file for the GraphSlam class. This class manipulates graph
 *         to implement a graph-based SLAM according to the g2o librairy. 
 *
 * \details   This class uses the GaussNewton Algorithm to solve the optimization.
 *        The slam is performed in unknown environement with observed features. 
 *        These are detected using MSIS. The robot localization is determined 
 *        with USBL and compass sensor. 
 *        Assumption : 
 *          The nodes and the robot position measurements are expressed
 *        in the global coordonate frame, whereas the landmark 
 *        measurements are in robot coodinate frame.
 *          The landmark node (localization) are a cartesian coordonates 
 *          and the landmark measurements in polar coordonates.
 */

// header include
#include <optim/graphSlam.h>

// namespaces
using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

//typedef using PCL librairy 
typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZINormal PointTNormal;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointTNormal> PointCloudTNormal;


#include <sstream>

 /* Constructor and Destructor ----------------------------------------------------------------------- */
  GraphSlam::GraphSlam(double landmarkCovariance, double robotPoseCovariance){

	/* Initialise the robot pose id and landmark id of the vertices */
	_globalId = -1;

	/* Initialisation of the optimizer */
    //linearSolver = new SlamLinearSolver();
    //linearSolver->setBlockOrdering(false);
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();

  //blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
      g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

  //solver = new OptimizationAlgorithmGaussNewton(blockSolver);
	_optimizer.setAlgorithm(solver);

   /* compute pose covariance  according to the measurement pose error */
   _landmarkCovariance.fill(0.);
   _landmarkCovariance(0, 0) = landmarkCovariance  * landmarkCovariance ; // 0.05 * 0.05
   _landmarkCovariance(1, 1) = landmarkCovariance  * landmarkCovariance ;

  
   /* compute pose covariance  according to the measurement pose error */
   _robotPoseCovariance.fill(0.);
   _robotPoseCovariance(0, 0) = robotPoseCovariance * robotPoseCovariance;
   _robotPoseCovariance(1, 1) = robotPoseCovariance * robotPoseCovariance;
   _robotPoseCovariance(2, 2) = DEG2RAD(1.5) * DEG2RAD(1.5);

   SE2 sensorOffsetTransf(0.0, 0.0, 0.0);
   sensorOffset->setOffset(sensorOffsetTransf);
   sensorOffset->setId(0);
   _optimizer.addParameter(sensorOffset);

   PointCloudT::Ptr map (new PointCloudT());
  
  }

  GraphSlam::~GraphSlam(){
	 
   updateOptimization(map,1.0);
   optimization();
	 Factory::destroy();
	 OptimizationAlgorithmFactory::destroy();
	 HyperGraphActionLibrary::destroy();

  }
/* ----------------------------------------------------------------------- constructor and destructor */

void GraphSlam::updateOptimization(PointCloudT::Ptr map, int lim){
  std::vector<int> erasedFeature;

    for (int i = 0; i < map->points.size() ; ++i)
    {
      if (map->points[i].intensity < lim){
        erasedFeature.push_back(i);
      }
    }
    std::sort(erasedFeature.begin(), erasedFeature.end());
    for (int i = erasedFeature.size()-1; i >= 0 ; --i)
    {
       std::cout << " erase " << erasedFeature.at(i) << ",  " << _lkId.size() << ", vertexid supprimé :  " << _lkId[erasedFeature.at(i)] <<"                                    : graphSlam.cpp"<< std::endl ;
       _optimizer.removeVertex(_optimizer.vertex(_lkId[erasedFeature.at(i)]));
       double idDel = erasedFeature.at(i) ; 
       _lkId.erase(_lkId.begin()+idDel);
    }

}


void GraphSlam::optimization(){
        // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(_optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  _optimizer.setVerbose(true);
  
  //std::cout << "Optimizing" << std::endl;
  _optimizer.initializeOptimization();
  _optimizer.computeInitialGuess ();
  bool cool =  _optimizer.save( (ros::package::getPath("mapping") + "/savedData/2015-08-19-11-25-05/02/pillar02_OPTIM_IG.g2o").c_str() );
  _optimizer.optimize(10);
  //std::cout << "done." << std::endl;
  //const VertexSE2* v1 = static_cast<const VertexSE2*>_optimizer.VertexContainer.(_vertices[0]) ;
 _optimizer.save((ros::package::getPath("mapping") + "/savedData/2015-08-19-11-25-05/02/pillar02_OPTIM_A.g2o").c_str());
}


void GraphSlam::add2DPose(Eigen::Vector3d posePS){


/* Update global graph id */
 	_globalId ++ ;
  
/* Add the first robot pose */
  	if (_globalId == 0){
		/* --Create nodes according to the pose received-- */ 
      SE2 firstPose = SE2(posePS(0),posePS(1),posePS(2)) ;
      // first pose is assume without noise
		VertexSE2* init =  new VertexSE2;
		init->setId(_globalId); 
		init->setEstimate(firstPose);
		_optimizer.addVertex(init);   
      /* ------------------------------------------------------ */

      /* Save the robot pose id : useful to create edge between a robot pose and the landmarks that were seen there */
		_posesId.push_back(_globalId); 
    _poses.push_back(firstPose);
    _kalmanPose.push_back(firstPose) ;
      /* Update global graph id */
		_globalId ++ ;
   }
	/* --Create nodes according to the pose received-- */ 
   SE2 kposeSE2 = SE2(posePS(0),posePS(1),posePS(2)) ;
     // compute of the motion between the two poses
   SE2 deltaMeasurement =  _kalmanPose.back().inverse() * kposeSE2 ;
     // compute the new pose : Xi = Xj + kposeSE2 (linear ) with J = Id
   SE2 poseEstimated = kposeSE2 ;

	VertexSE2* robot =  new VertexSE2;
	robot->setId(_globalId); 
	robot->setEstimate(poseEstimated);
	_optimizer.addVertex(robot);
 	/* ------------------------------------------------------ */

   /* --Create edge between the two previous poses -- */ 
	GraphSlam::add_pose_edge(_posesId.back(), _kalmanPose.back(), _globalId, kposeSE2);
   /* ------------------------------------------------------ */

   _optimizer.save((ros::package::getPath("mapping") + "/savedData/2015-08-19-11-25-05/02/pillar02_OPTIM_B.g2o").c_str());
   /* Save the robot pose id : useful to create edge between a robot pose and the landmarks that were seen there */
   _posesId.push_back(_globalId); 
   _poses.push_back(poseEstimated);
   _kalmanPose.push_back(kposeSE2) ;


//   optimization of the graph every 10 poses 
   // if( (_posesId.size() % 10 ) ==  0){
   //    optimization();
   // }
}








void GraphSlam::addPointTNodeVector(PointCloudT::Ptr locations, std::vector<int> & TdataAss, std::vector<int> & eraseId, PointCloudT::Ptr mapupdate){
    //update the map 
    map = mapupdate ;

   bool first = _lkId.empty() ; 
   PointCloudT::Ptr tmp_loca (new PointCloudT());
   if (first) { // only new landmark, sort the vector 
      for (int i = 0; i < TdataAss.size(); ++i)
      {
         tmp_loca->points.push_back(locations->points[TdataAss[i]]);
      }
   }
   else{
       tmp_loca = locations ;
   }

   for (int i = 0; i < tmp_loca->points.size(); ++i)
   {

        // from pointCloudT to vector2d 
      Eigen::Vector2d locaOdomFrame(tmp_loca->points.at(i).x,tmp_loca->points.at(i).y);
        // from cartesian coordonate to polar coordonate
      Eigen::Vector2d polarOdomFrame = GraphSlam::cartesianToPolar(locaOdomFrame);
        // point in odom frame transformed in a point on body frame  
      Eigen::Vector2d locaPoseFrame = _kalmanPose.back().inverse() *locaOdomFrame ;
        // from cartesian coordonate to polar coordonate
      Eigen::Vector2d polarPoseFrame = GraphSlam::cartesianToPolar(locaPoseFrame);

      if (TdataAss[i] >= 0 && first == false )// landmark already seen 
      {
        
         double idLdk = TdataAss[i]  ;
         /* --Create edge between the previous poses and the landmark already seen  -- */ 
         GraphSlam::add_landmark_edge(_posesId.back(), _poses.back()  ,_lkId.at(idLdk), polarPoseFrame );  
      }
      else {
         if(TdataAss[i] == (-1) || first == true) { // new landmark

            /* Update global graph id */
            _globalId ++ ;
            /* --Create nodes according to the landmark location -- */ 
              // the function predictLocaLandmark compute the estimate measurement between the robot and the landmark position
            VertexPointXY* landmark_ = new VertexPointXY;
            landmark_->setId(_globalId);
            // estimate in cartesian coordonate
            landmark_->setEstimate(locaOdomFrame);
            _optimizer.addVertex(landmark_);
            /* ------------------------------------------------------ */
              // add the new landmark on the landmark id list 
            _lkId.push_back(_globalId);
            /* --Create edge between the previous poses and and the landmark already seen -- */ 
            GraphSlam::add_landmark_edge(_posesId.back(),_kalmanPose.back(), _globalId, polarPoseFrame); 

            /*  display the landmark vertex in file  */
            // ofstream myfile;
            // string file = "/home/clarisse/Documents/srcTestOctave/pillars.txt";
            // myfile.open(file.c_str(), std::ofstream::out | std::ofstream::app);
            // myfile << "VERTEXP " << locaPoseFrame(0) << " " << locaPoseFrame(1) << "\n";
            // myfile.close();
         }         
      }
   }

      // Erase the uncertain points
      std::sort(eraseId.begin(), eraseId.end());
      for (int i = eraseId.size()-1; i >= 0 ; --i)
      {
         _optimizer.removeVertex(_optimizer.vertex(_lkId[eraseId.at(i)]));
         double idDel = eraseId.at(i) ; 
         _lkId.erase(_lkId.begin()+idDel);
      }
  _optimizer.save((ros::package::getPath("mapping") + "/savedData/2015-08-19-11-25-05/02/pillar02_OPTIM_B.g2o").c_str());
}









Eigen::Vector2d GraphSlam::cartesianToPolar(Eigen::Vector2d cartesian){
   double r = sqrt(cartesian(0) * cartesian(0) + cartesian(1) * cartesian(1));
   double phi = std::atan2(cartesian(1), cartesian(0));
   return Eigen::Vector2d(r, phi) ;
}









void GraphSlam::add_pose_edge(int prevPoseId, SE2 prevPose, int poseId, SE2 pose){

   /* Creation of a new edge between the node[prev_pose] and the node [pose] */
   EdgeSE2* odometry = new EdgeSE2;
   odometry->vertices()[0] = _optimizer.vertex(prevPoseId);
   odometry->vertices()[1] = _optimizer.vertex(poseId);
    // Compute the measurement 
   SE2 deltaMeasurement = prevPose.inverse() * pose;
   odometry->setMeasurement(deltaMeasurement);
    // Compute the information matrix
   _robotPoseInformation = _robotPoseCovariance.inverse();
   odometry->setInformation(_robotPoseInformation);
     // Add edge into the graph
   _optimizer.addEdge(odometry);
}










void GraphSlam::add_landmark_edge(int pose,SE2 robotPoseXYT, int landmark, Eigen::Vector2d polarPoseFrame){

   /* Creation of a new edge between the node[pose] and the landmark[landmark] seen*/
   EdgeSE2PointXY* landmarkObservation =  new EdgeSE2PointXY;
   landmarkObservation->vertices()[0] = _optimizer.vertex(pose) ;
   landmarkObservation->vertices()[1] = _optimizer.vertex(landmark) ;
   landmarkObservation->setMeasurement(polarPoseFrame);
   // Compute the information matrix
	_landmarkInformation = _landmarkCovariance.inverse();
   landmarkObservation->setInformation(_landmarkInformation);
   landmarkObservation->setParameterId(0, sensorOffset->id());
   // Add edge into the graph
   _optimizer.addEdge(landmarkObservation);
}










void GraphSlam::writePoseinTxt(SE2 pose_computed,double dt){
   ofstream myfile;
   string path = "/home/clarisse/Documents/srcTestOctave/";
   string type = ".txt";
   string file = path + "poses_" + type;

   myfile.open(file.c_str(), std::ofstream::out | std::ofstream::app);
   myfile << "POSE " << pose_computed.translation()(0) << " " << pose_computed.translation()(1) << " " << pose_computed.rotation().angle() <<  " " << dt << "\n";
   myfile.close();
}












