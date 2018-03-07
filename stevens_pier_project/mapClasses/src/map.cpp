#include "mapClasses/map/map.h"
#include "mapClasses/utils/utils.h"

#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <math.h>
#include <vector>

using namespace utils::params;

void sonarMap::initialize(ros::NodeHandle *nh){
    map = PointCloudT::Ptr (new PointCloudT());
    mapWalls = PointCloudTNormal::Ptr (new PointCloudTNormal());
    nh_ = nh;
    setParameters();

    // Publish to outputTransform
		// In this case the topic is : /mapBuilder/trans_update
		// a message of type geometry_msgs::Transform
    pubTransform_       = nh_->advertise<geometry_msgs::Transform>(par_.outputTransform, 1);

    // Publish to outputMapWalls
		// In this case the topic is : /mapBuilder/map_walls
		// a message of type PointCloudTNormal
    pubMapWalls_        = nh_->advertise<PointCloudTNormal>(par_.outputMapWalls, 1);

    // Publish to outputMapWallsAligned
		// In this case the topic is : /mapBuilder/map_walls_aligned
		// a message of type PointCloudTNormal
    pubMapWallsAligned_ = nh_->advertise<PointCloudTNormal>(par_.outputMapWallsAligned, 1);
    

    aligned_tmp = PointCloudT::Ptr (new PointCloudT());
    aligned_tmp->header.frame_id="odom";
    features = PointCloudT::Ptr (new PointCloudT());
    pillars = PointCloudT::Ptr (new PointCloudT());
    pillars->header.frame_id="odom";
    features->header.frame_id="odom";
}

bool sonarMap::newScan(PointCloudT::Ptr beam, std::vector<int> & TdataAss,  std::vector<int> & erasedId,const int type){

    if(type == 1){
        return sonarMap::newWallScan(beam);
    } else if(type == 2){
        return sonarMap::newPillarScan(beam, TdataAss, erasedId);
    }
    return false;
}

bool sonarMap::newWallScan(PointCloudT::Ptr beam){
    // TODO add feature as param to functions.
    // Do this like this for now since feature is a global variable accessed by multiple functions.
    features->clear();
    *features = *beam;
    features->header.frame_id = "odom";

    pcl::PointCloud<pcl::Normal> normals;
    normals.resize(beam->width * beam->height);
    PointCloudTNormal::Ptr beamNormal (new PointCloudTNormal());
    pcl::concatenateFields(*beam, normals, *beamNormal);
    // Check to see if the map has been properly initialized.

  
    if(!sonarMap::intializeMapWall(beamNormal, mapWalls, 15)) return false;

    // create a kdtree over the wall map
    // find the beam neighbors on map
    pcl::KdTreeFLANN<PointTNormal> kdtree;
    kdtree.setInputCloud(mapWalls);

    std::vector<int> pidx;
    std::vector<float> psd;
    PointCloudTNormal::Ptr filtered_beamNormal (new PointCloudTNormal());
    for(size_t i = 0; i < beam->points.size(); ++i){
        if(kdtree.radiusSearch(beamNormal->points.at(i), 5.0, pidx, psd) > 0){
            // if(features->points.size() <= 2){
            if(pidx.size() > 0){
                filtered_beamNormal->points.push_back(beamNormal->points.at(i));
            }
        }
    }

    PointCloudTNormal::Ptr aligned (new PointCloudTNormal());

    typedef pcl::registration::TransformationEstimationPointToPlane<PointTNormal, PointTNormal> te;
    boost::shared_ptr<te> teSVDscale (new te);

    pcl::IterativeClosestPoint<PointTNormal, PointTNormal> icp;
    // icp.setTransformationEstimation(teSVDscale);
    icp.setInputSource(filtered_beamNormal);
    icp.setInputTarget(mapWalls);
    icp.setMaxCorrespondenceDistance (5.0);
    icp.setRANSACOutlierRejectionThreshold(0.5);
    icp.setRANSACIterations(100);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-3);;
    // icp.setMaximumIterations(500);
    // icp.setTransformationEpsilon (1e-8);
    // icp.setEuclideanFitnessEpsilon (1.0);

    //estimates the transformation and returns the transformed source (input) as output.
    icp.align(*aligned);

    if(!(icp.getFitnessScore() < 1.5 && icp.hasConverged())){
        return false;
    }


    Eigen::Affine3f transform;
    transform = icp.getFinalTransformation();
    pcl::transformPointCloud (*beamNormal, *aligned, transform);

    if(!sonarMap::publishWallTransform(transform)){
        std::cout << "Aligned, but transformation to big....." << std::endl;
        return false;
    }

    for(int i = 0; i < aligned->points.size(); ++i)
    {
        if(kdtree.radiusSearch(aligned->points.at(i), 2.0, pidx, psd) == 0){
            mapWalls->points.push_back(aligned->points.at(i));
        }
    }

    pubMapWalls_.publish(mapWalls);
    pubMapWallsAligned_.publish(aligned);
    return true;
}





bool sonarMap::newPillarScan(PointCloudT::Ptr beam, std::vector<int> & TdataAss,std::vector<int> & erasedId){

    features->clear();
    *features = *beam;
    features->header.frame_id = "odom";

    /* Check to see if the map has been properly initialized */
    if(!sonarMap::intializeMapPillar(features, map, 3,1.0)) return false;


/* Data fitering for aligning                                        */
    /* Passthrough Filter, only align to map features with intensity higher than 2.0 */
    PointCloudT::Ptr        filtered_map (new PointCloudT());
    sonarMap::passThroughFilter(map, filtered_map, 2.0);
    /* Create a kd-tree from the filtered map */
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(filtered_map);
    /* Radius search variables */
    std::vector<int>        pidx ;                          // the resultant indices of the neighboring points 
    std::vector<float>      psd ;                           // the resultant squared distances to the neighboring points
    double                  radius = 5.0 ;                  // the radius of the sphere bounding all of p_q's neighbors 

    /* Filter out points far away from the map and only use those who are close for aligning */
    PointCloudT::Ptr         filtered_scan(new PointCloudT()) ;   // scan point who are are close enough 
    for(size_t i = 0; i < features->points.size(); ++i){
        if(kdtree.radiusSearch(features->points.at(i), radius, pidx, psd) > 0){
            if(pidx.size() > 0){
                filtered_scan->points.push_back(features->points.at(i));
            }
        }
    }
/* .................................................................... */

    // ICP variables
    Eigen::Affine3f         transform ;                     // Eigen transformation between filtered_map et filtered_scan 
    PointCloudT             aligned ;                       // filtered_scan transformed according to the transformation find by ICP 

    if(filtered_scan->points.size() > 2){
        /* Align filtered_scan (source) to filtered_map (target)*/       
        if(icpAlign(filtered_map, filtered_scan, kdtree,  aligned, transform)){

            /* Applied the transformation find by the ICP in all the feaure point*/
            pcl::transformPointCloud (*features, *aligned_tmp, transform);
            /* dataAssociation between the map and the aligned feature */
            sonarMap::dataAssociation(aligned_tmp,TdataAss,erasedId);
            /* piblication of the transformation to update the kalman */
            sonarMap::publishSonarTransform(transform);

            return true;
        }
    } else if(filtered_scan->points.size() > 1){
        if(sonarMap::matchTwoPoints(filtered_map, filtered_scan, kdtree, transform)){
             pcl::transformPointCloud (*features, *aligned_tmp, transform);
            if(std::abs(transform.matrix()(0,3)) > 1.5 || std::abs(transform.matrix()(1,3)) > 1.5){
                // std::cout << "hahahahahahha" << std::endl;
                return false;
            }
            ROS_ERROR("TWO POINTS");
            sonarMap::dataAssociation(aligned_tmp,TdataAss, erasedId);
            sonarMap::publishSonarTransform(transform);
            return true;
        }
    } else {
        // ROS_ERROR("LESS THAN TWO POINTS");
    }
    aligned_tmp->clear();
    return false;
}


void sonarMap::dataAssociation(PointCloudT::Ptr aligned,  std::vector<int>  & TdataAss, std::vector<int> & erasedId ){

    //* Create a kd-tree from map 
    pcl::KdTreeFLANN<PointT> kdtree5;
    kdtree5.setInputCloud(map);
    //* K nearest search variables 
    int                 K = 5;      // the number of neighbors to search for 
    std::vector<int>    pidx(K);    // the resultant indices of the K neighboring points
    std::vector<float>  psq(K);     // the resultant squared distances to the K neighboring points 
    int                 pidxSize;   // number of neighbors found

    std::vector<int>   p1;     // aligned index
    std::vector<int>   p2;     // map index
    std::vector<float> dist;   // Indexes to points not associated
    std::vector<int>   left;   // Indexes to points not associated

    std::vector <int>::iterator it;
    int nPosition;
    int idx; // id of the closest point 
    int idp2;





    bool end = false;
    int cptTmp = 0 ;
     for(size_t i = 0; i < aligned->points.size(); ++i){
        pidxSize = kdtree5.nearestKSearch(aligned->points.at(i), K, pidx, psq);
        if(pidxSize > 0)
        {
            // Find the closest definite feature within certain distance
            idx = 0;
            for (int j = 0; j < pidxSize; ++j)
            {
                if(pidx[j] < map->points.size() && map->points.at(pidx[j]).intensity >= 2.0 && psq[j] < 1.5){
                    idx = j;
                    break;
                }
            }

            if(psq[idx] < 1.5 && pidx[idx] < map->points.size()){
                // Close enough to a current feature: ASSOCIATED 
                it = find(p2.begin(), p2.end(), pidx[idx]);
                if (it != p2.end()){
                    nPosition = distance(p2.begin(), it);
                    // left.push_back(p1.at(nPosition));
                    if(psq[idx] < dist.at(nPosition)){
                        p1.at(nPosition) = i;
                        dist.at(nPosition) = psq[idx];
                    }
                } else {
                    p1.push_back(i);
                    p2.push_back(pidx[idx]);
                    dist.push_back(psq[idx]);
                }
                
            } else {
                // Probably a new feature
                left.push_back(i);
            }

        }
    }
    // for(size_t i = 0; i < aligned->points.size(); ++i){
        
    //     pidxSize = kdtree5.nearestKSearch(aligned->points.at(i), K, pidx, psq);

    //     do{

    //         //* if neighbors finded 
    //         if(pidxSize > 0)
    //         {
    //             //* Find the closest definite feature within certain distance
    //             idx = 0;
    //             for (int j = 0; j < pidxSize; ++j)
    //             {
    //                 //* keep the closest neighbors with an intensity less than 2.0 
    //                 if( map->points.at(pidx[j]).intensity >= 2.0  && psq[j] < 1.5 && pidx[j] < map->points.size() ){
    //                     idx = j;
    //                     break;
    //                 }
    //             }

        
                

    //             //* Close enough to a current feature: ASSOCIATED
    //             if(psq[idx] < 1.5 && pidx[idx] < map->points.size() && cptTmp != pidxSize+1)
    //             {

    //                 //* try to find the value of pidx[idx] in the range (p2.begin(), p2.end())
    //                 //* return an iterator is the value is finded and the last position of the range otherwise
    //                 it = find(p2.begin(), p2.end(), pidx[idx]);
                   
    //                 //* index find : Feature already associated 
    //                 if (it != p2.end() ){

    //                     nPosition = distance(p2.begin(), it);
    //                     // if new associate is closest than then the previous, the association is changed 
    //                     if(psq[idx] < dist.at(nPosition)){
    //                         p1.at(nPosition) = i;
    //                         dist.at(nPosition) = psq[idx];
    //                         end = true;
                                
    //                     } else {
    //                         cptTmp++;
    //                     }
    //                 } else {

    //                     //* Association aligned point i and map point pidx[idx] with the sqr distance psq[idx]
    //                     p1.push_back(i);
    //                     p2.push_back(pidx[idx]);
    //                     dist.push_back(psq[idx]);
    //                     end = true;
    //                 }

    //             } else {
    //                 // Probably a new feature
    //                 left.push_back(i);
    //                 end = true;
    //             }
        

    //         }
    //     }
    //     while(end==false);
    // }

     std::vector<int> tmp (aligned->points.size(),-3);
     TdataAss.clear();
     TdataAss = tmp ;
    for (int i = 0; i < p1.size(); ++i)
    {
         TdataAss.at(p1[i]) = p2[i];
    }
    for (int i = 0; i < left.size(); ++i)
    {
         TdataAss.at(left[i]) = -1;
    }


    

    
/* .................................................................... */

    //* Change the intensity of the map point associated : increase certainty detected points
    sonarMap::changeIntensity(0.0, 4.0, 0.2, p2);

    //*  Decrease certainty detected points 
    //* Create dtree from aligned 
    pcl::KdTreeFLANN<PointT> kdtree6;
    kdtree6.setInputCloud(aligned);
    std::vector<int>    pointIdxRadiusSearch;           // the resultant indices of the neighboring points 
    std::vector<float>  pointRadiusSquaredDistance;     // the resultant squared distances to the neighboring points

    for(size_t i = 0; i < map->points.size(); ++i){
        try{
            if(map->points.at(i).intensity < 2.0){ 
                if ( kdtree6.radiusSearch (map->points.at(i), 5.0, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                    map->points.at(i).intensity -= 0.1;
                }
            }
        }
        catch(int e){
            // ROS_WARN("OUT OF BOUNDS");
            // std::cout << "i: " << i << "size: " << map->points.size() << std::endl;
            // ROS_WARN("OUT OF BOUNDS");
        }
    }


/* .................................................................... */

/*          Add new points                                              */ 
/* detail : add left points : aligned point without map corresponding  */
    // Add new points
    // std::cout << "new points" << std::endl;
    for(size_t i = 0; i < left.size(); ++i){
        aligned->points.at(left.at(i)).intensity = 1.0;
        map->points.push_back(aligned->points.at(left.at(i)));

        PointCloudT::Ptr tmp (new PointCloudT());
        tmp->points.push_back(aligned->points.at(left.at(i)));
        pillarPoints.push_back(tmp);

        PointCloudT::Ptr tmp2 (new PointCloudT());
        tmp2->points.push_back(features->points.at(left.at(i)));
        featurePoints.push_back(tmp2);

        featureCount.push_back(1);
    }
/* .................................................................... */


/*          update the map points according to theirs corresponding points */
/* detail : update the time number where the feature have been seen        */ 
    for(size_t i = 0; i < p2.size(); ++i){

        if(p2[i] >= map->points.size() && map->points[p2[i]].intensity < 2.0){
            continue;
        }
        featureCount[p2[i]]++; 
        pillarPoints[p2[i]]->points.push_back(aligned->points.at(p1[i]));
        featurePoints[p2[i]]->points.push_back(features->points.at(p1[i]));
        map->points.at(p2[i]).x = (map->points.at(p2[i]).x*(featureCount[p2[i]]-1)+aligned->points.at(p1[i]).x) / featureCount[p2[i]];
        map->points.at(p2[i]).y = (map->points.at(p2[i]).y*(featureCount[p2[i]]-1)+aligned->points.at(p1[i]).y) / featureCount[p2[i]];
    }
    // std::cout << "finished updating featur" << std::endl;
/* .................................................................... */


/* ..................................................................................................................................................... */


    // std::cout << "pillars" << std::endl;
    // add pillars estimated outlines
    tf::StampedTransform transform;
    try{
        listener_.lookupTransform("/odom", "/body", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        std::cout << "exception: " << ex.what() << std::endl;
    }
    float theta;
    PointT p;
    for(size_t i = 0; i < p1.size(); ++i){
        if(p2[i] < map->points.size() && map->points[p2[i]].intensity >= 2.0){
            theta = atan2(aligned->points.at(p1.at(i)).y - transform.getOrigin().y(), aligned->points.at(p1.at(i)).x - transform.getOrigin().x());
            p.x = aligned->points.at(p1.at(i)).x - cos(theta)*0.0;
            p.y = aligned->points.at(p1.at(i)).y - sin(theta)*0.0;
            pillars->points.push_back(p);
        }
    }
    sonarMap::pruneMap(erasedId);
/* ..................................................................................................................................................... */

}










bool sonarMap::intializeMapPillar(const PointCloudT::Ptr scan, PointCloudT::Ptr mapPtr, const int size, int sqr_distance){
    if(mapPtr->points.size() >= size) return true;

    // check if the distance between the new points and the map points : here less than 1.O 
    bool toClose;
    for(size_t i = 0; i < scan->points.size(); ++i){
        toClose = false;
        for(size_t j = 0; j < mapPtr->points.size(); ++j){
            if(utils::Distance(mapPtr->points.at(j), scan->points.at(i)) < sqr_distance){
                toClose = true;
            }
        }
        // if the point is further than all the map points, we can add it on the map  
        if(!toClose){ 
            /* Add new point to map with big intensity */
            mapPtr->points.push_back(scan->points.at(i));
            mapPtr->points.back().intensity = 4.0;

            /* Add new point on the pillarPoins */
            PointCloudT::Ptr tmp (new PointCloudT());
            tmp->points.push_back(scan->points.at(i));
            pillarPoints.push_back(tmp);

            /* Add new point on the featurePoints */
            PointCloudT::Ptr tmp2 (new PointCloudT());
            *tmp2 += *tmp; 
            featurePoints.push_back(tmp2);

            /* the new point has been seen 1 time for the moment */
            featureCount.push_back(1);
        }
    }
    mapPtr->header.frame_id = scan->header.frame_id;

    // Check to see if map is initialized now
    if(mapPtr->points.size() >= size){
        return true;
    } 
        
    return false;
}










// add the new point on the map
bool sonarMap::intializeMapWall(const PointCloudTNormal::Ptr scan, PointCloudTNormal::Ptr mapPtr, const int size){
    if(mapPtr->points.size() >= size) return true;

    // use for loops to check new points, ok due to small size.
    bool toClose;
    for(size_t i = 0; i < scan->points.size(); ++i){
        toClose = false;
        for(size_t j = 0; j < mapPtr->points.size(); ++j){
            if(utils::Distance(mapPtr->points.at(j), scan->points.at(i)) < 1.0){
                toClose = true;
            }
        }
        // if(!toClose && map->points.size() < 4){ // Add new point to map
        if(!toClose){ // Add new point to map
            mapPtr->points.push_back(scan->points.at(i));
            mapPtr->points.back().intensity = 4.0;
        }
    }
    mapPtr->header.frame_id = scan->header.frame_id;
    // Check to see if map is initialized now
    if(mapPtr->points.size() >= size){
        return true;
    }

    return false;
}

void sonarMap::passThroughFilter(const PointCloudT::Ptr scan, PointCloudT::Ptr filtered, const float lim_low){
    filtered->header = scan->header;
    for(size_t i = 0; i < scan->points.size(); ++i){
        if(scan->points.at(i).intensity >= lim_low)
            filtered->points.push_back(scan->points.at(i));
    }
}

bool sonarMap::icpAlign(const PointCloudT::Ptr map, const PointCloudT::Ptr scan, const pcl::KdTreeFLANN<PointT> &kdtree, PointCloudT &aligned, Eigen::Affine3f &trans){
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(scan);
    icp.setInputTarget(map);
    icp.setMaxCorrespondenceDistance (5.0);
    // icp.setRANSACOutlierRejectionThreshold (1.0);
    // icp.setMaximumIterations(500);
    // icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (1.0);
    icp.align(aligned);

    // align point : successful
    if(icp.getFitnessScore() < 1.0 && icp.hasConverged()){
        trans = icp.getFinalTransformation();
        Eigen::Vector3f ea= trans.matrix().block<3,3>(0,0).eulerAngles(0, 1, 2);
        sonarMap::minimizeEuler(ea);
        if(std::abs(ea[2]) > M_PI/9){
            std::cout << "  ROTATION TO BIG" << std::endl;
            return false;
        }

        // PointCloudT::Ptr tmp_aligned(&aligned);
        // double free or corruption error
        // Stupid, use for loop to copy to PointCloud::Ptr
        PointCloudT::Ptr tmp_aligned(new PointCloudT());
        for(size_t i = 0; i < aligned.points.size(); ++i){
            tmp_aligned->points.push_back(aligned.points[i]);
        }

        pcl::KdTreeFLANN<PointT> kdtree3;
        kdtree3.setInputCloud(tmp_aligned);

        // std::vector<int> pointIdxRadiusSearch;
        // std::vector<float> pointRadiusSquaredDistance;

        // std::vector<int> featureIdx(features->points.size());
        // int count = 0;
        // for(size_t i = 0; i < map->points.size(); ++i){
        //     if ( kdtree3.radiusSearch (map->points.at(i), 3.0, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        //         if(featureIdx[pointIdxRadiusSearch[0]] == 0){
        //             featureIdx[pointIdxRadiusSearch[0]] = 1;
        //             count++;
        //         }
        //     }
        // }


        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<int> pIdxRS;
        std::vector<float> pRSD;

        std::vector<int> featureIdx(features->points.size());
        int count = 0;
        int featureNeighbours = 0;
        int mapNeighbours = 0;
        for(size_t i = 0; i < map->points.size(); ++i){
            // std::cout << "i: " << i << "   size: " << map->points.size() << std::endl;
            featureNeighbours = kdtree3.radiusSearch (map->points.at(i), 2.0, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            mapNeighbours = kdtree.radiusSearch (map->points.at(i), 4.0, pIdxRS, pRSD);
            if(featureNeighbours > 0){
                if(featureNeighbours-mapNeighbours > 0){
                    featureIdx[pointIdxRadiusSearch[0]] = 1;
                    count++;
                }
                else{
                    for(size_t j = 0; j < featureNeighbours; ++j){
                        // std::cout << "hmmmmm: " << j << "max: " << featureNeighbours << "map" << mapNeighbours << std::endl;
                        featureIdx[pointIdxRadiusSearch[j]] = 1;
                    }
                }
            }
        }


        if(count > 0){
            ROS_WARN("##############");
            ROS_WARN("ALIGNING AGAIN");
            ROS_WARN("##############");
            // More than one detected feature has been aligned to the same map feature.
            // Need to remove and align again.
            PointCloudT::Ptr new_aligned (new PointCloudT());
            for(size_t i = 0; i < tmp_aligned->points.size(); ++i){
                if(featureIdx[i] == 1){
                    new_aligned->points.push_back(tmp_aligned->points[i]);
                }
            }

            icp.setInputSource(new_aligned);
            icp.setInputTarget(map);
            icp.setMaxCorrespondenceDistance (3.0);
            // icp.setRANSACOutlierRejectionThreshold (1.0);
            // icp.setMaximumIterations(500);
            // icp.setTransformationEpsilon (1e-8);
            icp.setEuclideanFitnessEpsilon (1.0);
            icp.align(aligned);
            if(icp.getFitnessScore() < 1.0 && icp.hasConverged()){
                Eigen::Affine3f trans2;
                trans2 = icp.getFinalTransformation();
                trans = trans2 * trans;
            }
        }
        else{
            // std::cout << "NOT aligned again" << std::endl;
        }


    }

    // check that the transformation is sensible(does not have really small or really big values
    // Just put the values small and big enough
    for(int i = 0; i < trans.matrix().rows(); ++i){
        for(int j = 0; j < trans.matrix().cols(); ++j){
            if(trans.matrix()(i,j) != 0){
                if( std::abs(trans.matrix()(i,j)) < 1*std::pow(10,-20) || 1*std::pow(10,10) < std::abs(trans.matrix()(i,j)) ){
                // if( 10000000 < std::abs(trans.matrix()(i,j)) ){
                    ROS_WARN("Nonsensical transformation");
                    // std::cout << trans.matrix() << std::endl;
                    return false;
                }
            }
        }
    }
    return true;
}


void sonarMap::changeIntensity(float limLow, float limHigh, float change){
    for(size_t i = 0; i < map->points.size(); ++i){
        if(map->points.at(i).intensity >= limLow && map->points.at(i).intensity <= limHigh){
            map->points.at(i).intensity += change;
        }
    }
}

void sonarMap::changeIntensity(float limLow, float limHigh, float change, std::vector<int> idx){
    for(size_t i = 0; i < idx.size(); ++i){
        if(idx.at(i) >= map->points.size()){
            ROS_WARN("WHAT THE FUCKING FUCK");
            break;
        }
        if(map->points.at(idx.at(i)).intensity >= limLow && map->points.at(idx.at(i)).intensity <= limHigh){
            map->points.at(idx.at(i)).intensity += change;
        }
    }
}

void sonarMap::pruneMap(std::vector<int> & erasedId ){
    int tmpi = map->points.size();
    int feat = 0;
    for(int i = 0; i < map->points.size(); ++i){
        if(map->points.at(i).intensity >= 2.0){
            feat++;
        }
    }
    pcl::PointIndices pi;
    pcl::PassThrough<PointT> pass(true);
    pass.setInputCloud(map);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(0.0, 30);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*map);
    pass.getRemovedIndices(pi);

    int feat2 = 0;
    for(int i = 0; i < map->points.size(); ++i){
        if(map->points.at(i).intensity >= 2.0){
            feat2++;
        }
    }

    if(feat != feat2){
        ROS_WARN("############################################");
        ROS_WARN("############################################");
        ROS_WARN("FUCKING PASSTHROUGH");
        ROS_WARN("############################################");
        ROS_WARN("############################################");
    }


    if(pi.indices.size() > 0){
        ROS_WARN("############################################");
        std::cout << "indices: ";
    }
    for(size_t i = pi.indices.size(); i-- > 0;){
        erasedId.push_back(pi.indices[i]);
        featureCount.erase(featureCount.begin() + pi.indices[i]);
        pillarPoints.erase(pillarPoints.begin() + pi.indices[i]);
        featurePoints.erase(featurePoints.begin() + pi.indices[i]);
        std::cout << pi.indices[i] << ", ";
        // featureCount.at(i) = -1;
    }
    if(pi.indices.size() > 0){
        std::cout << "" << std::endl;
        ROS_WARN("############################################");
    }
    // featureCount.erase(remove(featureCount.begin(), featureCount.end(), -1), featureCount.end());
}

void sonarMap::minimizeEuler(Eigen::Vector3f &euler){
    if(euler[0] > M_PI/4){
        euler[0] =  euler[0] - M_PI;
        euler[1] = -euler[1];
        euler[2] =  euler[2] - M_PI;
    } else if(euler[0] < -M_PI/4){
        euler[0] =  euler[0] + M_PI;
        euler[1] = -euler[1];
        euler[2] =  euler[2] + M_PI;
    }
    if(euler[1] > M_PI/4){
        euler[0] = -euler[0];
        euler[1] =  euler[1] - M_PI;
        euler[2] = -euler[2];
    } else if(euler[1] < -M_PI/4){
        euler[0] = -euler[0];
        euler[1] =  euler[1] + M_PI;
        euler[2] = -euler[2];
    }
    if(euler[2] > M_PI)
        euler[2] = euler[2]-2*M_PI;
    if(euler[2] < -M_PI)
        euler[2] = euler[2]+2*M_PI;
}


bool sonarMap::matchTwoPoints(PointCloudT::Ptr map, PointCloudT::Ptr scan, const pcl::KdTreeFLANN<PointT> &kdtree, Eigen::Affine3f &trans){
    // ROS_ERROR("MatcheTwoPoints");
    std::vector<int> pi1(2);
    std::vector<float> pd1(2);
    std::vector<int> pi2(2);
    std::vector<float> pd2(2);
    if(kdtree.nearestKSearch(scan->points.at(0), 2, pi1, pd1) > 0){
        if(kdtree.nearestKSearch(scan->points.at(1), 2, pi2, pd2) > 0){
            // get the distance between the two points from the scan
            float dstScannedP = utils::Distance(scan->points.at(0), scan->points.at(1));
            // Check to see of the closest points is the same for both scanned points.
            if(pi1[0] != pi2[0]){
                // The points are not closest to the same points, Check the distance between the found points
                float dstFoundP = utils::Distance(map->points.at(pi1[0]), map->points.at(pi2[0]));
                if(std::abs(dstFoundP - dstScannedP) < std::min(0.1*dstFoundP, 1.0)){
                    // close to being the same distance between the points.
                    // Find transformation between the points
                    trans = sonarMap::transformationBetweenPairsOfPoints(map->points.at(pi1[0]), map->points.at(pi2[0]), scan->points.at(0), scan->points.at(1));
                    return true;
                }
            }else{
                // Need to figure out if any other combination can be used (pi1[0],pi2[1]), (pi1[1],pi2[0])
                float d12 = utils::Distance(map->points.at(pi1[0]), map->points.at(pi2[1]));
                float d21 = utils::Distance(map->points.at(pi1[1]), map->points.at(pi2[0]));
                float combined1 = 0;
                float combined2 = 0;
                if(std::abs(d12 - dstScannedP) < 0.5){
                    // Combination 12 is possible
                    // Find distance between both Matches
                    float p10 = utils::Distance(scan->points.at(0), map->points.at(pi1[0]));
                    float p21 = utils::Distance(scan->points.at(1), map->points.at(pi2[1]));
                    combined1 = p10 + p21 + std::abs(p10 - p21);

                    if(std::abs(d21 - dstScannedP) < 0.5){
                        // Combination 12 is possible
                        // Find distance between both Matches
                        float p11 = utils::Distance(scan->points.at(0), map->points.at(pi1[1]));
                        float p20 = utils::Distance(scan->points.at(1), map->points.at(pi2[0]));
                        combined2 = p11 + p20 + std::abs(p11 - p20);
                        if(combined2 < combined1){
                            trans = sonarMap::transformationBetweenPairsOfPoints(map->points.at(pi1[1]), map->points.at(pi2[0]), scan->points.at(0), scan->points.at(1));
                            return true;
                        }
                    }

                    trans = sonarMap::transformationBetweenPairsOfPoints(map->points.at(pi1[0]), map->points.at(pi2[1]), scan->points.at(0), scan->points.at(1));
                    return true;
                }

            }
        }
    }
    return false;
}

Eigen::Affine3f sonarMap::transformationBetweenPairsOfPoints(PointT &a1, PointT &a2, PointT &b1, PointT &b2){
    // get rotation and translation on xy plane

    // Find center of target
    float target_x = (a1.x + a2.x)/2;
    float target_y = (a1.y + a2.y)/2;

    // Find center of start
    float start_x = (b1.x + b2.x)/2;
    float start_y = (b1.y + b2.y)/2;

    // find ideal angle
    // float target_angle = std::atan2(a2.y-a1.y, a2.x-a1.x);
    // float start_angle  = std::atan2(b2.y-b1.y, b2.x-b1.x);
    // float angleDiff = target_angle - start_angle;

    // Eigen::Affine3f transformT1 = Eigen::Affine3f::Identity();
    // transformT1.translation() << -start_x, -start_y, 0;
    // Eigen::Affine3f transformR = Eigen::Affine3f::Identity();
    // transformR.rotate(Eigen::AngleAxisf (angleDiff, Eigen::Vector3f::UnitZ()));
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << target_x-start_x, target_y-start_y, 0;

    return transform;
    // return transformT2*transformR*transformT1;
}

void sonarMap::publishSonarTransform(Eigen::Affine3f translation){

    tf::StampedTransform stampedTr;
    Eigen::Affine3d trans;
    trans.matrix() = translation.matrix().cast<double>();
    tf::transformEigenToTF(trans, stampedTr);

    geometry_msgs::Transform m;
    tf::transformEigenToMsg (trans, m);
    pubTransform_.publish(m);

    // tf::TransformBroadcaster br;
    // br.sendTransform(tf::StampedTransform(stampedTr, ros::Time::now(), "/odom_sonar", "/odom"));
    // sb.updateBuffer(translation);

}

bool sonarMap::publishWallTransform(Eigen::Affine3f translation){


    Eigen::Quaternionf q(translation.rotation());

    // Check that the rotation is not to exessive.
    Eigen::Vector3f ea = translation.rotation().eulerAngles(0, 1, 2);

    sonarMap::minimizeEuler(ea);
    // std::cout << "Euler: " << ea[0]/M_PI*180 << " " << ea[1]/M_PI*180 << " " << ea[2]/M_PI*180 << std::endl;
    if(std::abs(ea[2]) > M_PI/6) return false;
    if(translation.matrix()(0,3) > 3.0) return false;
    if(translation.matrix()(1,3) > 3.0) return false;
    if(translation.matrix()(2,3) > 3.0) return false;


    // // Get the current position and check translation
    // tf::StampedTransform transform;
    // try{
    //     listener_.lookupTransform("/odom", "/body", ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //     std::cout << "exception: " << ex.what() << std::endl;
    // }

    // Eigen::VectorXf v(4);
    // v(0) = transform.getOrigin().x();
    // v(1) = transform.getOrigin().y();
    // v(2) = transform.getOrigin().z();
    // v(3) = 1;
    // Eigen::VectorXf out(4);
    // out = translation.matrix()*v - v;
    // // std::cout << "Translation: " << out(0) << " " << out(1) << " " << out(2) << std::endl;

    // if(out.cwiseAbs().maxCoeff() > 5.0) return false;

    // // Lets find the new heading based on the  translation and new heading
    // double roll, pitch, yaw;
    // tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    // Eigen::Vector3f ea2;
    // ea2 << roll, pitch, yaw;
    // // std::cout << "thetas  " << ea[2] << " " << ea2[2] << std::endl;
    // sonarMap::minimizeEuler(ea2);



    // geometry_msgs::Pose2D p2d;
    // p2d.x = out(0);
    // p2d.y = out(1);
    // p2d.theta = ea[2]-ea2[2];

    geometry_msgs::Transform m;
    m.rotation.x = q.x();
    m.rotation.y = q.y();
    m.rotation.z = q.z();
    m.rotation.w = q.w();

    m.translation.x = translation.matrix()(0,3);
    m.translation.y = translation.matrix()(1,3);
    m.translation.z = translation.matrix()(2,3);

    pubTransform_.publish(m);
    return true;

}

std::vector<PointCloudT::Ptr> sonarMap::returnPillarPoints(void){
    std::vector<PointCloudT::Ptr> tmp;
    for(size_t i = 0; i < map->points.size(); ++i){
        if(map->points[i].intensity >= 2.0){
            tmp.push_back(pillarPoints[i]);
        }
    }
    return tmp;
}

std::vector<PointCloudT::Ptr> sonarMap::returnFeaturePoints(void){
    std::vector<PointCloudT::Ptr> tmp;
    for(size_t i = 0; i < map->points.size(); ++i){
        if(map->points[i].intensity >= 2.0){
            tmp.push_back(featurePoints[i]);
        }
    }
    return tmp;
}

void sonarMap::setParameters(){
    par_.outputTransform       = loadParam<string>("outputTransform", nh_);
    par_.outputMapWalls        = loadParam<string>("outputMapWalls", nh_);
    par_.outputMapWallsAligned = loadParam<string>("outputMapWallsAligned", nh_);
}
