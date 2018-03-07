//This program filters the map using algorithums in Probablist Robotics chapter 6
#include <ros/ros.h>
#include <vector>
  // PCL specific includes
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/radius_outlier_removal.h>

#define _USE_MATH_DEFINES


using namespace std;

//declare functions
void makeCloud(const pcl::PCLPointCloud2ConstPtr& );
void calculateProp(pcl::PointCloud<pcl::PointXYZI> , float , float , float , float );
double getPHit(double , double ,double , double,double);
double getPShort(double , double ,double );
double getPRand(double ,double);
double getPMax(double , double );

//intilize global varibles
ros::Publisher pub;


//function that is called when the point cloud message is recieved.
void makeCloud(const pcl::PCLPointCloud2ConstPtr& inputCloud)
{

  //distance window
  float minD = 2;
  float maxD =  10;
  //hold the current distance
  float r = 0;
  //hols the last distance
  float lastR = 0;
  //holds the total distance
  float totalDistance = 0;
  //holds the total intensity
  float totalIntensity = 0;
  //intensity threashold
  float intensity = 65;
  //hold current intensity
  float currentI = 0;
  //holds the last intensity
  float lastI = 0;
  //hold the count of points
  int count = 0;
  //flag
  bool havePoint = false;

  //point cloud varibles
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> disFilter;
  pcl::fromPCLPointCloud2(*inputCloud,cloud);

  //filter the pointclouds
 for (int i = 0; i<cloud.points.size(); i++ )
 {
  //get the intensity of the point
  currentI  = cloud.points[i].intensity;
  //calcualted the distance for the point
  r = sqrt( (cloud.points[i].y)*(cloud.points[i].y) + (cloud.points[i].x)*(cloud.points[i].x) + (cloud.points[i].z)*(cloud.points[i].z)  );
  if (  (currentI >= lastI && r >= lastR ) && r < maxD && r >= minD && currentI   > intensity)
  {
    //add point to cloud
    disFilter.push_back(cloud.points[i]);
    // point = cloud.points[i];
    //save the information
    lastI = currentI;
    totalIntensity += currentI;
    totalDistance +=r;
    lastR = r;
    havePoint = true;
    count++;

  }

}

//if a cloud as been contructed pass it to the probablity filter
if(havePoint)
{
  disFilter.header.frame_id = cloud.header.frame_id;
    //pub.publish (disFilter);
  calculateProp( disFilter, lastR,lastI, totalDistance/count, totalIntensity/count );

} else {
  pcl::PointCloud<pcl::PointXYZI> cloud_tmp;
  pub.publish(cloud_tmp);
}

}


/*
  determines the likilhood of the point being a hit
  see chapter 6 of Probablist Robotics
  This method use the beam range finder algorithum to calculate the likilhood of the point being a hit.
  However a change was made to calculate the instric varibles on each beam using the tunig algorithum also in chapeter 6.

  The measurment varible is a wieghting of the distance and the intensity of the point, normilized to 1.
  then using a voxel filter to down sample the cloud.
*/
void calculateProp(pcl::PointCloud<pcl::PointXYZI> inputCloud, float maxD, float maxI, float dis, float inten)
{
  //clouds to hold new pointclouds
  pcl::PointCloud<pcl::PointXYZI> cloud;
  //temp cloud for pre voxel
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  //hold voxel sorted cloud
  pcl::PointCloud<pcl::PointXYZI> sorted;
  //voxel filter
  pcl::VoxelGrid<pcl::PointXYZI> sor;

  //tolerance to plot the points
  double tol = 0.2;
  //weighting of the distance
  double wd = .5;
  //weighting of the intensity
  double wi = 1-wd;

  //starting weights
  double z_Hit = 0.25;
  double z_Max = 0.25;
  double z_Short = 0.25;
  double z_Rand = 0.25;

  //hold the probablities desities of the points
  double pHit = 0;
  double pMax = 0;
  double pShort = 0;
  double pRand = 0;

  //need to calcuale the intrincisc parameters
  double eHit = 0;
  double eMax = 0;
  double eShort = 0;
  double eRand = 0;

  //hold data of a point
  double z = 0;
  double r = 0;
  double intensity = 0;
  double q = 1;
  double p = 0;
  double nu = 0;
  double eHitSum = 0;
  double eHitSumZ = 0;
  double eShortSum = 0;
  double eShortSumZ = 0;
  double totalZ = 0;

  //starting values for calcualting the intristic parameters
  double phi  =  .075;//*.075;//.005;//.075;
  double lamda = 2000;//2750;
  double det = 1.0/ (sqrt(2.*M_PI*phi));
  double expo = 0;
  double eta = 0;

  //get the referacne point
  double zStar  =  wd*dis/maxD + wi*inten/maxI;
  //max point
  double zMax  =   1;

  //size of the data
  int length = inputCloud.points.size();
  std::vector<double> zList;
  cloud.header.frame_id = inputCloud.header.frame_id;


  //get the total value of the points
  for (int i = 0; i<length; i++ )
  {

    r = sqrt( (inputCloud.points[i].y)*(inputCloud.points[i].y) + (inputCloud.points[i].x)*(inputCloud.points[i].x) + (inputCloud.points[i].z)*(inputCloud.points[i].z)  );
    intensity = inputCloud.points[i].intensity;
    z = (wd*r/maxD + wi*intensity/maxI);
    zList.push_back(z);
    expo = ((-0.5)*(z-zStar)*(z-zStar))/phi;
    eta += det*expo;
    totalZ += z;

  }

  expo = ((-0.5)*(zMax-zStar)*(zMax-zStar))/phi;
  eta+= det*expo;
  eta = 1/eta;

  //calculate the intrinsic varibles
  for (int i = 0; i<length; i++ )
  {
    //get the measument value
    z = zList[i];
    //get the probablities of the point
    pHit = getPHit(z,zStar,zMax,phi,eta);
    pRand = getPRand(z,zMax);
    pShort = getPShort(z,zStar,lamda);
    pMax = getPMax(z,zMax);


    nu = 1/(pHit+pRand+pShort+pMax);

    eHit+=nu*pHit;
    eShort+=nu*pShort;
    eRand+=nu*pRand;
    eMax+=nu*pMax;

    z_Hit = eHit/abs(totalZ);
    z_Max = eMax/abs(totalZ);
    z_Short = eShort/abs(totalZ);
    z_Rand = eRand/abs(totalZ);

    eHitSum+=eHit;
    eHitSumZ+=eHit*(z-zStar)*(z-zStar);
    eShortSum+=eShort;
    eShortSumZ+=eShort*z;

    lamda = eShortSum/eShortSumZ;
    phi = eHitSumZ/eHitSum;


  }

  //do the filtering
  for (int i = 0; i<inputCloud.points.size(); i++ )
  {

    z = zList[i];

    pHit = getPHit(z,zStar,zMax,phi,eta);
    pRand = getPRand(z,zMax);
    pShort = getPShort(z,zStar,lamda);
    pMax = getPMax(z,zMax);

    p = z_Hit*pHit+z_Short*pShort+z_Max*pMax+z_Rand*pRand;
    q = q*p;


    //add points to the cloud
    if(p > tol)
     cloud.push_back(inputCloud.points[i]);

 }

 //do the voxel filtering
 *cloud_filtered  = cloud;
 sor.setInputCloud (cloud_filtered);
 sor.setLeafSize (.0100, .0100, .0100);
 sor.filter (sorted);

  //publish the cloud
    pub.publish(cloud);
}
/*
  The following methods find the prob. densities
*/

double getPHit(double point, double zStar,double zMax, double phi,double eta)
{

  double det = 1.0/ (sqrt(2.*M_PI*phi));
  double expo = ((-0.5)*(point-zStar)*(point-zStar))/phi;

  if(point < zMax)
    return det*exp(expo)*eta;
  else
    return 0.0;

}

double getPShort(double point, double zStar, double lamda )
{

  double nu = 1.0/ ( 1.0 - exp(-1*lamda*zStar));

  if(point <= zStar)
    return nu*lamda*exp(-1*lamda*zStar);
  else
    return 0;

}

double getPRand(double point,double zMax)
{
  if(point < zMax )
    return 1.0/zMax;
  else
    return 0;

}

double getPMax(double point, double zMax)
{

  if (point = zMax)
    return 1;
  else
    return 0;

}

int main (int argc, char** argv)
{
  std::cout << "beamfilter.cpp" << std::endl;
    // Initialize ROS
  ros::init (argc, argv, "beamFilter");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/tritech_micron_node/sonarscan", 1, makeCloud);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >  ("beamFilter", 1);

    // Spin
  ros::spin ();
  std::cout << "beamfilter.cpp" << std::endl;
}
