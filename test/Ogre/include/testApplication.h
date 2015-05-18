/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#ifndef __testApplication_h_
#define __testApplication_h_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <exx_compression/compression.h>
#include <exx_compression/planes.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include "BaseApplication.h"
#include <bitset>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class testApplication : public BaseApplication
{
    // Parameters
    std::string configPath_, cloudPath_, savePath_;
    double SVVoxelResolution_, SVSeedResolution_, SVColorImportance_, SVSpatialImportance_;
    double RANSACDistanceThreshold_, VoxelLeafSize_; int RANSACMinInliers_;
    double GP3SearchRad_, GP3Mu_, GP3MaxNearestNeighbours_, GP3Ksearch_;
    double RWHullMaxDist_;
    bool simplifyHulls_;
    double ECClusterTolerance_;
    double hullAlpha_;
    int ECMinClusterSize_;
    PointCloudT::Ptr baseCloud_;

    bool first;
    int count;
    std::bitset<2> bs;
    Ogre::ManualObject* manual;

    EXX::compression cmprs;
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_subscriber;
public:
    testApplication(void);
    virtual ~testApplication(void);

protected:
    virtual void createScene(void);
    virtual void loadBaseCloud();
    virtual void loadParams();
    void updateScene(PointCloudT::Ptr nonPlanar, std::vector<PointCloudT::Ptr> planes, std::vector<PointCloudT::Ptr> hulls, std::vector<float> gp3_rad, std::vector<std::vector<float> > normals);
    void point_cloud_callback(const exx_compression::planes& cloud_msg);

};

#endif // #ifndef __testApplication_h_
  