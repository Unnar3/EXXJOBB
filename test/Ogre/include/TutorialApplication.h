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
#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include "BaseApplication.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class TutorialApplication : public BaseApplication
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

    ros::NodeHandle nh;
    ros::Subscriber point_cloud_subscriber;
public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);
    virtual void loadBaseCloud();

protected:
    virtual void createScene(void);
    virtual void loadParams();
    void updateScene(PointCloudT::Ptr cloud);
    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

};

#endif // #ifndef __TutorialApplication_h_
  