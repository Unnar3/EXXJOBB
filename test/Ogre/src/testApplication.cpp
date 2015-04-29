/*
-----------------------------------------------------------------------------
Filename:    testApplication.cpp
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

#include "testApplication.h"
#include <utils/utils.h>
#include <iostream>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PolygonMesh.h>

#include <OgreManualObject.h>
#include <CEGUI/CEGUI.h>
#include <CEGUI/CEGUISchemeManager.h>
#include <CEGUI/RendererModules/Ogre/CEGUIOgreRenderer.h>

#include <boost/thread/thread.hpp>
#include <cstdint>

using namespace EXX;
using namespace EXX::params;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//-------------------------------------------------------------------------------------
testApplication::testApplication(void)
{
    first = true;
    count = 0;
    bs.set(1);

    nh = ros::NodeHandle("~");
    PCL_ERROR("Could not load base cloud, baseCloud_ is empty.");
    testApplication::loadParams();
    testApplication::setConfigPath(loadParam<std::string>("configPath", nh));

    cmprs.setGP3SearchRad( loadParam<double>("GP3SearchRad", nh) );
    cmprs.setGP3Mu( loadParam<double>("GP3Mu", nh) );
    cmprs.setGP3MaxNearestNeighbours( loadParam<double>("GP3MaxNearestNeighbours", nh) );
    cmprs.setGP3Ksearch( loadParam<double>("GP3Ksearch", nh) );

    point_cloud_subscriber   = nh.subscribe(loadParam<std::string>("TOPIC_POINT_CLOUD", nh), 1, &testApplication::point_cloud_callback, this);
}
//-------------------------------------------------------------------------------------
testApplication::~testApplication(void)
{
}

//-------------------------------------------------------------------------------------
void testApplication::updateScene(PointCloudT::Ptr nonPlanar, std::vector<PointCloudT::Ptr> planes, std::vector<PointCloudT::Ptr> hulls, std::vector<float> gp3_rad){
    // Initialize the manual object if it hasnt been
    if (first){
        manual = mSceneMgr->createManualObject("manual");
    }

    std::vector<EXX::cloudMesh> cmesh;
    cmprs.greedyProjectionTriangulationPlanes(nonPlanar, planes, hulls, cmesh, gp3_rad);
    cmprs.improveTriangulation(cmesh, planes, hulls);

    manual->clear();
    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    PointCloudT::Ptr tmp_cloud (new PointCloudT ());
    for (int i = 0; i < cmesh.size()-1; ++i){
        *tmp_cloud += *cmesh[i].cloud;
    }

    int pic = 0;
    Ogre::Real r, g, b;
    for (std::vector<EXX::cloudMesh>::iterator ite = cmesh.begin(); ite < cmesh.end()-1; ++ite){

        for (size_t i = 0; i < (*ite).cloud->points.size(); i++){
            manual->position((*ite).cloud->points[i].x, -(*ite).cloud->points[i].y, -(*ite).cloud->points[i].z);
            r = (Ogre::Real)(*ite).cloud->points[i].r / (Ogre::Real)255;
            g = (Ogre::Real)(*ite).cloud->points[i].g / (Ogre::Real)255;
            b = (Ogre::Real)(*ite).cloud->points[i].b / (Ogre::Real)255;
            manual->colour(r, g, b);
        }
        for (size_t i = 0; i < (*ite).mesh.polygons.size(); ++i){
            // Add triangle facing one way
            for (size_t j = 0; j < (*ite).mesh.polygons[i].vertices.size(); ++j)
            {
                manual->index((*ite).mesh.polygons[i].vertices[j] + pic);
            }
            // Add same triangles facing the other way
            for (size_t j = (*ite).mesh.polygons[i].vertices.size(); j-- > 0; )
            {
                manual->index((*ite).mesh.polygons[i].vertices[j] + pic);
            }
        }
        pic += (*ite).cloud->points.size();
    }
    manual->end();  

    if (first){
        mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
        first = false;
    }
    std::cout << "end" << std::endl;
}

void testApplication::createScene(void){}

void testApplication::loadBaseCloud(){
    baseCloud_ = PointCloudT::Ptr (new PointCloudT ());
    if( pcl::io::loadPCDFile (cloudPath_, *baseCloud_) == -1 ){
        PCL_ERROR("Could not load base cloud, baseCloud_ is empty.");
    } else {
        std::cout << "baseCloud_ loaded;" << std::endl;
    }
}

void testApplication::point_cloud_callback(const exx_compression::planes& cloud_msg)
{
    std::vector<PointCloudT::Ptr> planes;
    std::vector<PointCloudT::Ptr> hulls;
    std::vector<float> gp3_rad;
    PointCloudT::Ptr nonPlanar (new PointCloudT ());

    for ( auto i : cloud_msg.planes ){
        PointCloudT::Ptr cloud (new PointCloudT ());
        pcl::fromROSMsg (i, *cloud);
        planes.push_back(cloud);            
    }

    for ( auto i : cloud_msg.hulls ){
        PointCloudT::Ptr cloud (new PointCloudT ());
        pcl::fromROSMsg (i, *cloud);
        hulls.push_back(cloud);            
    }

    for ( auto i : cloud_msg.gp3_rad ){
        gp3_rad.push_back(i);
    }

    pcl::fromROSMsg (cloud_msg.nonPlanar, *nonPlanar);

    testApplication::updateScene(nonPlanar, planes, hulls, gp3_rad);
}

void testApplication::loadParams(){
    // nh.param<std::string>("configPath", configPath_, "./");
    configPath_ = loadParam<std::string>("configPath", nh);
    nh.param<std::string>("cloudPath", cloudPath_, "./");
    nh.param<std::string>("savePath", savePath_, "./");
    nh.param<double>("SVVoxelResolution", SVVoxelResolution_, 0.1);
    nh.param<double>("SVSeedResolution", SVSeedResolution_, 0.3);
    nh.param<double>("SVColorImportance", SVColorImportance_, 1.0);
    nh.param<double>("SVSpatialImportance", SVSpatialImportance_, 0.01);
    nh.param<double>("RANSACDistanceThreshold", RANSACDistanceThreshold_, 0.04);
    nh.param<int>("RANSACMinInliers", RANSACMinInliers_, 200);
    nh.param<double>("VoxelLeafSize", VoxelLeafSize_, 0.02);
    nh.param<double>("GP3SearchRad", GP3SearchRad_, 0.3); 
    nh.param<double>("GP3Mu", GP3Mu_, 2.5);
    nh.param<double>("GP3MaxNearestNeighbours", GP3MaxNearestNeighbours_, 100);
    nh.param<double>("GP3Ksearch", GP3Ksearch_, 20);
    nh.param<double>("RWHullMaxDist", RWHullMaxDist_, 0.3);
    nh.param<bool>("simplifyHulls", simplifyHulls_, true);
    nh.param<double>( "ECClusterTolerance", ECClusterTolerance_, 0.05);
    nh.param<int>( "ECMinClusterSize", ECMinClusterSize_, 100);
    nh.param<double>("hullAlpha", hullAlpha_, 0.1);
}

#ifdef __cplusplus
extern "C" {
#endif
    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "rosOgre_node");
        int o = 0;
        // Create application object
        testApplication app;
        ros::Rate loop_rate(0.2);

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
        }

        ros::Time begin;
        ros::Duration timeout(1.0);
        bool keepRolling = true;

        // ros::AsyncSpinner spinner(1);
        // std::cout << "about to start spinner" << std::endl;
        // spinner.start();
        // std::cout << "spinner started" << std::endl;
        // app.renderFrames();
        // std::cout << "finished rendering" << std::endl;
        // spinner.stop();
        // std::cout << "spinner stopped" << std::endl;

        while(ros::ok() && keepRolling) {
            if ( o > 100 ){
                break;
            }
            begin = ros::Time::now();
            ros::spinOnce();
            while (ros::Time::now() - begin < timeout){
                if( !app.renderOneFrame() ){
                    keepRolling = false;
                    break;
                }
            }
            o++;
            //loop_rate.sleep();
        }
        // ros::spin();

        char a = 1;
        const int s = 3;    
        std::bitset<s> x(a);
        for (size_t j = 0; j < 10; ++j){
            x = utils::shiftRight<s>(x);
            std::cout << x << std::endl;
        }

        app.destroyScene();

        return 0;
    }

#ifdef __cplusplus
}
#endif
