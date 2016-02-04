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
    cmprs.setGP3MinAngle( loadParam<double>("GP3MinAngle", nh) );
    cmprs.setGP3MaxAngle( loadParam<double>("GP3MaxAngle", nh) );

    point_cloud_subscriber   = nh.subscribe(loadParam<std::string>("TOPIC_POINT_CLOUD", nh), 1, &testApplication::point_cloud_callback, this);
}
//-------------------------------------------------------------------------------------
testApplication::~testApplication(void)
{
}

//-------------------------------------------------------------------------------------
void testApplication::updateScene(PointCloudT::Ptr nonPlanar, std::vector<PointCloudT::Ptr> planes, std::vector<PointCloudT::Ptr> hulls, std::vector<float> gp3_rad, std::vector<std::vector<float> > normals){
    // Initialize the manual object if it hasnt been
    if (first){
        manual = mSceneMgr->createManualObject("manual");
    }

    // for ( size_t i = 0; i < hulls.size(); ++i){
    //     for ( size_t j = 0; j < hulls.at(i)->points.size(); ++j){
    //         hulls.at(i)->points[j].r = 255;
    //         hulls.at(i)->points[j].g = 255-2*j;
    //         hulls.at(i)->points[j].b = 0;
    //     }
    // }

    std::vector<EXX::cloudMesh> cmesh;
    std::cout << "trian" << std::endl;
    cmprs.greedyProjectionTriangulationPlanes(nonPlanar, planes, hulls, cmesh, gp3_rad);
    std::cout << "impr" << std::endl;
    cmprs.improveTriangulation2(cmesh, planes, hulls, normals);
    std::cout << "hmm" << std::endl;


    std::cout << "Normal: " << std::endl;
    std::for_each(normals[0].begin(), normals[0].end(), [](float val){std::cout << " " << val;});
    std::cout << " " << std::endl;
    manual->clear();
    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    PointCloudT::Ptr tmp_cloud (new PointCloudT ());
    for (int i = 0; i < cmesh.size(); ++i){
        *tmp_cloud += *cmesh[i].cloud;
    }

    int pic = 0;
    Ogre::Real r, g, b;
    for (std::vector<EXX::cloudMesh>::iterator ite = cmesh.begin(); ite < cmesh.end(); ++ite){

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
    std::vector<std::vector<float> > normals;
    PointCloudT::Ptr nonPlanar (new PointCloudT ());
    float k = 0;

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

    for ( auto i : cloud_msg.normal ){
        normals.push_back(i.normal);
    }
    // normals = cloud_msg.normal;

    pcl::fromROSMsg (cloud_msg.nonPlanar, *nonPlanar);

    testApplication::updateScene(nonPlanar, planes, hulls, gp3_rad, normals);
}

void testApplication::loadParams(){
    configPath_ = loadParam<std::string>("configPath", nh);
}

#ifdef __cplusplus
extern "C" {
#endif
    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "rosOgre_node");

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
        ros::Duration timeout(0.5);
        bool keepRolling = true;
        while(ros::ok() && keepRolling) {
            begin = ros::Time::now();
            ros::spinOnce();
            while (ros::Time::now() - begin < timeout){
                if( !app.renderOneFrame() ){
                    keepRolling = false;
                    break;
                }
            }
        }

        app.destroyScene();
        return 0;
    }

#ifdef __cplusplus
}
#endif
