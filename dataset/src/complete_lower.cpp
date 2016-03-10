#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <utils/utils.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <metaroom_xml_parser/load_utilities.h>

// OTHER
#include <tf_conversions/tf_eigen.h>

#include <boost/thread/thread.hpp>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <vector>
#include <stdlib.h>
#include <time.h>
// #include <filesystem.h>
// #include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>


// DEFINITIONS
#define PRINT                   1
#define BUFFER_SIZE             1
#define NODE_NAME               "copy_waypoints"

// using PointT = pcl::PointXYZRGB;
typedef pcl::PointXYZRGB PointT;
// using PointCloudT = pcl::PointCloud<PointT>;
// using PointNT = pcl::PointNormal;
// using PointNCloudT = pcl::PointCloud<PointNT>;
// using NormalT = pcl::Normal;
// using NormalCloudT = pcl::PointCloud<NormalT>;
// using SurfelT = SurfelType;
// using SurfelCloudT = pcl::PointCloud<SurfelT>;


int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh = ros::NodeHandle("~");

    std::string path = "/media/unnar/HDD/thesis/Dataset_upload/";
    std::string waypoint = "WayPoint16";
    // std::string txt_base = "/media/unnar/HDD/thesis/test.txt";
    // std::string dataset_base = "/media/unnar/HDD/thesis/Dataset";
    std::vector<std::string> rooms = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointT>(path, waypoint);

    for(auto str : rooms){
        std::size_t t = str.find_last_of("/\\");
        std::string path = str.substr(0, t+1);
        auto roomdata = SimpleXMLParser<PointT>::loadRoomFromXML(str);

        auto sweep = SimpleXMLParser<PointT>::loadRoomFromXML(path + "room.xml");

        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());

        std::cout << path << std::endl;
        for (size_t i = 0; i < 17; i++) {
            tf::StampedTransform rotationTF = sweep.vIntermediateRoomCloudTransformsRegistered[i];
            Eigen::Affine3d trans;
            tf::transformTFToEigen(rotationTF, trans);
            pcl::transformPointCloud (*sweep.vIntermediateRoomClouds[i], *sweep.vIntermediateRoomClouds[i], trans);
            *cloud += *sweep.vIntermediateRoomClouds[i];
        }


        pcl::PCDWriter writer;

        writer.write(path + "complete_cloud_lower.pcd", *cloud);

    }


    ros::Rate loop_rate(10);

    return 0;
}
