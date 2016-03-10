#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <utils/utils.h>
// #include <metaroom_xml_parser/simple_xml_parser.h>
#include <metaroom_xml_parser/load_utilities.h>

// OTHER
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

    std::string path = "/media/unnar/HDD/thesis/KTH_longterm_dataset_registered";
    std::string waypoint = "WayPoint5";
    std::string txt_base = "/media/unnar/HDD/thesis/test.txt";
    std::string dataset_base = "/media/unnar/HDD/thesis/Dataset";
    std::vector<std::string> rooms = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointT>(path, waypoint);
    std::ofstream output_file(txt_base);

    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(rooms.begin(), rooms.end(), output_iterator);

    std::ofstream myfile1 (txt_base);
    if (myfile1.is_open())
    {
        for(auto str : rooms){
            std::string out = str + "\n";
            myfile1 << out;
        }
        myfile1.close();
    }

    std::vector<std::string> test;
    std::string line;
    std::ifstream myfile2 (txt_base);
    if (myfile2.is_open())
        {
        while ( std::getline (myfile2,line) )
        {
            std::cout << line << '\n';
        }
        myfile2.close();
    }

    ros::Rate loop_rate(10);

    return 0;
}
