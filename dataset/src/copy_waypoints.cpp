#include <ros/ros.h>
// #include <utils/utils.h>
#include <metaroom_xml_parser/simple_xml_parser.h>

// OTHER
#include <boost/thread/thread.hpp>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <sstream>


// DEFINITIONS
#define PRINT                   1
#define BUFFER_SIZE             1
#define NODE_NAME               "copy_waypoints"

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointNT = pcl::PointNormal;
using PointNCloudT = pcl::PointCloud<PointNT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

using namespace EXX::params;
using namespace EXX;


int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh = ros::NodeHandle("~");

    ros::Rate loop_rate(10);

    return 0;
}
