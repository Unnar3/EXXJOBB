#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <exx_compression/compression.h>
#include <exx_compression/planes.h>
#include <dbscan/dbscan.h>
#include <utils/utils.h>        
#include <plane_features/plane_features.h>
#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <pcl/filters/extract_indices.h>    
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

// OTHER
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include <complex>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <sstream>


// DEFINITIONS
#define PRINT                   1
#define BUFFER_SIZE             1
#define NODE_NAME               "test_node"
#define TOPIC_POINT_CLOUD       "/camera/depth_registered/points"
#define TOPIC_EXTRACTED_PLANES  "/EXX/compressedPlanes"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
using namespace EXX::params;

class TestCompression
{ 
public:
    ros::NodeHandle nh;
private:
    ros::Subscriber point_cloud_subscriber;
    ros::Publisher point_cloud_publisher;
    EXX::compression cmprs;
    primitive_params params;

public:
    TestCompression()
    {
        nh = ros::NodeHandle("~");
        point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &TestCompression::point_cloud_callback, this);
        point_cloud_publisher  = nh.advertise<exx_compression::planes> (TOPIC_EXTRACTED_PLANES, BUFFER_SIZE);
        cmprs.setVoxelLeafSize(loadParam<double>("VoxelLeafSize", nh));
        cmprs.setSVVoxelResolution(loadParam<double>("SVVoxelResolution", nh));
        cmprs.setSVSeedResolution(loadParam<double>("SVSeedResolution", nh));
        cmprs.setSVColorImportance(loadParam<double>("SVColorImportance", nh));
        cmprs.setSVSpatialImportance(loadParam<double>("SVSpatialImportance", nh));
        cmprs.setRWHullMaxDist(loadParam<double>("RWHullMaxDist", nh));
        cmprs.setHULLAlpha(loadParam<double>("hullAlpha", nh));

        params.number_disjoint_subsets = loadParam<int>("disjoinedSet", nh);
        params.octree_res              = loadParam<double>("octree_res", nh);
        params.normal_neigbourhood     = loadParam<double>("normal_neigbourhood", nh);
        params.inlier_threshold        = loadParam<double>("inlier_threshold", nh);
        params.angle_threshold         = loadParam<double>("angle_threshold", nh);
        params.add_threshold           = loadParam<double>("add_threshold", nh);
        params.connectedness_res       = loadParam<double>("connectedness_res", nh);
        params.distance_threshold      = loadParam<double>("distance_threshold", nh);
    
        std::cout << "leaf size: " << cmprs.getVoxelLeafSize() << std::endl;
    }

    void testCompression(PointCloudT::Ptr cloud)
    {
        std::cout << "entering compression" << std::endl;
        

        // Load all params for compression and Ransac

        // VOXEL GRID FILTER
        PointCloudT::Ptr voxel_cloud (new PointCloudT ());
        cmprs.voxelGridFilter(cloud, voxel_cloud);

        params.min_shape               = voxel_cloud->points.size()*0.000001;
        params.inlier_min              = params.min_shape;

        // Perform the compression        
        // RANSAC
        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(voxel_cloud, primitives, params, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract(extracted);

        std::vector<PointCloudT::Ptr> plane_vec;
        std::vector<Eigen::Vector4d> normal;
        std::vector<int> ind;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ExtractIndices<PointT> extract;
        Eigen::VectorXd data;
        for (size_t j = 0; j < extracted.size(); ++j){
            ind = extracted[j]->supporting_inds;
            
            inliers->indices.reserve(inliers->indices.size() + ind.size());
            inliers->indices.insert(inliers->indices.end(), ind.begin(), ind.end());

            extracted.at(j)->shape_data(data); 
            normal.push_back(data.segment<4>(0));
            PointCloudT::Ptr test_cloud (new PointCloudT ());
            for (size_t i = 0; i < ind.size(); ++i){
                test_cloud->points.push_back(voxel_cloud->points[ind[i]]);
            }
            plane_vec.push_back(test_cloud);
        }

        PointCloudT::Ptr nonPlanar (new PointCloudT ());
        extract.setInputCloud (voxel_cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*nonPlanar);

        // Define all remaining data structures
        std::vector<PointCloudT::Ptr> hulls;
        std::vector<PointCloudT::Ptr> simplified_hulls;
        std::vector<EXX::densityDescriptor> dDesc;
        std::vector<PointCloudT::Ptr> super_planes;

        // PROJECT TO PLANE
        for ( size_t i = 0; i < normal.size(); ++i ){
            EXX::compression::projectToPlaneS( plane_vec[i], normal[i] );
        }

        // FIND CONCAVE HULL
        cmprs.planeToConcaveHull(&plane_vec, &hulls);
        cmprs.getPlaneDensity( plane_vec, hulls, dDesc);
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);
        cmprs.cornerMatching(plane_vec, simplified_hulls, normal);
        cmprs.superVoxelClustering(&plane_vec, &super_planes, dDesc);
        cloudPublish( nonPlanar, super_planes, simplified_hulls, dDesc );
    }

private:
    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        PointCloudT::Ptr cloud (new PointCloudT ());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        testCompression(cloud);
    }

    void cloudPublish(PointCloudT::Ptr nonPlanar ,std::vector<PointCloudT::Ptr> &planes, std::vector<PointCloudT::Ptr> &hulls, std::vector<EXX::densityDescriptor> &dDesc)
    {
        exx_compression::planes pmsgs;
        sensor_msgs::PointCloud2 output_p;
        sensor_msgs::PointCloud2 output_h;
        sensor_msgs::PointCloud2 output;
        
        for (size_t i = 0; i < planes.size(); ++i){
            pcl::toROSMsg(*planes[i] , output_p);
            pcl::toROSMsg(*hulls[i] , output_h);
            pmsgs.planes.push_back(output_p);
            pmsgs.hulls.push_back(output_h);
            pmsgs.gp3_rad.push_back(dDesc[i].gp3_search_rad);
        }
        pcl::toROSMsg(*nonPlanar , output);
        pmsgs.nonPlanar = output;
        point_cloud_publisher.publish (pmsgs);
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);

    TestCompression test;
    
    ros::Rate loop_rate(loadParam<int>("HZ", test.nh ));
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}