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
#include <simple_xml_parser.h>
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

class timeMeasurement
{
    std::string name;
    ros::Duration min;
    ros::Duration max;
    std::vector<ros::Duration> durs;
public:    

    void setName(std::string n){
        name = n;
    }   
    void addDuration(ros::Duration dur){
        if ( durs.size() == 0 ){
            min = dur;
            max = dur;
        }
        else if ( dur < min ){
            min = dur; 
        } else if ( dur > max ) {
            max = dur;
        }
        durs.push_back( dur );
    }
    double getMin(){
        return min.toSec();
    }
    double getMax(){
        return max.toSec();
    }
    double getAverage(){
        ros::Duration comb;
        for (auto i : durs){
            comb += i;
        }
        return comb.toSec() / durs.size();
    }
    void printStat(){
        std::cout << name << std::endl;
        std::cout << "  min: " << getMin() << " sec" << std::endl;
        std::cout << "  max: " << getMax() << " sec" << std::endl;
        std::cout << "  average: " << getAverage() << " sec" << std::endl;
    }
};

class TestCompression
{ 
public:
    ros::NodeHandle nh;
private:
    ros::Subscriber point_cloud_subscriber;
    ros::Publisher point_cloud_publisher;
    EXX::compression cmprs;
    primitive_params params;
    timeMeasurement comp, vox, rans, pp, dens, ch, sch, sv;

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

        vox.setName("voxelGridFilter");
        rans.setName("ransac");
        pp.setName("Project to Plane");
        dens.setName("Density measure");
        ch.setName("Concave Hull");
        sch.setName("Simplify Concave Hull");
        sv.setName("Super Voxels");
        comp.setName("Compression");
    }

    void testCompression()
    {

        PointCloudT::Ptr cloud (new PointCloudT ());
        auto sweep = SimpleXMLParser<PointT>::loadRoomFromXML("/home/unnar/catkin_ws/src/Metarooms/room_3/room.xml");
        
        for (size_t i = 0; i < sweep.vIntermediateRoomClouds.size(); i++){
        // for (size_t i = 0; i < 10; i++){
            cloud = sweep.vIntermediateRoomClouds[i];
            tf::StampedTransform rotationTF = sweep.vIntermediateRoomCloudTransforms[i];
            //pcl_ros::transformPointCloud(*cloud, *cloud, rotationTF);

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
            performCompression(cloud);
        }
        vox.printStat();
        rans.printStat();
        pp.printStat();
        dens.printStat();
        ch.printStat();
        sch.printStat();
        sv.printStat();
        comp.printStat();
    }

    void performCompression(PointCloudT::Ptr cloud)
    {    
        // Load all params for compression and Ransac
        // VOXEL GRID FILTER
        ros::Time t_v1,t_v2,t_r1,t_r2,t_p1,t_p2,t_ch1,t_ch2,t_d1,t_d2,t_sh1,t_sh2,t_sv1,t_sv2;
        PointCloudT::Ptr voxel_cloud (new PointCloudT ());
        t_v1 = ros::Time::now();
        cmprs.voxelGridFilter(cloud, voxel_cloud);
        t_v2 = ros::Time::now();
        vox.addDuration(t_v2 - t_v1);

        t_r1 = ros::Time::now();
        params.min_shape  = voxel_cloud->points.size()*0.00005;
        params.inlier_min = params.min_shape;

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
        t_r2 = ros::Time::now();
        rans.addDuration(t_r2-t_r1);
        // Define all remaining data structures
        std::vector<PointCloudT::Ptr> hulls;
        std::vector<PointCloudT::Ptr> simplified_hulls;
        std::vector<EXX::densityDescriptor> dDesc;
        std::vector<PointCloudT::Ptr> super_planes;

        t_p1 = ros::Time::now();
        // PROJECT TO PLANE
        for ( size_t i = 0; i < normal.size(); ++i ){
            EXX::compression::projectToPlaneS( plane_vec[i], normal[i] );
        }
        t_p2 = ros::Time::now();
        pp.addDuration(t_p2-t_p1);
        // FIND CONCAVE HULL
        t_ch1 = ros::Time::now();
        cmprs.planeToConcaveHull(&plane_vec, &hulls);
        t_ch2 = ros::Time::now();
        ch.addDuration(t_ch2-t_ch1);
        t_d1 = ros::Time::now();
        cmprs.getPlaneDensity( plane_vec, hulls, dDesc);
        t_d2 = ros::Time::now();
        dens.addDuration(t_d2-t_d1);
        t_sh1 = ros::Time::now();
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);
        t_sh2 = ros::Time::now();
        sch.addDuration(t_sh2 - t_sh1);
        // cmprs.cornerMatching(plane_vec, simplified_hulls, normal);
        t_sv1 = ros::Time::now();
        cmprs.superVoxelClustering(&plane_vec, &super_planes, dDesc);
        t_sv2 = ros::Time::now();
        sv.addDuration(t_sv2-t_sv1);
        comp.addDuration(t_sv2 - t_v1);
        cloudPublish( nonPlanar, super_planes, simplified_hulls, dDesc );
    }

private:
    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        PointCloudT::Ptr cloud (new PointCloudT ());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        // testCompression(cloud);
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
    // while(ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    // 
    test.testCompression();

    return 0;
}