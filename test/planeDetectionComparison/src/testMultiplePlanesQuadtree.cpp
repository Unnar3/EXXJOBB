#include <ros/ros.h>
#include <exx_compression/compression.h>
#include <exx_compression/planes.h>
#include <plane_extraction/plane_extraction.h>
#include <utils/utils.cpp>
#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <PointTypes/surfel_type.h>
#include <pcl/visualization/pcl_visualizer.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <quadtree/quadtree.h>
#include <quadtree/quadtreePCL.h>
#include <pcl/TextureMesh.h>
#include <quadtree/saveOBJFile.h>


// DEFINITIONS
#define NODE_NAME               "multipleQuadtree"

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;

using namespace EXX::params;
using namespace EXX;

class Comparison
{
public:
    ros::NodeHandle nh;
private:
    ros::Publisher point_cloud_publisher;
    EXX::compression cmprs;
    EXX::planeExtraction planeEx;
    primitive_params params;

public:

    Comparison()
    {
        nh = ros::NodeHandle("~");
        cmprs.setVoxelLeafSize(loadParam<double>("VoxelLeafSize", nh));
        cmprs.setSVVoxelResolution(loadParam<double>("SVVoxelResolution", nh));
        cmprs.setSVSeedResolution(loadParam<double>("SVSeedResolution", nh));
        cmprs.setSVColorImportance(loadParam<double>("SVColorImportance", nh));
        cmprs.setSVSpatialImportance(loadParam<double>("SVSpatialImportance", nh));
        cmprs.setRWHullMaxDist(loadParam<double>("RWHullMaxDist", nh));
        cmprs.setHULLAlpha(loadParam<double>("hullAlpha", nh));
        cmprs.setGP3SearchRad( loadParam<double>("GP3SearchRad", nh) );
        cmprs.setGP3Mu( loadParam<double>("GP3Mu", nh) );
        cmprs.setGP3MaxNearestNeighbours( loadParam<double>("GP3MaxNearestNeighbours", nh) );
        cmprs.setGP3Ksearch( loadParam<double>("GP3Ksearch", nh) );

        params.number_disjoint_subsets = loadParam<int>("disjoinedSet", nh);
        params.octree_res              = loadParam<double>("octree_res", nh);
        params.normal_neigbourhood     = loadParam<double>("normal_neigbourhood", nh);
        params.inlier_threshold        = loadParam<double>("inlier_threshold", nh);
        params.angle_threshold         = loadParam<double>("angle_threshold", nh);
        params.add_threshold           = loadParam<double>("add_threshold", nh);
        params.connectedness_res       = loadParam<double>("connectedness_res", nh);
        params.distance_threshold      = loadParam<double>("distance_threshold", nh);
        params.inlier_min              = loadParam<int>("inlier_min", nh);
        params.min_shape               = loadParam<int>("min_shape", nh);

        planeEx.setPrimitiveParameters(params);
        std::cout << "leaf size: " << cmprs.getVoxelLeafSize() << std::endl;
    }

    void testComparison(void){
        std::string path = loadParam<std::string>("path_test", nh);

        PointCloudT::Ptr segment (new PointCloudT());
        NormalCloudT::Ptr normals (new NormalCloudT());

        pcl::PCDReader reader;
        reader.read (path + "testCloud_complete.pcd", *segment);

        std::cout << "segment: " << segment->points.size() << std::endl;


        // std::vector<PointCloudT::Ptr> plane_vec;
        // std::vector<Eigen::Vector4d> normal_vec;

        ////////////////////////////////////////////////////////////////////////
        // EFFICIENT RANSAC with PPR
        ////////////////////////////////////////////////////////////////////////

        PointCloudT::Ptr nonPlanar (new PointCloudT());

        std::vector<PointCloudT::Ptr> plane_vec;
        std::vector<pcl::ModelCoefficients::Ptr> normal_vec;

        std::cout << "Efficient PPR..................." << std::endl;
        // planeDetection::planeSegmentationEfficientPPR(segment, params, plane_vec, normal_vec, nonPlanar);
        planeEx.planeSegmentationEfficientPPR(segment, normals, plane_vec, normal_vec, nonPlanar);
        // PROJECT TO PLANE
        for ( size_t i = 0; i < normal_vec.size(); ++i ){
            EXX::compression::projectToPlaneS( plane_vec[i], normal_vec[i] );
        }
        // PointCloudT::Ptr outCloudEfficientPPR( new PointCloudT() );
        // planeEx.combinePlanes(plane_vec_efficient_ppr, outCloudEfficientPPR, true);

        EXX::compression cmprs;
        cmprs.setRWHullMaxDist(0.02);
        cmprs.setHULLAlpha(0.07);

        std::vector<PointCloudT::Ptr> hulls;
        cmprs.planeToConcaveHull(&plane_vec, &hulls);
        std::vector<EXX::densityDescriptor> dDesc;
        EXX::densityDescriptor dens;
        dens.rw_max_dist = 0.15;
        dDesc.push_back(dens);
        std::vector<PointCloudT::Ptr> simplified_hulls;
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);


        objStruct<PointT> object(1);
        for ( size_t i = 0; i < plane_vec.size(); ++i ){
        // for ( size_t i = 0; i < 2; ++i ){

            QuadTreePCL<PointT> qtpcl(1,10,0,0);
            // qtpcl.setMaxLevel(10);
            qtpcl.setMaxWidth(1);
            qtpcl.setNormal(normal_vec[i]);

            PointCloudT::Ptr out (new PointCloudT());
            std::vector< pcl::Vertices > vertices;
            qtpcl.insertBoundary(hulls[i]);
            qtpcl.createMesh<PointT>(out, vertices);

            cv::Mat image;
            std::vector<Eigen::Vector2f> vertex_texture;
            qtpcl.createTexture<PointT>(plane_vec[i], out, image, vertex_texture);

            object.clouds.push_back(out);
            object.polygons.push_back(vertices);
            object.images.push_back(image);
            object.texture_vertices.push_back(vertex_texture);
            object.coefficients.push_back(normal_vec[i]);

        }
        saveOBJFile("/home/unnar/Desktop/Mesh/multiQuad.obj", object, 5);

        pcl::PCDWriter writer;
        for ( size_t i = 1; i < plane_vec.size(); ++i ){
            *plane_vec[0] += *plane_vec[i];
            *hulls[0] += *hulls[i];
        }
        writer.write("/home/unnar/Desktop/Mesh/outCloudEfficientPPR.pcd", *plane_vec[0]);
        writer.write("/home/unnar/Desktop/Mesh/outCloudEfficientPPRHulls.pcd", *hulls[0]);

    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);

    Comparison test;

    ros::Rate loop_rate(loadParam<int>("HZ", test.nh ));
    test.testComparison();

    return 0;
}
