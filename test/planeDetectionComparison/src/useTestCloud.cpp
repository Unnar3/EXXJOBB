#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <exx_compression/compression.h>
#include <exx_compression/planes.h>
// #include <utils/utils.h>
#include <dbscan/dbscan.h>
#include <utils/utils.cpp>
#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <PointTypes/surfel_type.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/common.h>

// OTHER
#include <planeDetectionComparison/planeSegmentationPCL.h>
#include <planeDetectionComparison/planeSegmentationPPR.h>
#include <planeDetectionComparison/planeSegmentationEfficient.h>
#include <planeDetectionComparison/cgalTriangulation.h>
#include <planeDetectionComparison/utils.h>
#include <Refinement/SurfaceRefinement.h>
#include <boost/thread/thread.hpp>
#include <tf_conversions/tf_eigen.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include <algorithm>
#include <complex>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <sstream>


// DEFINITIONS
#define PRINT                   1
#define BUFFER_SIZE             1
#define NODE_NAME               "use_test_cloud"
#define TOPIC_POINT_CLOUD       "/camera/depth_registered/points"
#define TOPIC_EXTRACTED_PLANES  "/EXX/compressedPlanes"

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

class UseTestCloud
{
public:
    ros::NodeHandle nh;
private:
    ros::Publisher point_cloud_publisher;
    EXX::compression cmprs;
    primitive_params params;

public:

    UseTestCloud()
    {
        nh = ros::NodeHandle("~");
        point_cloud_publisher  = nh.advertise<exx_compression::planes> (TOPIC_EXTRACTED_PLANES, BUFFER_SIZE);
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

        std::cout << "leaf size: " << cmprs.getVoxelLeafSize() << std::endl;
    }

    void testCloud(void){
        std::string path = loadParam<std::string>("path_test", nh);

        // SurfelCloudT::Ptr surfel_cloud (new SurfelCloudT());
        PointCloudT::Ptr segment (new PointCloudT());
        NormalCloudT::Ptr normals (new NormalCloudT());

        pcl::PCDReader reader;
        reader.read (path + "testCloud_complete.pcd", *segment);

        std::cout << "segment: " << segment->points.size() << std::endl;


        std::vector<PointCloudT::Ptr> plane_vec;
        std::vector<pcl::ModelCoefficients::Ptr> normal_vec;
        PointCloudT::Ptr nonPlanar (new PointCloudT());

        std::cout << "Efficient PPR..................." << std::endl;
        planeDetection::planeSegmentationEfficientPPR(segment, params, plane_vec, normal_vec, nonPlanar);
        // PROJECT TO PLANE
        for ( size_t i = 0; i < normal_vec.size(); ++i ){
            EXX::compression::projectToPlaneS( plane_vec[i], normal_vec[i] );
        }

        // Define all remaining data structures
        std::vector<PointCloudT::Ptr> hulls;
        std::vector<PointCloudT::Ptr> simplified_hulls;
        std::vector<EXX::densityDescriptor> dDesc;
        std::vector<PointCloudT::Ptr> super_planes;

        // FIND CONCAVE HULL/* message */
        cmprs.planeToConcaveHull(&plane_vec, &hulls);
        cmprs.getPlaneDensity( plane_vec, hulls, dDesc);
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);
        // cmprs.cornerMatching(plane_vec, simplified_hulls, normal_vec);
        cmprs.superVoxelClustering(&plane_vec, &super_planes, dDesc);

        int s = 0;
        PointCloudT::Ptr combined (new PointCloudT());
        pcl::PolygonMesh mesh;
        for (size_t i = 0; i < plane_vec.size(); i++) {

            std::vector<Point> plane_2d;
            std::vector<Point> boundary_2d;

            pclPlaneToCGAL<pcl::PointXYZRGB>(super_planes[i], hulls[i], normal_vec[i], plane_2d, boundary_2d);
            std::vector<std::vector<unsigned int> > idx;
            constrainedDelaunayTriangulation(plane_2d, boundary_2d, idx);

            *combined += *super_planes[i];
            for(auto &p : hulls[i]->points){
                p.r = 150;
                p.g = 150;
                p.b = 150;
            }
            *combined += *hulls[i];


            pcl::Vertices vert;
            vert.vertices.resize(3);
            for(auto poly : idx){
                vert.vertices[0] = poly[0]+s;
                vert.vertices[1] = poly[1]+s;
                vert.vertices[2] = poly[2]+s;
                mesh.polygons.push_back(vert);
                // std::cout << poly[0] << ", " << poly[1] << ", " << poly[2] << std::endl;
                // std::cout << "-----------------------" << std::endl;
            }
            s = s + super_planes[i]->points.size() + hulls[i]->points.size();

        }
        planeDetection::toMeshCloud(*combined, mesh.cloud);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPolygonMesh(mesh,"meshes",0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        while (!viewer->wasStopped ()){
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }


        PointCloudT::Ptr outCloudEfficientPPR( new PointCloudT() );
        combinePlanes(plane_vec, outCloudEfficientPPR);

        pcl::PCDWriter writer;
        writer.write(path + "outCloudEfficientPPR.pcd", *outCloudEfficientPPR);

    }

private:


    void planeSegmentationPPR(){}


    void planeSegmentationNILS( const PointCloudT::Ptr cloud_in,
                            const NormalCloudT::Ptr normals,
                            std::vector<PointCloudT::Ptr> &plane_vec,
                            std::vector<Eigen::Vector4d> &normal_vec,
                            PointCloudT::Ptr nonPlanar){
        // Perform the compression
        // RANSAC
        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(cloud_in, normals, primitives, params, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract(extracted);


        std::vector<int> ind;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ExtractIndices<PointT> extract;
        Eigen::VectorXd data;
        for (size_t j = 0; j < extracted.size(); ++j){
            ind = extracted[j]->supporting_inds;

            if(ind.size() < 1000) continue;

            inliers->indices.reserve(inliers->indices.size() + ind.size());
            inliers->indices.insert(inliers->indices.end(), ind.begin(), ind.end());

            extracted.at(j)->shape_data(data);
            normal_vec.push_back(data.segment<4>(0));
            PointCloudT::Ptr test_cloud (new PointCloudT ());
            for (size_t i = 0; i < ind.size(); ++i){
                test_cloud->points.push_back(cloud_in->points[ind[i]]);
            }
            plane_vec.push_back(test_cloud);
        }

        extract.setInputCloud (cloud_in);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*nonPlanar);
    }



    NormalCloudT::Ptr compute_surfel_normals(SurfelCloudT::Ptr& surfel_cloud, PointCloudT::Ptr& segment)
    {
        pcl::KdTreeFLANN<SurfelT> kdtree;
        kdtree.setInputCloud(surfel_cloud);
        NormalCloudT::Ptr normals(new NormalCloudT);
        normals->reserve(segment->size());
        for (const PointT& p : segment->points) {
            if (!pcl::isFinite(p)) {
                NormalT crap; crap.normal_x = 0; crap.normal_y = 0; crap.normal_z = 0;
                normals->push_back(crap);
                continue;
            }
            vector<int> indices;
            vector<float> distances;
            SurfelT s; s.x = p.x; s.y = p.y; s.z = p.z;
            kdtree.nearestKSearchT(s, 1, indices, distances);
            if (distances.empty()) {
                cout << "Distances empty, wtf??" << endl;
                exit(0);
            }
            SurfelT q = surfel_cloud->at(indices[0]);
            NormalT n; n.normal_x = q.normal_x; n.normal_y = q.normal_y; n.normal_z = q.normal_z;
            normals->push_back(n);
        }
        return normals;
    }

    void generateRandomColor(int mix_red, int mix_green, int mix_blue,
        int &red, int &green, int& blue) {

        red = ((rand() % 255) + mix_red) / 2;
        green = ((rand() % 255) + mix_green) / 2;
        blue = ((rand() % 255) + mix_blue) / 2;
    }


    void combinePlanes( const   std::vector<PointCloudT::Ptr>   &planes,
                                PointCloudT::Ptr                out)
    {
        // Check if out is empty an clear it if is not
        if( out->points.size() == 0 ) out->clear();
        int red, green, blue;
        for( auto plane : planes ){
            generateRandomColor(137,196,244, red,green,blue);
            int sizeOut = out->points.size();
            *out += *plane;
            for (size_t i = sizeOut; i < out->points.size(); i++) {
                out->points[i].r = red;
                out->points[i].g = green;
                out->points[i].b = blue;
            }
        }
    }

    void combinePlanes( const   std::vector<PointCloudT::Ptr>   &planes,
                        const   PointCloudT::Ptr                nonPlanar,
                                PointCloudT::Ptr                out)
    {
        // Check if out is empty an clear it if is not
        if( out->points.size() == 0 ) out->clear();

        combinePlanes(planes, out);

        int red, green, blue;
        generateRandomColor(137,196,244, red,green,blue);
        int sizeOut = out->points.size();
        *out += *nonPlanar;
        for (size_t i = sizeOut; i < out->points.size(); i++) {
            out->points[i].r = 255;
            out->points[i].g = 255;
            out->points[i].b = 255;
        }
    }


    void cloudPublish(PointCloudT::Ptr nonPlanar ,std::vector<PointCloudT::Ptr> &planes, std::vector<PointCloudT::Ptr> &hulls, std::vector<EXX::densityDescriptor> &dDesc,std::vector<Eigen::Vector4d> &normal)
    {
        exx_compression::planes pmsgs;
        exx_compression::normal norm;
        sensor_msgs::PointCloud2 output_p;
        sensor_msgs::PointCloud2 output_h;
        sensor_msgs::PointCloud2 output;
        Eigen::Vector4f tmpnormal;

        for (size_t i = 0; i < planes.size(); ++i){
            pcl::toROSMsg(*planes[i] , output_p);
            pcl::toROSMsg(*hulls[i] , output_h);
            pmsgs.planes.push_back(output_p);
            pmsgs.hulls.push_back(output_h);
            pmsgs.gp3_rad.push_back(dDesc[i].gp3_search_rad);
            tmpnormal = normal[i].cast<float>();
            norm.normal.push_back(tmpnormal[0]);
            norm.normal.push_back(tmpnormal[1]);
            norm.normal.push_back(tmpnormal[2]);
            pmsgs.normal.push_back(norm);
        }
        pcl::toROSMsg(*nonPlanar , output);
        pmsgs.nonPlanar = output;
        point_cloud_publisher.publish (pmsgs);
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);

    UseTestCloud test;

    ros::Rate loop_rate(loadParam<int>("HZ", test.nh ));
    test.testCloud();

    return 0;
}
