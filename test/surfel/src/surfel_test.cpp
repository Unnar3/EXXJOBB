#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <exx_compression/compression.h>
#include <exx_compression/planes.h>
// #include <utils/utils.h>
#include <dbscan/dbscan.h>
#include <utils/utils.cpp>
#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <simple_xml_parser.h>
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

// OTHER
#include <boost/thread/thread.hpp>
#include <tf_conversions/tf_eigen.h>
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
#define NODE_NAME               "test_surfel"
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

class TestSurfel
{
public:
    ros::NodeHandle nh;
private:
    ros::Subscriber point_cloud_subscriber;
    ros::Publisher point_cloud_publisher;
    EXX::compression cmprs;
    primitive_params params;

public:

    TestSurfel()
    {
        nh = ros::NodeHandle("~");
        point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &TestSurfel::point_cloud_callback, this);
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

    void testSurfel(void){
        std::string path = loadParam<std::string>("path", nh);

        SurfelCloudT::Ptr surfel_cloud (new SurfelCloudT());
        PointCloudT::Ptr segment (new PointCloudT());
        NormalCloudT::Ptr normals (new NormalCloudT());

        pcl::PCDReader reader;
        reader.read (path + "surfel_map.pcd", *surfel_cloud);
        reader.read (path + "complete_cloud.pcd", *segment);

        auto sweep = SimpleXMLParser<PointT>::loadRoomFromXML(path + "room.xml");
        tf::StampedTransform rotationTF = sweep.vIntermediateRoomCloudTransforms.front();
        Eigen::Affine3d trans;
        tf::transformTFToEigen(rotationTF, trans);

        pcl::transformPointCloud (*segment, *segment, trans);
        pcl::transformPointCloudWithNormals (*surfel_cloud, *surfel_cloud, trans);

        // EXX::utils::OBBcube obbcube;
        // EXX::utils::getOBB(segment, obbcube);
        //
        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // transform.translation() << obbcube.position[0], obbcube.position[1], obbcube.position[2];
        // transform.rotate(obbcube.rot);
        //
        // pcl::transformPointCloud (*segment, *segment, transform.inverse());


        // Eigen::Vector3f position;
        // position[0] = 0; position[1] = 0; position[2] = 0;
        // quat =  Eigen::Quaternionf();
        // Eigen::Quaternionf quat (obbcube.rot);
        // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        // viewer->setBackgroundColor (0, 0, 0);
        // viewer->addCoordinateSystem (1.0);
        // viewer->initCameraParameters ();
        // viewer->addPointCloud<pcl::PointXYZRGB> (segment, "sample cloud");
        // viewer->addCube (obbcube.position, quat,
        //     obbcube.x,
        //     obbcube.y,
        //     obbcube.z,
        //     "OBB");
        //
        // while(!viewer->wasStopped())
        // {
        //     viewer->spinOnce (100);
        //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        // }

        // return;

        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // transform.translation() << obbcube.position[0], obbcube.position[1], obbcube.position[2];
        // transform.rotate(obbcube.rot);
        // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        // transform_1.block<3,3>(0,0) = obbcube.rot.transpose();


        // pcl::transformPointCloud (*segment, *segment, transform.inverse());
        // pcl::transformPointCloudWithNormals (*surfel_cloud, *surfel_cloud, transform.inverse());
        // pcl::transformPointCloud (*segment, *segment, transform_1);
        // pcl::transformPointCloudWithNormals (*surfel_cloud, *surfel_cloud, transform_1);

        // Compute OBB



        normals = compute_surfel_normals(surfel_cloud, segment);


        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*normals, *normals, indices);
        // pcl::removeNaNFromPointCloud(*segment, *segment, indices);


        std::vector<PointCloudT::Ptr> plane_vec;
        std::vector<Eigen::Vector4d> normal_vec;
        PointCloudT::Ptr nonPlanar (new PointCloudT());
        planeSegmentation(segment, normals, plane_vec, normal_vec, nonPlanar);

        // PROJECT TO PLANE
        for ( size_t i = 0; i < normal_vec.size(); ++i ){
            EXX::compression::projectToPlaneS( plane_vec[i], normal_vec[i] );
        }
        // for(auto normal : normal_vec)
        // std::cout << "normal: " << normal[0] << ", " << normal[1] << ", " << normal[2]  << std::endl;
        Eigen::Vector3d mean_norm = findMainNorm(normal_vec);

        // return;

        // Define all remaining data structures
        std::vector<PointCloudT::Ptr> hulls;
        std::vector<PointCloudT::Ptr> simplified_hulls;
        std::vector<EXX::densityDescriptor> dDesc;
        std::vector<PointCloudT::Ptr> super_planes;

        // FIND CONCAVE HULL
        cmprs.planeToConcaveHull(&plane_vec, &hulls);
        cmprs.getPlaneDensity( plane_vec, hulls, dDesc);
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);
        // cmprs.cornerMatching(plane_vec, simplified_hulls, normal_vec);
        cmprs.superVoxelClustering(&plane_vec, &super_planes, dDesc);
        // cloudPublish( nonPlanar, super_planes, simplified_hulls, dDesc );

        for ( size_t i = 0; i < normal_vec.size(); ++i ){
            EXX::compression::projectToPlaneS( super_planes[i], normal_vec[i] );
            EXX::compression::projectToPlaneS( simplified_hulls[i], normal_vec[i] );
        }




        int k = 0;
        int red, green, blue;
        PointCloudT::Ptr segmented_cloud (new PointCloudT());
        for (size_t i = 0; i < super_planes.size(); i++) {
            // if(super_planes[i]->points.size()<2000) continue;
            generateRandomColor(137,196,244, red,green,blue);
            // for(auto &point : super_planes[i]->points){
            //     point.r = red;
            //     point.g = green;
            //     point.b = blue;
            // }
            *segmented_cloud += *super_planes[i];
            // for(auto &point : simplified_hulls[i]->points){
            //     point.r = 255;//red+20;
            //     point.g = 255;//green+20;
            //     point.b = 255;//blue+20;
            // }
            *segmented_cloud += *simplified_hulls[i];
        }
        pcl::PCDWriter writer;
        writer.write(path + "segmented_cloud_planar.pcd", *segmented_cloud);

        PointCloudT::Ptr allPlanes (new PointCloudT());
        PointCloudT::Ptr nonPlanarFiltered (new PointCloudT());
        for(auto cloud : plane_vec){
            *allPlanes += *cloud;
        }


        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(allPlanes);

        for(auto point : nonPlanar->points){

            vector<int> indices;
            vector<float> distances;
            kdtree.nearestKSearchT(point, 1, indices, distances);
            if (distances.empty()) {
                cout << "Distances empty, wtf??" << endl;
                continue;
            }
            if (distances[0] > 0.005){
                nonPlanarFiltered->points.push_back(point);
            }
        }

        std::vector<PointCloudT::Ptr> vNonPlanar;
        ecClustering<PointT>(nonPlanarFiltered, 0.02, 500, 100000, vNonPlanar);

        for(auto cluster : vNonPlanar){
            generateRandomColor(137,196,244, red,green,blue);
            for(auto &point : cluster->points){
                point.r = red;
                point.g = green;
                point.b = blue;
            }
            *segmented_cloud += *cluster;
        }

        // pcl::PCDWriter writer;
        writer.write(path + "segmented_cloud.pcd", *segmented_cloud);
        cloudPublish( nonPlanarFiltered, super_planes, simplified_hulls, dDesc, normal_vec );
    }

private:


    Eigen::Vector3d findMainNorm(const std::vector<Eigen::Vector4d> &normals){
        std::vector<double> x;
        std::vector<double> y;
        x.reserve(normals.size());
        y.reserve(normals.size());
        Eigen::Vector3d tmp;

        for(auto normal : normals){
            int idx;
            normal.head(3).maxCoeff(&idx);
            if(idx != 2 && normal[2] < 0.1){
                tmp = normal.head(3);
                tmp[2] = 0;
                tmp = tmp/tmp.norm();
                if(std::abs(tmp[1]) > std::abs(tmp[0])){
                    // make x axis bigger by turning 90 degrees
                    double value = tmp[0];
                    tmp[0] = tmp[1];
                    tmp[1] = -value;
                }
                if(tmp[0] < 0){
                    tmp[0] *= -1;
                    tmp[1] *= -1;
                }
                x.push_back(tmp[0]);
                y.push_back(tmp[1]);
                std::cout << "x: " << x.back() << "  y: " << y.back() << std::endl;
            }
        }
        // median of vector
        auto median_func = [](std::vector<double> x)->double{
            std::sort(x.begin(), x.end());
            if(x.size()%2 == 0)
                return (x[x.size()/2 - 1] + x[x.size()/2]) / 2.0;
            // else return 0.0;
            return x[x.size()/2];
        };

        double median1 = median_func(x);
        std::cout << "median; " << median1 << std::endl;



        return tmp;
    }


    // Euclidean Cluster Extraction from PCL.
    // Inputs:
    //  cloud, single PointCloud that we wan't to extract clusters from.
    //  tolerance, maximum distance between points so that they belong to same cluster.
    //  min_size, minimum number of points in a cluster.
    //  max_size, maximum number of points in a cluster.
    // Output:
    //  outvec, vector containing PointClouds where each PointCloud represents a single cluster.
    template<typename PT>
    void ecClustering(const typename pcl::PointCloud<PT>::Ptr cloud, const float tolerance, const float min_size, const float max_size,
        std::vector<typename pcl::PointCloud<PT>::Ptr> &outvec){

        typename pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
        tree->setInputCloud(cloud);

    	std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PT> ec;
        ec.setClusterTolerance(tolerance); // 2cm
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            PT p;
            typename pcl::PointCloud<PT>::Ptr tmp_cloud (new pcl::PointCloud<PT>());
            tmp_cloud->points.reserve(it->indices.size());
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                p = cloud->points[*pit];
                tmp_cloud->points.push_back(p);
            }
            tmp_cloud->width = tmp_cloud->points.size ();
            tmp_cloud->height = 1;
            tmp_cloud->is_dense = true;
            outvec.push_back(tmp_cloud);
        }
    };



    void generateRandomColor(int mix_red, int mix_green, int mix_blue,
                                     int &red, int &green, int& blue) {

        red = ((rand() % 255) + mix_red) / 2;
        green = ((rand() % 255) + mix_green) / 2;
        blue = ((rand() % 255) + mix_blue) / 2;
    }


    void planeSegmentation( const PointCloudT::Ptr cloud_in,
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

            if(ind.size() < 500) continue;

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


    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        PointCloudT::Ptr cloud (new PointCloudT ());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        // testCompression(cloud);
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

    TestSurfel test;

    ros::Rate loop_rate(loadParam<int>("HZ", test.nh ));
    // while(ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    //
    // test.testCompression();
    test.testSurfel();

    return 0;
}
