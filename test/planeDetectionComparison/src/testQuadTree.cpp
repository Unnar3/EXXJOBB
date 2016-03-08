#include <ros/ros.h>
#include <quadtree/quadtree.h>
#include <quadtree/quadtreePCL.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <exx_compression/compression.h>
#include <utils/utils.cpp>
#include <pcl/TextureMesh.h>
#include <planeDetectionComparison/utils.h>
#include <random>
#include <sensor_msgs/PointField.h>
#include <quadtree/saveOBJFile.h>
#include <locale>
#include <planeDetectionComparison/utils.h>
// #include <pcl/pcl_config.h>

// typedef pcl::PointXYZ PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGB PointTC;
typedef pcl::PointCloud<PointTC> PointCloudTC;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;


using namespace EXX::params;
using namespace EXX;
void removeIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr indices, bool inside){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (indices);
    extract.setNegative (!inside);
    extract.filter (*out);
    *cloud=*out;
}


  // Build a passthrough filter to reduce field of view.
void removePart(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, bool inside)
{

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices());
    for (size_t i = 0; i < cloud->points.size(); i++) {
        if(cloud->points[i].z > zmin && cloud->points[i].z < zmax){
          if(cloud->points[i].x > xmin && cloud->points[i].x < xmax){
            if(cloud->points[i].y > ymin && cloud->points[i].y < ymax){
              indices->indices.push_back(i);
            //   std::cout << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << std::endl;
            }
          }
        }
    }
    removeIndices(cloud,indices,inside);
    // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // extract.setInputCloud (cloud);
    // extract.setIndices (indices);
    // extract.setNegative (!inside);
    // extract.filter (*out);
    // *cloud=*out;

    // Remove the planar inliers, extract the rest
}

// TEST(QuadTree, typeHasColor) {
//     QuadTree<pcl::PointXYZRGB> quad(1, 100, 0, 2);
//     quad.useColor(false);
//     ASSERT_EQ(false, quad.useColor()) << "Color exists, color not set, still used.";
//
//     quad.useColor(true);
//     ASSERT_EQ(true, quad.useColor()) << "Color exists, use color still not set.";
// }
//
// TEST(QuadTree, typeHasNoColor) {
//     QuadTree<pcl::PointXYZ> quad(1,100, 0,2);
//
//     quad.useColor(false);
//     ASSERT_EQ(false, quad.useColor()) << "Color doesn't exist, color not set.";
//
//     quad.useColor(true);
//     ASSERT_EQ(false, quad.useColor()) << "Color doesn't exist, color set.";
// }
//
// TEST(QuadTree, testSetterGetterMethods){
//     QuadTree<pcl::PointXYZ> quad(1,100, 0,2);
//     EXPECT_EQ(1, quad.getLevel()) << "Level initialized wrong";
//     EXPECT_EQ(100, quad.getWidth()) << "Width initialized wrong";
//     EXPECT_EQ(-1, quad.getMaxLevel()) << "Max level initialized wrong";
//     EXPECT_EQ(-1, quad.getMaxWidth()) << "Max Width initialized wrong";
//     EXPECT_EQ(true, quad.isLeaf()) << "is Leaf initialized wrong";
//     EXPECT_EQ(false, quad.useColor()) << "Use Color Initialized wrong";
//
//     // change max level and max width.
//     quad.setMaxLevel(7);
//     quad.setMaxWidth(1.0);
//     quad.isLeaf(false);
//     EXPECT_EQ(7, quad.getMaxLevel()) << "Max level changed wrong";
//     EXPECT_EQ(1.0, quad.getMaxWidth()) << "Max Width changed wrong";
//     EXPECT_EQ(false, quad.isLeaf()) << "is leaf changed wrong";
// }
//
// TEST(QuadTree, testInsertBoundary){
//
//     std::cout << "   " << std::endl;
//     std::cout << "   " << std::endl;
//     std::cout << "   " << std::endl;
//     std::cout << "   " << std::endl;
//     std::cout << "testInsertBoundary" << std::endl;
//
//     QuadTree<pcl::PointXYZ> quad(1,100, 0,0);
//     quad.setMaxLevel(5);
//     quad.setMaxWidth(0.1);
//
//     pcl::PointXYZ p;
//
//     p.x = 10; p.y = 5; p.z = 1;
//     EXPECT_EQ(1,quad.insert(p, false)) << "create new level";
//
//     std::vector<int> idx(0);
//
//     p.x = 60; p.y = 40; p.z = 1;
//     EXPECT_EQ(2,quad.insert(p, false)) << "create new level";
//     //
//     p.x = 30; p.y = 80; p.z = 1;
//     EXPECT_EQ(3,quad.insert(p, false)) << "create new level";
//     //
//     p.x = 70; p.y = 80; p.z = 1;
//     EXPECT_EQ(4,quad.insert(p, false)) << "create new level";
//
//     // quad.printTree(idx);
//
//     EXPECT_EQ(5,quad.getTreeDepth()) << "tree depth calculation wrong";
//     quad.clear();
//     EXPECT_EQ(1,quad.getTreeDepth()) << "clear method wrong";
//
//     p.x = 120; p.y = 80; p.z = 1;
//     EXPECT_EQ(5,quad.insert(p, false)) << "create new level";
// }
//
// TEST(QuadTree, decemation){
//     QuadTree<pcl::PointXYZRGB> quad(1,10, 0,0);
//     quad.setMaxWidth(0.1);
//     // quad.setMaxLevel(5);
//
//     srand(time(0));
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr original (new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::PointXYZRGB p;
//     p.z = 0;
//     for (size_t i = 0; i < 1000; i++) {
//         p.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/4.9)) + 0.1;
//         p.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0));
//         p.r = 255; p.g = 255; p.b = 255;
//         quad.insert(p, false);
//         original->push_back(p);
//     }
//
//     for (size_t i = 0; i < 1000; i++) {
//         p.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0)) + 5.0;
//         p.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0));
//         if(p.x < 7.5 && p.y > 2.5){
//             p.r = 255; p.g = 255; p.b = 0;
//         } else {
//             p.r = 255; p.g = 255; p.b = 255;
//         }
//         quad.insert(p, false);
//         original->push_back(p);
//     }
//
//     for (size_t i = 0; i < 1000; i++) {
//         p.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0)) + 5.0;
//         p.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0)) + 5.0;
//         p.r = 255; p.g = 255; p.b = 255;
//         quad.insert(p, false);
//         original->push_back(p);
//     }
//
//     for (size_t i = 0; i < 50; i++) {
//         p.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.05));
//         p.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0));
//         p.r = 255; p.g = 0; p.b = 255;
//         quad.insert(p, true);
//         original->push_back(p);
//     }
//
//     p.x = 4.0;
//     p.y = 6.0;
//     p.r = 255; p.g = 0; p.b = 255;
//     quad.insert(p, true);
//     // p.x = 0.0;
//     // p.y = 0.0;
//     // quad.insert(p, true);
//     // p.x = 0.05;
//     // p.y = 1.0;
//     // quad.insert(p, true);
//     std::cout << "tree Depth: " << quad.getTreeDepth() << std::endl;
//     quad.decemate();
//     std::cout << "tree Depth: " << quad.getTreeDepth() << std::endl;
//
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
//     std::vector< pcl::Vertices > vertices;
//     quad.createPointCloud(cloud, vertices);
//
//     pcl::PCDWriter writer;
//     std::string path = "/home/unnar/Desktop/";
//     writer.write(path + "decemated.pcd", *cloud);
//     writer.write(path + "decemated_original.pcd", *original);
//
//     std::cout << "size: " << cloud->points.size() << std::endl;
//
//     // pcl::visualization::PCLVisualizer::Ptr viewer;
//     // viewer.reset(new pcl::visualization::PCLVisualizer);
//     // viewer->addPolygonMesh<pcl::PointXYZRGB>(cloud, vertices);
//     // viewer->spin();
//
// }

// TEST(QuadTree, insertBoundary){
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     cloud->resize(10);
//     int add = 1;
//
//     cloud->points[0].x = 1;
//     cloud->points[0].y = 2;
//     cloud->points[1].x = 3;
//     cloud->points[1].y = 1;
//     cloud->points[2].x = 5;
//     cloud->points[2].y = 2;
//     cloud->points[3].x = 8;
//     cloud->points[3].y = 1;
//     cloud->points[4].x = 9;
//     cloud->points[4].y = 4;
//     cloud->points[5].x = 7;
//     cloud->points[5].y = 8;
//     cloud->points[6].x = 7;
//     cloud->points[6].y = 9;
//     cloud->points[7].x = 3;
//     cloud->points[7].y = 9;
//     cloud->points[8].x = 1;
//     cloud->points[8].y = 7;
//     cloud->points[9].x = 2;
//     cloud->points[9].y = 3;
//
//     QuadTree<pcl::PointXYZ> quad(1,10, 0,0);
//     quad.setMaxWidth(0.05);
//
//     EXPECT_EQ(true, quad.insertBoundary(cloud));
//
//     Polygon poly;
//     poly.push_back(Point(2,4));
//     poly.push_back(Point(3,5));
//     poly.push_back(Point(4,8));
//     poly.push_back(Point(3,7));
//     quad.insertHole(poly);
//
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>());
//     std::vector< pcl::Vertices > vertices;
//     quad.createPointCloudBoundary(cloud_out, vertices);
//
//
//     pcl::PointXYZRGB p1, p2;
//     pcl::getMinMax3D (*cloud_out, p1, p2);
//
//     std::cout << "x: " << p1.x << ", " << p2.x << "  y: " << p1.y << ", " << p2.y << std::endl;
//
//     pcl::PCDWriter writer;
//     std::string path = "/home/unnar/Desktop/";
//     writer.write(path + "decemated_boundary.pcd", *cloud_out);
//
//     pcl::visualization::PCLVisualizer::Ptr viewer;
//     viewer.reset(new pcl::visualization::PCLVisualizer);
//     viewer->addPolygonMesh<pcl::PointXYZRGB>(cloud_out, vertices);
//     viewer->spin();
//
// }

// TEST(QuadTreePCL, testCLass){
//     Eigen::Vector3f normal;
//     normal << 0,-1,0;
//     Eigen::Vector3f result;
//
//     QuadTreePCL<PointTC> qtpcl(1,10,0,0);
//     // qtpcl.setMaxLevel(10);
//     qtpcl.setMaxWidth(0.1);
//     qtpcl.setNormal(normal);
//     // result = qtpcl.getNormal();
//     // EXPECT_EQ(true, result == normal);
//
//     PointTC p;
//     p.y = 0; p.r = 255; p.g = 255; p.b = 255;
//     PointCloudTC::Ptr cloud (new PointCloudTC());
//     p.x = 0; p.z = 2;
//     cloud->points.push_back(p);
//     p.x = 0; p.z = 0;
//     cloud->points.push_back(p);
//     p.x = 7; p.z = 0;
//     cloud->points.push_back(p);
//     p.x = 7; p.z = 6;
//     cloud->points.push_back(p);
//     p.x = 0; p.z = 6;
//     cloud->points.push_back(p);
//     p.x = 0; p.z = 3;
//     cloud->points.push_back(p);
//
//     p.x = 2; p.z = 2;
//     cloud->points.push_back(p);
//     p.x = 3; p.z = 1;
//     cloud->points.push_back(p);
//     p.x = 5; p.z = 3;
//     cloud->points.push_back(p);
//     p.x = 4; p.z = 4;
//     cloud->points.push_back(p);
//     p.x = 2; p.z = 3;
//     cloud->points.push_back(p);
//
//     // PointCloudTC::Ptr tmp (new PointCloudTC());
//     // *tmp += *cloud;
//     qtpcl.insertBoundary(cloud);
//
//     PointCloudTC::Ptr out (new PointCloudTC());
//     std::vector< pcl::Vertices > vertices;
//     qtpcl.createMesh<PointTC>(out, vertices);
//
//     pcl::Vertices vert;
//     vert.vertices.resize(3);
//     int size = out->size();
//     vert.vertices[0] = size;
//     vert.vertices[1] = size+1;
//     vert.vertices[2] = size+2;
//     vertices.push_back(vert);
//     vert.vertices[0] = size;
//     vert.vertices[1] = size+3;
//     vert.vertices[2] = size+2;
//     vertices.push_back(vert);
//
//     // *out += *tmp;
//
//
//     // pcl::visualization::PCLVisualizer::Ptr viewer;
//     // viewer.reset(new pcl::visualization::PCLVisualizer);
//     // viewer->addPolygonMesh<pcl::PointXYZRGB>(out, vertices);
//     // // viewer->addPointCloud<pcl::PointXYZRGB>(out);
//     // viewer->spin();
// }

TEST(QuadTreePCL, testCloud){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_wall1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_wall1->width  = 40000;
    cloud_wall1->height = 1;
    cloud_wall1->points.resize (cloud_wall1->width * cloud_wall1->height);

    double tableDistFromWall = 0.05;
    double variance = 0.02;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_wall1(0, 5);
    std::uniform_real_distribution<> disz_wall1(0, 3);
    std::normal_distribution<> d_wall1(0,variance);
    for (size_t i = 0; i < cloud_wall1->points.size (); ++i){
        cloud_wall1->points[i].x = dis_wall1(gen);
        cloud_wall1->points[i].z = disz_wall1(gen);
        cloud_wall1->points[i].y = 0;
        cloud_wall1->points[i].r = 50*cloud_wall1->points[i].x;
        cloud_wall1->points[i].g = 50*cloud_wall1->points[i].x;
        cloud_wall1->points[i].b = 0;
    }

    removePart(cloud_wall1, 0, 0.7,-100, 100,  0, 1.0, false);
    removePart(cloud_wall1, 1.5, 3.5,-100, 100,  0.8, 2.7, false);

    EXX::compression cmprs;
    cmprs.setRWHullMaxDist(0.02);
    cmprs.setHULLAlpha(0.07);

    std::vector<PointCloudT::Ptr> plane_vec;
    plane_vec.push_back(cloud_wall1);
    std::vector<PointCloudT::Ptr> hulls;
    cmprs.planeToConcaveHull(&plane_vec, &hulls);
    std::vector<EXX::densityDescriptor> dDesc;
    EXX::densityDescriptor dens;
    dens.rw_max_dist = 0.3;
    dDesc.push_back(dens);
    std::vector<PointCloudT::Ptr> simplified_hulls;
    cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);

    QuadTreePCL<PointTC> qtpcl(1,10,0,0);
    // qtpcl.setMaxLevel(10);
    qtpcl.setMaxWidth(1);
    Eigen::Vector3f normal;
    normal << 0,1,0;
    qtpcl.setNormal(normal);

    qtpcl.insertBoundary(simplified_hulls[0]);
    PointCloudTC::Ptr out (new PointCloudTC());

    std::vector< pcl::Vertices > vertices;
    qtpcl.createMesh<PointTC>(out, vertices);
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
    coeff->values.resize(3);
    coeff->values[0] = normal[0]; coeff->values[1] = normal[1]; coeff->values[2] = normal[2];


    cv::Mat image;
    std::vector<Eigen::Vector2f> vertex_texture;
    qtpcl.createTexture<PointTC>(cloud_wall1, out, image, vertex_texture);

    objStruct<PointTC> object(1);
    object.clouds.push_back(out);
    object.polygons.push_back(vertices);
    object.images.push_back(image);
    object.texture_vertices.push_back(vertex_texture);
    object.coefficients.push_back(coeff);

    std::string path = "/home/unnar/Desktop/";
    cv::imwrite(path + "texture.png", image);
    std::vector<std::vector<pcl::Vertices> > polygons;
    polygons.push_back(vertices);
    std::vector<std::vector<Eigen::Vector2f>> texture_vertices;
    texture_vertices.push_back(vertex_texture);
    std::vector<pcl::ModelCoefficients::Ptr> coeffs;
    coeffs.push_back(coeff);
    std::vector<pcl::TexMaterial> materials;
    pcl::TexMaterial tmat;
    tmat.tex_Ka.r = 0.2f;
    tmat.tex_Ka.g = 0.2f;
    tmat.tex_Ka.b = 0.2f;

    tmat.tex_Kd.r = 0.8f;
    tmat.tex_Kd.g = 0.8f;
    tmat.tex_Kd.b = 0.8f;

    tmat.tex_Ks.r = 1.0f;
    tmat.tex_Ks.g = 1.0f;
    tmat.tex_Ks.b = 1.0f;
    tmat.tex_d = 1.0f;
    tmat.tex_Ns = 1.0f;
    tmat.tex_illum = 2;

    tmat.tex_name = "texture";
    tmat.tex_file = path + "texture.png";
    materials.push_back(tmat);
    saveOBJFile<PointTC>(path + "texture.obj", out, polygons, texture_vertices, coeffs, materials);


    // pcl::TextureMesh tmesh;
    // tmesh.tex_polygons = polygons;
    // tmesh.tex_coordinates = texture_vertices;
    // tmesh.tex_materials = materials;
    // planeDetection::toMeshCloud(*out, tmesh.cloud);



    // pcl::visualization::PCLVisualizer::Ptr viewer;
    // viewer.reset(new pcl::visualization::PCLVisualizer);
    // viewer->addTextureMesh(tmesh);
    // viewer->addPointCloud<pcl::PointXYZRGB>(out);
    // viewer->spin();

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "testQuadTree");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ros::Rate loop_rate(10);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}
