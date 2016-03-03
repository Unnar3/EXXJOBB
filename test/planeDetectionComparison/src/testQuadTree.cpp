#include <ros/ros.h>
#include <quadtree/quadtree.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


TEST(QuadTree, typeHasColor) {
    QuadTree<pcl::PointXYZRGB> quad(1, 100, 0, 2);
    quad.useColor(false);
    ASSERT_EQ(false, quad.useColor()) << "Color exists, color not set, still used.";

    quad.useColor(true);
    ASSERT_EQ(true, quad.useColor()) << "Color exists, use color still not set.";
}

TEST(QuadTree, typeHasNoColor) {
    QuadTree<pcl::PointXYZ> quad(1,100, 0,2);

    quad.useColor(false);
    ASSERT_EQ(false, quad.useColor()) << "Color doesn't exist, color not set.";

    quad.useColor(true);
    ASSERT_EQ(false, quad.useColor()) << "Color doesn't exist, color set.";
}

TEST(QuadTree, testSetterGetterMethods){
    QuadTree<pcl::PointXYZ> quad(1,100, 0,2);
    EXPECT_EQ(1, quad.getLevel()) << "Level initialized wrong";
    EXPECT_EQ(100, quad.getWidth()) << "Width initialized wrong";
    EXPECT_EQ(-1, quad.getMaxLevel()) << "Max level initialized wrong";
    EXPECT_EQ(-1, quad.getMaxWidth()) << "Max Width initialized wrong";
    EXPECT_EQ(true, quad.isLeaf()) << "is Leaf initialized wrong";
    EXPECT_EQ(false, quad.useColor()) << "Use Color Initialized wrong";

    // change max level and max width.
    quad.setMaxLevel(7);
    quad.setMaxWidth(1.0);
    quad.isLeaf(false);
    EXPECT_EQ(7, quad.getMaxLevel()) << "Max level changed wrong";
    EXPECT_EQ(1.0, quad.getMaxWidth()) << "Max Width changed wrong";
    EXPECT_EQ(false, quad.isLeaf()) << "is leaf changed wrong";
}

TEST(QuadTree, testInsertBoundary){

    std::cout << "   " << std::endl;
    std::cout << "   " << std::endl;
    std::cout << "   " << std::endl;
    std::cout << "   " << std::endl;
    std::cout << "testInsertBoundary" << std::endl;

    QuadTree<pcl::PointXYZ> quad(1,100, 0,0);
    quad.setMaxLevel(5);
    quad.setMaxWidth(0.1);

    pcl::PointXYZ p;

    p.x = 10; p.y = 5; p.z = 1;
    EXPECT_EQ(1,quad.insert(p, false)) << "create new level";

    std::vector<int> idx(0);

    p.x = 60; p.y = 40; p.z = 1;
    EXPECT_EQ(2,quad.insert(p, false)) << "create new level";
    //
    p.x = 30; p.y = 80; p.z = 1;
    EXPECT_EQ(3,quad.insert(p, false)) << "create new level";
    //
    p.x = 70; p.y = 80; p.z = 1;
    EXPECT_EQ(4,quad.insert(p, false)) << "create new level";

    quad.printTree(idx);

    EXPECT_EQ(5,quad.getTreeDepth()) << "tree depth calculation wrong";
    quad.clear();
    EXPECT_EQ(1,quad.getTreeDepth()) << "clear method wrong";

    p.x = 120; p.y = 80; p.z = 1;
    EXPECT_EQ(5,quad.insert(p, false)) << "create new level";
}

TEST(QuadTree, decemation){
    QuadTree<pcl::PointXYZ> quad(1,10, 0,0);
    quad.setMaxWidth(1.0);
    // quad.setMaxLevel(5);

    srand(time(0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr original (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p;
    p.z = 0;
    for (size_t i = 0; i < 1000; i++) {
        p.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/4.9)) + 0.1;
        p.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0));
        quad.insert(p, false);
        original->push_back(p);
    }

    for (size_t i = 0; i < 1000; i++) {
        p.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0)) + 5.0;
        p.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0));
        quad.insert(p, false);
        original->push_back(p);
    }

    for (size_t i = 0; i < 1000; i++) {
        p.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0)) + 5.0;
        p.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5.0)) + 5.0;
        quad.insert(p, false);
        original->push_back(p);
    }

    p.x = 4.0;
    p.y = 6.0;
    quad.insert(p, true);
    p.x = 0.0;
    p.y = 0.0;
    quad.insert(p, true);
    p.x = 0.05;
    p.y = 1.0;
    quad.insert(p, true);
    std::cout << "tree Depth: " << quad.getTreeDepth() << std::endl;
    quad.decemate();
    std::cout << "tree Depth: " << quad.getTreeDepth() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    std::vector< pcl::Vertices > vertices;
    quad.createPointCloud(cloud, vertices);

    pcl::PCDWriter writer;
    std::string path = "/home/unnar/Desktop/";
    writer.write(path + "decemated.pcd", *cloud);
    writer.write(path + "decemated_original.pcd", *original);

    std::cout << "size: " << cloud->points.size() << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer);
    viewer->addPolygonMesh<pcl::PointXYZRGB>(cloud, vertices);
    viewer->spin();

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "testQuadTree");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ros::Rate loop_rate(10);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}
