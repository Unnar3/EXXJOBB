#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <random>

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


// Build a passthrough filter to reduce field of view.
 void removePart(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, bool inside)
 {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices());
    for (size_t i = 0; i < cloud->points.size(); i++) {
        if(cloud->points[i].z > zmin && cloud->points[i].z < zmax){
          if(cloud->points[i].x > xmin && cloud->points[i].x < xmax){
            if(cloud->points[i].y > ymin && cloud->points[i].y < ymax){
              indices->indices.push_back(i);
              std::cout << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << std::endl;
            }
          }
        }
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (indices);
    extract.setNegative (!inside);
    extract.filter (*out);
    *cloud=*out;

    // Remove the planar inliers, extract the rest
 }


int main(int argc, char **argv) {

    // ros::init(argc, argv, "createTestCloud");
    // ros::Rate loop_rate(10);

    double tableDistFromWall = 0.03;
    double variance = 0.01;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_floor (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_wall1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_wall2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shelf1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shelf2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shelf3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_table (new pcl::PointCloud<pcl::PointXYZRGB>);

    // resize clouds
    cloud_floor->width  = 20000;
    cloud_floor->height = 1;
    cloud_floor->points.resize (cloud_floor->width * cloud_floor->height);

    cloud_wall1->width  = 10000;
    cloud_wall1->height = 1;
    cloud_wall1->points.resize (cloud_wall1->width * cloud_wall1->height);

    cloud_wall2->width  = 10000;
    cloud_wall2->height = 1;
    cloud_wall2->points.resize (cloud_wall2->width * cloud_wall2->height);

    cloud_shelf1->width  = 4000;
    cloud_shelf1->height = 1;
    cloud_shelf1->points.resize (cloud_shelf1->width * cloud_shelf1->height);

    cloud_shelf2->width  = 4000;
    cloud_shelf2->height = 1;
    cloud_shelf2->points.resize (cloud_shelf2->width * cloud_shelf2->height);

    cloud_shelf3->width  = 4000;
    cloud_shelf3->height = 1;
    cloud_shelf3->points.resize (cloud_shelf3->width * cloud_shelf3->height);

    cloud_table->width  = 2000;
    cloud_table->height = 1;
    cloud_table->points.resize (cloud_table->width * cloud_table->height);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 5);
    std::normal_distribution<> d(0,variance);

    for (size_t i = 0; i < cloud_floor->points.size (); ++i){
        cloud_floor->points[i].x = dis(gen);
        cloud_floor->points[i].y = dis(gen);
        cloud_floor->points[i].z = d(gen);
        cloud_floor->points[i].r = 255;
        cloud_floor->points[i].g = 255;
        cloud_floor->points[i].b = 255;
    }

    std::uniform_real_distribution<> dis_wall1(0, 5);
    std::uniform_real_distribution<> disz_wall1(0, 3);
    std::normal_distribution<> d_wall1(0,variance);
    for (size_t i = 0; i < cloud_wall1->points.size (); ++i){
        cloud_wall1->points[i].y = dis_wall1(gen);
        cloud_wall1->points[i].z = disz_wall1(gen);
        cloud_wall1->points[i].x = d_wall1(gen);
        cloud_wall1->points[i].r = 200;
        cloud_wall1->points[i].g = 200;
        cloud_wall1->points[i].b = 200;
    }

    for (size_t i = 0; i < cloud_wall2->points.size (); ++i){
        cloud_wall2->points[i].x = dis_wall1(gen);
        cloud_wall2->points[i].z = disz_wall1(gen);
        cloud_wall2->points[i].y = d_wall1(gen);
        cloud_wall2->points[i].r = 220;
        cloud_wall2->points[i].g = 220;
        cloud_wall2->points[i].b = 220;
    }

    std::uniform_real_distribution<> dis_shelf3(0, 0.7);
    std::normal_distribution<> d_shelf3(1.0,variance);
    for (size_t i = 0; i < cloud_shelf3->points.size(); ++i){
      cloud_shelf3->points[i].x = dis_shelf3(gen);
      cloud_shelf3->points[i].y = dis_shelf3(gen);
      cloud_shelf3->points[i].z = d_shelf3(gen);
      cloud_shelf3->points[i].r = 182+20;
      cloud_shelf3->points[i].g = 155+20;
      cloud_shelf3->points[i].b = 76+20;
    }

    std::uniform_real_distribution<> dis_shelf1(0, 0.7);
    std::uniform_real_distribution<> disz_shelf1(0, 1.0);
    std::normal_distribution<> d_shelf1(0.7,variance);
    for (size_t i = 0; i < cloud_shelf1->points.size (); ++i){
        cloud_shelf1->points[i].y = dis_shelf1(gen);
        cloud_shelf1->points[i].z = disz_shelf1(gen);
        cloud_shelf1->points[i].x = d_shelf1(gen);
        cloud_shelf1->points[i].r = 182;
        cloud_shelf1->points[i].g = 155;
        cloud_shelf1->points[i].b = 76;
    }

    for (size_t i = 0; i < cloud_shelf2->points.size(); ++i){
        cloud_shelf2->points[i].x = dis_shelf1(gen);
        cloud_shelf2->points[i].z = disz_shelf1(gen);
        cloud_shelf2->points[i].y = d_shelf1(gen);
        cloud_shelf2->points[i].r = 182+10;
        cloud_shelf2->points[i].g = 155+10;
        cloud_shelf2->points[i].b = 76+10;
    }

    std::uniform_real_distribution<> dis_table(1.5, 3.5);
    std::uniform_real_distribution<> disz_table(0.8, 2.7);
    std::normal_distribution<> d_table(tableDistFromWall,variance);
    for (size_t i = 0; i < cloud_table->points.size(); ++i){
        cloud_table->points[i].y = dis_table(gen);
        cloud_table->points[i].z = disz_table(gen);
        cloud_table->points[i].x = d_table(gen);
        cloud_table->points[i].r = 135;
        cloud_table->points[i].g = 206;
        cloud_table->points[i].b = 250;
        // cloud_table->points[i].r = 255;
        // cloud_table->points[i].g = 255;
        // cloud_table->points[i].b = 255;
    }

    removePart(cloud_floor, 0, 0.7, 0, 0.7, -100, 100, false);
    removePart(cloud_wall1, -100, 100, 0, 0.7, 0, 1.0, false);
    removePart(cloud_wall1, -100, 100, 1.5, 3.5, 0.8, 2.7, false);
    removePart(cloud_wall2, 0, 0.7, -100, 100, 0, 1.0, false);
    removePart(cloud_wall2, 1.5, 3.5, -100, 100, 1.2, 2.4, false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud = *cloud_floor + *cloud_wall1;
    *cloud += *cloud_wall2;
    *cloud += *cloud_shelf1;
    *cloud += *cloud_shelf2;
    *cloud += *cloud_shelf3;
    *cloud += *cloud_table;

    pcl::PCDWriter writer;
    std::string path = "/home/unnar/catkin_ws/src/EXXJOBB/test/planeDetectionComparison/src/";
    writer.write(path + "testCloud.pcd", *cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(cloud);

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


    return 0;

}
