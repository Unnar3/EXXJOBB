
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <Refinement/SurfaceRefinement.h>
#include <planeDetectionComparison/ppr.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> PointCloudN;
typedef pcl::PointXYZRGBA PointTA;
typedef pcl::PointCloud<PointTA> PointCloudTA;
typedef pcl::ModelCoefficients ModelCoeffT;
typedef std::vector<PointCloudT::Ptr> vPointCloudT;
typedef std::vector<PointCloudTA::Ptr> vPointCloudTA;

namespace planeDetection{

    void planeSegmentationPPR(  const PointCloudT::Ptr          cloud,
                                const PointCloudN::Ptr          normals,
                                std::vector<PointCloudT::Ptr>   &planes,
                                std::vector<Eigen::Vector4d>    &coeffs,
                                PointCloudT::Ptr                nonPlanar)
    {
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        // pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        PointCloudT::Ptr cloud_in (new PointCloudT ());
        PointCloudT::Ptr cloud_f (new PointCloudT ());

        PointCloudN::Ptr normal_in (new PointCloudN());
        PointCloudN::Ptr cloud_n (new PointCloudN());

        *cloud_in = *cloud;
        *normal_in = *normals;

        // Eigen::Vector3f ax;
        // ax << 1.0,1.0,0;

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.1);
        seg.setEpsAngle (0.5);
        seg.setNormalDistanceWeight (0.1);
        // seg.setAxis(ax);

        int i=0, nr_points = cloud->points.size ();
        while (i < 100 && cloud->points.size() > 0 && cloud->points.size() > 0.1 * nr_points)
        {
            // Define for each plane we find
            ModelCoeffT::Ptr coefficients (new ModelCoeffT);
            PointCloudT::Ptr cloud_plane (new PointCloudT ());

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_in);
            seg.setInputNormals(normal_in);
            seg.segment (*inliers, *coefficients);


            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            if(inliers->indices.size() < 3000){
                i++;
                break;
            }
            inliers = segmentPPR(cloud_in, coefficients);

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud (cloud_in);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_plane);

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud_in.swap (cloud_f);

            pcl::ExtractIndices<pcl::Normal> extractN;
            extractN.setInputCloud(normal_in);
            extractN.setIndices (inliers);
            extractN.setNegative (true);
            extractN.filter(*cloud_n);
            normal_in.swap(cloud_n);

            planes.push_back(cloud_plane);
            Eigen::Vector4d eq;
            eq[0] = coefficients->values[0];
            eq[1] = coefficients->values[1];
            eq[2] = coefficients->values[2];
            eq[3] = coefficients->values[3];
            coeffs.push_back(eq);
            i++;
        }
        *nonPlanar = *cloud_in;
    }

}
