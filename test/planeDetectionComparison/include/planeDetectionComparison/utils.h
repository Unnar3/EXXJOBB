#ifndef PLANE_SEGMENTATION_UTILS_H
#define PLANE_SEGMENTATION_UTILS_H

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/for_each_type.h>
#include <pcl/point_traits.h>

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


    // Euclidean Cluster Extraction from PCL.
    // Inputs:
    //  cloud, single PointCloud that we wan't to extract clusters from.
    //  tolerance, maximum distance between points so that they belong to same cluster.
    //  min_size, minimum number of points in a cluster.
    //  max_size, maximum number of points in a cluster.
    // Output:
    //  outvec, vector containing PointClouds where each PointCloud represents a single cluster.
    template<typename PT>
    void ecClustering(const typename pcl::PointCloud<PT>::Ptr cloud, const float tolerance, const int min_size, const int max_size,
        std::vector<typename pcl::PointCloud<PT>::Ptr> &outvec,
        std::vector<pcl::PointIndices> &cluster_indices){

        typename pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
        tree->setInputCloud(cloud);

        // std::vector<pcl::PointIndices> cluster_indices;
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

    void extractIndices(PointCloudT::Ptr                cloud,
                        std::vector<PointCloudT::Ptr>   &planes,
                        pcl::PointIndices::Ptr          inliers){

            //
            PointCloudT::Ptr cloud_tmp (new PointCloudT());
            PointCloudT::Ptr cloud_tmpt (new PointCloudT());
            PointCloudT::Ptr plane (new PointCloudT());
            PointCloudT::Ptr planet (new PointCloudT());
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*plane);

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_tmp);
            (*cloud).swap (*cloud_tmp);

            std::vector<pcl::PointIndices> cluster_indices;
            ecClustering<PointT>(plane, 0.1, 400, 1000000, planes, cluster_indices);

            std::cout << "planes size: " << planes.size() << std::endl;
            *planet = *plane;
            pcl::PointIndices::Ptr tmp (new pcl::PointIndices());
            for(auto ind : cluster_indices){

                tmp->indices.reserve(tmp->indices.size() + ind.indices.size());
                tmp->indices.insert(tmp->indices.end(), ind.indices.begin(), ind.indices.end());
            }

            extract.setInputCloud (planet);
            extract.setIndices (tmp);
            extract.setNegative (true);
            extract.filter (*cloud_tmpt);
            *cloud += *cloud_tmpt;

        }


        template<typename PointT> void
        toMeshCloud (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
        {
            // Ease the user's burden on specifying width/height for unorganized datasets
            if (cloud.width == 0 && cloud.height == 0)
            {
              msg.width  = static_cast<uint32_t>(cloud.points.size ());
              msg.height = 1;
            }
            else
            {
              assert (cloud.points.size () == cloud.width * cloud.height);
              msg.height = cloud.height;
              msg.width  = cloud.width;
            }

            // Fill point cloud binary data (padding and all)
            size_t data_size = sizeof (PointT) * cloud.points.size ();
            msg.data.resize (data_size);
            memcpy (&msg.data[0], &cloud.points[0], data_size);

            // Fill fields metadata
            msg.fields.clear ();
            pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::detail::FieldAdder<PointT>(msg.fields));

            msg.header     = cloud.header;
            msg.point_step = sizeof (PointT);
            msg.row_step   = static_cast<uint32_t> (sizeof (PointT) * msg.width);
            msg.is_dense   = cloud.is_dense;
            /// @todo msg.is_bigendian = ?;
        }
}


#endif
