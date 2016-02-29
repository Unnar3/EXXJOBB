#ifndef PLANEEXTRACTION_H
#define PLANEEXTRACTION_H


// #include <planeDetectionComparison/utils.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common.h>

#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <Eigen/Dense>

namespace EXX{

    class planeExtraction{

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef pcl::Normal PointN;
        typedef pcl::PointCloud<PointN> PointCloudN;
        typedef pcl::ModelCoefficients ModelCoeffT;

    public:
        planeExtraction(){}
        ~planeExtraction(){}


        void planeSegmentationEfficientPPRSinglePlanes( const PointCloudT::Ptr              cloud,
                                                        PointCloudN::Ptr                    normals,
                                                        const primitive_params              params,
                                                        std::vector<PointCloudT::Ptr>     & plane_vec,
                                                        std::vector<ModelCoeffT::Ptr>     & coeff_vec,
                                                        PointCloudT::Ptr                    nonPlanar);

    private:

        void estimateNormals(const PointCloudT::Ptr cloud, PointCloudN::Ptr normals, float rad);


        bool extractPlaneEfficientRANSAC(   const PointCloudT::Ptr  cloud_in,
                                            const PointCloudN::Ptr  normals,
                                            const primitive_params  params,
                                            pcl::PointIndices::Ptr  indices,
                                            ModelCoeffT::Ptr        coeff);



        void runPPRSinglePlane( PointCloudT::Ptr        nonPlanar,
                                PointCloudN::Ptr        normals,
                                ModelCoeffT::Ptr        coeff,
                                PointCloudT::Ptr        plane);


        pcl::PointIndices::Ptr segmentPPR(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud,
                                            pcl::ModelCoefficients::Ptr             coefficients,
                                            float                                   max_dist);


        // Euclidean Cluster Extraction from PCL.
        // Inputs:
        //  cloud, single PointCloud that we wan't to extract clusters from.
        //  tolerance, maximum distance between points so that they belong to same cluster.
        //  min_size, minimum number of points in a cluster.
        //  max_size, maximum number of points in a cluster.
        // Output:
        //  outvec, vector containing PointClouds where each PointCloud represents a single cluster.
        template<typename PT>
        void ecClustering(  const typename pcl::PointCloud<PT>::Ptr         cloud,
                            const float                                     tolerance,
                            const int                                       min_size,
                            const int                                       max_size,
                            std::vector<typename pcl::PointCloud<PT>::Ptr>  &outvec,
                            std::vector<pcl::PointIndices>                  &cluster_indices);


        template <typename T>
        void extractIndicesAndRemoveFromOriginal(
                typename pcl::PointCloud<T>::Ptr    cloud,
                typename pcl::PointCloud<T>::Ptr    extracted,
                pcl::PointIndices::Ptr              inliers);

        template <typename T>
        void extractIndicesFromOriginal(
                typename pcl::PointCloud<T>::Ptr    cloud,
                typename pcl::PointCloud<T>::Ptr    extracted,
                pcl::PointIndices::Ptr              inliers);

        void extractIndices(
                PointCloudT::Ptr        cloud,
                PointCloudN::Ptr        normals,
                PointCloudT::Ptr        plane,
                pcl::PointIndices::Ptr  inliers);

    };
}

#include "impl/plane_extraction.hpp"
#endif
