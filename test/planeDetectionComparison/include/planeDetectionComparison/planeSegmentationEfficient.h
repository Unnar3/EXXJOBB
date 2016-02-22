
#include <planeDetectionComparison/utils.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <Eigen/Dense>

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
    void planeSegmentationEfficientPlanes(const PointCloudT::Ptr cloud_in,
                            const PointCloudN::Ptr          normals,
                            const primitive_params          params,
                            std::vector<pcl::PointIndices::Ptr> &indicesv,
                            std::vector<ModelCoeffT::Ptr>   &coeffv
                            ){
        // Perform the compression
        // RANSAC
        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(cloud_in, normals, primitives, params, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract(extracted);

        std::vector<int> ind;
        coeffv.reserve(extracted.size());
        indicesv.reserve(extracted.size());
        pcl::ExtractIndices<PointT> extract;
        Eigen::VectorXd data;
        for (size_t j = 0; j < extracted.size(); ++j){
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            inliers->indices = extracted[j]->supporting_inds;
            indicesv.push_back(inliers);

            extracted.at(j)->shape_data(data);
            ModelCoeffT::Ptr coefficients (new ModelCoeffT);
            coefficients->values.resize(4);
            coefficients->values[0] = data[0];
            coefficients->values[1] = data[1];
            coefficients->values[2] = data[2];
            coefficients->values[3] = data[3];
            coeffv.push_back(coefficients);
        }

    }

    void planeSegmentationEfficientPlanes(const PointCloudT::Ptr cloud_in,
                            const primitive_params          params,
                            std::vector<pcl::PointIndices::Ptr> &indicesv,
                            std::vector<ModelCoeffT::Ptr>   &coeffv
                            ){
        // Perform the compression
        // RANSAC
        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(cloud_in, primitives, params, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract(extracted);

        std::vector<int> ind;
        coeffv.reserve(extracted.size());
        indicesv.reserve(extracted.size());
        pcl::ExtractIndices<PointT> extract;
        Eigen::VectorXd data;
        for (size_t j = 0; j < extracted.size(); ++j){
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            inliers->indices = extracted[j]->supporting_inds;
            indicesv.push_back(inliers);

            extracted.at(j)->shape_data(data);
            ModelCoeffT::Ptr coefficients (new ModelCoeffT);
            coefficients->values.resize(4);
            coefficients->values[0] = data[0];
            coefficients->values[1] = data[1];
            coefficients->values[2] = data[2];
            coefficients->values[3] = data[3];
            coeffv.push_back(coefficients);
        }

    }


    void runPPR(const PointCloudT::Ptr          cloud_in,
                std::vector<pcl::PointIndices::Ptr> &indicesv,
                std::vector<ModelCoeffT::Ptr>   &coeffv,
                std::vector<PointCloudT::Ptr>   &plane_vec,
                std::vector<ModelCoeffT::Ptr>   &normal_vec,
                PointCloudT::Ptr                nonPlanar){


        if(plane_vec.size() > 0){
            plane_vec.clear();
            normal_vec.clear();
        }
        plane_vec.reserve(indicesv.size());
        normal_vec.reserve(indicesv.size());

        PointCloudT::Ptr cloud_tmp (new PointCloudT());
        *nonPlanar = *cloud_in;
        for(auto &coeff: coeffv){
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            inliers = segmentPPR(nonPlanar, coeff);

            std::vector<PointCloudT::Ptr> planes;
            // PointCloudT::Ptr cloud_plane (new PointCloudT());
            extractIndices(nonPlanar, planes, inliers);
            for (size_t i = 0; i < planes.size(); i++) {
                /* code */
                plane_vec.push_back(planes[i]);
                normal_vec.push_back(coeff);
            }
        }
    }




    void planeSegmentationEfficientPPR( const PointCloudT::Ptr              cloud,
                                        const PointCloudN::Ptr              normals,
                                        const primitive_params              params,
                                        std::vector<PointCloudT::Ptr>     & plane_vec,
                                        std::vector<ModelCoeffT::Ptr>     & coeff_vec,
                                        PointCloudT::Ptr                    nonPlanar){

        std::vector<pcl::PointIndices::Ptr> indicesv;
        std::vector<ModelCoeffT::Ptr>       coeffv;
        planeSegmentationEfficientPlanes(cloud, normals, params, indicesv, coeffv);
        runPPR(cloud, indicesv, coeffv, plane_vec, coeff_vec, nonPlanar);


    }

    void planeSegmentationEfficientPPR( const PointCloudT::Ptr              cloud,
                                        const primitive_params              params,
                                        std::vector<PointCloudT::Ptr>     & plane_vec,
                                        std::vector<ModelCoeffT::Ptr>     & coeff_vec,
                                        PointCloudT::Ptr                    nonPlanar){

        std::vector<pcl::PointIndices::Ptr> indicesv;
        std::vector<ModelCoeffT::Ptr>       coeffv;
        planeSegmentationEfficientPlanes(cloud, params, indicesv, coeffv);
        runPPR(cloud, indicesv, coeffv, plane_vec, coeff_vec, nonPlanar);

    }

}
