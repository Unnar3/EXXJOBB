
#include <planeDetectionComparison/utils.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common.h>

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

    void estimateNormals(const PointCloudT::Ptr cloud, PointCloudN::Ptr normals, float rad){

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        typedef pcl::search::KdTree<PointT> kd_tree_type;
        typedef typename kd_tree_type::Ptr kd_tree_type_ptr;
        kd_tree_type_ptr tree(new kd_tree_type());
        //pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr ptr(octree);
        ne.setSearchMethod(tree);

        // Use all neighbors in a sphere of radius normal_radius m
        ne.setRadiusSearch(rad);


        PointT min,max;
        pcl::getMinMax3D(*cloud,min,max);

        ne.setViewPoint (min.x - 5.0, min.y - 5.0, min.z - 5.0);
        // Compute the features
        ne.compute(*normals);

    }


    bool extractPlaneEfficientRANSAC(   const PointCloudT::Ptr  cloud_in,
                                        const PointCloudN::Ptr  normals,
                                        const primitive_params  params,
                                        pcl::PointIndices::Ptr  indices,
                                        ModelCoeffT::Ptr        coeff){


        // RANSAC
        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(cloud_in, normals, primitives, params, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract_single_param(true);
        extractor.extract(extracted);

        if( extracted.size() > 0){
            indices->indices = extracted.at(0)->supporting_inds;

            Eigen::VectorXd data;
            extracted.at(0)->shape_data(data);
            coeff->values.resize(4);
            coeff->values[0] = data[0];
            coeff->values[1] = data[1];
            coeff->values[2] = data[2];
            coeff->values[3] = data[3];
            return true;
        }
        return false;

    }

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
        // extractor.extract_single_param(true);
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
        // extractor.extract_single_param(true);
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


    void runPPRSinglePlane( PointCloudT::Ptr        nonPlanar,
                            PointCloudN::Ptr        normals,
                            ModelCoeffT::Ptr        coeff,
                            PointCloudT::Ptr        plane){



        PointCloudT::Ptr cloud_tmp (new PointCloudT());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers = segmentPPR(nonPlanar, coeff, 0.02);

        // PointCloudT::Ptr plane (new PointCloudT);
        extractIndices(nonPlanar, normals, plane, inliers);

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
            inliers = segmentPPR(nonPlanar, coeff, 0.1);

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

    void planeSegmentationEfficientPPRSinglePlanes( const PointCloudT::Ptr              cloud,
                                                    PointCloudN::Ptr                    normals,
                                                    const primitive_params              params,
                                                    std::vector<PointCloudT::Ptr>     & plane_vec,
                                                    std::vector<ModelCoeffT::Ptr>     & coeff_vec,
                                                    PointCloudT::Ptr                    nonPlanar){

        *nonPlanar = *cloud;

        if(cloud->points.size() != normals->points.size()){
            std::cout << "Empty normal cloud. Need to estimate the normals !!!" << std::endl;
            estimateNormals(cloud, normals, params.normal_neigbourhood);
        }

        while(1){

            pcl::PointIndices::Ptr indices (new pcl::PointIndices());
            ModelCoeffT::Ptr coeff (new ModelCoeffT());
            if(extractPlaneEfficientRANSAC(nonPlanar, normals, params, indices, coeff)){

                PointCloudT::Ptr plane (new PointCloudT());

                runPPRSinglePlane(nonPlanar, normals, coeff, plane);
                if(plane->points.size() > 500){
                    plane_vec.push_back(plane);
                    coeff_vec.push_back(coeff);
                    continue;
                }
            }
            break;
        }

    }




}
