#include <plane_extraction/plane_extraction.h>
#include <Refinement/SurfaceRefinement.h>

namespace EXX{

    void planeExtraction::planeSegmentationEfficientPPRSinglePlanes(
                const PointCloudT::Ptr              cloud,
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


    void planeExtraction::runPPRSinglePlane(
            PointCloudT::Ptr    nonPlanar,
            PointCloudN::Ptr    normals,
            ModelCoeffT::Ptr    coeff,
            PointCloudT::Ptr    plane){



        PointCloudT::Ptr cloud_tmp (new PointCloudT());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers = segmentPPR(nonPlanar, coeff, 0.02);

        // PointCloudT::Ptr plane (new PointCloudT);
        extractIndices(nonPlanar, normals, plane, inliers);

    }


    bool planeExtraction::extractPlaneEfficientRANSAC(
            const PointCloudT::Ptr  cloud_in,
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

    void planeExtraction::estimateNormals(
            const PointCloudT::Ptr  cloud,
            PointCloudN::Ptr        normals,
            float                   rad){

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


    pcl::PointIndices::Ptr planeExtraction::segmentPPR(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud,
            pcl::ModelCoefficients::Ptr             coefficients,
            float                                   max_dist){

		ppr::SurfaceRefinement * refinement;
		refinement = new ppr::SurfaceRefinement();
		refinement->use_colors = true;
		refinement->setDebugg(false);
		refinement->setVisualize(false);
		refinement->setMaxDistance(max_dist);


		pcl::PointIndices::Ptr inl (new pcl::PointIndices);

		int width = cloud->width;
		int height = cloud->height;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		normals->width = width;
		normals->height = height;
		normals->points.resize(width*height);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_normals);

		ppr::Plane * p = new ppr::Plane();
		p->normal_x = coefficients->values[0];
		p->normal_y = coefficients->values[1];
		p->normal_z = coefficients->values[2];
		float d = -coefficients->values[3];
		p->point_x	= p->normal_x*d;
		p->point_y	= p->normal_y*d;
		p->point_z	= p->normal_z*d;

		// TODO
		// Shouldnt have to check the distances again, already have list of indices.
		bool ** inliers = new bool*[width];
		for(int i = 0; i < width; i++){
			inliers[i] = new bool[height];
			for(int j = 0; j < height; j++){
				// TODO This loop is probably not needed
				inliers[i][j] = true;
			}
		}

		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				pcl::PointXYZRGBNormal & point = cloud_normals->points.at(h*cloud->width+w);
				if(!isnan(point.z)){
                    refinement->use_colors = true;
					float d =  fabs(p->distance(point.x,point.y,point.z));
					inliers[w][h] = d < 0.1;
				}else{
					inliers[w][h] = false;
				}
			}
		}

		refinement->improve(inliers,cloud_normals,p);

		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				if(inliers[w][h]){
					inl->indices.push_back(h*width+w);
				}
			}
		}


		// std::cout << "Coefficients" << std::endl;
        // std::cout << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
        // std::cout << p->normal_x << ", " << p->normal_y << ", " << p->normal_z << std::endl;
		coefficients->values[0] = p->normal_x;
		coefficients->values[1] = p->normal_y;
		coefficients->values[2] = p->normal_z;
		coefficients->values[3] = -p->distance(0.0,0.0,0.0);



		delete(refinement);
		delete(p);
		delete(inliers);
		return inl;
	}

    void planeExtraction::extractIndices(PointCloudT::Ptr        cloud,
                        PointCloudN::Ptr        normals,
                        PointCloudT::Ptr        plane,
                        pcl::PointIndices::Ptr  inliers){

        extractIndicesAndRemoveFromOriginal<PointT>(cloud, plane, inliers);
        PointCloudN::Ptr normals_plane (new PointCloudN());
        extractIndicesAndRemoveFromOriginal<PointN>(normals, normals_plane, inliers);

        std::vector<PointCloudT::Ptr> planes;

        std::vector<pcl::PointIndices> cluster_indices;
        ecClustering<PointT>(plane, 0.1, 0, 1000000, planes, cluster_indices);

        if(planes.size() > 1){
            std::cout << "ecClustering" << std::endl;
            auto result = std::max_element(planes.begin(), planes.end(),
                [](PointCloudT::Ptr a,PointCloudT::Ptr b){
                    return a->points.size() < b->points.size();
                }
            );
            int ind = std::distance(planes.begin(), result);
            // *planet = *plane;

            for (size_t i = 0; i < planes.size(); i++) {
                if(i == ind){
                    *plane = *planes[i];
                } else {
                    *cloud += *planes[i];
                    PointCloudN::Ptr normals_tmp (new PointCloudN());
                    pcl::PointIndices::Ptr inliers_tmp (new pcl::PointIndices());
                    inliers_tmp->indices = cluster_indices[i].indices;
                    extractIndicesFromOriginal<PointN>(normals_plane, normals_tmp, inliers_tmp);
                    *normals += *normals_tmp;
                }
            }
        }

    }


}
