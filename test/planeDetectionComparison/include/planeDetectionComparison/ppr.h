#include <Refinement/SurfaceRefinement.h>

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

    //Gets initial guess using ransac to find the largest plane
	//Refines the result using Probabilistic Plane Refinement
	//Visualize
	pcl::PointIndices::Ptr segmentPPR(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, float max_dist){

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
}
