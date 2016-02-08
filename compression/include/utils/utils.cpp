#include "utils.h"
#include <moment_of_inertia/moment_of_inertia_estimation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include "pcl/common/common.h"

namespace EXX {
namespace utils {

        void getOBB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, OBBcube &obb){
                pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
                pcl::PointXYZRGB min_point_OBB;
                pcl::PointXYZRGB max_point_OBB;
                pcl::PointXYZRGB min_point_OBB_original;
                pcl::PointXYZRGB max_point_OBB_original;
                pcl::PointXYZRGB position_OBB;
                Eigen::Matrix3f rotational_matrix_OBB;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);


                pcl::getMinMax3D(*cloud, min_point_OBB_original, max_point_OBB_original);

                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
                coefficients->values.resize (4);
                coefficients->values[0] = coefficients->values[1] = 0;
                coefficients->values[2] = 1;
                coefficients->values[3] = 0;

                // Create the filtering object
                pcl::ProjectInliers<pcl::PointXYZRGB> proj;
                proj.setModelType(pcl::SACMODEL_PLANE);
                proj.setInputCloud(cloud);
                proj.setModelCoefficients(coefficients);
                proj.filter(*cloud_projected);

                feature_extractor.setInputCloud(cloud_projected);
                feature_extractor.compute();

                feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);


                obb.position = Eigen::Vector3f(position_OBB.x, position_OBB.y, (max_point_OBB_original.z+min_point_OBB_original.z)/2.0);
                obb.rot = rotational_matrix_OBB;
                obb.x = max_point_OBB.x - min_point_OBB.x;
                obb.y = max_point_OBB.y - min_point_OBB.y;
                obb.z = max_point_OBB_original.z - min_point_OBB_original.z;
        };
}
}
