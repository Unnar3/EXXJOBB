#include <exx_compression/compression.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

namespace EXX{

	void compression::voxelGridFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr out_cloud){
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud (cloud);
		sor.setLeafSize (v_leaf_size_, v_leaf_size_, v_leaf_size_);
		sor.filter (*out_cloud);
	}

	void compression::superVoxelClustering(vPointCloudT *cloud, vPointCloudT *out_vec){
		std::vector<PointCloudTA::Ptr> out_cloud;
		std::vector<PointCloudT::Ptr>::iterator it = cloud->begin();
		for (; it != cloud->end(); ++it)
		{
			(*out_vec).push_back(  compression::superVoxelClustering_s(*it) );
		}
	}

	PointCloudT::Ptr compression::superVoxelClustering_s(PointCloudT::Ptr cloud){

		pcl::SupervoxelClustering<PointT> super (sv_voxel_res_, sv_seed_res_, false);
		super.setColorImportance (sv_color_imp_);
		super.setSpatialImportance (sv_spatial_imp_);
		super.setNormalImportance(0.0f);
		super.setInputCloud (cloud);
		std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
		super.extract (supervoxel_clusters);

		PointCloudTA::Ptr out (new PointCloudTA ());
		std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >::iterator it = supervoxel_clusters.begin();
		for ( ; it != supervoxel_clusters.end() ; ++it ){
			out->points.push_back( it->second->centroid_ );
		}
		return compression::PointRGBAtoRGB(out);
	} 

	void compression::euclideanClusterPlanes(vPointCloudT* cloud, vPointCloudT* out_vec){
		// Loop through all planes
		vPointCloudT::iterator ite = cloud->begin();
		for ( ; ite != cloud->end(); ++ite){
			// Creating the KdTree object for the search method of the extraction
			pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
			tree->setInputCloud (*ite);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<PointT> ec;
			ec.setClusterTolerance (ec_cluster_tolerance_); // 2cm
			ec.setMinClusterSize (ec_min_cluster_size_);
			// ec.setMaxClusterSize (25000);
			ec.setSearchMethod (tree);
			ec.setInputCloud (*ite);
			ec.extract (cluster_indices);

			int j = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				PointCloudT::Ptr cloud_cluster (new PointCloudT);
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
					cloud_cluster->points.push_back ((*ite)->points[*pit]); //*
				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
				(*out_vec).push_back(cloud_cluster);
			}
		}
	}

	void compression::extractPlanesRANSAC(PointCloudT::Ptr cloud, planesAndCoeffs *pac)
	{
		pcl::SACSegmentation<PointT> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		PointCloudT::Ptr cloud_f (new PointCloudT ());
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (max_ite_);
		seg.setDistanceThreshold (dist_thresh_);

		int i=0, nr_points = cloud->points.size ();
		while (i < max_number_planes_ && cloud->points.size() > 0 && cloud->points.size() > 0.05 * nr_points)
		{
		    // Define for each plane we find
		    ModelCoeffT::Ptr coefficients (new ModelCoeffT);
		    PointCloudT::Ptr cloud_plane (new PointCloudT ());

		    // Segment the largest planar component from the remaining cloud
		    seg.setInputCloud (cloud);
		    seg.segment (*inliers, *coefficients);
		    if (inliers->indices.size () == 0)
		    {
		        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		        break;
		    }
		    if(inliers->indices.size() < min_inliers_){
		        i++;
		        continue;
		    }

		    // Extract the planar inliers from the input cloud
		    pcl::ExtractIndices<PointT> extract;
		    extract.setInputCloud (cloud);
		    extract.setIndices (inliers);
		    extract.setNegative (false);

		    // Get the points associated with the planar surface
		    extract.filter (*cloud_plane);

		    // Remove the planar inliers, extract the rest
		    extract.setNegative (true);
		    extract.filter (*cloud_f);
		    cloud.swap (cloud_f);

		    pac->coeff.push_back(coefficients);
		    pac->cloud.push_back(cloud_plane);
		    i++;
		}
	}

	void compression::projectToPlane(planesAndCoeffs *pac){

		// Create the projection object
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);

		for (int i = 0; i < pac->cloud.size(); i++){
		    proj.setInputCloud ( pac->cloud.at(i) );
		    proj.setModelCoefficients ( pac->coeff.at(i) );
		    proj.filter ( *(pac->cloud.at(i)) );
		}
	}

	void compression::planeToConcaveHull(vPointCloudT *planes, vPointCloudT *hulls){

		vPointCloudT::iterator it = planes->begin();
		for ( ; it < planes->end(); ++it)
		{
			(*hulls).push_back( compression::planeToConcaveHull_s(*it) );
		}
	}

	PointCloudT::Ptr compression::planeToConcaveHull_s(PointCloudT::Ptr cloud){
		
		pcl::ConcaveHull<PointT> chull;
		PointCloudT::Ptr cloud_hull (new PointCloudT ());
		chull.setInputCloud ( cloud );
        chull.setAlpha ( hull_alpha_ );
        chull.setKeepInformation ( true );
        chull.reconstruct (*cloud_hull);

		return cloud_hull;
	}

	void compression::reumannWitkamLineSimplification(vPointCloudT* hulls, vPointCloudT* s_hulls){

		vPointCloudT::iterator it = hulls->begin();
		for ( ; it < hulls->end(); ++it){
			(*s_hulls).push_back( compression::reumannWitkamLineSimplification_s(*it) );
		}

	}

	PointCloudT::Ptr compression::reumannWitkamLineSimplification_s(PointCloudT::Ptr cloud){
	    
	    double distToLine, distBetweenPoints;
	    int j_current, j_next, j_nextCheck, j_last;
	    PointT current, next, last, nextCheck;
	    PointCloudT::Ptr cloud_out (new PointCloudT ());

	    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	    pcl::ExtractIndices<PointT> extract;

        j_current = 0;
        j_next = j_current + 1;
        j_last = j_next;
        j_nextCheck = j_next + 1;

        // Loop through all points in the plane and find redundant points.
        while(j_nextCheck < cloud->points.size() ){
            current = cloud->points.at(j_current);
            next = cloud->points.at(j_next);
            last = cloud->points.at(j_last);
            nextCheck = cloud->points.at(j_nextCheck);

            // Check that the point is not to far away current point.
            distBetweenPoints = compression::distBetweenPoints(current, nextCheck);
            if (distBetweenPoints > rw_hull_max_dist_){
            	inliers->indices.push_back(j_next);
            	j_current = j_nextCheck;
            	j_next = j_current + 1;
            	j_last = j_next;
            	j_nextCheck = j_next + 1;
                continue;
            }

            // Check that the point is not to far away from the line.
            distToLine = pointToLineDistance(current, next, nextCheck);
            if ( distToLine < rw_hull_eps_ ){
                if ( j_next != j_last ){
                    inliers->indices.push_back(j_last);
                }
                j_last++;
                j_nextCheck++;
            } else {
                inliers->indices.push_back(j_next);
                j_current = j_nextCheck;
                j_next = j_current + 1;
                j_last = j_next;
                j_nextCheck = j_next + 1;
            }
        }
        // Remove the redundant points.
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_out);

        return cloud_out;
	}

	void compression::greedyProjectionTriangulation(PointCloudT::Ptr nonPlanar, vPointCloudT *planes, vPointCloudT *hulls, std::vector<cloudMesh> *cm){
		PointCloudT::Ptr tmp_cloud (new PointCloudT ());
		for (size_t i = 0; i < sv_planes_.size(); i++){
			*tmp_cloud += *planes->at(i);
			*tmp_cloud += *hulls->at(i);
		}
		*tmp_cloud += *nonPlanar;
		(*cm).push_back( compression::greedyProjectionTriangulation_s( tmp_cloud ) );
	}

	void compression::greedyProjectionTriangulationPlanes(PointCloudT::Ptr nonPlanar, vPointCloudT *planes, vPointCloudT *hulls, std::vector<cloudMesh> *cm){
		PointCloudT::Ptr tmp_cloud (new PointCloudT ());
		for (size_t i = 0; i < planes->size(); ++i){
			*tmp_cloud = *planes->at(i)+*hulls->at(i);
			(*cm).push_back( compression::greedyProjectionTriangulation_s( tmp_cloud ));
		}
		(*cm).push_back( compression::greedyProjectionTriangulation_s( nonPlanar ));
	}

	cloudMesh compression::greedyProjectionTriangulation_s(PointCloudT::Ptr cloud){
	    cloudMesh cloud_mesh;

	    // Normal estimation*
	    pcl::NormalEstimation<PointT, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

	    // Initialize objects
	    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

	    // Set the maximum distance between connected points (maximum edge length)
	    gp3.setSearchRadius ( gp3_search_rad_ );

	    // Set typical values for the parameters
	    gp3.setMu ( gp3_Mu_ );
	    gp3.setMaximumNearestNeighbors( gp3_max_nearest_neighbours_ );
	    gp3.setMaximumSurfaceAngle( gp3_max_surface_angle_ );
	    gp3.setMinimumAngle( gp3_min_angle_ );
	    gp3.setMaximumAngle( gp3_max_angle_ ); // 120 degrees
	    gp3.setNormalConsistency(false);

	    tree->setInputCloud (cloud);
	    n.setInputCloud (cloud);
	    n.setSearchMethod (tree);
	    n.setKSearch ( gp3_Ksearch_ );
	    n.compute (*normals);

	    // Concatenate the XYZ and normal fields*
	    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	    tree2->setInputCloud (cloud_with_normals);

	    // Get result
	    gp3.setInputCloud (cloud_with_normals);
	    gp3.setSearchMethod (tree2);
	    gp3.reconstruct (cloud_mesh.mesh);
	    cloud_mesh.cloud = cloud;
	    return cloud_mesh;
	}

	double compression::pointToLineDistance(PointT current, PointT next, PointT nextCheck){
		std::vector<float> x0x1;
		x0x1.push_back(nextCheck.x-current.x);
		x0x1.push_back(nextCheck.y-current.y);
		x0x1.push_back(nextCheck.z-current.z);

		std::vector<float> x0x2;
		x0x2.push_back(nextCheck.x-next.x);
		x0x2.push_back(nextCheck.y-next.y);
		x0x2.push_back(nextCheck.z-next.z);

		std::vector<float> x1x2;
		x1x2.push_back(current.x-next.x);
		x1x2.push_back(current.y-next.y);
		x1x2.push_back(current.z-next.z);

		std::vector<float> cross;
		cross.push_back(x0x1[1]*x0x2[2] - x0x1[2]*x0x2[1]);
		cross.push_back(x0x1[2]*x0x2[0] - x0x1[0]*x0x2[2]);
		cross.push_back(x0x1[0]*x0x2[1] - x0x1[1]*x0x2[0]);

		return std::sqrt(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]) / std::sqrt(x1x2[0]*x1x2[0] + x1x2[1]*x1x2[1] + x1x2[2]*x1x2[2]);
	}

	double compression::distBetweenPoints(PointT a, PointT b){
		double x = a.x - b.x;
		double y = a.y - b.y;
		double z = a.z - b.z;

		return std::sqrt( x*x + y*y + z*z );
	}
	
	PointCloudT::Ptr compression::PointRGBAtoRGB( PointCloudTA::Ptr cloudRGBA ){
		PointCloudT::Ptr cloud (new PointCloudT ());
		pcl::copyPointCloud(*cloudRGBA, *cloud);
		return cloud;
	}

	void compression::triangulate(){
		// compression::voxelGridFilter();
		// compression::extractPlanesRANSAC();
		// compression::projectToPlane();
		// compression::planeToConcaveHull();
		// compression::reumannWitkamLineSimplification();
		// compression::superVoxelClustering();
		// compression::greedyProjectionTriangulation();
	}

	void compression::triangulatePlanes(){
		// compression::voxelGridFilter();
		// std::cout << "voxel" << std::endl;
		// compression::extractPlanesRANSAC();
		// std::cout << "ransac" << std::endl;
		// compression::projectToPlane();
		// std::cout << "project" << std::endl;
		// compression::planeToConcaveHull();
		// std::cout << "hull" << std::endl;
		// compression::reumannWitkamLineSimplification();
		// std::cout << "simple hull" << std::endl;
		// compression::superVoxelClustering();
		// std::cout << "super" << std::endl;
		// compression::greedyProjectionTriangulationPlanes();
		// std::cout << "triangulation" << std::endl;
	}


	void compression::savePCD(PointCloudT::Ptr cloud, std::string name){
		pcl::io::savePCDFileASCII (name + ".pcd", *cloud);
	}

	void compression::savePCD(std::vector<PointCloudT::Ptr> cloud, std::string name){
		PointCloudT::Ptr tmp_cloud (new PointCloudT ());
		for (size_t i = 0; i < cloud.size(); i++){
			*tmp_cloud += *(cloud.at(i));
		}
		pcl::io::savePCDFileASCII (name + ".pcd", *tmp_cloud);
	}

	void compression::saveVTK(pcl::PolygonMesh mesh, std::string name){
		pcl::io::saveVTKFile (name + ".vtk", mesh);
	}
}