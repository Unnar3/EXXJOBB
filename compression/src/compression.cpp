#include <exx_compression/compression.h>
#include <utils/utils.h>
#include <intersections/intersections.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/transforms.h>
#include <math.h>

namespace EXX{

	void compression::voxelGridFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr out_cloud){
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud (cloud);
		sor.setLeafSize (v_leaf_size_, v_leaf_size_, v_leaf_size_);
		sor.filter (*out_cloud);
	}

	void compression::superVoxelClustering(vPointCloudT *cloud, vPointCloudT *out_vec, std::vector<densityDescriptor> &dDesc){
		std::vector<PointCloudTA::Ptr> out_cloud;
		std::vector<PointCloudT::Ptr>::iterator it = cloud->begin();
		int i = 0;
		for (; it != cloud->end(); ++it)
		{
			(*out_vec).push_back(  compression::superVoxelClustering_s(*it, dDesc[i++]) );
		}
	}

	PointCloudT::Ptr compression::superVoxelClustering_s(PointCloudT::Ptr cloud, densityDescriptor &dDesc){

		pcl::SupervoxelClustering<PointT> super (dDesc.voxel_res, dDesc.seed_res, false);
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

	void compression::euclideanClusterPlanes(vPointCloudT* cloud, vPointCloudT* out_vec, std::vector<int> *normalIndex){
		// Loop through all planes
		int ind = 0;
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
				normalIndex->push_back(ind);
			}
			ind++;
		}
	}

	void compression::extractPlanesRANSAC(PointCloudT::Ptr cloud, planesAndCoeffs *pac)
	{
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		PointCloudT::Ptr cloud_f (new PointCloudT ());

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud<pcl::Normal>);

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (max_ite_);
		seg.setDistanceThreshold (dist_thresh_);
		seg.setEpsAngle (2*M_PI);
		seg.setNormalDistanceWeight (0.2);

		// Estimate point normals
		ne.setSearchMethod (tree);
		ne.setInputCloud (cloud);
		ne.setKSearch (10);
		ne.compute (*cloud_normals);

		int i=0, nr_points = cloud->points.size ();
		while (i < max_number_planes_ && cloud->points.size() > 0 && cloud->points.size() > 0.05 * nr_points)
		{
		    // Define for each plane we find
		    ModelCoeffT::Ptr coefficients (new ModelCoeffT);
		    PointCloudT::Ptr cloud_plane (new PointCloudT ());

		    // Segment the largest planar component from the remaining cloud
		    seg.setInputCloud (cloud);
		    seg.setInputNormals(cloud_normals);
		    seg.segment (*inliers, *coefficients);

		    if (inliers->indices.size () == 0)
		    {
		        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		        break;
		    }
		    if(inliers->indices.size() < min_inliers_){
		        i++;
		        break;
		    }

		    // Extract the planar inliers from the input cloud
		    pcl::ExtractIndices<PointT> extract;
		    extract.setInputCloud (cloud);
		    extract.setIndices (inliers);
		    extract.setNegative (false);
		    extract.filter (*cloud_plane);

		    // Remove the planar inliers, extract the rest
		    extract.setNegative (true);
		    extract.filter (*cloud_f);
		    cloud.swap (cloud_f);

		    pcl::ExtractIndices<pcl::Normal> extractN;
		    extractN.setInputCloud(cloud_normals);
		    extractN.setIndices (inliers);
		    extractN.setNegative (true);
		    extractN.filter(*cloud_n);
		    cloud_normals.swap(cloud_n);

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

	void compression::projectToPlaneS(PointCloudT::Ptr cloud, Eigen::Vector4d coeff){
		pcl::ModelCoefficients::Ptr m_coeff (new pcl::ModelCoefficients ());
		m_coeff->values.resize (4);
		m_coeff->values[0] = float(coeff(0));
		m_coeff->values[1] = float(coeff(1));
		m_coeff->values[2] = float(coeff(2));
		m_coeff->values[3] = float(coeff(3));
		compression::projectToPlaneS(cloud, m_coeff);
	}

	void compression::projectToPlaneS(PointCloudT::Ptr cloud, ModelCoeffT::Ptr coeff){

		// Create the projection object
		pcl::ProjectInliers<PointT> proj;
		proj.setModelType (pcl::SACMODEL_PLANE);
	    proj.setInputCloud ( cloud );
	    proj.setModelCoefficients ( coeff );
	    proj.filter ( *cloud );
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

	void compression::planeToConvexHull(vPointCloudT &planes, vPointCloudT &hulls, std::vector<double> &area){

		vPointCloudT::iterator it = planes.begin();
		double are = 0;
		for ( ; it < planes.end(); ++it)
		{
			PointCloudT::Ptr out (new PointCloudT ());;
			compression::planeToConvexHull_s(*it, out, are);
			hulls.push_back( out );
			area.push_back(are);
		}
	}

	void compression::planeToConvexHull_s(const PointCloudT::Ptr cloud, PointCloudT::Ptr out, double &area ){

		pcl::ConvexHull<PointT> chull;
		chull.setInputCloud ( cloud );
        //chull.setAlpha ( hull_alpha_ );
        chull.setComputeAreaVolume(true);
        chull.reconstruct (*out);
        area = chull.getTotalArea();
	}

	void compression::reumannWitkamLineSimplification(vPointCloudT* hulls, vPointCloudT* s_hulls, std::vector<densityDescriptor> &dDesc){

		vPointCloudT::iterator it = hulls->begin();
		int i = 0;
		for ( ; it < hulls->end(); ++it){
			(*s_hulls).push_back( compression::reumannWitkamLineSimplification_s(*it, dDesc[++i]) );
		}

	}

	PointCloudT::Ptr compression::reumannWitkamLineSimplification_s(PointCloudT::Ptr cloud, densityDescriptor &dDesc){

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
            if (distBetweenPoints > dDesc.rw_max_dist ){
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

	// TODO: check that both planes belonging to a line are close to the line.
	void compression::cornerMatching(vPointCloudT &planes, vPointCloudT &hulls, const std::vector<Eigen::Vector4d> &coeff){
		Eigen::VectorXd line;
		std::vector<std::vector<Eigen::VectorXd> > lines(planes.size());
		Eigen::Vector4d point;
		point(3) = 0;
		Eigen::Vector4d l_point;
		l_point(3) = 0;
		Eigen::Vector4d d_point;
		d_point(3) = 0;

		pcl::ProjectInliers<PointT> proj;
		proj.setModelType (pcl::SACMODEL_LINE);
		proj.setCopyAllData(true);

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		coefficients->values.resize(6);

		// Convert pcl::PointT to Eigen::Vector4d
		auto pointToEigen = [](Eigen::Vector4d& point, PointT& p){
			point(0) = p.x;
			point(1) = p.y;
			point(2) = p.z;
		};

		// Given bondary points, interior points, line and indexes we project points to the line and
		// fill in empty space between original point and projected point.
		auto projectToLine = [&pointToEigen, &proj, &coefficients](PointCloudT::Ptr hull, PointCloudT::Ptr plane, pcl::PointIndices::Ptr inl, Eigen::VectorXd l){
			if (inl->indices.size()==0){ return; }
			std::vector<Eigen::Vector4d> eigenVecs;
			Eigen::Vector4d eigenVec;
			eigenVec(3) = 0;

			// Store location of all points that will be projected.
			for (auto i : inl->indices){
				pointToEigen(eigenVec, hull->points.at(i));
				eigenVecs.push_back(eigenVec);
			}

			// change line equation from eigen to pcl::modelcoefficients.
			for (size_t i = 0; i < l.size(); ++i){
				coefficients->values[i] = l(i);
			}

			// Project the points.
			PointCloudT::Ptr tmpCloud (new PointCloudT ());
			proj.setInputCloud(hull);
			proj.setIndices(inl);
			proj.setModelCoefficients(coefficients);
			proj.filter(*hull);

			Eigen::Vector4d projectedPoint;
			projectedPoint(3) = 0;
			Eigen::Vector4d vecDiff;
			int numberPoints = 0;
			double vecNorm = 0;
			PointT newPoint;
			// create new points.
			for (size_t i = 0; i < inl->indices.size(); ++i){
				pointToEigen(projectedPoint, hull->points.at(inl->indices.at(i)));
				vecDiff = eigenVecs[i] - projectedPoint;
				vecNorm = vecDiff.norm();
				if ( vecNorm > 0.005 ){
					// Create points inbetween if distance is greater than a threshold.
					for (int j=1; j<numberPoints; ++j){
						newPoint.x = eigenVecs[i](0)-vecDiff(0)/numberPoints*j;
						newPoint.y = eigenVecs[i](1)-vecDiff(1)/numberPoints*j;
						newPoint.z = eigenVecs[i](2)-vecDiff(2)/numberPoints*j;
						newPoint.r = hull->points.at(inl->indices.at(i)).r;
						newPoint.g = hull->points.at(inl->indices.at(i)).g;
						newPoint.b = hull->points.at(inl->indices.at(i)).b;
						plane->points.push_back(newPoint);
					}
				}
			}
		};

		// Find all plane to plane intersections.
		for (size_t i = 0; i < coeff.size()-1; ++i){
			for (size_t k = i+1; k < coeff.size(); ++k){
				if(pcl::planeWithPlaneIntersection( coeff.at(i), coeff.at(k), line, 0.3 )){
					lines[i].push_back(line);
					lines[k].push_back(line);
				}
			}
		}

		float dist;
		float tmpDist;
		int distIdx;
		std::vector<pcl::PointIndices::Ptr> indices;
		// Project hull points to closest intersection line.
		for (size_t i = 0; i < hulls.size(); ++i){
			// Check to see if the plane has any intersections
			if (lines[i].size()>0){
				indices.clear();
				indices.resize(lines[i].size());
				for (size_t j = 0; j < indices.size(); ++j){
					indices[j] = pcl::PointIndices::Ptr (new pcl::PointIndices ());
				}
				// Go through all boundary points on a plane
				for (size_t j = 0; j < hulls.at(i)->points.size(); ++j){
					pointToEigen(point, hulls.at(i)->points.at(j));
					// for each point calculate dist to all lines and map to closest
					for (size_t k = 0; k < lines[i].size(); ++k){
						l_point.head(3) = lines[i][k].head(3);
						d_point.head(3) = lines[i][k].tail(3);
						tmpDist = pcl::sqrPointToLineDistance(point.cast<float>(), l_point.cast<float>(), d_point.cast<float>());
						if (k == 0){
							dist = tmpDist;
							distIdx = 0;
						} else if (tmpDist < dist) {
							dist = tmpDist;
							distIdx = k;
						}
					}
					// apply threshold on distance from intersection
					if(dist < 0.20*0.20){
						indices[distIdx]->indices.push_back(j);
					}
				}
				// Project the points
				for (size_t k = 0; k < indices.size(); ++k){
					projectToLine(hulls.at(i), planes.at(i), indices[k], lines[i][k]);
				}
			}
		}
	}

	void compression::getPlaneDensity( vPointCloudT &planes, vPointCloudT &hulls,  std::vector<densityDescriptor> &dDesc){

		// check if planes and hulls are same size
		if ( planes.size() != hulls.size() ){
			return;
		}

		// Start by finding area of the planes
		pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
        PointT min_point_OBB;
        PointT max_point_OBB;
        PointT pos_OBB;
        Eigen::Matrix3f rot_mat_OBB;
        densityDescriptor dens;

        for (size_t i = 0; i < hulls.size(); ++i){
            feature_extractor.setInputCloud (hulls[i]);
            feature_extractor.compute ();
			feature_extractor.getOBB (min_point_OBB, max_point_OBB, pos_OBB, rot_mat_OBB);
			float x = float( std::abs( max_point_OBB.x - min_point_OBB.x) );
			float y = float( std::abs( max_point_OBB.y - min_point_OBB.y) );
			float area = x * y;
			float max_points = area / ( v_leaf_size_ * v_leaf_size_ );
			float pRatio = float(planes[i]->points.size()) / max_points;
			dens.seed_res = std::min( std::max( std::min( x, y ) / 5.0f * utils::fast_sigmoid(pRatio, 3.0f), v_leaf_size_), sv_seed_res_ );
			// dens.seed_res = sv_seed_res_;
			dens.voxel_res = std::min( dens.seed_res, sv_voxel_res_ );
			dens.rw_max_dist = std::min( dens.seed_res / 2.0f, rw_hull_max_dist_ );
			dens.gp3_search_rad = std::min( 3.0f * utils::l2_norm(dens.seed_res), gp3_search_rad_ );
			dDesc.push_back( dens );
		}

	}


	void compression::greedyProjectionTriangulation(PointCloudT::Ptr nonPlanar, vPointCloudT *planes, vPointCloudT *hulls, std::vector<cloudMesh> *cm){
		PointCloudT::Ptr tmp_cloud (new PointCloudT ());
		for (size_t i = 0; i < sv_planes_.size(); i++){
			*tmp_cloud += *planes->at(i);
			*tmp_cloud += *hulls->at(i);
		}
		*tmp_cloud += *nonPlanar;
		(*cm).push_back( compression::greedyProjectionTriangulation_s( tmp_cloud, gp3_search_rad_ ) );
	}

	void compression::greedyProjectionTriangulationPlanes(PointCloudT::Ptr nonPlanar, vPointCloudT *planes, vPointCloudT *hulls, std::vector<cloudMesh> *cm, std::vector<densityDescriptor> &dDesc){
		PointCloudT::Ptr tmp_hull (new PointCloudT ());
		for (size_t i = 0; i < planes->size(); ++i){
			PointCloudT::Ptr tmp_cloud (new PointCloudT ());
			*tmp_hull += *hulls->at(i);
			*tmp_cloud = *planes->at(i)+*hulls->at(i);
			(*cm).push_back( compression::greedyProjectionTriangulation_s( tmp_cloud, dDesc[i].gp3_search_rad ));
		}
		std::cout << "finished with planes" << std::endl;
		(*cm).push_back( compression::greedyProjectionTriangulation_s( tmp_hull, 0.5f));
		(*cm).push_back( compression::greedyProjectionTriangulation_s( nonPlanar, utils::l2_norm(v_leaf_size_) * 1.5f ));
	}

	void compression::greedyProjectionTriangulationPlanes(PointCloudT::Ptr nonPlanar, vPointCloudT &planes, vPointCloudT &hulls, std::vector<cloudMesh> &cm,std::vector<float> &dDesc){
		for (size_t i = 0; i < planes.size(); ++i){
			PointCloudT::Ptr tmp_cloud (new PointCloudT ());
			*tmp_cloud = *planes.at(i)+*hulls.at(i);
			cm.push_back( compression::greedyProjectionTriangulation_s( tmp_cloud, dDesc[i] ));
		}
		if (nonPlanar->points.size() > 0){
			cm.push_back( compression::greedyProjectionTriangulation_s( nonPlanar, utils::l2_norm(v_leaf_size_) * 1.5f ));
		}
	}

	cloudMesh compression::greedyProjectionTriangulation_s(PointCloudT::Ptr cloud, float gp3_rad){
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
	    gp3.setSearchRadius ( gp3_rad );

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

	void compression::improveTriangulation(std::vector<cloudMesh> &cm, vPointCloudT &planes, vPointCloudT &hulls){

		int inlier_cnt = 0;
		std::vector<int> points;
		std::vector<int> polys;
		int max = 0;
		int min = 0;

		for ( size_t i = 0; i < cm.size()-1; ++i ){
			inlier_cnt = planes.at(i)->points.size();
			polys.clear();
			for ( size_t j = 0; j < cm.at(i).mesh.polygons.size(); ++j ){
				points.clear();
				for ( size_t k = 0; k < cm[i].mesh.polygons[j].vertices.size(); ++k ){
					if ( cm[i].mesh.polygons[j].vertices[k] >= inlier_cnt ){
						points.push_back(k);
					}
				}
				if ( points.size() > 2 ){
					max = *std::max_element(points.begin(), points.end());
					min = *std::min_element(points.begin(), points.end());
					if (max - min < 5){
						polys.push_back(j);
					}
				}
			}
			std::sort(polys.begin(), polys.end(), std::greater<int>());
			for ( auto j : polys ){
				cm[i].mesh.polygons.erase(cm[i].mesh.polygons.begin() + j);
			}
		}

	}

	void compression::improveTriangulation2(std::vector<cloudMesh> &cm, vPointCloudT &planes, vPointCloudT &hulls,  std::vector<std::vector<float> > &normals){

		int inlier_cnt = 0;
		std::vector<uint32_t> points;
		std::vector<int> polys;
		std::vector<PointT> tri(3);
		std::vector<float> tmpNorm;
		int bcount = 0;

		int max = 0;
		int min = 0;

		// find the normal of first triangle consisting of two boundary points and interior point.

		for ( size_t i = 0; i < cm.size()-1; ++i ){
			inlier_cnt = planes.at(i)->points.size();
			std::vector<std::vector<int> > boundary_connections(hulls[i]->points.size());
			std::vector<int> boundary_count(hulls[i]->points.size());
			polys.clear();

			for ( size_t j = 0; j < cm.at(i).mesh.polygons.size(); ++j ){
				bcount = 0;

				points = cm[i].mesh.polygons[j].vertices;
				std::sort(points.begin(), points.end());

				// Check if triangle is outside boundary
				// Count number of boundary points in the triangle
				bcount = std::count_if(points.begin(), points.end(),
				[inlier_cnt](int number){return (number >= inlier_cnt);});
				if ( bcount > 2 ){ // Three boundary points forming a triangle
					points = cm[i].mesh.polygons[j].vertices;
					// std::sort(points.begin(), points.end());
                    for (size_t k = 0; k < points.size(); k++) {
						tri[k] = cm.at(i).cloud->points.at(points[k]);
					}
					tmpNorm = utils::triNorm(tri[0], tri[1], tri[2]);
					if( utils::vecDot(normals[i], tmpNorm) > 0 ){
						polys.push_back(j);
					}
				}
			}

			std::sort(polys.begin(), polys.end(), std::greater<int>());
			for ( auto j : polys ){
				cm[i].mesh.polygons.erase(cm[i].mesh.polygons.begin() + j);
			}
		}
	}


	Eigen::Vector3d compression::findMainNorm(const std::vector<Eigen::Vector4d> &normals){
        std::vector<double> x;
        std::vector<double> y;
        x.reserve(normals.size());
        y.reserve(normals.size());
        Eigen::Vector3d tmp;

        for(auto normal : normals){
            int idx;
            normal.head(3).cwiseAbs().maxCoeff(&idx);
            if(idx != 2 && normal[2] < 0.1){
                tmp = normal.head(3);
                tmp[2] = 0;
                tmp = tmp/tmp.norm();
                // for (size_t i = 0; i < tmp.size(); i++) {
                //     std::cout << tmp[i] << std::endl;
                // }
                // return tmp;
                if(std::abs(tmp[1]) > std::abs(tmp[0])){
                    // make x axis bigger by turning 90 degrees
                    double value = tmp[0];
                    tmp[0] = tmp[1];
                    tmp[1] = -value;
                }
                if(tmp[0] < 0){
                    tmp[0] *= -1;
                    tmp[1] *= -1;
                }
                x.push_back(tmp[0]);
                y.push_back(tmp[1]);
                std::cout << "x: " << x.back() << "  y: " << y.back() << std::endl;
            }
        }
        // median of vector
        auto median_func = [](std::vector<double> x)->double{
            std::sort(x.begin(), x.end());
            if(x.size()%2 == 0)
                return (x[x.size()/2 - 1] + x[x.size()/2]) / 2.0;
            return x[x.size()/2];
        };

        double median = median_func(x);
        std::vector<double> x_median(x.size());
        std::transform(x.begin(),x.end(),x_median.begin(),
            [median](double x_val){
                return std::abs(x_val - median);
            }
        );
        double mad = median_func(x_median);

        int k = 0;
        double sum = 0;
        for(auto val : x){
            if(std::abs(val - median) < 2*mad){
                k++;
                sum += val;
            }
        }
        double mean = sum / k;
        tmp[0] = mean;
        tmp[1] = std::sqrt(1-std::pow(mean,2));
        tmp[3] = 0;
        for (size_t i = 0; i < tmp.size(); i++) {
            std::cout << tmp[i] << std::endl;
        }
        return tmp;
    }

	void compression::rotateToAxis(	const Eigen::Vector3d           normal,
                    				std::vector<PointCloudT::Ptr>   &planes,
                    				std::vector<Eigen::Vector4d>    &normal_vec,
                    				PointCloudT::Ptr                nonPlanar){

		// as.fkj
		if(planes.size() != normal_vec.size()){
            std::cerr << "planes and normals of different size, exiting!!!" << std::endl;
            exit(0);
        }

        // Get angle between [1 0 0] and normal.
        double theta = std::atan2(normal[1], normal[0]);
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));
        pcl::transformPointCloud (*nonPlanar, *nonPlanar, transform);

        // need to rotate each plane and the associated normal vector.
        for (size_t i = 0; i < planes.size(); i++) {
            // Transform boundaries and super voxels according to theta.
            pcl::transformPointCloud (*planes[i], *planes[i], transform);

            // Change the direction of the normal and find new distance from origin for the plane equation.
            // Also perfectly align to x/y/z axis if similar enough.
            normal_vec[i][0] = normal_vec[i][0]*std::cos(-theta) - normal_vec[i][1]*std::sin(-theta);
            normal_vec[i][1] = normal_vec[i][0]*std::sin(-theta) + normal_vec[i][1]*std::cos(-theta);
            int idx;
            normal_vec[i].head(3).cwiseAbs().maxCoeff(&idx);
            if(idx == 0){
                if(std::abs(normal_vec[i][2]) < 0.25){
                    // This is a wall like structures
                    if(std::abs(normal_vec[i][1]) < 0.3){
                        // Should most likely be a wall, align it to the axis.
                        normal_vec[i][0] = 1;
                        normal_vec[i][1] = 0;
                        normal_vec[i][2] = 0;
                    }
                }
                // Need to fix distance from origin after aligning.
                PointT p1, p2;
                pcl::getMinMax3D(*planes[i], p1, p2);
                p1.x = (p1.x + p2.x)/2.0;
                p1.y = (p1.y + p2.y)/2.0;
                p1.z = (p1.z + p2.z)/2.0;
                normal_vec[i][3] = -(p1.x*normal_vec[i][0] + p1.y*normal_vec[i][1] + p1.z*normal_vec[i][2]);

            } else if (idx == 1){
                if(std::abs(normal_vec[i][2]) < 0.25){
                    // This is a wall like structures
                    if(std::abs(normal_vec[i][0]) < 0.3){
                        // Should most likely be a wall, align it to the axis.
                        normal_vec[i][0] = 0;
                        normal_vec[i][1] = 1;
                        normal_vec[i][2] = 0;
                    }
                }
                // Need to fix distance from origin after aligning.
                PointT p1, p2;
                pcl::getMinMax3D(*planes[i], p1, p2);
                p1.x = (p1.x + p2.x)/2.0;
                p1.y = (p1.y + p2.y)/2.0;
                p1.z = (p1.z + p2.z)/2.0;
                normal_vec[i][3] = -(p1.x*normal_vec[i][0] + p1.y*normal_vec[i][1] + p1.z*normal_vec[i][2]);
            } else {
                if(std::abs(normal_vec[i][2]) > 0.90){
                    // floor like object
                    normal_vec[i][0] = 0;
                    normal_vec[i][1] = 0;
                    normal_vec[i][2] = 1;

                    PointT p1, p2;
                    pcl::getMinMax3D(*planes[i], p1, p2);
                    p1.x = (p1.x + p2.x)/2.0;
                    p1.y = (p1.y + p2.y)/2.0;
                    p1.z = (p1.z + p2.z)/2.0;
                    normal_vec[i][3] = -(p1.x*normal_vec[i][0] + p1.y*normal_vec[i][1] + p1.z*normal_vec[i][2]);
                }
            }
        }
	}

	void compression::rotateToAxis(	std::vector<PointCloudT::Ptr>   &planes,
									std::vector<Eigen::Vector4d>    &normal_vec,
									PointCloudT::Ptr                nonPlanar){

		compression::rotateToAxis(compression::findMainNorm(normal_vec),planes,normal_vec,nonPlanar);
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
		// compression::voxelGridFilter()lastd::cout << "voxel" << std::endl;
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
}
