#ifndef CGALTRIANGULATION_H
#define CGALTRIANGULATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>

#include <PointTypes/surfel_type.h>
#include <typedefs/typedef.h>
#include <Eigen/Dense>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel                 Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel>   Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                            Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                         Delaunay;
typedef Kernel::Point_2                                                     Point;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel>                 Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                         TDS;
typedef CGAL::Exact_predicates_tag                                          Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, Itag>       CDT;
typedef CGAL::Spatial_sort_traits_adapter_2<Kernel,Point*>                  Search_traits;

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointNT = pcl::PointNormal;
using PointNCloudT = pcl::PointCloud<PointNT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;


template <class InputIterator>
void insert_with_info(CDT& cdt, InputIterator first,InputIterator last);


void constrainedDelaunayTriangulation(  std::vector<Point>                      points,
                                        std::vector<Point>                      constraines,
                                        std::vector<std::vector<unsigned int> > &idx);

template <class T>
void pclPlaneToCGAL(    typename pcl::PointCloud<T>::Ptr    plane,
                        typename pcl::PointCloud<T>::Ptr    boundary,
                        pcl::ModelCoefficients::Ptr         coeff,
                        std::vector<Point >               & plane_2d,
                        std::vector<Point >               & boundary_2d);


/////////////////////////////////////////////////////////////
// Implementation
/////////////////////////////////////////////////////////////

// Takes in PointCloud where all points are lying on a plane and projects that plane to 2d.
template <class T>
void pclPlaneToCGAL(    typename pcl::PointCloud<T>::Ptr    plane,
                        typename pcl::PointCloud<T>::Ptr    boundary,
                        pcl::ModelCoefficients::Ptr         coeff,
                        std::vector<Point >               & plane_2d,
                        std::vector<Point >               & boundary_2d){

    // Make sure 2d vectors are empty and efficient.
    if(plane_2d.size() != 0){
        plane_2d.clear();
        plane_2d.reserve(plane->points.size());
    }
    if(boundary_2d.size() != 0){
        boundary_2d.clear();
        boundary_2d.reserve(boundary->points.size());
    }

    // Create the normal vector
    Eigen::Vector3f N;
    for (size_t i = 0; i < N.size(); i++) {
        N[i] = coeff->values[i];
    }
    N = N / N.norm();

    // Create arbitrary vector U not parallell to N to find a new X/Y axis
    Eigen::Vector3f U(1,0,0);
    if( std::abs(N.dot(U)) > 0.8 ){
        // U to similar to N change it to [0 1 0].
        U[0] = 0; U[1] = 1;
    }

    // create the new x y axis orthogonal to N.
    Eigen::Vector3f X = U - (U.dot(N) * N);
    Eigen::Vector3f Y = N.cross(X);

    // normalize the axis
    X = X / X.norm();
    Y = Y / Y.norm();

    // Loop through all points and project them to new 2d coordinate system
    Eigen::Vector3f point;
    for(auto p : plane->points){
        point[0] = p.x;
        point[1] = p.y;
        point[2] = p.z;
        plane_2d.push_back( Point( point.dot(X), point.dot(Y) ) );
    }

    for(auto p : boundary->points){
        point[0] = p.x;
        point[1] = p.y;
        point[2] = p.z;
        boundary_2d.push_back( Point( point.dot(X), point.dot(Y) ) );
    }
}


void constrainedDelaunayTriangulation(  std::vector<Point>                      points,
                                        std::vector<Point>                      constraines,
                                        std::vector<std::vector<unsigned int> > &idx){

    CDT cdt;

    if(points.size() < 3) return;
    int size = points.size();
    int points_size = size;

    // Insert the points.
    insert_with_info(cdt, points.begin(),points.end());

    // Insert the constraints (boundary).
    if(constraines.size() > 1){
        for (size_t i = 0; i < constraines.size()-1; i++) {
            cdt.insert_constraint(constraines[i], constraines[i+1]);
        }

        // put the correct index on the constraine points.
        auto it = cdt.vertices_begin();
        std::cout << "size: " << size << std::endl;
        std::advance(it, size);
        for(; it != cdt.vertices_end(); ++it){
            it->info() = size;
            std::cout << size << std::endl;
            size++;
        }
    }

    // Clear Idx if not empty and reserve approximate size;
    if(idx.size() != 0) idx.clear();
    idx.reserve( cdt.number_of_faces() );


    // Fill idx with indices.
    std::vector<unsigned int> poly(3);

    auto check_boundary = [points_size](std::vector<unsigned int> vec){
        for(auto point : vec){
            if( point < points_size ) return true;
        }
        return false;
    };

    for(CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
        fit != cdt.finite_faces_end(); ++fit) {


        CDT::Face_handle face = fit;

        poly[0] = face->vertex(0)->info();
        poly[1] = face->vertex(1)->info();
        poly[2] = face->vertex(2)->info();

        if( check_boundary(poly) ){
            idx.push_back(poly);
        }
    }
}



template <class InputIterator>
void insert_with_info(CDT& cdt, InputIterator first,InputIterator last)
{
  std::vector<std::ptrdiff_t> indices;
  std::vector<Point> points;
  std::ptrdiff_t index=0;

  for (InputIterator it=first;it!=last;++it){
    points.push_back( *it);
    indices.push_back(index++);
  }

  CGAL::spatial_sort(indices.begin(),indices.end(),Search_traits(&(points[0]),cdt.geom_traits()));

  CDT::Vertex_handle v_hint;
  CDT::Face_handle hint;
  for (typename std::vector<std::ptrdiff_t>::const_iterator
    it = indices.begin(), end = indices.end();
    it != end; ++it){
    v_hint = cdt.insert(points[*it], hint);
    if (v_hint!=CDT::Vertex_handle()){
      v_hint->info()=*it;
      hint=v_hint->face();
    }
  }
}


#endif