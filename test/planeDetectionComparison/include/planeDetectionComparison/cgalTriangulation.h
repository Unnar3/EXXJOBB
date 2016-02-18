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


template <class InputIterator>
void insert_with_info(CDT& cdt, InputIterator first,InputIterator last);


void constrainedDelaunayTriangulation(  std::vector<Point>                      points,
                                        std::vector<Point>                      constraines,
                                        std::vector<std::vector<unsigned int> > &idx);

template <typename PointT>
void pclPlaneToCGAL(    pcl::PointCloud<PointT>::Ptr plane,
                        pcl::PointCloud<PointT>::Ptr boundary,
                        pcl::ModelCoefficients::Ptr coeff
                        std::vector<std::vector<unsigned int> > &idx);


/////////////////////////////////////////////////////////////
// Implementation
/////////////////////////////////////////////////////////////

// Takes in PointCloud where all points are lying on a plane and projects that plane to 2d.
template <typename PointT>
void pclPlaneToCGAL(    pcl::PointCloud<PointT>::Ptr plane,
                        pcl::PointCloud<PointT>::Ptr boundary,
                        pcl::ModelCoefficients::Ptr coeff,
                        std::vector<Point > &plane_2d,
                        std::vector<Point > &boundary_2d){

    // Create the normal vector
    Eigen::Vector3f normal;
    for (size_t i = 0; i < normal.size(); i++) {
        normal[i] = coeff->values[i];
    }
    normal = normal/normal.norm();

    // Create arbitrary vector U not parallell to normal to find a new X/Y axis
    Eigen::Vector3f U(1,0,0);
    if( std::abs(normal.dot(U)) > 0.8 ){
        // U to similar to normal change it to [0 1 0].
        U[0] = 0; U[1] = 1;
    }

}


void constrainedDelaunayTriangulation(  std::vector<Point>                      points,
                                        std::vector<Point>                      constraines,
                                        std::vector<std::vector<unsigned int> > &idx){

    CDT cdt;

    // Insert the points.
    insert_with_info(cdt, points.begin(),points.end());

    // Insert the constraints (boundary).
    for (size_t i = 0; i < constraines.size()-1; i++) {
        cdt.insert_constraint(constraines[i], constraines[i+1]);
    }

    // put the correct index on the constraine points.
    auto it = cdt.vertices_begin();
    int size = points.size();
    int points_size = size;
    std::cout << "size: " << size << std::endl;
    std::advance(it, size);
    for(; it != cdt.vertices_end(); ++it){
        it->info() = size;
        std::cout << size << std::endl;
        size++;
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
