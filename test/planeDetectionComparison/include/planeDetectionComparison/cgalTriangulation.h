#ifndef CGALTRIANGULATION_H
#define CGALTRIANGULATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>
#include <CGAL/Polygon_2.h>

#include <PointTypes/surfel_type.h>
#include <typedefs/typedef.h>
#include <Eigen/Dense>
#include <vector>

// struct FaceInfo2
// {
//   FaceInfo2(){}
//   int nesting_level;
//
//   bool in_domain(){
//     return nesting_level%2 == 1;
//   }
// };

typedef CGAL::Exact_predicates_inexact_constructions_kernel                 Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel>   Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                            Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                         Delaunay;
typedef Kernel::Point_2                                                     Point;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel>                 Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                         TDS;
typedef CGAL::Exact_predicates_tag                                          Itag;
// typedef CGAL::No_intersection_tag                                           Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, Itag>       CDT;
typedef CGAL::Spatial_sort_traits_adapter_2<Kernel,Point*>                  Search_traits;
typedef CGAL::Polygon_2<Kernel>                                             Polygon_2;

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


    std::cout << "plane size: " << plane->points.size() << std::endl;
    std::cout << "boundary size: " << boundary->points.size() << std::endl;
    std::cout << "combined size: " << plane->points.size() + boundary->points.size() << std::endl;

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


// void
// mark_domains(CDT& ct,
//              CDT::Face_handle start,
//              int index,
//              std::list<CDT::Edge>& border )
// {
//   if(start->info().nesting_level != -1){
//     return;
//   }
//   std::list<CDT::Face_handle> queue;
//   queue.push_back(start);
//
//   while(! queue.empty()){
//     CDT::Face_handle fh = queue.front();
//     queue.pop_front();
//     if(fh->info().nesting_level == -1){
//       fh->info().nesting_level = index;
//       for(int i = 0; i < 3; i++){
//         CDT::Edge e(fh,i);
//         CDT::Face_handle n = fh->neighbor(i);
//         if(n->info().nesting_level == -1){
//           if(ct.is_constrained(e)) border.push_back(e);
//           else queue.push_back(n);
//         }
//       }
//     }
//   }
// }
//
// //explore set of facets connected with non constrained edges,
// //and attribute to each such set a nesting level.
// //We start from facets incident to the infinite vertex, with a nesting
// //level of 0. Then we recursively consider the non-explored facets incident
// //to constrained edges bounding the former set and increase the nesting level by 1.
// //Facets in the domain are those with an odd nesting level.
// void
// mark_domains(CDT& cdt)
// {
//   for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
//     it->info().nesting_level = -1;
//   }
//
//   int index = 0;
//   std::list<CDT::Edge> border;
//   mark_domains(cdt, cdt.infinite_face(), index++, border);
//   while(! border.empty()){
//     CDT::Edge e = border.front();
//     border.pop_front();
//     CDT::Face_handle n = e.first->neighbor(e.second);
//     if(n->info().nesting_level == -1){
//       mark_domains(cdt, n, e.first->info().nesting_level+1, border);
//     }
//   }
// }


void insert_polygon(CDT& cdt,const Polygon_2& polygon){
  if ( polygon.is_empty() ) return;
  CDT::Vertex_handle v_prev=cdt.insert(*CGAL::cpp11::prev(polygon.vertices_end()));
  for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin();
       vit!=polygon.vertices_end();++vit)
  {
    CDT::Vertex_handle vh=cdt.insert(*vit);
    cdt.insert_constraint(vh,v_prev);
    v_prev=vh;
  }
}


void constrainedDelaunayTriangulation(  std::vector<Point>                      points,
                                        std::vector<Point>                      constraines,
                                        std::vector<std::vector<unsigned int> > &idx){


    auto dist_func = [](Point a, Point b){
        return std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2) < 0.5;
    };

    Polygon_2 polygon;
    std::vector<Polygon_2> polygons;
    polygons.push_back(polygon);
    int k = 0;
    int startP = 0;
    for (size_t i = 0; i < constraines.size()-1; i++) {
        if(dist_func(constraines[i], constraines[i+1])){
            if(i == startP){
                polygons[k].push_back(constraines[startP]);
            }
            polygons[k].push_back(constraines[i+1]);
        } else {
            k++;
            startP = i+1;
            polygons.push_back(polygon);
            // break;
        }

    }
    int l = 0;
    for(auto poly : polygons){
        std::cout << "poly: " << l++ << std::endl;
        // for(auto p : poly) {
        for (size_t i = 0; i < poly.size(); i++) {
            /* code */
            std::cout << poly[i] << std::endl;
        }
    }


    CDT cdt;

    if(points.size() < 3) return;
    int size = points.size();
    int points_size = size;

    // Insert the points.
    insert_with_info(cdt, points.begin(),points.end());

    // Insert the constraints (boundary).
    int hull_close = 0;
    if(constraines.size() > 1){
        for(auto pol : polygons){
            if(pol.size() > 5){
                insert_polygon(cdt, pol);
            }
        }
        //Mark facets that are inside the domain bounded by the polygon
        // mark_domains(cdt);


        // put the correct index on the constraine points.
        auto it = cdt.vertices_begin();
        std::advance(it, size);
        for(; it != cdt.vertices_end(); ++it){
            it->info() = size;
            // std::cout << size << std::endl;
            size++;
        }
        // std::cout << "vertices count: " << cdt.number_of_vertices()  << std::endl;
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
