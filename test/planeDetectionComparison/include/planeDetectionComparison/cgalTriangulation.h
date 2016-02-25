#ifndef CGALTRIANGULATION_H
#define CGALTRIANGULATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/intersections.h>

#include <PointTypes/surfel_type.h>
#include <typedefs/typedef.h>
#include <Eigen/Dense>
#include <vector>

struct FaceInfo2
{
  FaceInfo2(){}
  int nesting_level;

  bool in_domain(){
    return nesting_level%2 == 1;
  }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel            K;
// typedef CGAL::Exact_predicates_exact_constructions_kernel K;

typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K>   Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>         Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>            Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                    TDS;
// typedef CGAL::Exact_predicates_tag                                     Itag;
// typedef CGAL::Exact_intersections_tag                                  Itag;
typedef CGAL::No_intersection_tag                                      Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>       CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT>                    CDTplus;
typedef K::Point_2                                                     Point;
typedef CGAL::Spatial_sort_traits_adapter_2<K,Point*>                  Search_traits;
typedef CGAL::Polygon_2<K>                                             Polygon_2;

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

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void mark_domains( CDT& cdt );


void mark_domains(  CDT&                   ct,
                    CDT::Face_handle       start,
                    int                    index,
                    std::list<CDT::Edge>&  border );


void insert_polygon(    CDT&                cdt,
                        const Polygon_2&    polygon);

bool check_winding(     std::vector<Point>          &points,
                        std::vector<unsigned int>   idx,
                        std::vector<int>            breaks);

float polygon_winding_order( std::vector<Point> &points, int start, int end );


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


void constrainedDelaunayTriangulation(  std::vector<Point>                      points,
                                        std::vector<Point>                      constraines,
                                        std::vector<std::vector<unsigned int> > &idx){


    auto dist_func = [](Point a, Point b){
        return std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2) < 0.5;
    };

    if(points.size() < 3) return;
    int size = points.size();
    int points_size = size;
    for(auto p : constraines){
        points.push_back(p);
    }

    Polygon_2 polygon;
    std::vector<Polygon_2> polygons;
    polygons.push_back(polygon);
    std::vector<int> breaks;
    int k = 0;
    int startP = 0;
    breaks.push_back(startP);
    breaks.push_back(points_size);
    for (size_t i = 0; i < constraines.size()-1; i++) {
        if(dist_func(constraines[i], constraines[i+1])){
            if(i == startP){
                polygons[k].push_back(constraines[startP]);
            }
            polygons[k].push_back(constraines[i+1]);
        } else {
            k++;
            startP = i+1;
            breaks.push_back(startP + points_size);
            polygons.push_back(polygon);
            // break;
        }
    }

    std::cout << "size: " << points_size << std::endl;
    for(auto p: breaks){
        std::cout << p << ", ";
    }
    std::cout << " " << std::endl;


    CDT cdt;


    // Insert the points.
    insert_with_info(cdt, points.begin(),points.end());

    int hull_close = 0;

    // make it conforming Delaunay
    // CGAL::make_conforming_Delaunay_2(cdt);

    // Clear Idx if not empty and reserve approximate size;
    if(idx.size() != 0) idx.clear();
    idx.reserve( cdt.number_of_faces() );


    // Fill idx with indices.
    std::vector<unsigned int> poly(3);

    for(CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
        fit != cdt.finite_faces_end(); ++fit) {

        // if ( !fit->info().in_domain() ) continue;

        CDT::Face_handle face = fit;

        poly[0] = face->vertex(0)->info();
        poly[1] = face->vertex(1)->info();
        poly[2] = face->vertex(2)->info();

        if( check_winding(points, poly, breaks) ){
            idx.push_back(poly);
        }
    }
    for(auto p : constraines){
        std::cout << p << std::endl;
    }
}



void
mark_domains(CDT& ct,
             CDT::Face_handle start,
             int index,
             std::list<CDT::Edge>& border )
{
  if(start->info().nesting_level != -1){
    return;
  }
  std::list<CDT::Face_handle> queue;
  queue.push_back(start);

  while(! queue.empty()){
    CDT::Face_handle fh = queue.front();
    queue.pop_front();
    if(fh->info().nesting_level == -1){
      fh->info().nesting_level = index;
      for(int i = 0; i < 3; i++){
        CDT::Edge e(fh,i);
        CDT::Face_handle n = fh->neighbor(i);
        if(n->info().nesting_level == -1){
          if(ct.is_constrained(e)) border.push_back(e);
          else queue.push_back(n);
        }
      }
    }
  }
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void
mark_domains(CDT& cdt)
{
  for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
    it->info().nesting_level = -1;
  }

  int index = 0;
  std::list<CDT::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), index++, border);
  while(! border.empty()){
    CDT::Edge e = border.front();
    border.pop_front();
    CDT::Face_handle n = e.first->neighbor(e.second);
    if(n->info().nesting_level == -1){
      mark_domains(cdt, n, e.first->info().nesting_level+1, border);
    }
  }
}


void insert_polygon(CDT& cdt,const Polygon_2& polygon){
  if ( polygon.is_empty() ) return;
  bool first = true;
  CDT::Vertex_handle v_prev=cdt.insert(*CGAL::cpp11::prev(polygon.vertices_end()));
  for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin();
       vit!=polygon.vertices_end();++vit)
  {
    CDT::Vertex_handle vh=cdt.insert(*vit);
    cdt.insert_constraint(vh,v_prev);
    std::cout << *v_prev << " -- " << *vh << std::endl;
    v_prev=vh;
  }
}


float polygon_winding_order( std::vector<Point> &points, int start, int end ){
    if( start > end || end > points.size()){
        std::cerr << "Illeagal inputs for polygon_winding order!!" << std::endl;
        exit(0);
    }

    float sum = 0;
    for (size_t i = start; i < end; i++) {
        sum +=  (points[i+1][0] - points[i][0]) * (points[i+1][1] + points[i][1]);
    }
    sum += (points[start][0] - points[end-1][0]) * (points[start][1] + points[end-1][1]);

    return sum;

}


bool check_winding(std::vector<Point> &points, std::vector<unsigned int> idx, std::vector<int> breaks){
    // Start by finding number of points in external alpha shape.

    if(breaks.size() <= 1 ) return true;

    breaks.push_back(points.size());
    bool external = true;
    float clockwise = 0;

    int count = 0;
    // std::vector<int> count(breaks.size()-1);
    for(auto i = 1; i < breaks.size() - 1; ++i){
        count = 0;
        for(auto p : idx){
            if( p >= breaks[i] && p < breaks[i+1] ){
                count++;
            }
        }
        if( count == 3 ){
            clockwise = polygon_winding_order(points, breaks[i], breaks[i+1]);
            external = i == 1;
            break;
        }
    }

    // Check if sufficient number of boundary points.
    if( count < 3 ) return true;

    auto triangle_clockwise = [](Point a, Point b, Point c){
        float sum = 0;
        sum +=  (b[0] - a[0]) * (b[1] + a[1]);
        sum +=  (c[0] - b[0]) * (c[1] + b[1]);
        sum +=  (a[0] - c[0]) * (a[1] + c[1]);
        return sum > 0;
    };

    std::sort(idx.begin(), idx.end());
    // Eigen::Vector3f a;
    // Eigen::Vector3f b;
    // a[0] = points[idx[1]][0] - points[idx[0]][0];
    // a[1] = points[idx[1]][1] - points[idx[0]][1];
    // b[0] = points[idx[2]][0] - points[idx[0]][0];
    // b[1] = points[idx[2]][1] - points[idx[0]][1];
    // b[2] = 0;
    //
    // bool order = (a.cross(b))[2] <= 0;

    bool order = triangle_clockwise(points[idx[0]], points[idx[1]], points[idx[2]]);

    bool out = ((clockwise > 0) == order) == external;
    if(!out){

        std::cout << "-------------" << std::endl;
        std::cout << points[idx[0]] << ", " << points[idx[1]] << ", " << points[idx[2]] << std::endl;
        std::cout << idx[0] << ", " << idx[1] << ", " << idx[2] << std::endl;
        std::cout << "breaks: " << std::endl;
        std::cout << "clockwise: " << clockwise << std::endl;
        for(auto i : breaks){
            std::cout << i << ",";
        }
        std::cout << " " << std::endl;

        std::cout << "E: " << external << "  C: " << clockwise << "  P: " << order << "   Out: " << out << std::endl;
    }

    return (clockwise == order) == external;

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
