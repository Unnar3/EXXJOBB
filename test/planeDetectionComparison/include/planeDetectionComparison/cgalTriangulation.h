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
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/intersections.h>

#include <PointTypes/surfel_type.h>
#include <typedefs/typedef.h>
#include <Eigen/Dense>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel            K;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K>   Vb;
typedef CGAL::Triangulation_face_base_2<K>         Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>            Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                    TDS;
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


// Inserts points into the constrained Delaunay triangulation and
// assigns it the correct index.
template <class InputIterator>
void insert_with_info(CDT& cdt, InputIterator first,InputIterator last);


// Returns the euclidean distance between two Points.
// INPUTS:
//  a   :   Point 1.
//  b   :   Point 2.
// OUTPUTS:
//  float Ueclidean distance between a and b.
//  out = sqrt( (b[0] - b[0])² + (b[1] - b[1])² )
float distanceBetweenPoints(Point a, Point b);




// Checks the winding order of a single triangle and checks if it is inside or
// outside the respective alpha shape.
// points : vector containing all points in the plane.
// idx    : Vector of size 3 containing idices to three points forming a triangle.
// breaks : classifies points,
//              [ breaks[0]-breaks[1] )     -> interior points.
//              [ breaks[1]-breaks[2] )     -> interior points.
//              [ breaks[3]-breaks[4] )     -> Points outlining first hole.
//              [ breaks[N-1]-breaks[N] )   -> Points outlining last hole.
bool check_winding(     std::vector<Point>          &points,
                        std::vector<unsigned int>   idx,
                        std::vector<int>            breaks,
                        std::vector<bool>           clockwisev,
                        bool log);

// Given a list of points, a starting index and end index the algorithm checks
// the winding order of the polygon.
// It assumes the the points are already ordered around the plane.
bool isPolygonClockwise( std::vector<Point> &points, int start, int end );

// Checks if a given triangle is clockwise or not.
bool isPolygonClockwise( Point a, Point b, Point c);

// Given a list of points, a starting index and end index the algorithm finds
// the area of the triangle.
// It assumes the the points are already ordered around the plane.
float polygonArea(std::vector<Point> &points, int start, int end);

// Creates a Delaunay triangulation from a set of 2D points.
// INPUTS:
//  points      : Vector of interior points.
//  constraines : Vector containing the convex hull and any holes.
// OUTPUTS:
//  idx         : Vector containing indexes to points forming triangles,
//                  std::vector<unsigned int> a(3) = idx[0];
void constrainedDelaunayTriangulation(  std::vector<Point>                      points,
                                        std::vector<Point>                      constraines,
                                        std::vector<std::vector<unsigned int> > &idx,
                                        bool log);

// Takes in PointCloud where all points are lying on a plane and projects that plane to 2d.
// INPUTS:
//  plane       : 3D point cloud containing interior points.
//  boundary    : 3D point cloud containing boundary points.
//  coeff       : plane equation describing the points.
//                  coeff[0]*x + coeff[1]*y + coeff[2]*z + coeff[3] = 0
// OUTPUTS:
//  plane_2d    : vector containing the 2d interior points.
//  boundary_2d : vector containing the 2d boundary points.
template <class T>
void pclPlaneToCGAL(    typename pcl::PointCloud<T>::Ptr    plane,
                        typename pcl::PointCloud<T>::Ptr    boundary,
                        pcl::ModelCoefficients::Ptr         coeff,
                        std::vector<Point >               & plane_2d,
                        std::vector<Point >               & boundary_2d);


/////////////////////////////////////////////////////////////
// Implementation
/////////////////////////////////////////////////////////////


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
                                        std::vector<std::vector<unsigned int> > &idx,
                                        bool log){

    if(points.size() < 3) return;
    int size = points.size();
    int points_size = size;
    for(auto p : constraines){
        points.push_back(p);
    }

    int startP = 0;
    std::vector<int> breaks;
    breaks.push_back(startP);
    breaks.push_back(points_size);
    for (size_t i = 0; i < constraines.size()-1; i++) {
        if( distanceBetweenPoints(constraines[i], constraines[i+1]) > 0.3 ){
            startP = i+1;
            breaks.push_back(startP + points_size);
        }
    }
    breaks.push_back(points.size());

    std::vector<bool> clockwise(breaks.size()-2);

    // std::cout << "breaks: ";
    // for(auto p: breaks){
    //     std::cout << p << ", ";
    // }
    // std::cout << "" << std::endl;
    float maxarea = 0;
    int maxareaint = 0;
    for (size_t i = 1; i < breaks.size()-1; i++) {
        clockwise[i-1] = isPolygonClockwise(points, breaks[i], breaks[i+1]);
        if( std::abs(polygonArea(points, breaks[i], breaks[i+1])) > maxarea ){
            maxarea = polygonArea(points, breaks[i], breaks[i+1]);
            maxareaint = i;
        }
    }



    CDT cdt;

    // Insert the points.
    insert_with_info(cdt, points.begin(),points.end());

    // make it conforming Delaunay
    // CGAL::make_conforming_Delaunay_2(cdt);

    // Clear Idx if not empty and reserve approximate size;
    if(idx.size() != 0) idx.clear();
    idx.reserve( cdt.number_of_faces() );

    // Vector containing indices to a single triangle.
    std::vector<unsigned int> poly(3);

    log = false;
    for(CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
        fit != cdt.finite_faces_end(); ++fit) {

        CDT::Face_handle face = fit;

        poly[0] = face->vertex(0)->info();
        poly[1] = face->vertex(1)->info();
        poly[2] = face->vertex(2)->info();

        if( check_winding(points, poly, breaks, clockwise, log) ){
            idx.push_back(poly);
        }
    }
    // if(maxareaint != 1){
    //     std::cout << "whaaaaaaaaaat:  " << maxareaint << std::endl;
    //     std::cout << "breaks : ";
    //     for(auto p : breaks){
    //         std::cout << p << ", ";
    //     }
    //     std::cout << "" << std::endl;
    // } else {
    //     std::cout << "ok" << std::endl;
    // }
    // std::cout << "clockwise : ";
    // for(auto p : clockwise){
    //     std::cout << p << ", ";
    // }
    // std::cout << "" << std::endl;
}

float polygonArea( std::vector<Point> &points, int start, int end ){
    if( start > end || end > points.size()){
        std::cerr << "Illeagal inputs for polygonArea!!" << std::endl;
        exit(0);
    }

    float sum = 0;
    for (size_t i = start; i < end; i++) {
        sum +=  (points[i+1][0] - points[i][0]) * (points[i+1][1] + points[i][1]);
    }
    sum += (points[start][0] - points[end-1][0]) * (points[start][1] + points[end-1][1]);

    return sum;

}



bool isPolygonClockwise( std::vector<Point> &points, int start, int end ){
    return polygonArea(points, start, end) > 0;
}

bool isPolygonClockwise( Point a, Point b, Point c){
    float sum = 0;
    sum +=  (b[0] - a[0]) * (b[1] + a[1]);
    sum +=  (c[0] - b[0]) * (c[1] + b[1]);
    sum +=  (a[0] - c[0]) * (a[1] + c[1]);
    return sum > 0;
}


bool check_winding(std::vector<Point> &points, std::vector<unsigned int> idx, std::vector<int> breaks, std::vector<bool> clockwisev, bool log){
    // Start by finding number of points in external alpha shape.

    if(breaks.size() <= 1 ) return true;

    // breaks.push_back(points.size());
    bool external = true;
    bool clockwise = 0;

    int count = 0;
    for(auto i = 1; i < breaks.size() - 1; ++i){
        count = 0;
        for(auto p : idx){
            if( p >= breaks[i] && p < breaks[i+1] ){
                count++;
            }
        }
        if( count == 3 ){
            // clockwise = isPolygonClockwise(points, breaks[i], breaks[i+1]);
            clockwise = clockwisev[i-1];

            external = i == 1;
            break;
        }
    }

    // Check if sufficient number of boundary points.
    if( count < 3 ) return true;

    std::sort(idx.begin(), idx.end());

    bool order = isPolygonClockwise(points[idx[0]], points[idx[1]], points[idx[2]]);

    // if(log){
    //     std::cout << "----------------" << std::endl;
    //     std::cout << idx[0] << ", " << idx[1] << ", " << idx[2]  << std::endl;
    //     std::cout << "clockwise : ";
    //     for(auto p : clockwisev){
    //         std::cout << p << ", ";
    //     }
    //     std::cout << "" << std::endl;
    //     bool out = (clockwise == order) == external;
    //     std::cout << "E: " << external << "  C: " << clockwise << "  P: " << order << "  Out: " << out << std::endl;
    //
    // }
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


float distanceBetweenPoints(Point a, Point b){
    return std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2);
}

#endif
