#ifndef EXX_RECONSTRUCTION_H
#define EXX_RECONSTRUCTION_H

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
// #include <typedefs/typedef.h>
#include <Eigen/Dense>
#include <vector>


typedef CGAL::Exact_predicates_inexact_constructions_kernel            K;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K>   Vb;
typedef CGAL::Triangulation_face_base_2<K>                             Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>            Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                    TDS;
typedef CGAL::No_intersection_tag                                      Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>       CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT>                    CDTplus;
typedef K::Point_2                                                     Point;
typedef CGAL::Spatial_sort_traits_adapter_2<K,Point*>                  Search_traits;
typedef CGAL::Polygon_2<K>                                             Polygon_2;


namespace EXX{

    class reconstruction{

    public:
        reconstruction(){}
        ~reconstruction(){}

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


    private:

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

    };

}

#include "impl/reconstruction.hpp"

#endif
