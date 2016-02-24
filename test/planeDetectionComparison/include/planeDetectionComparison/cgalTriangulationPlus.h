#ifndef CGALTRIANGULATIONPLUS_H
#define CGALTRIANGULATIONPLUS_H

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Polygon_2.h>

#include <cassert>
#include <iostream>

namespace triangulationPlus{

typedef CGAL::Exact_predicates_exact_constructions_kernel K;

typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K>    Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>                  Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                     TDS;
typedef CGAL::Exact_intersections_tag                                   Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K,TDS,Itag>          CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT>                     CDTplus;
typedef CGAL::Polygon_2<K>                                              Polygon_2;
typedef CDTplus::Point                                                  Point;

void insert_polygon_constriant(CDT& cdt, const Polygon_2& polygon){

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


void cdtp(   std::vector<Point>                      points,
                std::vector<Point>                      constraines,
                std::vector<std::vector<unsigned int> > &idx){

    // blah
    // auto dist_func = [](Point a, Point b){
    //     return std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2) < 0.5;
    // };

    // Polygon_2 polygon;
    // std::vector<Polygon_2> polygons;
    // polygons.push_back(polygon);
    // std::vector<int> breaks;
    // int k = 0;
    // int startP = 0;
    // breaks.push_back(startP);
    // for (size_t i = 0; i < constraines.size()-1; i++) {
        // if(dist_func(constraines[i], constraines[i+1])){
        //     if(i == startP){
        //         polygons[k].push_back(constraines[startP]);
        //     }
        //     polygons[k].push_back(constraines[i+1]);
        // } else {
        //     k++;
        //     startP = i+1;
        //     breaks.push_back(startP);
        //     polygons.push_back(polygon);
        //     // break;
        // }
    // }


    CDTplus cdt;

    // if(points.size() < 3) return;
    // int size = points.size();
    // int points_size = size;
    //
    // // Insert the points.
    // insert_with_info(cdt, points.begin(),points.end());
    //
    // // Insert the constraints (boundary).
    // int hull_close = 0;
    // if(constraines.size() > 1){
    //     // for(auto pol : polygons){
    //     //     if(pol.size() > 5){
    //     //         insert_polygon(cdt, pol);
    //     //     }
    //     // }
    //     //Mark facets that are inside the domain bounded by the polygon
    //     insert_polygon(cdt, polygons[0]);
    //     mark_domains(cdt);
    //
    //
    //     // put the correct index on the constraine points.
    //     auto it = cdt.vertices_begin();
    //     std::advance(it, size);
    //     for(; it != cdt.vertices_end(); ++it){
    //         it->info() = size;
    //         // std::cout << size << std::endl;
    //         size++;
    //     }
    // }

    // make it conforming Delaunay
    // CGAL::make_conforming_Delaunay_2(cdt);

    // Clear Idx if not empty and reserve approximate size;
    // if(idx.size() != 0) idx.clear();
    // idx.reserve( cdt.number_of_faces() );


    // Fill idx with indices.
    // std::vector<unsigned int> poly(3);

    // auto check_boundary = [points_size](std::vector<unsigned int> vec){
    //     for(auto point : vec){
    //         if( point < points_size ) return true;
    //     }
    //     return false;
    // };

    // auto func_closness = [](std::vector<unsigned int> vec, int idx){
    //     return std::count_if(vec.begin(), vec.end(), [idx](int value){
    //         if( idx == value || idx == value+1) return true;
    //         // if(value == 0) return true;
    //         return false;
    //     }) > 1;
    // };

    // for(CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
    //     fit != cdt.finite_faces_end(); ++fit) {
    //
    //     if ( !fit->info().in_domain() ) continue;
    //
    //     CDT::Face_handle face = fit;
    //
    //     poly[0] = face->vertex(0)->info();
    //     poly[1] = face->vertex(1)->info();
    //     poly[2] = face->vertex(2)->info();
    //
    //     if(func_closness(poly, breaks[1])){
    //         std::cout << "break: " << breaks[1] << ", ";
    //         std::cout << poly[0] << ", " << poly[1] << ", " << poly[2] << std::endl;
    //         continue;
    //     }
    //
    //     // if(poly[0] == points_size || poly[1] == points_size || poly[2] == points_size){
    //     //     if(poly[0] == size-1 || poly[1] == size-1 || poly[2] == size-1){
    //     //         continue;
    //     //     }
    //     // }
    //
    //     if( check_boundary(poly) ){
    //         idx.push_back(poly);
    //     }
    // }


}


// int
// main( )
// {
//   CDTplus cdt;
//   std::cout  << "Inserting a grid 5 x 5 of constraints " << std::endl;
//   for (int i = 1; i < 6; ++i)
//     cdt.insert_constraint( Point(0,i), Point(6,i));
//   for (int j = 1; j < 6; ++j)
//     cdt.insert_constraint( Point(j,0), Point(j,6));
//
//   assert(cdt.is_valid());
//   int count = 0;
//   for (CDTplus::Subconstraint_iterator scit = cdt.subconstraints_begin();
//        scit != cdt.subconstraints_end();
//        ++scit)  ++count;
//   std::cout << "The number of resulting constrained edges is  "
// 	    <<  count << std::endl;
//   return 0;
// }

}
# endif
