#include <ros/ros.h>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>
#include <planeDetectionComparison/cgalTriangulation.h>

#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                       Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                    Delaunay;
typedef Kernel::Point_2                                                Point;

typedef CGAL::Constrained_triangulation_face_base_2<Kernel>              Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                 TDS;
typedef CGAL::Exact_predicates_tag                                  Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, Itag>    CDT;
typedef CGAL::Spatial_sort_traits_adapter_2<Kernel,Point*> Search_traits;

// template <class InputIterator>
// void insert_with_info(CDT& cdt, InputIterator first,InputIterator last)
// {
//   std::vector<std::ptrdiff_t> indices;
//   std::vector<Point> points;
//   std::ptrdiff_t index=0;
//
//   for (InputIterator it=first;it!=last;++it){
//     points.push_back( *it);
//     indices.push_back(index++);
//   }
//
//   CGAL::spatial_sort(indices.begin(),indices.end(),Search_traits(&(points[0]),cdt.geom_traits()));
//
//   CDT::Vertex_handle v_hint;
//   CDT::Face_handle hint;
//   for (typename std::vector<std::ptrdiff_t>::const_iterator
//     it = indices.begin(), end = indices.end();
//     it != end; ++it){
//     v_hint = cdt.insert(points[*it], hint);
//     if (v_hint!=CDT::Vertex_handle()){
//       v_hint->info()=*it;
//       hint=v_hint->face();
//     }
//   }
// }

// template <class InputIterator>
// void insert_constraint_with_info(CDT& cdt, InputIterator first,InputIterator last,int idx)
// {
//   std::vector<std::ptrdiff_t> indices;
//   std::vector<Point> points;
//   std::ptrdiff_t index=0;
//
//   for (InputIterator it=first;it!=last;++it){
//     points.push_back( *it);
//     indices.push_back(index++);
//   }
//
//   CGAL::spatial_sort(indices.begin(),indices.end(),Search_traits(&(points[0]),cdt.geom_traits()));
//
//   CDT::Vertex_handle v_hint;
//   CDT::Face_handle hint;
//   for (typename std::vector<std::ptrdiff_t>::const_iterator
//     it = indices.begin(), end = indices.end();
//     it != end; ++it){
//     v_hint = cdt.insert_constraint(points[*it], hint);
//     if (v_hint!=CDT::Vertex_handle()){
//       v_hint->info()=*it+idx;
//       hint=v_hint->face();
//     }
//   }
// }

int main(int argc, char **argv) {
//
    ros::init(argc, argv,"test");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ros::Rate loop_rate(10);




  std::vector<Point> points;
  points.push_back(Point(1,1));
  points.push_back(Point(2,1));
  points.push_back(Point(3,1));
  points.push_back(Point(4,1));
  points.push_back(Point(5,1));
  points.push_back(Point(1,2));
  points.push_back(Point(2,2));
  points.push_back(Point(3,2));
  points.push_back(Point(4,2));
  points.push_back(Point(5,2));
  points.push_back(Point(1,3));
  points.push_back(Point(2,3));
  points.push_back(Point(1,4));
  points.push_back(Point(2,4));
  points.push_back(Point(4,3));
  points.push_back(Point(5,3));
  points.push_back(Point(4,4));
  points.push_back(Point(5,4));

  std::vector<Point> constrained;
  constrained.push_back(Point(2.5,4));
  constrained.push_back(Point(2.5,3));
  constrained.push_back(Point(3,3));
  constrained.push_back(Point(3.5,3));
  constrained.push_back(Point(3.5,4));

  std::vector<std::vector<unsigned int> > idx;
  constrainedDelaunayTriangulation(points, constrained, idx);


  for(auto poly : idx){
      std::cout << poly[0] << ", " << poly[1] << ", " << poly[2] << std::endl;
      std::cout << "-----------------------" << std::endl;
  }

  // Delaunay triangulation;
  // triangulation.insert(points.begin(),points.end());
  //
  // CDT cdt;
  // CDT::Vertex_handle verttest;
  // insert_with_info(cdt, points.begin(),points.end());
  // for (size_t i = 0; i < constrained.size()-1; i++) {
  //     cdt.insert_constraint(constrained[i], constrained[i+1]);
  // }
  //
  // auto it = cdt.vertices_begin();
  // std::advance(it, points.size());
  // int size = points.size();
  // for(; it != cdt.vertices_end(); ++it){
  //     it->info() = size;
  //     std::cout << it->info() << std::endl;
  //     size++;
  // }
  //
  // // insert_constraint_with_info(cdt, constrained.begin(), constrained.end(), points.size());
  // // cdt.insert_constraint(Point(2.5, 0), Point(2.5,2));
  //
  // for(CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
  //     fit != cdt.finite_faces_end(); ++fit) {
  //
  //   CDT::Face_handle face = fit;
  //   // std::cout << "Triangle:\t" << cdt.triangle(face) << std::endl;
  //   std::cout << "Vertex 0:\t" << face->vertex(0)->info() << ", ";
  //   std::cout << face->vertex(1)->info() << ", ";
  //   std::cout << face->vertex(2)->info() << std::endl;
  //   std::cout << "----------------------------" << std::endl;
  // }


}
