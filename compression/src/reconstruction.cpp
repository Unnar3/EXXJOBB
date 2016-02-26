#include <reconstruction/reconstruction.h>

namespace EXX{

void reconstruction::constrainedDelaunayTriangulation(  std::vector<Point>                      points,
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
        if( reconstruction::distanceBetweenPoints(constraines[i], constraines[i+1]) > 0.3 ){
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
        clockwise[i-1] = reconstruction::isPolygonClockwise(points, breaks[i], breaks[i+1]);
        if( std::abs(reconstruction::polygonArea(points, breaks[i], breaks[i+1])) > maxarea ){
            maxarea = reconstruction::polygonArea(points, breaks[i], breaks[i+1]);
            maxareaint = i;
        }
    }



    CDT cdt;

    // Insert the points.
    reconstruction::insert_with_info(cdt, points.begin(),points.end());

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

        if( reconstruction::check_winding(points, poly, breaks, clockwise, log) ){
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

float reconstruction::polygonArea( std::vector<Point> &points, int start, int end ){
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



bool reconstruction::isPolygonClockwise( std::vector<Point> &points, int start, int end ){
    return reconstruction::polygonArea(points, start, end) > 0;
}

bool reconstruction::isPolygonClockwise( Point a, Point b, Point c){
    float sum = 0;
    sum +=  (b[0] - a[0]) * (b[1] + a[1]);
    sum +=  (c[0] - b[0]) * (c[1] + b[1]);
    sum +=  (a[0] - c[0]) * (a[1] + c[1]);
    return sum > 0;
}


bool reconstruction::check_winding(std::vector<Point> &points, std::vector<unsigned int> idx, std::vector<int> breaks, std::vector<bool> clockwisev, bool log){
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

    bool order = reconstruction::isPolygonClockwise(points[idx[0]], points[idx[1]], points[idx[2]]);

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
void reconstruction::insert_with_info(CDT& cdt, InputIterator first,InputIterator last)
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


float reconstruction::distanceBetweenPoints(Point a, Point b){
    return std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2);
}



}
