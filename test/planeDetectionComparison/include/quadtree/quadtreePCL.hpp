#include <pcl/common/transforms.h>
#include <algorithm>
#include <pcl/common/common.h>

template <typename PointT>
void QuadTreePCL<PointT>::insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary){
    // Convert to cgal polygon
    if(boundary->size() == 0) return;

    QuadTreePCL<PointT>::rotateToAxis(boundary);
    PointT min, max;
    pcl::getMinMax3D(*boundary, min, max);

    std::cout << "min: " << min << std::endl;
    std::cout << "max: " << max << std::endl;

    if(!inserted_){
        // Determin the inital size of the quadtree
        z_ = boundary->points[0].z;
        float x = QuadTreePCL<PointT>::roundDown(min.x);
        float y = QuadTreePCL<PointT>::roundDown(min.y);
        float width = std::max(QuadTreePCL<PointT>::roundUp(max.x - x), QuadTreePCL<PointT>::roundUp(max.y - y));

        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "width: " << width << std::endl;

        // initialize the quadtree
        quad = QuadTree(1,width,x,y);
        quad.setMaxWidth(0.05);
    }

    Polygon polygon;
    for (size_t i = 0; i < boundary->size(); i++) {
        polygon.push_back(Point(boundary->points[i].x, boundary->points[i].y));
    }
    std::vector<Polygon> polygons;
    // if(!polygon.is_simple()){
    std::cout << "Polygon isn't simple, returning" << std::endl;
    QuadTreePCL<PointT>::makePolygonSimple(polygon, polygons);
    polygon = polygons[0];
        // return;
    // }
    CGAL::Orientation orientation = polygon.orientation();
    if(orientation == CGAL::NEGATIVE){
        std::cout << "need to invert polygon" << std::endl;
        std::reverse(polygon.vertices_begin(), polygon.vertices_end());
    }
    std::cout << "inserting" << std::endl;
    quad.insertBoundary(polygon);
    for(size_t i = 1; i < polygons.size(); ++i){
        orientation = polygons[i].orientation();
        if(orientation == CGAL::NEGATIVE){
            std::cout << "need to invert polygon" << std::endl;
            std::reverse(polygons[i].vertices_begin(), polygons[i].vertices_end());
        }
        quad.insertHole(polygons[i]);
    }
    std::cout << "inserted" << std::endl;
    inserted_ = true;
}

template <typename PointT>
template <typename T>
void QuadTreePCL<PointT>::createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices){

    std::vector<QuadTree::Cell> cells;
    quad.extractCells(cells);

    cloud->reserve(cells.size() * 4);
    vertices.reserve(cells.size() * 2);

    T p;
    p.z = z_;
    pcl::Vertices vert;
    vert.vertices.resize(3);
    for(auto c : cells){
        int size = cloud->size();
        if(c.cellType == QuadTree::Interior){
            p.r = 255; p.g = 255; p.b = 255;
        } else if(c.cellType == QuadTree::Boundary){
            p.r = 255; p.g = 255; p.b = 0;
        } else {
            p.r = 255; p.g = 0; p.b = 255;
        }
        p.x = c.x;
        p.y = c.y;
        cloud->push_back(p);
        p.x = c.x + c.width;
        p.y = c.y;
        cloud->push_back(p);
        p.x = c.x + c.width;
        p.y = c.y + c.width;
        cloud->push_back(p);
        p.x = c.x;
        p.y = c.y + c.width;
        cloud->push_back(p);

        vert.vertices[0] = size;
        vert.vertices[1] = size + 1;
        vert.vertices[2] = size + 2;
        vertices.push_back(vert);

        vert.vertices[0] = size;
        vert.vertices[1] = size + 3;
        vert.vertices[2] = size + 2;
        vertices.push_back(vert);

    }
    QuadTreePCL<PointT>::rotateFromAxis(cloud);
}

template <typename PointT>
void QuadTreePCL<PointT>::setNormal(Eigen::Vector3f normal){
    normal_ = normal;
    Eigen::Vector3f znorm;
    znorm << 0,0,1;
    quaternion_ = Eigen::Quaternion<float>::FromTwoVectors(normal_, znorm);
    normalVectorSet = true;
}

template <typename PointT>
void QuadTreePCL<PointT>::rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud){

    if(quaternion_.x() == 0 &&quaternion_.y() == 0 && quaternion_.z() == 0){
        // no rotation needed.
        return;
    }
    // first check if we need to rotate
    Eigen::Affine3f rot(quaternion_.matrix());
    pcl::transformPointCloud (*cloud, *cloud, rot);

}

template <typename PointT>
void QuadTreePCL<PointT>::rotateFromAxis(typename pcl::PointCloud<PointT>::Ptr cloud){

    if(quaternion_.x() == 0 &&quaternion_.y() == 0 && quaternion_.z() == 0){
        // no rotation needed.
        return;
    }
    // first check if we need to rotate
    Eigen::Affine3f rot(quaternion_.conjugate().matrix());
    pcl::transformPointCloud (*cloud, *cloud, rot);

}
template <typename PointT>
bool QuadTreePCL<PointT>::makePolygonSimple(Polygon &polygon, std::vector<Polygon> &polygons){

    float dist_threshold = 0.5*0.5;
    auto squared_point_distance = [dist_threshold](Point a, Point b){
        return std::pow((b[0]-a[0]),2) + std::pow((b[1]-a[1]),2) < dist_threshold;
    };

    // Very stupid test based on distance.
    int last_idx = 0;
    for(size_t i = 0; i < polygon.size() - 1; ++i){
        if( !squared_point_distance(polygon[i], polygon[i+1]) ){
            std::cout << "distance to great i: " << i << std::endl;
            std::cout << "point a: " << polygon[i] << std::endl;
            std::cout << "point b: " << polygon[i+1] << std::endl;
            if(squared_point_distance(polygon[i], polygon[last_idx])){
                // we have closed the polygon
                std::cout << "close to last" << std::endl;
                Polygon polygon_tmp;
                for(size_t j = last_idx; j < i+1; ++j){
                    polygon_tmp.push_back(polygon[j]);
                }
                polygons.push_back(polygon_tmp);
                Polygon hole_tmp;
                for(size_t j = i+1; j < polygon.size(); ++j){
                    hole_tmp.push_back(polygon[j]);
                }
                polygons.push_back(hole_tmp);
            }
        }
    }
    if(polygons.size() == 0){
        polygons.push_back(polygon);
    }

}
