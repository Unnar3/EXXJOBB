#include <pcl/common/transforms.h>

template <typename PointT>
void QuadTreePCL<PointT>::insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary){
    // Convert to cgal polygon

    QuadTreePCL<PointT>::rotateToAxis(boundary);

    Polygon polygon;
    for (size_t i = 0; i < boundary->size(); i++) {
        polygon.push_back(Point(boundary->points[i].x, boundary->points[i].y));
    }
    std::cout << "muhahaah" << std::endl;
    quad.insertBoundary(polygon);
    std::cout << "hahahahh" << std::endl;
}

template <typename PointT>
template <typename T>
void QuadTreePCL<PointT>::createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices){

    std::vector<QuadTree::Cell> cells;
    quad.extractCells(cells);

    cloud->reserve(cells.size() * 4);
    vertices.reserve(cells.size() * 2);

    T p;
    p.z = 0;
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
}

template <typename PointT>
void QuadTreePCL<PointT>::setNormal(Eigen::Vector3f normal){
    normal_ = normal;
    Eigen::Vector3f znorm;
    znorm << 0,0,1;
    quaternion = Eigen::Quaternion<float>::FromTwoVectors(normal_, znorm);
    normalVectorSet = true;

    std::cout << quaternion.x() << std::endl;
    std::cout << quaternion.y() << std::endl;
    std::cout << quaternion.z() << std::endl;
    std::cout << quaternion.w() << std::endl;
}

template <typename PointT>
void QuadTreePCL<PointT>::rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud){

    if(quaternion.x() == 0 &&quaternion.y() == 0 && quaternion.z() == 0){
        // no rotation needed.
        return;
    }
    // first check if we need to rotate
    Eigen::Affine3f rot(quaternion.matrix());
    pcl::transformPointCloud (*cloud, *cloud, rot);

}
