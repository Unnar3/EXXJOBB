
template <typename PointT>
QuadTreePCL<PointT>::QuadTreePCL(){
    quad = QuadTree(1,10,0,0);
}

template <typename PointT>
QuadTreePCL<PointT>::QuadTreePCL(int level, float width, float x, float y){
    quad = QuadTree(level, width, x, y);
}
//
// template <typename PointT>
// void QuadTreePCL<PointT>::insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary){
//     // Convert to cgal polygon
//
//     Polygon polygon;
//     for (size_t i = 0; i < boundary->size(); i++) {
//         polygon.push_back(Point(boundary->points[i].x, boundary->points[i].y));
//     }
//     QuadTree<PointT>::insertBoundary(polygon);
// }
//
// template <typename PointT>
// template <typename T>
// void QuadTreePCL<PointT>::createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices){
//     if(qt::isLeaf()){
//         std::cout << "Only one cell, returning empty cloud!" << std::endl;
//         return;
//     } else {
//         // loop through all nodes.
//         typename std::vector<QuadTree<PointT> >::iterator n = qt::nodesBegin();
//         typename std::vector<QuadTree<PointT> >::iterator end = qt::nodesEnd();
//         for(; n < end; n++){
//             qtpcl::createMesh<T>(*n, cloud, vertices);
//         }
//
//         // for(auto &n : nodes){
//         // }
//     }

    // if(qt::isLeaf()){
    //     int size = cloud->points.size();
    //     // if(cellType != Exterior){
    //     if(qt::cellType() != qt::Parent && qt::cellType() != qt::Exterior && qt::cellType() != qt::Hole){
    //
    //         pcl::Vertices vert;
    //         vert.vertices.resize(3);
    //         vert.vertices[0] = size;
    //         vert.vertices[1] = size + 3;
    //         vert.vertices[2] = size + 2;
    //         vertices.push_back(vert);
    //
    //         vert.vertices[0] = size;
    //         vert.vertices[1] = size + 1;
    //         vert.vertices[2] = size + 3;
    //         vertices.push_back(vert);
    //
    //         cloud->reserve(size + 4);
    //         // add the 4 corners.
    //
    //         T p; p.z = 0;
    //
    //         // if(qt::pointHasColor(p)){
    //         //     qtpcl::colorBasedOnCellType(p);
    //         // }
    //
    //         if(qt::cellType() == qt::Interior){
    //             p.r = 255; p.g = 255; p.b = 255;
    //         } else if(qt::cellType() == qt::Boundary){
    //             p.r = 255; p.g = 255; p.b = 0;
    //         } else{
    //             p.r = 255; p.g = 0; p.b = 255;
    //         }
    //
    //         p.x = qt::x(); p.y = qt::y();
    //         cloud->push_back(p);
    //
    //         p.x = qt::x() + qt::width(); p.y = qt::y();
    //         cloud->push_back(p);
    //
    //         p.x = qt::x(); p.y = qt::y() + qt::width();
    //         cloud->push_back(p);
    //
    //         p.x = qt::x() + qt::width(); p.y = qt::y() + qt::width();
    //         cloud->push_back(p);
    //     }
    // }// else {
    //     for(auto &n : qt::nodes){
    //         n.createPointCloudBoundary(cloud, vertices);
    //     }
    // }
// }

// template <typename PointT>
// template <typename T>
// void QuadTreePCL<PointT>::createMesh(typename qt::QuadTree &node, typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices){
//     if(thisisLeaf()){
//         // write information to cloud.
//         return;
//     // } else {
//     //     // loop through all nodes.
//     //     for(auto &n : nodes){
//     //         qtPCL::createMesh(n, cloud, vertices);
//     //     }
//     }
// }

// template <typename PointT>
// void QuadTreePCL<PointT>::colorBasedOnCellType(PointT &p){
//     if(this->cellType == this->Interior){
//         p.r = 255; p.g = 255; p.b = 255;
//     } else if(this->cellType == this->Boundary){
//         p.r = 255; p.g = 255; p.b = 0;
//     } else{
//         p.r = 255; p.g = 0; p.b = 255;
//     }
// }
