#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>

template <typename PointT>
QuadTree<PointT>::QuadTree(int level, float width, float x, float y){
    x_ = x;
    y_ = y;
    width_ = width;
    level_ = level;
    max_width_ = -1;
    max_level_ = -1;
    use_color_ = false;
    is_leaf_ = true;
    is_boundary_ = false;
    cellType = Parent;
}

template <typename PointT>
int QuadTree<PointT>::insert(PointT point, bool is_boundary){
    if(max_level_ < 1){
        if(max_width_ <= 0){
            std::cout << "max_width: " << max_width_ << std::endl;
            std::cout << "Neither max_level_ nor max_width_ have been set!!!" << std::endl;
            std::cout << "Setting max_level_ to 10." << std::endl;
            max_level_ = 10;
        }
    }
    if( is_leaf_ ){
        // This is leaf, check if we should go deeper
        if( !QuadTree<PointT>::maxDepth() ){
            // Create new leafs.
            QuadTree<PointT>::createSubNodesAndInherit();
        } else {
            // Insert point in vector
            if(is_boundary){
                if(!is_boundary_){
                    points.clear();
                    is_boundary_ = true;
                }
                points.push_back(point);
            } else if(!is_boundary){
                points.push_back(point);
            }
            return 0;
        }
    }
    int quadrant = QuadTree<PointT>::returnQuadrant(point);
    if(quadrant < 4){
        nodes[quadrant].insert(point, is_boundary);
    }
    return quadrant+1;
}


template <typename PointT>
void QuadTree<PointT>::useColor(bool use_color){
    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    if(use_color == true){
        bool is_rgb = false;
        float rgb_val;
        PointT p;
        pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<PointT, float> (p, "rgb", is_rgb, rgb_val));
        if(is_rgb){
            use_color_ = true;
        } else {
            use_color_ = false;
            std::cout << "The point type does not include Color!!!  Setting use_color to false. " << std::endl;
        }
    } else {
        use_color_ = use_color;
    }
}

template <typename PointT>
void QuadTree<PointT>::setMaxLevel(int max_level){
    max_level_ = max_level;
}

template <typename PointT>
void QuadTree<PointT>::setMaxWidth(float max_width){
    max_width_ = max_width;
}

template <typename PointT>
void QuadTree<PointT>::isLeaf(bool is_leaf){
    is_leaf_ = is_leaf;
}

template <typename PointT>
void QuadTree<PointT>::isBoundary(bool is_boundary){
    is_boundary_ = is_boundary;
}

template <typename PointT>
bool QuadTree<PointT>::maxDepth(){

    if( level_ >= max_level_ && max_level_ > 0 ){
        return true;
    }
    if( width_ < max_width_ && max_width_> 0 ){
        return true;
    }
    return false;

    // return !(level_ >= max_level_ && max_level_ > 0) || (width_ < max_width_ && max_width_> 0);
}

template <typename PointT>
void QuadTree<PointT>::createSubNodesAndInherit(){

    nodes.resize(4);
    float half =  width_/2.0;
    std::cout << "half: " << half << std::endl;
    nodes[0] = typename QuadTree<PointT>::QuadTree(level_ + 1, half, x_, y_);
    nodes[1] = typename QuadTree<PointT>::QuadTree(level_ + 1, half, x_ + half, y_);
    nodes[2] = typename QuadTree<PointT>::QuadTree(level_ + 1, half, x_, y_ + half);
    nodes[3] = typename QuadTree<PointT>::QuadTree(level_ + 1, half, x_ + half, y_ + half);

    for(auto &n : nodes){
        n.useColor(use_color_);
        n.setMaxWidth(max_width_);
        n.setMaxLevel(max_level_);
    }

    is_leaf_ = false;

}

template <typename PointT>
int QuadTree<PointT>::returnQuadrant(PointT p){

    // First check that we are in the correct cell.
    if(p.x < x_ || p.x > (x_ + width_)){
        std::cout << "hmmmmm1" << std::endl;
        return 4;
    }
    if(p.y < y_ || p.y > (y_ + width_)){
        std::cout << "hmmmmm2" << std::endl;
        return 4;
    }

    if( p.y < y_ + width_/2.0 ){
        if( p.x < x_ + width_/2.0 )
            return 0;
        return 1;
    } else if( p.x < x_ + width_/2.0 )
        return 2;
    return 3;
}

template <typename PointT>
void QuadTree<PointT>::printTree(std::vector<int> idx){
    if(is_leaf_){
        if( points.size() != 0 ){
            std::cout << "level: " << level_ << std::endl;
            std::cout << "idx: ";
            for(auto i : idx){
                std::cout << i << ", ";
            }
            std::cout << " " << std::endl;
            for(auto p : points){
                std::cout << "x: " << p.x << "  y: " << p.y << std::endl;
            }
        }
    } else {
        idx.resize(idx.size()+1);
        for (size_t i = 0; i < nodes.size(); i++) {
            idx.back() = i;
            nodes[i].printTree(idx);
        }
    }
}


template <typename PointT>
void QuadTree<PointT>::clear(){

    // TODO either copy all points to parent node or get average color.

    // Clear all points.
    // points.clear();
    // Recursively clear all subnodes;
    for(auto &n : nodes){
        points.insert(points.end(), n.pointsBegin(), n.pointsEnd());
        n.clear();
    }
    // Clear nodes vector;
    nodes.clear();
    is_leaf_ = true;
}


template <typename PointT>
int QuadTree<PointT>::getTreeDepth(){
    if(is_leaf_){
        return level_;
    } else {
        int max = 0;
        int tmp = 0;
        for(auto n : nodes){
            tmp = n.getTreeDepth();
            if(tmp > max){
                max = tmp;
            }
        }
        return max;
    }
}

template <typename PointT>
bool QuadTree<PointT>::decemate(){

    // return True -> can be merged.
    // return False -> can't be merged;

    // boundary nodes are never merged.
    if(is_boundary_) return false;



    if(!use_color_){
        // if it is leaf it can be merged
        if(is_leaf_) return true;

        bool mergable = true;
        for(auto &n : nodes){
            if(!n.decemate()) mergable = false;
        }
        if(mergable){
            QuadTree<PointT>::clear();
            is_leaf_ = true;
        }
        return mergable;
    }
}

template <typename PointT>
void QuadTree<PointT>::createPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector< pcl::Vertices > &vertices){
    if(is_leaf_){

        int size = cloud->points.size();
        if(is_boundary_){
            cloud->reserve(size + points.size());
            for(auto p : points){
                pcl::PointXYZRGB tmp;
                tmp.x = p.x;
                tmp.y = p.y;
                tmp.z = p.z;
                tmp.r = 255;
                tmp.g = 255;
                tmp.b = 0;
                cloud->push_back(tmp);
            }
        } else if(points.size() >= 0){
            pcl::Vertices vert;
            vert.vertices.resize(3);
            vert.vertices[0] = size;
            vert.vertices[1] = size + 3;
            vert.vertices[2] = size + 2;
            vertices.push_back(vert);

            vert.vertices[0] = size;
            vert.vertices[1] = size + 1;
            vert.vertices[2] = size + 3;
            vertices.push_back(vert);

            cloud->reserve(size + 4);
            // add the 4 corners.

            pcl::PointXYZRGB p; p.z = 0;
            p.r = 255; p.g = 255; p.b = 255;
            p.x = x_; p.y = y_;
            cloud->push_back(p);

            p.x = x_ + width_; p.y = y_;
            cloud->push_back(p);

            p.x = x_; p.y = y_ + width_;
            cloud->push_back(p);

            p.x = x_ + width_; p.y = y_ + width_;
            cloud->push_back(p);
        }
    } else {
        for(auto &n : nodes){
            n.createPointCloud(cloud, vertices);
        }
    }
}

template <typename PointT>
bool QuadTree<PointT>::insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary){
    // Convert to cgal polygon

    Polygon polygon;
    for (size_t i = 0; i < boundary->size(); i++) {
        polygon.push_back(Point(boundary->points[i].x, boundary->points[i].y));
    }
    return QuadTree<PointT>::insertBoundary(polygon);
}

template <typename PointT>
bool QuadTree<PointT>::insertBoundary(Polygon polygon){

    Polygon cell;
    cell.push_back( Point(x_,          y_) );
    cell.push_back( Point(x_ + width_, y_) );
    cell.push_back( Point(x_ + width_, y_ + width_) );
    cell.push_back( Point(x_,          y_+width_) );


    // check to see if polygon is contained completely by the cell
    if(QuadTree<PointT>::polygonCompletelyWithinCell(polygon)){
        if( !QuadTree<PointT>::maxDepth() ){
            // Create new leafs.
            QuadTree<PointT>::createSubNodesAndInherit();
            for(auto &n : nodes){
                n.insertBoundary(polygon);
            }
        } else {
            cellType = Boundary;
        }
    }


    if(cellType == Parent){
        // This cell hasn't been processed
        bool intersect = CGAL::do_intersect(polygon, cell);
        if(intersect){

            // need to check if cell completely inside polygon
            bool inside = false;
            bool outside = false;
            bool edge = false;
            for (size_t i = 0; i < cell.size(); i++) {
                auto bounded = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), cell[i], K());

                if(bounded == CGAL::ON_BOUNDED_SIDE){
                    // std::cout << " is inside the polygon.\n";
                    inside = true;
                } else if(bounded == CGAL::ON_BOUNDARY){
                    // std::cout << " The polygon and cells edges touch.\n";
                    edge = true;
                } else {
                    // std::cout << " is outside the polygon.\n";
                    outside = true;
                }
            }

            if(inside && outside || edge || (outside && !inside && !edge) ){
                // The boundary travels through the cell.
                // if the cell isn't a smallest possible size or biggest depth split it.
                if( !QuadTree<PointT>::maxDepth() ){
                    // Create new leafs.
                    QuadTree<PointT>::createSubNodesAndInherit();
                    for(auto &n : nodes){
                        n.insertBoundary(polygon);
                    }
                } else {
                    cellType = Boundary;
                }
            } else if(inside){
                cellType = Interior;
            } else {
                cellType = Exterior;
            }

        } else {
            // std::cout << "does not intersect" << std::endl;
            cellType = Exterior;
            return false;
        }
    }
    return true;
}


template <typename PointT>
void QuadTree<PointT>::createPointCloudBoundary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector< pcl::Vertices > &vertices){
    if(is_leaf_){
        int size = cloud->points.size();
        // if(cellType != Exterior){
        if(cellType != Parent && cellType != Exterior && cellType != Hole){

            pcl::Vertices vert;
            vert.vertices.resize(3);
            vert.vertices[0] = size;
            vert.vertices[1] = size + 3;
            vert.vertices[2] = size + 2;
            vertices.push_back(vert);

            vert.vertices[0] = size;
            vert.vertices[1] = size + 1;
            vert.vertices[2] = size + 3;
            vertices.push_back(vert);

            cloud->reserve(size + 4);
            // add the 4 corners.

            pcl::PointXYZRGB p; p.z = 0;
            if(cellType == Interior){
                p.r = 255; p.g = 255; p.b = 255;
            } else if(cellType == Boundary){
                p.r = 255; p.g = 255; p.b = 0;
            } else{
                p.r = 255; p.g = 0; p.b = 255;
            }
            p.x = x_; p.y = y_;
            cloud->push_back(p);

            p.x = x_ + width_; p.y = y_;
            cloud->push_back(p);

            p.x = x_; p.y = y_ + width_;
            cloud->push_back(p);

            p.x = x_ + width_; p.y = y_ + width_;
            cloud->push_back(p);
        }
    } else {
        for(auto &n : nodes){
            n.createPointCloudBoundary(cloud, vertices);
        }
    }
}





template <typename PointT>
void QuadTree<PointT>::insertHole(Polygon polygon){

    // Check to see if exterior node.
    // check to see if polygon is contained completely by the cell
    if(QuadTree<PointT>::polygonCompletelyWithinCell(polygon)){
        if(cellType == Parent){
            for(auto &n : nodes){
                n.insertHole(polygon);
            }
        } else if( !QuadTree<PointT>::maxDepth() && cellType == Interior ){
            // Create new leafs.
            QuadTree<PointT>::createSubNodesAndInherit();
            for(auto &n : nodes){
                n.setCellType(Interior);
                n.insertHole(polygon);
            }
        } else {
            cellType = Boundary;
        }
        return;
    }

    Polygon cell;
    cell.push_back( Point(x_,          y_) );
    cell.push_back( Point(x_ + width_, y_) );
    cell.push_back( Point(x_ + width_, y_ + width_) );
    cell.push_back( Point(x_,          y_+width_) );

    bool intersect = CGAL::do_intersect(polygon, cell);

    if(intersect){
        // std::cout << "intersect: " << cellType << std::endl;
        if(cellType == Interior){
            // std::cout << "Interior" << std::endl;
            // need to check if cell completely inside polygon
            bool inside = false;
            bool outside = false;
            bool edge = false;
            for (size_t i = 0; i < cell.size(); i++) {
                auto bounded = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), cell[i], K());

                if(bounded == CGAL::ON_BOUNDED_SIDE){
                    // std::cout << " is inside the polygon.\n";
                    inside = true;
                } else if(bounded == CGAL::ON_BOUNDARY){
                    // std::cout << " The polygon and cells edges touch.\n";
                    edge = true;
                } else {
                    // std::cout << " is outside the polygon.\n";
                    outside = true;
                }
            }

            if(inside && outside || edge || (outside && !inside && !edge) ){
                // The boundary travels through the cell.
                // if the cell isn't a smallest possible size or biggest depth split it.
                if(cellType == Parent){
                    for(auto &n : nodes){
                        n.insertHole(polygon);
                    }
                }
                else if( !QuadTree<PointT>::maxDepth() ){
                    // Create new leafs.
                    QuadTree<PointT>::createSubNodesAndInherit();
                    for(auto &n : nodes){
                        n.setCellType(Interior);
                        n.insertHole(polygon);
                    }
                } else {
                    cellType = Boundary;
                }
            } else if(inside){
                cellType = Hole;
            } else {
                cellType = Interior;
            }
        } else if(cellType == Parent){
            for(auto &n : nodes){
                n.insertHole(polygon);
            }
        }
    }
}

template <typename PointT>
bool QuadTree<PointT>::polygonCompletelyWithinCell(Polygon &polygon){
    auto left = CGAL::left_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    auto right = CGAL::right_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    auto top = CGAL::top_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    auto bottom = CGAL::bottom_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    if( (*left)[0] >= x_) {
        if( (*right)[0] <= x_ + width_) {
            if( (*bottom)[1] >= y_) {
                if( (*top)[1] <= y_ + width_) {
                    return true;
                }
            }
        }
    }
    return false;
}
