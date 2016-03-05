#include <quadtree/quadtreePCL.h>

void QuadTree::extractCells(std::vector<QuadTree::Cell> &cells){

    if(is_leaf_){
        QuadTree::Cell c;
        c.x = x_;
        c.y = y_;
        c.width = width_;
        c.r = 0;
        c.g = 0;
        c.b = 0;
        cells.push_back(c);
    } else {
        for(auto &n : nodes){
            n.extractCells(cells);
        }
    }

}
