#include "quadtree.h"

QuadTree::useColor(bool value){
    if(value == true){
        if( pcl::traits::has_field<PointT, pcl::fields::rgb>::value ){
            use_color_ = true;
        }
        use_color_ = false;
    } else
}
