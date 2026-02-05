#include "frontend/Map.hpp"

namespace frontend {

void Map::addMapPoint(const MapPoint::Ptr& map_point) {
    if (!map_point) {
        return;
    }
    map_points_.insert(map_point);
}

void Map::addKeyFrame(const Frame::Ptr& frame) {
    if (!frame) {
        return;
    }
    keyframes_.insert(frame);
}   

}// namespace frontend

