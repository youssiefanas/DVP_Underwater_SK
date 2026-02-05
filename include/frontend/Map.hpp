#pragma once 

#include <unordered_set> 

#include <memory>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "Frame.hpp"
#include "MapPoint.hpp"

namespace frontend {
class KeyFrame;

class Map {
public:
    using Ptr = std::shared_ptr<Map>;
    
    Map() = default;
    ~Map() = default;

    void addKeyFrame(const Frame::Ptr& frame);
    void addMapPoint(const MapPoint::Ptr& map_point);
    
    const std::unordered_set<Frame::Ptr>& getFrames() const { return keyframes_; }
    const std::unordered_set<MapPoint::Ptr>& getMapPoints() const { return map_points_; }

private:
    std::unordered_set<Frame::Ptr> keyframes_;
    std::unordered_set<MapPoint::Ptr> map_points_;
};
}
