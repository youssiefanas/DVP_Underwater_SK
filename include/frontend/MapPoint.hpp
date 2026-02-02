#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <list>
#include <opencv2/core.hpp>
#include <Eigen/Core>

namespace frontend {

// Forward declarations
class Frame;
class KeyFrame;

/**
 * @brief MapPoint represents a 3D point in the world.
 * Role: Persistent 3D geometric constraint.
 */
class MapPoint {
public:
    using Ptr = std::shared_ptr<MapPoint>;

    MapPoint(const Eigen::Vector3d& pos);
    
    // --- Getters & Setters ---
    
    Eigen::Vector3d getPosition() const;
    void setPosition(const Eigen::Vector3d& pos);

    cv::Mat getDescriptor() const;
    void setDescriptor(const cv::Mat& des);

    // --- Observation Management ---
    
    void addObservation(std::shared_ptr<KeyFrame> kf, size_t idx);
    void removeObservation(std::shared_ptr<KeyFrame> kf);
    
    // Get number of observations (KeyFrames that see this point)
    int observationCount() const;

    // --- Quality ---
    
    bool isBad() const;
    void setBadFlag();

    // --- Projectability ---
    // Helper to get Representative Descriptor (e.g. median of observations)
    // For now, we might just set it explicitly.

public:
    // Metadata for Graph
    long unsigned int id_;
    static std::atomic<long unsigned int> next_id_;

private:
    Eigen::Vector3d pos_;
    cv::Mat descriptor_;
    
    // Observations: KeyFrame observing this point + index of feature in that KeyFrame
    // Using weak_ptr for KeyFrames to avoid cyclic dependencies
    // (MapPoint <-> KeyFrame)
    // Actually, usually MapPoints are owned by the Map, and referenced by KeyFrames.
    // But KeyFrames also reference MapPoints.
    // Let's store raw ptr or weak_ptr to KeyFrame to be safe, but usually 
    // KeyFrame -> MapPoint (shared_ptr)
    // MapPoint -> KeyFrame (weak_ptr/raw)
    // For simplicity in this frontend-focused prototype, we just track that we are observed.
    // We can store a map: KeyFrame* -> feature_index
    // But we need to include KeyFrame.hpp which is circular.
    // We will use a structure defined here or void* if necessary, but forward decl should work 
    // with pointers.
    
    // Simplification: Just count for now, or use forward declared shared_ptr
    // std::map<std::shared_ptr<KeyFrame>, size_t> observations_; 
    // MapPoints need to know who observes them to refine their position or descriptor.
    
    // Mutex for thread safety
    mutable std::mutex mutex_;
    
    bool is_bad_ = false;
};

} // namespace frontend
