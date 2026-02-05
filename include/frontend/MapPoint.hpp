#pragma once

#include <memory>
#include <map>
#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace frontend {
    class Frame;

    class MapPoint {
        public:
        using Ptr = std::shared_ptr<MapPoint>;
        using FramePtr = std::shared_ptr<Frame>;

        /**
         * @brief Create a new MapPoint object
         * 
         * @param pos 
         * @return Ptr 
         */
        static Ptr create(const Eigen::Vector3d& pos);

        Eigen::Vector3d position_;
        cv::Mat descriptor_;

        std::map<FramePtr, size_t> observations_;
        // Observations: which frames observe this point and the keypoint index in that frame
        bool isBad_ = false;
        
        /**
         * @brief Add an observation of the MapPoint in a specific frame
         * 
         * @param frame 
         * @param keypoint_idx 
         */
        void addObservation(const FramePtr& frame, size_t keypoint_idx);

        private:
        MapPoint(const Eigen::Vector3d& pos);
        

    };
} // namespace frontend