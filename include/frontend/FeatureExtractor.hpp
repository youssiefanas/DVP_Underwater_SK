#pragma once
#include "VisualTypes.hpp"
#include "Frame.hpp"
#include <opencv2/features2d.hpp>

namespace frontend {

class FeatureExtractor {
public:
    FeatureExtractor(const ORBParams& params);

    void extract(Frame& frame);
    

private:
    cv::Ptr<cv::ORB> orb_;
};

} // namespace frontend
