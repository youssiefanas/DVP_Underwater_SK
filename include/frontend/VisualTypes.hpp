#pragma once
#include <string>

namespace frontend {

struct ORBParams {
    int n_features = 1000;
    float scale_factor = 1.2f;
    int n_levels = 8;
    int edge_threshold = 31;
    int first_level = 0;
    int wta_k = 2;
    int score_type = 0; // HARRIS_SCORE
    int patch_size = 31;
    int fast_threshold = 20;
    std::string matcher_type = "NORM_HAMMING";
};

} // namespace frontend
