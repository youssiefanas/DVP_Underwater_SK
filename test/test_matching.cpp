#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// Include your library headers
#include "frontend/FeatureExtractor.hpp"
#include "frontend/FeatureMatcher.hpp"
#include "frontend/Frame.hpp"

using namespace frontend;

// Test Fixture to setup common objects
class MatchingTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize your params
    ORBParams params;
    params.n_features = 1000;

    extractor = std::make_shared<FeatureExtractor>(params);
    matcher = std::make_shared<FeatureMatcher>("NORM_HAMMING");

    // Path to your test images
    // PRO TIP: In real projects, use CMake to configure this path
    data_path = "/home/anas/test_data/";
  }

  std::shared_ptr<FeatureExtractor> extractor;
  std::shared_ptr<FeatureMatcher> matcher;
  std::string data_path;
};

TEST_F(MatchingTest, VisualStepThrough) {
  // Load a sequence of images (e.g., 0.png to 3.png)
  int num_images = 600; // Adjust based on your dataset
  Frame::Ptr prev_frame = nullptr;

  for (int i = 100; i < num_images; i++) {
    // 1. Load Image
    // i is 06d for example image_000543.png
    std::string filename = data_path + "image_000" + std::to_string(i) + ".png";
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    // Assert image exists (Fail test if path is wrong)
    ASSERT_FALSE(img.empty()) << "Could not load image: " << filename;

    // 2. Create Frame
    double timestamp = i * 0.1; // Fake timestamp
    auto current_frame = Frame::createFrame(img, timestamp);

    // 3. Extract Features
    extractor->extract(*current_frame);

    // 4. Match (only if we have a previous frame)
    if (prev_frame) {
      auto matches = matcher->match(prev_frame, current_frame);

      // LOGIC CHECK: We should find *some* matches in consecutive frames
      // EXPECT_GT(matches.size(), 20) << "Too few matches between frame " <<
      // i-1 << " and " << i;

      // --- VISUALIZATION BLOCK ---
      cv::Mat out_img;
      cv::drawMatches(prev_frame->getImage(), prev_frame->getKeypoints(),
                      current_frame->getImage(), current_frame->getKeypoints(),
                      matches, out_img);
      std::string label = "Frame " + std::to_string(i - 1) + " -> " +
                          std::to_string(i) +
                          " | Matches: " + std::to_string(matches.size());

      cv::putText(out_img, label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                  0.8, cv::Scalar(0, 255, 0), 2);

      cv::imshow("Sequence Matcher (Press Any Key for Next)", out_img);

      // --- WAIT FOR KEY PRESS ---
      // 0 = Wait indefinitely until key press
      // 1 = Wait 1ms (plays like a video)
      // 27 = ESC key (usually)
      char key = (char)cv::waitKey(0);

      if (key == 27) { // If 'ESC' is pressed, break the loop
        std::cout << "Test aborted by user." << std::endl;
        break;
      }
    }
    // Update previous frame
    prev_frame = current_frame;
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}