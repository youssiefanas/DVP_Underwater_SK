cmake_minimum_required(VERSION 3.20)
project(dv_slam)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set (OPENCV_LIBS
  ${OpenCV_LIBS}
  opencv_calib3d
  opencv_core
  opencv_imgproc
  opencv_highgui
  )

ament_auto_add_library(${PROJECT_NAME}_frontend
  src/frontend/VisualFrontend.cpp
  src/frontend/Viewer.cpp
  src/frontend/FeatureExtractor.cpp
  src/frontend/FeatureMatcher.cpp
  src/frontend/Frame.cpp
  src/frontend/PoseEstimator.cpp
)

target_include_directories(${PROJECT_NAME}_frontend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_frontend
  # ${GTSAM_LIBRARIES}
  gtsam
 ${OPENCV_LIBS} 
)

ament_auto_add_executable(vo_node 
 src/visual_odom/visual_odom_node.cpp
 src/visual_odom/visual_odom_main.cpp
)
target_include_directories(vo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vo_node 
 ${PROJECT_NAME}_frontend
 ${OPENCV_LIBS}
)


 if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_matching test/test_matching.cpp)
  target_include_directories(test_matching PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_matching
    ${PROJECT_NAME}_frontend
    ${OPENCV_LIBS}
  )
  
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
