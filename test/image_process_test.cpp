// see http://wiki.ros.org/gtest
// Bring in my package's API, which is what I'm testing
#include "food_perception/identify_food_pixel.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

// Declare a test
TEST(TestSuite, test_find_red)
{
  cv::Mat image;
  image = cv::imread("/home/traversrhodes/demo_ws/src/food_perception/test/input_data/MosaicView.jpg", CV_LOAD_IMAGE_COLOR);
  FoodPixelIdentifier food_identifier;
  cv::Point2d point;
  bool found = food_identifier.GetFoodPixelCenter(image, point);
  //std::cout << "point" << point.x << "," << point.y << "\n";
}

TEST(TestSuite, test_mask)
{
  // load test image
  cv::Mat image;
  image = cv::imread("/home/traversrhodes/demo_ws/src/food_perception/test/input_data/MosaicView.jpg", CV_LOAD_IMAGE_COLOR);

  // make a mask
  std::vector<cv::Point> image_filter_vertices;
  image_filter_vertices.push_back(cv::Point(image.cols/3,image.rows/3));
  image_filter_vertices.push_back(cv::Point(image.cols/3,2 * image.rows/3));
  image_filter_vertices.push_back(cv::Point(2*image.cols/3,2 * image.rows/3));
  image_filter_vertices.push_back(cv::Point(2*image.cols/3,image.rows/3));
  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
  cv::fillConvexPoly(mask, image_filter_vertices.data(), image_filter_vertices.size(), cv::Scalar(1));
  //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  //cv::imshow( "Display window", mask * 255);     
  //cv::waitKey(0);
  
  FoodPixelIdentifier food_identifier;
  cv::Point2d point;
  bool found = food_identifier.GetFoodPixelCenter(image, point, &mask);
  //std::cout << "point" << point.x << "," << point.y << "\n";

  //cv::circle(image,point,10,cv::Scalar( 0, 0, 255 ));
  //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  //cv::imshow( "Display window", image);     
  //cv::waitKey(0);

  //simple regression test:
  EXPECT_EQ(489, point.x);
  EXPECT_EQ(383, point.y);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
