#include "food_perception/identify_food_pixel.h"
#include <iostream>


FoodPixelIdentifier::FoodPixelIdentifier(std::vector<std::string> positive_img_filenames, std::string negative_img_filename)
{
  for (std::vector<std::string>::iterator itr = positive_img_filenames.begin(); itr != positive_img_filenames.end(); itr++)
  { 
    cv::Mat positive = cv::imread(*itr, CV_LOAD_IMAGE_COLOR);
    cv::Mat positive_vec = positive.reshape(3,positive.cols*positive.rows).reshape(1);
    positive_vecs_.push_back(positive_vec);
  }
  cv::Mat negative = cv::imread(negative_img_filename, CV_LOAD_IMAGE_COLOR);
  cv::Mat negative_vec = negative.reshape(3,negative.cols*negative.rows).reshape(1);
  negative_vecs_.push_back(negative_vec); 
}

// given an input image and positive and negative examples of the food
// and (optionally) a mask
// fill in the pixels vector (out parameter) with all the centers of the food
// that lie within the mask
std::vector<bool> FoodPixelIdentifier::GetFoodPixelCenter(const cv::Mat &image, 
         std::vector<cv::Point2d> &pixels, 
         cv::Mat *mask)
{
  //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  //cv::imshow( "Display window", image );     
  //cv::waitKey(0);

  // minimum number of pixels in order to count as a tomato
  int min_num_pixels = 200;
  
  // avoid the computational complexity of overly large images
  int scaled_size_x = 200, scaled_size_y = 200;
  int smaller_image_x, smaller_image_y;

  cv::Mat scaledImage;
  bool resized = false;
  if (image.rows > scaled_size_x || image.cols > scaled_size_y)
  {
    cv::resize(image, scaledImage, cv::Size(scaled_size_y, scaled_size_x),0,0);
    resized = true;
    smaller_image_x = scaled_size_x;
    smaller_image_y = scaled_size_y;
  }
  else
  {
    scaledImage = image;
    scaled_size_x = image.rows;
    scaled_size_y = image.cols;
  }

  cv::Mat image_vec = scaledImage.reshape(3,scaled_size_x * scaled_size_y).reshape(1);

  std::vector<cv::Mat> min_dist_positives;
  for (std::vector<cv::Mat>::iterator itr = positive_vecs_.begin(); itr != positive_vecs_.end(); itr++)
  {
    cv::Mat dist_positive;
    cv::batchDistance(image_vec, *itr, dist_positive, -1, cv::noArray());
    cv::Mat min_dist_positive;
    cv::reduce(dist_positive, min_dist_positive, 1, cv::REDUCE_MIN);
    min_dist_positives.push_back(min_dist_positive);
  }
  
  cv::Mat dist_negative; 
  cv::batchDistance(image_vec, negative_vecs_[0], dist_negative, -1, cv::noArray());
  cv::Mat min_dist_negative;
  cv::reduce(dist_negative, min_dist_negative, 1, cv::REDUCE_MIN);

  // New strategy: find min distance across all options, then check if
  // our array is equal to that min distance. Treats equality slightly differently
  // than originally planned (could fix with some epsilon if we wanted to)
  // but makes computation smooth and painless
  cv::Mat min_dist_cumulative = min_dist_negative;
  for (std::vector<cv::Mat>::iterator itr = min_dist_positives.begin(); itr != min_dist_positives.end(); itr++)
  {
    cv::min(*itr, min_dist_cumulative, min_dist_cumulative);
  }
    
  std::vector<bool> success_vec;
  // Generate binary image, find food point, and save it off for each type of food
  for (std::vector<cv::Mat>::iterator itr = min_dist_positives.begin(); itr != min_dist_positives.end(); itr++)
  {
    cv::Mat binary_image;
    cv::compare(*itr, min_dist_cumulative, binary_image, cv::CMP_EQ);
    binary_image = binary_image.reshape(1,scaled_size_x);
    
    cv::Mat binary_image_unscaled; 
    if (resized) {
      cv::resize(binary_image, binary_image_unscaled, cv::Size(image.cols,image.rows),0,0);
    } else {
      binary_image_unscaled = binary_image;
    }

    // apply the mask to the binary "relevant pixels" image 
    if (mask)
    {
      binary_image_unscaled = mask->mul(binary_image_unscaled);
    }

    
    cv::Moments moments = cv::moments(binary_image_unscaled, true);
    int x_center = moments.m10/moments.m00;
    int y_center = moments.m01/moments.m00;
    //std::cout << "x_center: " << x_center << "; y_center: " << y_center << "; count: " << moments.m00 << "\n";

    cv::Point2d pixel;
    if (moments.m00 < min_num_pixels)
    {
      // no object detected
      success_vec.push_back(false);
      pixels.push_back(pixel);
      continue;
    }
    
    success_vec.push_back(true);
    pixel.x = x_center;
    pixel.y = y_center;
    pixels.push_back(pixel);
  }
  //cv::circle(binary_image_unscaled,pixel,10,cv::Scalar( 0, 0, 255 ));
  //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  //cv::imshow( "Display window", binary_image_unscaled);     
  //cv::waitKey(0);
  return(success_vec);
}
