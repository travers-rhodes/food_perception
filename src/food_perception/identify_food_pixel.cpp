#include "food_perception/identify_food_pixel.h"
#include <iostream>

// given an input image and positive and negative examples of the food
// and (optionally) a mask
// fill in the pixels vector (out parameter) with all the centers of the food
// that lie within the mask
bool FoodPixelIdentifier::GetFoodPixelCenter(const cv::Mat &image, 
         std::string positive_img_filename,
         std::vector<std::string> negative_img_filenames,
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

  cv::Mat positive = cv::imread(positive_img_filename, CV_LOAD_IMAGE_COLOR);
  cv::Mat positive_vec = positive.reshape(3,positive.cols*positive.rows).reshape(1);

  //http://answers.opencv.org/question/1368/concatenating-matrices/
  cv::Mat negatives_vec;
  for (std::vector<std::string>::iterator itr = negative_img_filenames.begin(); itr != negative_img_filenames.end(); itr++)
  {
    cv::Mat negative = cv::imread(*itr, CV_LOAD_IMAGE_COLOR);
    cv::Mat negative_vec = negative.reshape(3,negative.cols*negative.rows).reshape(1);
    if (itr == negative_img_filenames.begin())
    {
      negatives_vec = negative_vec;
    }
    else
    {
      // TODO: check that this code branch actually runs with multiple images
      cv::hconcat(negative, negatives_vec, negatives_vec);
    }
  }

  //std::cout << image.cols << "," << image.rows << " I guess\n";
  //std::cout << scaledImage.cols << "," << scaledImage.rows << " I guess\n";
  //std::cout << tomato.cols << "," << tomato.rows << " I guess\n";
  //std::cout << noTomato.cols << "," << noTomato.rows << " I guess\n";

  cv::Mat image_vec = scaledImage.reshape(3,scaled_size_x * scaled_size_y).reshape(1);

  //std::cout << image_vec.cols << "," << image_vec.rows << " I guess\n";
  //std::cout << tomato_vec.cols << "," << tomato_vec.rows << " I guess\n";
  //std::cout << noTomato_vec.cols << "," << noTomato_vec.rows << " I guess\n";

  cv::Mat dist_positive, dist_negative; 
  cv::batchDistance(image_vec, positive_vec, dist_positive, -1, cv::noArray());
  cv::batchDistance(image_vec, negatives_vec, dist_negative, -1, cv::noArray());

  cv::Mat min_dist_positive, min_dist_negative;
  cv::reduce(dist_positive, min_dist_positive, 1, cv::REDUCE_MIN);
  cv::reduce(dist_negative, min_dist_negative, 1, cv::REDUCE_MIN);
  

  cv::Mat binary_image;
  cv::compare(min_dist_positive, min_dist_negative, binary_image, cv::CMP_LE);
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

  if (moments.m00 < min_num_pixels)
  {
    // no tomato detected
    return false;
  }
  
  cv::Point2d pixel;
  pixel.x = x_center;
  pixel.y = y_center;
  pixels.push_back(pixel);
  //cv::circle(binary_image_unscaled,pixel,10,cv::Scalar( 0, 0, 255 ));
  //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  //cv::imshow( "Display window", binary_image_unscaled);     
  //cv::waitKey(0);
  return true;
}
