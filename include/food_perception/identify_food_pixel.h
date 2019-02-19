#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class FoodPixelIdentifier
{
  public:
    bool GetFoodPixelCenter(const cv::Mat &image, std::string positive_img_filename, std::vector<std::string> negative_img_filenames, std::vector<cv::Point2d> &pixel, cv::Mat *mask = NULL);
};

