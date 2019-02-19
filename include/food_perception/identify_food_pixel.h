#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class FoodPixelIdentifier
{
  private:
    std::vector<cv::Mat> positive_vecs_;
    // I find using a vector more convenient than using a shared_ptr, even though
    // we expect only one negative vec for now.
    std::vector<cv::Mat> negative_vecs_;
  public:
    FoodPixelIdentifier(std::vector<std::string> positive_img_filenames, std::string negative_img_filename);
    std::vector<bool> GetFoodPixelCenter(const cv::Mat &image, std::vector<cv::Point2d> &pixel, cv::Mat *mask = NULL);
};

