#include "food_perception/food_tracker.h"

// Params:
// * image_topic: the ros topic streaming camera images
// * plane_frame: the table plane (z=0) we project food pixels onto
// * table_polygon_of_interest: a vector of points defining a polygon of interest (ignore food outside this polygon). All points should have z = 0 (should lie on table).
FoodTracker::FoodTracker(std::string image_topic, std::string plane_frame, std::vector<geometry_msgs::Point> *table_polygon_of_interest) : it_(nh_), plane_frame_(plane_frame), table_polygon_of_interest_(table_polygon_of_interest)
{  
  ROS_WARN("[food_tracker] subscribing to camera");
  sub_ = it_.subscribeCamera(image_topic, 1, &FoodTracker::imageCb, this);
  ROS_WARN("[food_tracker] advertizing food topic");
  food_loc_pub_ = nh_.advertise<geometry_msgs::PointStamped>("food",1);
}

void FoodTracker::StartTracking()
{
  active_ = true;
}

void FoodTracker::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (!pix_proj_)
  {
    ROS_WARN("[food_tracker] Initializing pixel projector");
    camera_frame_ = info_msg->header.frame_id;
    std::shared_ptr<PixelProjector> pix_proj(new PixelProjector(*info_msg, camera_frame_, plane_frame_)); 
    pix_proj_ = pix_proj;
  }
  
  if (!pix_identifier_)
  {
    ROS_WARN("[food_tracker] Initializing pixel identifier");
    std::shared_ptr<FoodPixelIdentifier> pix_identifier(new FoodPixelIdentifier());
    pix_identifier_ = pix_identifier;
  }

  // give some time for the newly initialized tf listener to hear the tf transform
  if (!active_)
  {
    ROS_WARN("[food_tracker] Waiting because not active yet...");
    return;
  }

  cv::Mat image;
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image = input_bridge->image;
  }
  catch (cv_bridge::Exception& ex){
    ROS_ERROR("[food_tracker] Failed to convert image");
    return;
  }

  cv::Mat *mask_pointer = NULL; 
  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);

  // If we have a region of interest,
  // generate a mask so we only look at pixels that fall inside that polygon
  if (table_polygon_of_interest_)
  {
    geometry_msgs::PointStamped stamped_vertex;
    // fill out point header to make a template for a stamped point 
    stamped_vertex.header.stamp = image_msg->header.stamp;
    stamped_vertex.header.frame_id = plane_frame_;
    std::vector<cv::Point> image_filter_vertices;
    for (int i = 0; i < table_polygon_of_interest_->size(); i++)
    {
      stamped_vertex.point = table_polygon_of_interest_->at(i);
      // the following converts to integer, which I'm fine with
      cv::Point image_filter_vertex = pix_proj_->PointStampedProjectedToPixel(stamped_vertex);
      image_filter_vertices.push_back(image_filter_vertex);
      // https://stackoverflow.com/questions/43443127/opencv-how-to-create-a-mask-in-the-shape-of-a-polygon
      cv::fillConvexPoly(mask, image_filter_vertices.data(), image_filter_vertices.size(), cv::Scalar(1));
    }
    mask_pointer = &mask;
  }

  cv::Point2d food_pixel;
  if (!pix_identifier_->GetFoodPixelCenter(image, food_pixel, mask_pointer))
  {
    ROS_WARN("[food_tracker] No tomato seen");
    return;
  }

 
  ROS_WARN("[food_tracker] Actually using tf");
  geometry_msgs::PointStamped food_loc_msg = pix_proj_->PixelProjectedOnXYPlane(food_pixel, info_msg->header.stamp);

  food_loc_pub_.publish(food_loc_msg);
}

