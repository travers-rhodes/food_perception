#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "food_perception/food_tracker.h"
#include "food_perception/param_parser.h"
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/algorithm/string.hpp>

int main(int argc, char** argv)
{
  ROS_WARN("Starting main method");
  ros::init(argc, argv, "food_publisher");
  ros::NodeHandle nh;
  std::string camera_topic, table_frame_id, roi_polygon_raw_str;
  ros::param::get("~camera_topic", camera_topic); 
  ros::param::get("~table_frame_id", table_frame_id); 
  // this is the syntax to include a default value
  ros::param::param<std::string>("~roi_polygon", roi_polygon_raw_str, ""); 
  std::vector<geometry_msgs::Point> *poly_of_interest_ptr = NULL; 
  std::vector<geometry_msgs::Point> poly_of_interest;
  if (roi_polygon_raw_str.size() > 0)
  {
    if (!ParsePolygonParam(roi_polygon_raw_str, poly_of_interest))
    {
      ROS_ERROR("Error parsing roi_polygon parameter. That parameter (if included) should be a string of the form '(x0,y0), (x1, y1), ..., (xn, yn)'");
      throw;
    }
    poly_of_interest_ptr = &poly_of_interest;
  }
  std::string image_topic = nh.resolveName(camera_topic);
  FoodTracker foodTracker(image_topic, table_frame_id, poly_of_interest_ptr);
  ROS_WARN("Sleeping for a spell");
  for (int i=0; i < 10; i++)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  foodTracker.StartTracking();
  ros::spin();
}
