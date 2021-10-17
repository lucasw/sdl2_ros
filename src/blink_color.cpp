/** Copyright 2021 Lucas Walter
 *
 * Blink a full image color on and off, publish a timestamp when it changes
 */
#include <SDL.h>
#include <ros/ros.h>
#include <sdl2_ros/view_image.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>


struct BlinkColor
{
  BlinkColor()
  {
    int width = 1000;
    ros::param::get("~width", width);
    int height = 800;
    ros::param::get("~height", height);

    blank_image_ = boost::make_shared<sensor_msgs::Image>();
    blank_image_->width = width;
    blank_image_->height = height;
    blank_image_->step = width * 3;
    blank_image_->data.resize(width * 3 * height);
    blank_image_->encoding = "bgr8";

    color_image_ = boost::make_shared<sensor_msgs::Image>();
    color_image_->width = width;
    color_image_->height = height;
    color_image_->step = width * 3;
    color_image_->data.resize(width * 3 * height);
    for (size_t i = 0; i < color_image_->data.size(); i+= 3) {
      color_image_->data[i] = 255;
    }
    color_image_->encoding = "bgr8";

    view_image_.imageCallback(color_image_);
    timer_ = nh_.createTimer(ros::Duration(0.5), &BlinkColor::update, this);
  }

  void update(const ros::TimerEvent& event)
  {
    if (count_ % 2 == 0) {
      ROS_INFO_STREAM("blank");
      view_image_.imageCallback(blank_image_);
    } else {
      ROS_INFO_STREAM("red");
      view_image_.imageCallback(color_image_);
    }
    count_ += 1;
  }

  size_t count_ = 0;

  sensor_msgs::Image::Ptr blank_image_;
  sensor_msgs::Image::Ptr color_image_;

  ros::NodeHandle nh_;
  ros::Timer timer_;

  ViewImage view_image_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "blink_color");
  BlinkColor blink_color;
  ros::spin();
  return 0;
}
