/** Copyright 2021 Lucas Walter
 */
#ifndef SDL2_ROS_VIEW_IMAGE_H
#define SDL2_ROS_VIEW_IMAGE_H

#include <SDL.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

struct ViewImage {
  ViewImage();
  ~ViewImage();

  std::vector<Uint8> makeTestPixels(const size_t msg_width, const size_t msg_height,
      const size_t chan, size_t& pitch, uint8_t r, uint8_t g, uint8_t b);

  void setSurface(const sensor_msgs::ImageConstPtr& msg);
  void setTexture(const sensor_msgs::ImageConstPtr& msg);
  void textureFromImageMsg(const sensor_msgs::ImageConstPtr& msg, SDL_Texture* texture);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  const Uint32 width = 2500;
  const Uint32 height = 1000;

  SDL_Window* win_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Surface* surface_ = nullptr;
  SDL_Texture* texture_ = nullptr;

  sensor_msgs::ImageConstPtr msg_ = nullptr;
};

#endif  // SDL2_ROS_VIEW_IMAGE_H
