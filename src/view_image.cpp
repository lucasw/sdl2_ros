/** Copyright 2021 Lucas Walter
 */
#include <SDL.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

struct ViewImage {
  ViewImage() {
    SDL_Init(SDL_INIT_EVERYTHING);
    win_ = SDL_CreateWindow("view_image", 50, 50, width, height, SDL_WINDOW_SHOWN);
    if (!win_) {
      ROS_ERROR_STREAM(SDL_GetError());
      SDL_Quit();
      ros::shutdown();
    }

    // need the software renderer to use surfaces
    renderer_ = SDL_CreateRenderer(win_, -1, SDL_RENDERER_SOFTWARE);
    // SDL_RENDERER_ACCELERATED);  // | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer_) {
      ROS_ERROR_STREAM(SDL_GetError());
      SDL_Quit();
      ros::shutdown();
    }

    {
      SDL_RendererInfo info;
      SDL_GetRendererInfo(renderer_, &info);
      ROS_INFO_STREAM("Renderer name: " << info.name);
      ROS_INFO_STREAM("Texture formats: ");
      for (Uint32 i = 0; i < info.num_texture_formats; i++) {
        ROS_INFO_STREAM(SDL_GetPixelFormatName(info.texture_formats[i]));
      }
    }

#if 1
    // TODO(lucasw) only with software rendering?
    surface_ = SDL_GetWindowSurface(win_);
    if (!surface_) {
      // TODO(lucasw) do I need a surface?
      // No hardware accelerated renderers available
      ROS_ERROR_STREAM(SDL_GetError());
      SDL_Quit();
      ros::shutdown();
    }
#endif

    SDL_SetRenderDrawColor(renderer_, 30, 40, 35, 255);
    SDL_RenderClear(renderer_);
    SDL_RenderPresent(renderer_);

    image_sub_ = nh_.subscribe<sensor_msgs::Image>("image_in", 2, &ViewImage::imageCallback, this);
    // timer_ = nh_.createTimer(ros::Duration(0.1), &ViewImage::update, this);
    ROS_INFO_STREAM("done init");
  }

  ~ViewImage() {
    if (surface_) {
      SDL_FreeSurface(surface_);
    }
    if (win_) {
      SDL_DestroyWindow(win_);
    }
    SDL_Quit();
  }

  std::vector<Uint8> makeTestPixels(const size_t msg_width, const size_t msg_height,
      const size_t chan, size_t& pitch)
  {
    pitch = msg_width * chan;
    std::vector<Uint8> pixels;
    pixels.resize(pitch * msg_height);
    for (size_t i = 0; i < pixels.size() / chan; ++i) {
      // slow but produces nice gradation
      const size_t x = i % msg_width;
      const size_t y = i / msg_width;
      const float fr_x = static_cast<float>(x) / static_cast<float>(msg_width);
      const float fr_y = static_cast<float>(y) / static_cast<float>(msg_height);
      pixels[i * chan + 0] = 0xff;  // alpha channel
      // no alpha channel if chan == 3
      const size_t offset = chan == 4 ? 1 : 0;
      pixels[i * chan + offset + 0] = fr_x * 255;
      pixels[i * chan + offset + 1] = fr_x * fr_y * 255;  // (i % pitch) * fr;
      pixels[i * chan + offset + 2] = fr_y * 255;
    }
    return pixels;
  }

  void setSurface(const sensor_msgs::ImageConstPtr& msg) {
    const auto t0 = ros::Time::now();
    if (surface_ == nullptr) {
      ROS_WARN_STREAM("no surface to blit to");
      return;
    }
    const size_t msg_width = msg->width;
    const size_t msg_height = msg->height;
    const size_t chan = 3;
    size_t pitch = msg->step;
    // auto pixels = makeTestPixels(msg_width, msg_height, chan, pitch);
    SDL_Surface* surface = SDL_CreateRGBSurfaceFrom(
        // pixels.data(),
        const_cast<void*>(reinterpret_cast<const void*>(msg->data.data())),
        msg_width, msg_height, 24, pitch,
        0xff0000,
        0x00ff00,
        0x0000ff,
        0x000000);
    if (surface == nullptr) {
      ROS_WARN_STREAM("bad surface " << SDL_GetError());
      return;
    }
    const int rv0 = SDL_BlitSurface(surface, nullptr, surface_, nullptr);
    SDL_FreeSurface(surface);
    if (rv0 != 0) {
      ROS_WARN_STREAM("bad blit surface " << SDL_GetError());
      return;
    }
    const auto t1 = ros::Time::now();
    ROS_DEBUG_STREAM("surface " << (t1 - t0).toSec());
    return;
  }

  void setTexture(const sensor_msgs::ImageConstPtr& msg) {
    texture_ = SDL_CreateTexture(
        renderer_,
        // TODO(lucasw) need to match on the msg->encoding
        // TODO(lucasw) This doesn't display correctly - the internal representation isn't
        // what I assumed it would be, maybe using Surfaces will work better?
        SDL_PIXELFORMAT_BGR888,
        // SDL_PIXELFORMAT_BGRA8888,
        // SDL_TEXTUREACCESS_STREAMING,
        SDL_TEXTUREACCESS_STATIC,
        msg->width,
        msg->height);
    if (texture_ == nullptr) {
      ROS_ERROR_STREAM("sdl create texture failed: " << SDL_GetError());
      return;
    }
    textureFromImageMsg(msg, texture_);
    // TODO(lucasw) keep this around in case next message is same size and type?
    SDL_DestroyTexture(texture_);
  }

  void textureFromImageMsg(const sensor_msgs::ImageConstPtr& msg, SDL_Texture* texture) {
    Uint32 format;
    int access;
    int w;
    int h;
    SDL_QueryTexture(texture, &format, &access, &w, &h);
    const size_t msg_width = msg->width;
    const size_t msg_height = msg->height;
    const size_t chan = (format == SDL_PIXELFORMAT_BGRA8888) ? 4 : 3;
    ROS_INFO_STREAM(SDL_GetPixelFormatName(format) << " " << chan);

#if 1
    size_t pitch;
    auto pixels = makeTestPixels(msg_width, msg_height, chan, pitch);
    const int rv0 = SDL_UpdateTexture(texture, nullptr, pixels.data(), pitch);
#else
    const size_t pitch = msg->step;
    // This seg faults unless 50 lines of padding are added to the texture
    const int rv0 = SDL_UpdateTexture(texture, nullptr, msg->data.data(), pitch);
#endif
    if (rv0 != 0) {
      ROS_ERROR_STREAM("sdl update texture failed: " << SDL_GetError());
      return;
    }

    int wd;
    int ht;
    SDL_GetRendererOutputSize(renderer_, &wd, &ht);
    if ((wd == 0) || (ht == 0)) {
      ROS_WARN_STREAM("zero in output size " << wd << " " << ht);
      return;
    }
    const float renderer_aspect = static_cast<float>(wd) / static_cast<float>(ht);
    const float msg_aspect = static_cast<float>(msg_width) / static_cast<float>(msg_height);
    SDL_Rect dst_rect;
    dst_rect.x = 0;
    dst_rect.y = 0;
    dst_rect.w = wd;
    dst_rect.h = ht;
    if (msg_aspect == renderer_aspect) {
    } else if (msg_aspect > renderer_aspect) {
      dst_rect.h = wd / msg_aspect;
      dst_rect.y = (ht - dst_rect.h) / 2;
    } else if (msg_aspect < renderer_aspect) {
      dst_rect.w = msg_aspect * ht;
      dst_rect.x = (wd - dst_rect.w) / 2;
    }
    ROS_INFO_STREAM_THROTTLE(10.0, "update "
        << "msg " << msg_width << " " << msg_height << " " << msg->step << " " << msg->data.size() << ", "
        << " x " << dst_rect.x << " y " << dst_rect.y << ","
        << " w " << dst_rect.w << " h " << dst_rect.h);
    const int rv1 = SDL_RenderCopy(renderer_, texture, NULL, &dst_rect);
    if (rv1 != 0) {
      ROS_ERROR_STREAM("sdl render copy failed: " << SDL_GetError());
      return;
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    msg_ = msg;

    // this is still taking up too much cpu- maybe the vsync?
    const auto t0 = ros::Time::now();

    if (msg->encoding != "bgr8") {
      ROS_WARN_STREAM_THROTTLE(4.0, "can't handle non-bgr8 currently " << msg->encoding);
      return;
    }

    // ROS_INFO_STREAM("msg " << msg->encoding << " " << msg->width << " " << msg->height
    //                  << " " << msg->step << " " << msg->data.size());
    if (msg->data.size() != msg->height * msg->step) {
      ROS_WARN_STREAM("msg " << msg->width << " " << msg->height << " " << msg->step << " " << msg->data.size());
      // TODO(lucasw) later show partial image known to be safe
      return;
    }

    setSurface(msg);
    // setTexture(msg);
    SDL_RenderPresent(renderer_);

    const auto t1 = ros::Time::now();
    // ROS_INFO_STREAM("image update " << (t1 - t0).toSec());
  }

  void update(const ros::TimerEvent& event) {
    ROS_INFO_STREAM_THROTTLE(5.0, "update");
  }

  const Uint32 width = 2500;
  const Uint32 height = 1000;

  SDL_Window* win_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Surface* surface_ = nullptr;
  SDL_Texture* texture_ = nullptr;

  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  sensor_msgs::ImageConstPtr msg_ = nullptr;
  ros::Timer timer_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "view_image");
  ViewImage view_image;
  ros::spin();
  return 0;
}
