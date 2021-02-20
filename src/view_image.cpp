#include <SDL.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

struct ViewImage {
  ViewImage() {
    SDL_Init(SDL_INIT_EVERYTHING);
    win_ = SDL_CreateWindow("view_image", 50, 50, width, height, SDL_WINDOW_SHOWN);
    if (!win_) {
      ROS_ERROR_STREAM(SDL_GetError());
      SDL_Quit();
      ros::shutdown();
    }
    // Disabling these two doesn't help performance
    ren_ = SDL_CreateRenderer(win_, -1, 0);  // SDL_RENDERER_ACCELERATED);  // | SDL_RENDERER_PRESENTVSYNC);
    if (!ren_) {
      ROS_ERROR_STREAM(SDL_GetError());
      SDL_Quit();
      ros::shutdown();
    }

    {
      SDL_RendererInfo info;
      SDL_GetRendererInfo(ren_, &info);
      ROS_INFO_STREAM("Renderer name: " << info.name);
      ROS_INFO_STREAM("Texture formats: ");
      for(Uint32 i = 0; i < info.num_texture_formats; i++) {
        ROS_INFO_STREAM(SDL_GetPixelFormatName(info.texture_formats[i]));
      }
    }

#if 0
    surface_ = SDL_GetWindowSurface(win_);
    if (!surface_) {
      // TODO(lucasw) do I need a surface?
      // No hardware accelerated renderers available
      ROS_ERROR_STREAM(SDL_GetError());
      SDL_Quit();
      ros::shutdown();
    }
#endif

    SDL_SetRenderDrawColor(ren_, 100, 50, 20, 255);
    SDL_RenderClear(ren_);
    SDL_RenderPresent(ren_);

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

  void textureFromImageMsg(const sensor_msgs::ImageConstPtr& msg, SDL_Texture* buffer) {
    std::vector<Uint8> pixels;
    pixels.resize(msg->step * msg->height);
    for (size_t i = 0; i < pixels.size() - 3; i+=3) {
    // for (size_t i = 0; i < pixels.size() - 4; i+=4) {
      // test pattern
      pixels[i] = (i % msg->step) % 255;
      pixels[i + 1] = 0xff;
      pixels[i + 2] = (i / msg->step) % 255;
      // pixels[i + 3] = 0xff;
    }

    SDL_Rect msg_rect;
    msg_rect.x = 0;
    msg_rect.y = 0;
    msg_rect.w = msg->width;
    msg_rect.h = msg->height;
    // This seg faults unless 50 lines of padding are added to the texture
    // SDL_UpdateTexture(buffer_, &msg_rect, msg->data.data(), msg->step);
    const int rv0 = SDL_UpdateTexture(buffer, &msg_rect, pixels.data(), msg->step);
    if (rv0 != 0) {
      ROS_ERROR_STREAM("sdl update texture failed: " << SDL_GetError());
      return;
    }
    SDL_Rect dst_rect;
    dst_rect.x = 0;
    dst_rect.y = 0;
    dst_rect.w = std::min(msg->width, width);
    dst_rect.h = std::min(msg->height, height);
    // ROS_INFO_STREAM("updated " << dst_rect.w << " " << dst_rect.h);
    SDL_Rect src_rect = dst_rect;
    const int rv1 = SDL_RenderCopy(ren_, buffer, &src_rect, &dst_rect);
    if (rv1 != 0) {
      ROS_ERROR_STREAM("sdl render copy failed: " << SDL_GetError());
      return;
    }
    SDL_RenderPresent(ren_);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    msg_ = msg;

    // this is still taking up too much cpu- maybe the vsync?
    const auto t0 = ros::Time::now();

    if (msg->encoding != "bgr8") {
      ROS_WARN_STREAM("can't handle non-bgr8 currently " << msg->encoding);
      return;
    }

    // ROS_INFO_STREAM("msg " << msg->encoding << " " << msg->width << " " << msg->height
    //                  << " " << msg->step << " " << msg->data.size());
    if (msg->data.size() != msg->height * msg->step) {
      ROS_WARN_STREAM("msg " << msg->width << " " << msg->height << " " << msg->step << " " << msg->data.size());
      // TODO(lucasw) later show partial image known to be safe
      return;
    }
    buffer_ = SDL_CreateTexture(
        ren_,
        // TODO(lucasw) need to match on the msg->encoding
        // This doesn't display correctly
        SDL_PIXELFORMAT_BGR888,
        // SDL_PIXELFORMAT_BGRA8888,
        // SDL_TEXTUREACCESS_STREAMING,
        SDL_TEXTUREACCESS_STATIC,
        msg->width,
        msg->height);
    if (buffer_ == nullptr) {
      ROS_ERROR_STREAM("sdl create texture failed: " << SDL_GetError());
      return;
    }
    textureFromImageMsg(msg, buffer_);
    // TODO(lucasw) keep this around in case next message is same size and type?
    SDL_DestroyTexture(buffer_);

    const auto t1 = ros::Time::now();
    // ROS_INFO_STREAM("image update " << (t1 - t0).toSec());
  }

  void update(const ros::TimerEvent& event) {
    ROS_INFO_STREAM_THROTTLE(5.0, "update");
  }

  const Uint32 width = 2000;
  const Uint32 height = 1000;

  SDL_Window* win_ = nullptr;
  SDL_Renderer* ren_ = nullptr;
  SDL_Surface* surface_ = nullptr;
  SDL_Texture* buffer_ = nullptr;

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
