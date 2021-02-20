#include <SDL.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

struct ViewImage {
  ViewImage() {
    SDL_Init(SDL_INIT_EVERYTHING);
    win_ = SDL_CreateWindow("view_image", 50, 50, width, height, SDL_WINDOW_SHOWN);
    if (!win_) {
      SDL_Quit();
    }
    // Disabling these two doesn't help performance
    ren_ = SDL_CreateRenderer(win_, -1, 0);  // SDL_RENDERER_ACCELERATED);  // | SDL_RENDERER_PRESENTVSYNC);
    if (!ren_) {
      SDL_Quit();
    }
    surface_ = SDL_GetWindowSurface(win_);

    // SDL_SetRenderDrawColor(ren_, 100, 50, 20, 255);
    // SDL_RenderClear(ren_);
    {
      const auto t0 = ros::Time::now();
      buffer_ = SDL_CreateTexture(
          ren_,
          SDL_PIXELFORMAT_BGRA8888,
          // SDL_TEXTUREACCESS_STREAMING,
          SDL_TEXTUREACCESS_STATIC,
          width,
          height);

      // this is okay for speed, mabye
      // std::array<int, width * height> pixels;
      std::vector<int> pixels;
      pixels.resize(width * height);
      for (size_t i = 0; i < pixels.size(); ++i ) {
        pixels[i] = 0x442288ff;
      }
      ROS_INFO_STREAM("int size: " << sizeof(int));
      int* pixels_ptr = pixels.data();
      int pitch = width * sizeof(int);
      // SDL_LockTexture(buffer_, NULL, reinterpret_cast<void**>(&pixels_ptr), &pitch);
      // SDL_UnlockTexture(buffer_);
      SDL_UpdateTexture(buffer_, NULL, pixels.data(), pitch);
      SDL_RenderCopy(ren_, buffer_, NULL, NULL);

/*
      // This is super slow
      const auto t0 = ros::Time::now();
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          SDL_SetRenderDrawColor(ren_, 255, x % 255, 0, 255);
          SDL_RenderDrawPoint(ren_, x, y);
        }
      }
*/
      const auto t1 = ros::Time::now();
      SDL_RenderPresent(ren_);
      ROS_INFO_STREAM((t1 - t0).toSec());
    }

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

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    msg_ = msg;

    // this is still taking up too much cpu- maybe the vsync?
    const auto t0 = ros::Time::now();
    buffer_ = SDL_CreateTexture(
        ren_,
        SDL_PIXELFORMAT_BGRA8888,
        // SDL_TEXTUREACCESS_STREAMING,
        SDL_TEXTUREACCESS_STATIC,
        msg->width,
        msg->height);

    // std::array<int, msg->width * msg->height> pixels;
    std::vector<int> pixels;
    pixels.resize(msg->width * msg->height);

    for (size_t i = 0; i < pixels.size(); ++i ) {
      size_t msg_ind = i * 3;
      pixels[i] = (msg->data[msg_ind + 0] << 24 |
                   msg->data[msg_ind + 1] << 16 |
                   msg->data[msg_ind + 2] << 8 |
                   0xff);
    }
    int* pixels_ptr = pixels.data();
    int pitch = msg->width * sizeof(int);
    // SDL_LockTexture(buffer_, NULL, reinterpret_cast<void**>(&pixels_ptr), &pitch);
    // SDL_UnlockTexture(buffer_);
    SDL_UpdateTexture(buffer_, NULL, pixels.data(), pitch);

    // SDL_SetRenderDrawColor(ren_, 100, 50, 20, 255);
    // SDL_RenderClear(ren_);
    SDL_Rect dst_rect;
    dst_rect.x = 0;
    dst_rect.y = 0;
    dst_rect.w = std::min(msg->width, width);
    dst_rect.h = std::min(msg->height, height);
    SDL_Rect src_rect = dst_rect;
    SDL_RenderCopy(ren_, buffer_, &src_rect, &dst_rect);

    // SDL_UpdateWindowSurface(win_);
/*
    // This is super slow 10x slower than above
    const auto t0 = ros::Time::now();
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        SDL_SetRenderDrawColor(ren_, 255, x % 255, 0, 255);
        SDL_RenderDrawPoint(ren_, x, y);
      }
    }
*/
    const auto t1 = ros::Time::now();
    SDL_RenderPresent(ren_);
    ROS_INFO_STREAM("image update " << (t1 - t0).toSec());
    SDL_DestroyTexture(buffer_);
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
