#include <SDL.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

struct ViewImage {
  ViewImage() {
    SDL_Init(SDL_INIT_EVERYTHING);
    const size_t width = 2000;
    const size_t height = 1000;
    win_ = SDL_CreateWindow("view_image", 50, 50, width, height, SDL_WINDOW_SHOWN);
    if (!win_) {
      SDL_Quit();
    }
    ren_ = SDL_CreateRenderer(win_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren_) {
      SDL_Quit();
    }
    surface_ = SDL_GetWindowSurface(win_);

    {
      SDL_SetRenderDrawColor(ren_, 100, 50, 20, 255);
      SDL_RenderClear(ren_);
#if 0
      buffer_ = SDL_CreateTexture(
          ren_,
          SDL_PIXELFORMAT_BGRA8888,
          SDL_TEXTUREACCESS_STREAMING,
          width,
          height);
      // SDL_BlitSurface(buffer_, NULL, surface_, NULL);

      std::array<int, width * height> pixels;
      for (size_t i = 0; i < pixels.size(); ++i ) {
        pixels[i] = 0xff0ff0ff;
      }
      ROS_INFO_STREAM("int size: " << sizeof(int));
      int* pixels_ptr = pixels.data();
      int pitch = width * sizeof(int);
      SDL_LockTexture(buffer_, NULL, reinterpret_cast<void**>(&pixels_ptr), &pitch);
      SDL_UnlockTexture(buffer_);
      SDL_RenderCopy(ren_, buffer_, NULL, NULL);
#endif
      SDL_RenderPresent(ren_);
      // SDL_UpdateWindowSurface(win_);
    }

    nh_.subscribe("image", 2, &ViewImage::imageCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &ViewImage::update, this);
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

  }

  void update(const ros::TimerEvent& event) {
  }

  SDL_Window* win_ = nullptr;
  SDL_Renderer* ren_ = nullptr;
  SDL_Surface* surface_ = nullptr;
  SDL_Texture* buffer_ = nullptr;

  ros::NodeHandle nh_;
  sensor_msgs::ImageConstPtr msg_ = nullptr;
  ros::Timer timer_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "view_image");
  ViewImage view_image;
  ros::spin();
  return 0;
}
