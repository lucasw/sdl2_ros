#include <SDL.h>
#include <SDL_mixer.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sdl2_mixer");
  ros::NodeHandle nh;

  if (SDL_Init(SDL_INIT_AUDIO) < 0)
  {
    ROS_ERROR_STREAM("Couldn't initialize SDL " << SDL_GetError());
    return -1;
  }

  if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0)
  {
    ROS_ERROR_STREAM("Couldn't initialize SDL audio " << Mix_GetError());
    return -2;
  }

  std::string sound_file = "test.wav";
  ros::param::get("~sound", sound_file);

  // std::unique_ptr<Mix_Chunk> sound(Mix_LoadWAV(sound_file));
  Mix_Chunk* sound = NULL;
  sound = Mix_LoadWAV(sound_file.c_str());
  if (!sound)
  {
    ROS_ERROR_STREAM("Couldn't load sound" << sound_file << ", " << Mix_GetError());
    return -3;
  }

  Mix_PlayChannel(-1, sound, 0);
  // ros::Duration(1.0).sleep();
  while (ros::ok())
  {
    if (!Mix_Playing(-1))
      break;
    ros::Duration(0.2).sleep();
  }
  // shutdown
  Mix_FreeChunk(sound);
  sound = NULL;
  Mix_Quit();
  SDL_Quit();
  return 0;
}
