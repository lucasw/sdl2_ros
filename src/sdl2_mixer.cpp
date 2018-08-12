#include <SDL.h>
#include <SDL_mixer.h>
#include <std_msgs/Int16MultiArray.h>
#include <ros/ros.h>

// combined ros message and sdl mixer audio format
struct AudioChunk
{
  AudioChunk(const std_msgs::Int16MultiArray::ConstPtr& audio) :
    audio(audio)
  {
    chunk.allocated = 0;
    chunk.abuf = reinterpret_cast<Uint8*>(const_cast<int16_t*>(&audio->data[0]));
    chunk.alen = audio->data.size() * sizeof(int16_t) / sizeof(Uint8);
    chunk.volume = 100;  // out of 128
  }
  // audio_common_msgs::AudioData::ConstPtr audio_data;
  std_msgs::Int16MultiArray::ConstPtr audio;
  Mix_Chunk chunk;
};

class Mixer
{
public:
  Mixer()
  {
    if (SDL_Init(SDL_INIT_AUDIO) < 0)
    {
      std::string msg = "Couldn't initialize SDL: ";
      msg += SDL_GetError();
      throw std::runtime_error(msg);
    }

    if (Mix_OpenAudio(MIX_DEFAULT_FREQUENCY, MIX_DEFAULT_FORMAT,
        MIX_DEFAULT_CHANNELS, 2048) < 0)
    {
      std::string msg = "Couldn't open SDL audio: ";
      msg += Mix_GetError();
      throw std::runtime_error(msg);
    }

    audio_sub_ = nh_.subscribe("audio", 3, &Mixer::audioCallback, this);
  }

  void audioCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
  {
    ROS_INFO_STREAM(msg->data.size());
    AudioChunk audio_chunk(msg);
    const int channel = Mix_PlayChannel(-1, &audio_chunk.chunk, 0);
    while (Mix_Playing(channel))
    {
      ros::Duration(0.2).sleep();
    }
    ROS_INFO_STREAM("finished");
  }
#if 0
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
#endif

  ~Mixer()
  {
    // shutdown
    // Mix_FreeChunk(sound);
    // sound = NULL;
    Mix_Quit();
    SDL_Quit();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber audio_sub_;
};

int main(int argc, char **argv)
{
  // don't hear anything with these
  // std::cout << '\a' << std::endl;
  // std::cout << '\7' << std::endl;

  ros::init(argc, argv, "sdl2_mixer");
  Mixer mixer;
  ros::spin();
  return 0;
}
