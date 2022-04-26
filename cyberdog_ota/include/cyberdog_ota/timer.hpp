#ifndef CYBERDOG_OTA_TIMER_HPP_
#define CYBERDOG_OTA_TIMER_HPP_

#include <chrono>

namespace cyberdog
{

class Timer 
{
public:
  Timer();

  void Start();
  void Restart();
  void Pause();
  void Resume();
  void Reset();

  double ElapsedMicroSeconds() const;
  double ElapsedSeconds() const;
  double ElapsedMinutes() const;
  double ElapsedHours() const;

  void PrintSeconds() const;
  void PrintMinutes() const;
  void PrintHours() const;

private:
  bool started_;
  bool paused_;
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::high_resolution_clock::time_point pause_time_;
};

}  // namespace colmap

#endif  // CYBERDOG_OTA_TIMER_HPP_
