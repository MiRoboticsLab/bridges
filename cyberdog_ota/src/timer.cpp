#include "cyberdog_ota/timer.hpp"
#include "cyberdog_ota/string.hpp"

#include <iostream>

using namespace std::chrono;

namespace cyberdog
{

Timer::Timer() : started_(false), paused_(false) {}

void Timer::Start() {
  started_ = true;
  paused_ = false;
  start_time_ = high_resolution_clock::now();
}

void Timer::Restart() {
  started_ = false;
  Start();
}

void Timer::Pause() {
  paused_ = true;
  pause_time_ = high_resolution_clock::now();
}

void Timer::Resume() {
  paused_ = false;
  start_time_ += high_resolution_clock::now() - pause_time_;
}

void Timer::Reset() {
  started_ = false;
  paused_ = false;
}

double Timer::ElapsedMicroSeconds() const {
  if (!started_) {
    return 0.0;
  }
  if (paused_) {
    return duration_cast<microseconds>(pause_time_ - start_time_).count();
  } else {
    return duration_cast<microseconds>(high_resolution_clock::now() -
                                       start_time_)
        .count();
  }
}

double Timer::ElapsedSeconds() const { return ElapsedMicroSeconds() / 1e6; }

double Timer::ElapsedMinutes() const { return ElapsedSeconds() / 60; }

double Timer::ElapsedHours() const { return ElapsedMinutes() / 60; }

void Timer::PrintSeconds() const 
{
  std::cout << StringPrintf("Elapsed time: %.5f [seconds]", ElapsedSeconds())
            << std::endl;
}

void Timer::PrintMinutes() const 
{
  std::cout << StringPrintf("Elapsed time: %.3f [minutes]", ElapsedMinutes())
            << std::endl;
}

void Timer::PrintHours() const 
{
  std::cout << StringPrintf("Elapsed time: %.3f [hours]", ElapsedHours())
            << std::endl;
}

}  // namespace colmap
