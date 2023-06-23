/*
 * timer.cpp
 *
 *  Created on: 21.06.2023
 *      Author: Carst
 */
#include "timer.h"

static std::uint32_t tick;
void SysTick_Handler(void)
{
  tick++;
}

namespace CarstenKeller
{

  Timer::Timer(std::uint32_t timeout_ms, bool start, bool restart)
  {
    this->timeout_ms = timeout_ms;
    this->running = start;
    if(start)
    {
      this->timestamp = tick;
    }
    this->restart = restart;
  }

  Timer::~Timer()
  {

  }

  bool Timer::elapsed(void)
  {
    bool elapsed = false;
    if(this->running && ((this->timestamp + this->timeout_ms) < tick))
    {
      elapsed = true;
      if(this->restart)
      {
        this->timestamp = tick;
      }
      else
      {
        this->running = false;
      }
    }
    return elapsed;
  }

  void Timer::stop(void)
  {
    this->running = false;
  }

  void Timer::start(void)
  {
    this->running = true;
    this->timestamp = tick;
  }

  bool Timer::is_running(void)
  {
    return this->running;
  }
}
