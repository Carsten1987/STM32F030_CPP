/*
 * timer.h
 *
 *  Created on: 21.06.2023
 *      Author: Carst
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <cstdint>
namespace CarstenKeller
{
  class Timer {
  public:
      Timer(std::uint32_t timeout_ms, bool start = false, bool restart = true);
      virtual ~Timer();
      bool elapsed(void);
      void stop(void);
      void start(void);
      bool is_running(void);

  private:
      std::uint32_t timeout_ms;
      std::uint32_t timestamp;
      bool running;
      bool restart;
  };
}

#endif /* TIMER_H_ */
