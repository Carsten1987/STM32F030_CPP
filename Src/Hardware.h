/*
 * Hardware.h
 *
 *  Created on: Dec 27, 2022
 *      Author: Carst
 */

#ifndef SRC_HARDWARE_H_
#define SRC_HARDWARE_H_

#include <cstdint>

namespace Beispiel
{
  class Hardware
  {
    public:
      Hardware();
      virtual ~Hardware();
      void run(void);
  };
}

#endif /* SRC_HARDWARE_H_ */
