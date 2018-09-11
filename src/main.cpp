/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @brief This file is based on the MAPLE MINI example from ChibiOS
 * 
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-11
 */

#include "ch.hpp"

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"
#include <stdlib.h>
#include <cstring>

#include "MPU6050.h"
#include "CanBusHandler.hpp"

using namespace chibios_rt;

/**
 * Commands
 */

/**
 * @brief the working area for the command shell thread, where the thread's own heap and stack is located
 */
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static void setCurrent(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 1)
  {
    CanBusHandler::current_1 =
        strtol(argv[0], NULL, 0);
  }
}

bool logEnable = true;

static void toggleLog(BaseSequentialStream *chp, int argc, char *argv[])
{
  logEnable = !logEnable;
  chprintf(chp, "\r\n\log mode %s\r\n", logEnable ? "started" : "stopped");
}

static const ShellCommand commands[] = {
    {"sc", setCurrent},
    {"log", toggleLog},
    {NULL, NULL}};

static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&SD1,
    commands};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Blinker thread, times are in milliseconds.
 * C++ version
 */
class BlinkerThread : public BaseStaticThread<128>
{
protected:
  void main(void) override
  {
    setName("blinker");

    while (true)
    {
      palClearLine(LINE_LED);
      chThdSleepMilliseconds(500);
      palSetLine(LINE_LED);
      chThdSleepMilliseconds(500);
    }
  }

public:
  BlinkerThread(void) : BaseStaticThread<128>()
  {
  }
};

/* C version :

static THD_WORKING_AREA(waThread1, 128);
static __attribute__((noreturn)) THD_FUNCTION(Thread1, arg)
{

  (void)arg;
  chRegSetThreadName("blinker");
  while (true)
  {
    palClearLine(LINE_LED);
    chThdSleepMilliseconds(500);
    palSetLine(LINE_LED);
    chThdSleepMilliseconds(500);
  }

} */

/** do not create thread objects inside functions if you dont know what you are doing
 * and do not define variables in header files, and do not define them twice, use extern to share variables in files
 * making sure that things are static
 * 
 * **Note that the keyword "static" acts differently in C and C++
 * while all global variables are actually static
 */
BlinkerThread blinker;

static const I2CConfig i2c2cfg = {
    OPMODE_I2C,
    100000,
    FAST_DUTY_CYCLE_2,
};

static const SerialConfig sd1Config =
    {115200,
     0,
     USART_CR2_STOP1_BITS,
     0};

/*
 * Application entry point.
 */
int main(void)
{

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the CAN driver 1, Serial Driver 1, and I2C 2.
   */
  sdStart(&SD1, &sd1Config);
  i2cStart(&I2CD2, &i2c2cfg);

  /*
   * Creates the blinker thread, C method. *just for reference
   * chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
   */

  /*
   * Creates the blinker thread, C++ method.
   */
  blinker.start(NORMALPRIO);

  /**
  * start the can bus test
  * 
  */
  CanBusHandler::caninit(&CAND1);

  sdWrite(&SD1, (const uint8_t *)"\n\n\n\n", 4);

  //initiallize the mpu6050 reader
  chThdSleepMilliseconds(500);
  MPU6050(MPU6050_ADDRESS_AD0_LOW);
  MPUinitialize();
  chprintf((BaseSequentialStream *)&SD1, "MPU6050 connection %s\n",
           MPUtestConnection() ? "OK" : "FAILED");

  chThdSleepMilliseconds(500);

  /*
   * Shell manager initialization.
   */
  shellInit();

  /**
   * @brief start the shell thread over UART1
   * notice that here the thread is created dynamically
   * calling this statement will occupy the heap of the caller thread
   */
  thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                          "shell", NORMALPRIO + 1,
                                          shellThread, (void *)&shell_cfg1);

  /*
   * Normal main() thread activity
   */
  int16_t rx = 0, ry = 0, rz = 0, ax = 0, ay = 0, az = 0;
  while (true)
  {
    systime_t startT = chibios_rt::System::getTime();
    MPUgetRotation(&rx, &ry, &rx);
    MPUgetAcceleration(&ax, &ay, &az);
    if (logEnable)
    {
      chprintf((BaseSequentialStream *)&SD1,
               "rx: %d ry:%d rz:%d ax: %d ay:%d az:%d\n",
               rx, ry, rz, ax, ay, az);
      chprintf((BaseSequentialStream *)&SD1,
               "rCnt: %D\n", CanBusHandler::receiveCount);
      chprintf((BaseSequentialStream *)&SD1,
               "rid: %d\n", CanBusHandler::f.SID);
      chprintf((BaseSequentialStream *)&SD1,
               "chibios_rt::System::getTime: %d\n", startT);
    }
    chibios_rt::BaseThread::sleepUntil(startT + TIME_MS2I(500));
    //chThdSleepMilliseconds(500);
  }
}
