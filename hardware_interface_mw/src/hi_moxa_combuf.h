/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HI_MOXA_COMBUF_H_
#define HI_MOXA_COMBUF_H_

#include <stdint.h>

namespace hi_moxa {

#define SERVO_ST_BUF_LEN 30
const char START_BYTE = '#';

// commands
const int COMMAND_MODE_PWM = 0x00;
const int COMMAND_MODE_CURRENT = 0x01;
const int COMMAND_MODE_POSITION = 0x02;
const int COMMAND_SET_PARAM = 0x0f;

// command parameters
const int COMMAND_PARAM_SYNCHRO = 0x10;

// SET_PARAM command parameters
const int PARAM_SYNCHRONIZED = 0x10;
const int PARAM_MAXCURRENT = 0x20;
const int PARAM_PID_POS_P = 0x30;
const int PARAM_PID_POS_I = 0x40;
const int PARAM_PID_POS_D = 0x50;
const int PARAM_PID_CURR_P = 0x60;
const int PARAM_PID_CURR_I = 0x70;
const int PARAM_PID_CURR_D = 0x80;
const int PARAM_DRIVER_MODE = 0x90;

// DRIVER_MODE values
const int PARAM_DRIVER_MODE_MANUAL = 0x00;
const int PARAM_DRIVER_MODE_PWM = 0x03;
const int PARAM_DRIVER_MODE_ERROR = 0x04;

// error flags returned by hi::read_write_hardware (defined in servo_gr.h)
const uint64_t ALL_RIGHT = 0x0000000000000000ULL;
const uint64_t SYNCHRO_ZERO = 0x0000000000000001ULL;
const uint64_t SYNCHRO_SWITCH_ON = 0x0000000000000002ULL;
const uint64_t LOWER_LIMIT_SWITCH = 0x0000000000000004ULL;
const uint64_t UPPER_LIMIT_SWITCH = 0x0000000000000008ULL;
const uint64_t OVER_CURRENT = 0x0000000000000010ULL;

struct status_St {
  uint8_t startByte;
  uint8_t sw1 :1;
  uint8_t sw2 :1;
  uint8_t swSynchr :1;
  uint8_t synchroZero :1;
  uint8_t powerStageFault :1;
  uint8_t overcurrent :1;
  uint8_t error :1;
  uint8_t isSynchronized :1;
  int16_t current;
  int32_t position;
}__attribute__((__packed__));

struct pwm_St {
  int16_t pwm;
}__attribute__((__packed__));

union param_Un {
  int8_t synchronized;
  int8_t driver_mode;
  int16_t maxcurrent;
  int16_t pid_coeff;
  int16_t largest;
}__attribute__((__packed__));
// packed dopisał słoń

struct servo_St {
  uint8_t txBuf[256];
  uint8_t txCnt;
  uint8_t commandArray[256];
  uint8_t commandCnt;
  int32_t current_absolute_position;
  int32_t previous_absolute_position;
  double current_position_inc;
  double previous_position_inc;
  int first_hardware_reads;
  bool trace_resolver_zero;
}__attribute__((__packed__));

}  // namespace hi_moxa

#endif  // HI_MOXA_COMBUF_H_
