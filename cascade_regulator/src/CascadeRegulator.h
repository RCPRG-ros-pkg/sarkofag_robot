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

#ifndef CASCADEREGULATOR_H_
#define CASCADEREGULATOR_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <string>

class CascadeRegulator : public RTT::TaskContext {
 public:
  explicit CascadeRegulator(const std::string& name);
  ~CascadeRegulator();

  int doServo_cas(double, double, int);
  void reset();

 private:
  bool configureHook();
  void updateHook();

  RTT::InputPort<double> desired_position_;
  RTT::InputPort<double> measured_position_;
  RTT::InputPort<double> deltaInc_in;
  RTT::InputPort<bool> synchro_state_in_;

  RTT::OutputPort<double> computedPwm_out;
  RTT::OutputPort<bool> emergency_stop_out_;

  double desired_position_increment_;
  double desired_position_old_, desired_position_new_;
  double measured_position_old_, measured_position_new_;
  double measured_increment_old_, measured_increment_new_;

  bool synchro_state_old_, synchro_state_new_;

  int64_t update_hook_iteration_number_;
  int64_t new_position_iteration_number_;

  // Properties
  int reg_number_;
  bool debug_;
  double KP_POS_;
  double TI_POS_;
  double TD_POS_;
  double KP_INC_;
  double TI_INC_;
  double TD_INC_;
  bool current_mode_;
  double max_output_current_;
  double current_reg_kp_;
  double output_multiplicator_;
  double max_desired_increment_;
  double enc_res_;

  double position_err_new;  // e_pos[k]
  double position_err_old;  // e_pos[k-1]
  double position_err_very_old;  // e_pos[k-2]

  double increment_err_new;  // e_inc[k]
  double increment_err_old;  // e_inc[k-1]
  double increment_err_very_old;  // e_inc[k-2]

  double position_set_value_new;  // u_pos[k]
  double position_set_value_old;  // u_pos[k-1]
  double increment_set_value_new;  // u_inc[k]
  double increment_set_value_old;  // u_inc[k-1]

  double output_value;  // y[k];

  double kp_pos, Ti_pos, Td_pos;
  double kp_inc, Ti_inc, Td_inc;
  double r0_pos, r1_pos, r2_pos;
  double r0_inc, r1_inc, r2_inc;

  double A_;
  double BB0_;
  double BB1_;
  double eint_dif_;

  double position_increment_old;
  double position_increment_new;
  double step_old_pulse;
  double step_new;
  double step_old;
  double set_value_new;
  double set_value_old;
  double set_value_very_old;
  double delta_eint;
  double delta_eint_old;

  double a_, b0_, b1_;
};
#endif  // CASCADEREGULATOR_H_
