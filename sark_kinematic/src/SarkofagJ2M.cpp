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

#include "../../sark_kinematic/src/SarkofagJ2M.h"

#include <rtt/Component.hpp>
#include <string>

#include "../../sark_kinematic/src/SarkofagTransmission.h"

SarkofagJ2M::SarkofagJ2M(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("JointPosition", port_joint_position_);
}

SarkofagJ2M::~SarkofagJ2M() {
}

bool SarkofagJ2M::configureHook() {
  motor_position_.resize(1);
  joint_position_.resize(1);
  return true;
}

void SarkofagJ2M::updateHook() {
  if (port_joint_position_.read(joint_position_) == RTT::NewData) {
    if (i2mp(&joint_position_(0), &motor_position_(0))) {
      port_motor_position_.write(motor_position_);
    }
  }
}

bool SarkofagJ2M::i2mp(const double* joints, double* motors) {
  motors[0] = (joints[0] + OFFSET) * GEAR ;
  return true;
}

ORO_CREATE_COMPONENT(SarkofagJ2M)

