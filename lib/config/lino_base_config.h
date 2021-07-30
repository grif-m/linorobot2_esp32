// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define LED_PIN 13 //used for debugging status

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER            // 4WD robot
// #define LINO_BASE MECANUM               // Mecanum drive robot

//uncomment the motor driver you're using
#define USE_GENERIC_2_IN_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
// #define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

//uncomment the IMU you're using
#define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define K_P 0.6                             // P constant
#define K_I 0.8                             // I constant
#define K_D 0.5                             // D constant

//define your robot' specs here
#define MAX_RPM 100                         // motor's maximum RPM
#define COUNTS_PER_REV 2200                 // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.090                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.200            // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.300            // distance between front and rear wheels. Ignore this if you're on 2WD
#define PWM_BITS 8                          // PWM Resolution of the microcontroller

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

/// ENCODER PINS
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B 15 
#define MOTOR1_ENCODER_INV false 

#define MOTOR2_ENCODER_A 11
#define MOTOR2_ENCODER_B 12 
#define MOTOR2_ENCODER_INV false 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 
#define MOTOR3_ENCODER_INV false 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10
#define MOTOR4_ENCODER_INV false 

//MOTOR PINS
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
  #define MOTOR_DRIVER L298

  #define MOTOR1_PWM 21
  #define MOTOR1_IN_A 1
  #define MOTOR1_IN_B 20
  #define MOTOR1_INV false

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8
  #define MOTOR2_INV false

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0
  #define MOTOR3_INV false

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 3
  #define MOTOR4_INV false

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
  #define MOTOR_DRIVER L298

  #define MOTOR1_PWM 21
  #define MOTOR1_IN_A 1
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_INV false

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_INV false

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_INV false

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_INV false

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR_DRIVER BTS7960  

  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 21
  #define MOTOR1_IN_B 20
  #define MOTOR1_INV false

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6
  #define MOTOR2_INV false

  #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23
  #define MOTOR3_INV false

  #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3
  #define MOTOR4_INV false

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_ESC_MOTOR_DRIVER
  #define MOTOR_DRIVER ESC  

  #define MOTOR1_PWM 1 
  #define MOTOR1_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_INV false

  #define MOTOR2_PWM 8 
  #define MOTOR2_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_INV false

  #define MOTOR3_PWM 0 
  #define MOTOR3_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_INV false

  #define MOTOR4_PWM 2 
  #define MOTOR4_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_INV false

  #define PWM_MAX 400
  #define PWM_MIN -PWM_MAX
#endif

#define STEERING_PIN 7

#endif
