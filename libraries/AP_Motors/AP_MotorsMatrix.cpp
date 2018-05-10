/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include "../AP_HAL/AP_HAL.h"
#include "AP_MotorsMatrix.h"

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsMatrix::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // record requested frame class and type
    _last_frame_class = frame_class;
    _last_frame_type = frame_type;

    // setup the motors
    setup_motors(frame_class, frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate( uint16_t speed_hz )
{
    uint8_t i;

    // record requested speed
    _speed_hz = speed_hz;

    // check each enabled motor
    uint32_t mask = 0;
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
		mask |= 1U << i;
        }
    }
    rc_set_freq( mask, _speed_hz );
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsMatrix::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // exit immediately if armed or no change
    if (armed() || (frame_class == _last_frame_class && _last_frame_type == frame_type)) {
        return;
    }
    _last_frame_class = frame_class;
    _last_frame_type = frame_type;

    // setup the motors
    setup_motors(frame_class, frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsMatrix::enable()
{
    int8_t i;

    // enable output channels
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_enable_ch(i);
        }
    }
}

void AP_MotorsMatrix::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_mode) {
        case SHUT_DOWN: {
            // sends minimum values out to the motors
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                        motor_out[i] = 0;
                    } else {
                        motor_out[i] = get_pwm_output_min();
                    }
                }
            }
            break;
        }
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_spin_up_to_pwm();
                }
            }
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                }
            }
            break;
    }

    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
                rc_write(i, motor_out[i]);
        }
    }
}

/////////////////////////////////////
// MURILLO
/////////////////////////////////////
void AP_MotorsMatrix::output_to_motors(int srv_5, int srv_6, int srv_7, int srv_8)
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_mode) {
        case SHUT_DOWN: {
            // sends minimum values out to the motors
            // set motor output based on thrust requests
            // MURILLO: Coloquei 4 no limite do FOR
            for (i=0; i<4; i++) {
                if (motor_enabled[i]) {
                    if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                        motor_out[i] = 0;
                    } else {
                        motor_out[i] = get_pwm_output_min();
                    }
                }
            }
            break;
        }
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            // MURILLO: Coloquei 4 no limite do FOR
            for (i=0; i<4; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_spin_up_to_pwm();
                }
            }
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            // MURILLO: Coloquei 4 no limite do FOR
            for (i=0; i<4; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                }
            }
            break;
    }
    /////////////////////////////////////
    // MURILLO
    /////////////////////////////////////
    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
                rc_write(i, motor_out[i]);
        }
    }
    // Writing on Aux_Out Channels
    hal.rcout->write(8,srv_5);  // Servo 5
    hal.rcout->write(9,srv_6);  // Servo 6
    hal.rcout->write(10,srv_7); // Servo 7
    hal.rcout->write(11,srv_8); // Servo 8
}


// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsMatrix::get_motor_mask()
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }
    return rc_map_mask(mask);
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   rpy_low = 0.0f;             // lowest motor value
    float   rpy_high = 0.0f;            // highest motor value
    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
    float   unused_range;               // amount of yaw we can fit in the current channel
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain();
    throttle_thrust = get_throttle() * get_compensation_gain();

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller

    throttle_thrust_best_rpy = MIN(0.5f, _throttle_avg_max);

    // calculate roll and pitch for each motor
    // calculate the amount of yaw input that each motor can accept
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            if (!is_zero(_yaw_factor[i])){
                if (yaw_thrust * _yaw_factor[i] > 0.0f) {
                    unused_range = fabsf((1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]))/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                } else {
                    unused_range = fabsf((throttle_thrust_best_rpy + _thrust_rpyt_out[i])/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                }
            }
        }
    }

    // todo: make _yaw_headroom 0 to 1
    yaw_allowed = MAX(yaw_allowed, (float)_yaw_headroom/1000.0f);

    if (fabsf(yaw_thrust) > yaw_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw to intermediate numbers for each motor
    rpy_low = 0.0f;
    rpy_high = 0.0f;
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] > rpy_high) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }

    // check everything fits
    throttle_thrust_best_rpy = MIN(0.5f - (rpy_low+rpy_high)/2.0, _throttle_avg_max);
    if (is_zero(rpy_low)){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy/rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f){
        // Full range is being used by roll, pitch, and yaw.
        limit.roll_pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < -(throttle_thrust_best_rpy+rpy_low)){
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy+rpy_low);
        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy+rpy_high)){
            // Throttle can't be increased to desired value
            thr_adj = 1.0f - (throttle_thrust_best_rpy+rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + rpy_scale*_thrust_rpyt_out[i];
        }
    }

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);
        }
    }
}

//MURILLO
// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing(float &vlr_yaw)
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   rpy_low = 0.0f;             // lowest motor value
    float   rpy_high = 0.0f;            // highest motor value
    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
    float   unused_range;               // amount of yaw we can fit in the current channel
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
//    yaw_thrust = _yaw_in * get_compensation_gain();
    yaw_thrust = 0;
    throttle_thrust = get_throttle() * get_compensation_gain();
    // MURILLO
    //throttle_thrust = (1.0 + (fabsf(vlr_yaw)/10)/100) * get_throttle() * get_compensation_gain();
    vlr_yaw = _yaw_in * get_compensation_gain();

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller

    throttle_thrust_best_rpy = MIN(0.5f, _throttle_avg_max);

    // calculate roll and pitch for each motor
    // calculate the amount of yaw input that each motor can accept
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            if (!is_zero(_yaw_factor[i])){
                if (yaw_thrust * _yaw_factor[i] > 0.0f) {
                    unused_range = fabsf((1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]))/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                } else {
                    unused_range = fabsf((throttle_thrust_best_rpy + _thrust_rpyt_out[i])/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                }
            }
        }
    }

    // todo: make _yaw_headroom 0 to 1
    yaw_allowed = MAX(yaw_allowed, (float)_yaw_headroom/1000.0f);

    if (fabsf(yaw_thrust) > yaw_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw to intermediate numbers for each motor
    rpy_low = 0.0f;
    rpy_high = 0.0f;
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] > rpy_high) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }

    // check everything fits
    throttle_thrust_best_rpy = MIN(0.5f - (rpy_low+rpy_high)/2.0, _throttle_avg_max);
    if (is_zero(rpy_low)){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy/rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f){
        // Full range is being used by roll, pitch, and yaw.
        limit.roll_pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < -(throttle_thrust_best_rpy+rpy_low)){
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy+rpy_low);
        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy+rpy_high)){
            // Throttle can't be increased to desired value
            thr_adj = 1.0f - (throttle_thrust_best_rpy+rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + rpy_scale*_thrust_rpyt_out[i];
        }
    }

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);
        }
    }
}

//MURILLO
// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing(float &mtr1, float &mtr2, float &mtr3, float &mtr4, float &Fx, float &thr_MFS, float &roll_MFS, float &pitch_MFS, float &yaw_MFS, float &srv5, float &srv6, float &srv7, float &srv8, int tp)
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
//    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
//    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
//    float   rpy_low = 0.0f;             // lowest motor value
//    float   rpy_high = 0.0f;            // highest motor value
//    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
//    float   unused_range;               // amount of yaw we can fit in the current channel
//    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // MURILLO
    //float vlr_pitch;
//    float mtr1=0.0, mtr2=0.0, mtr3=0.0, mtr4=0.0;
//    float Fx=0.0, roll_MFS=0.0, pitch_MFS=0.0, yaw_MFS=0.0;
//    float thr_MFS=0.0;

    // Saving last motors signals
    mtr1 = constrain_float(_thrust_rpyt_out[0], 0.2f, 1.0f);
    mtr2 = constrain_float(_thrust_rpyt_out[1], 0.2f, 1.0f);
    mtr3 = constrain_float(_thrust_rpyt_out[2], 0.2f, 1.0f);
    mtr4 = constrain_float(_thrust_rpyt_out[3], 0.2f, 1.0f);

    // apply voltage and air pressure compensation
    roll_thrust  = _roll_in;// * get_compensation_gain();
    pitch_thrust = _pitch_in;// * get_compensation_gain();
    yaw_thrust   = _yaw_in;// * get_compensation_gain();
    throttle_thrust = get_throttle();// * get_compensation_gain();

//     MURILLO
//    throttle_thrust = (1.0 + (fabsf(vlr_yaw)/10)/100) * get_throttle() * get_compensation_gain();

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

//    _throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);

//    // calculate throttle that gives most possible room for yaw which is the lower of:
//    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
//    //      2. the higher of:
//    //            a) the pilot's throttle input
//    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
//    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
//    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
//    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
//    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

//    // calculate amount of yaw we can fit into the throttle range
//    // this is always equal to or less than the requested yaw from the pilot or rate controller

//    throttle_thrust_best_rpy = MIN(0.5f, _throttle_avg_max);

//    // calculate roll and pitch for each motor
//    // calculate the amount of yaw input that each motor can accept
//    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
//        if (motor_enabled[i]) {
//            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
//            if (!is_zero(_yaw_factor[i])){
//                if (yaw_thrust * _yaw_factor[i] > 0.0f) {
//                    unused_range = fabsf((1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]))/_yaw_factor[i]);
//                    if (yaw_allowed > unused_range) {
//                        yaw_allowed = unused_range;
//                    }
//                } else {
//                    unused_range = fabsf((throttle_thrust_best_rpy + _thrust_rpyt_out[i])/_yaw_factor[i]);
//                    if (yaw_allowed > unused_range) {
//                        yaw_allowed = unused_range;
//                    }
//                }
//            }
//        }
//    }

//    // todo: make _yaw_headroom 0 to 1
//    yaw_allowed = MAX(yaw_allowed, (float)_yaw_headroom/1000.0f);

//    if (fabsf(yaw_thrust) > yaw_allowed) {
//        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
//        limit.yaw = true;
//    }

//    // add yaw to intermediate numbers for each motor
//    rpy_low = 0.0f;
//    rpy_high = 0.0f;
//    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
//        if (motor_enabled[i]) {
//            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

//            // record lowest roll+pitch+yaw command
//            if (_thrust_rpyt_out[i] < rpy_low) {
//                rpy_low = _thrust_rpyt_out[i];
//            }
//            // record highest roll+pitch+yaw command
//            if (_thrust_rpyt_out[i] > rpy_high) {
//                rpy_high = _thrust_rpyt_out[i];
//            }
//        }
//    }

//    // check everything fits
//    throttle_thrust_best_rpy = MIN(0.5f - (rpy_low+rpy_high)/2.0, _throttle_avg_max);
//    if (is_zero(rpy_low)){
//        rpy_scale = 1.0f;
//    } else {
//        rpy_scale = constrain_float(-throttle_thrust_best_rpy/rpy_low, 0.0f, 1.0f);
//    }

//    // calculate how close the motors can come to the desired throttle
//    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
//    if (rpy_scale < 1.0f){
//        // Full range is being used by roll, pitch, and yaw.
//        limit.roll_pitch = true;
//        limit.yaw = true;
//        if (thr_adj > 0.0f) {
//            limit.throttle_upper = true;
//        }
//        thr_adj = 0.0f;
//    } else {
//        if (thr_adj < -(throttle_thrust_best_rpy+rpy_low)){
//            // Throttle can't be reduced to desired value
//            thr_adj = -(throttle_thrust_best_rpy+rpy_low);
//        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy+rpy_high)){
//            // Throttle can't be increased to desired value
//            thr_adj = 1.0f - (throttle_thrust_best_rpy+rpy_high);
//            limit.throttle_upper = true;
//        }
//    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
//    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
//        if (motor_enabled[i]) {
//            _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + rpy_scale*_thrust_rpyt_out[i];
//        }
//    }

    // MURILLO

//    vlr_pitch  = tilt_angle_Vx()/(1928 - _mid_chn);
//    tilt_angle_full_TRUAV(mtr1, mtr2, mtr3, mtr4, srv5, srv6, srv7, srv8, Fx, Fz, T_roll, T_pitch, T_yaw);
//throttle_thrust/100

    // Checking if Normalized Throttle Control is <0.1.
//    if((throttle_thrust)<0.1){
//        throttle_thrust = 0.1;
//    }else{
//        throttle_thrust = throttle_thrust;
//    }

//    Fx = norm_Pitch_Channel(Fx);
    // Ajustando os valores para uma nova faixa de valores
//  return = norm_VLR_inputs(double inpt, double med_n, double max_n)

    Fx        = norm_VLR_inputs2(Fx, 0.0, 0.4);//0.3 para k1 = 0.25
//    norm_VLR_inputs1(double min_n, double max_n);
    thr_MFS   = norm_VLR_inputs1(throttle_thrust,0.2, 1.0);

//    norm_VLR_inputs2(double inpt, double med_n, double max_n)
    roll_MFS  = norm_VLR_inputs2(roll_thrust , 0.0, 0.05);// 0.09
    pitch_MFS = norm_VLR_inputs2(pitch_thrust, 0.0, 0.06);//0.1
    yaw_MFS   = norm_VLR_inputs2(yaw_thrust  , 0.0, 0.0175);

    // Arrumar o problema de estourar os valores dos motores dependendo das ações de controle.
//    tilt_angle_full_TRUAV(tp, mtr1, mtr2, mtr3, mtr4, srv5, srv6, srv7, srv8, 0, thr_MFS, 0, 0, 0);
    tilt_angle_full_TRUAV(tp, mtr1, mtr2, mtr3, mtr4, srv5, srv6, srv7, srv8, Fx, thr_MFS, roll_MFS, pitch_MFS, yaw_MFS);

    _thrust_rpyt_out[0] = mtr1;
    _thrust_rpyt_out[1] = mtr2;
    _thrust_rpyt_out[2] = mtr3;
    _thrust_rpyt_out[3] = mtr4;

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
//    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
    // MURILLO
    for (i=0; i<4; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);
        }
    }
}

///////////////////////
/// MURILLO ///
/////////////////////// (ot1, ot2, ot3, ot4, srv5, srv6, srv7, srv8, CANAL_PITCH, throttle_MFS, roll_MFS, pitch_MFS, yaw_MFS)
void AP_MotorsMatrix::tilt_angle_full_TRUAV(int tp, float &mtr1, float &mtr2, float &mtr3, float &mtr4, float &srv5, float &srv6, float &srv7, float &srv8, float Fx, float Fz, float T_roll, float T_pitch, float T_yaw)
{
    //////////////////////////////////////////
    // Saturação das variáveis de entrada
//    Fx      = constrain_float(Fx,      -0.9, 0.9);
//    Fz      = constrain_float(Fz,       0.0, 0.9);
//    T_roll  = constrain_float(T_roll,  -0.9, 0.9);
//    T_pitch = constrain_float(T_pitch, -0.9, 0.9);
//    T_yaw   = constrain_float(T_yaw,   -0.9, 0.9);
    //////////////////////////////////////////

    int8_t it, i;

    float out1=0.0, out2=0.0, out3=0.0, out4=0.0;
    float ag5_c=0.0, ag6_c=0.0, ag7_c=0.0, ag8_c=0.0;
    float ag5_s=0.0, ag6_s=0.0, ag7_s=0.0, ag8_s=0.0;
    float arcsin1=0.0, arcsin2=0.0, arcsin3=0.0, arcsin4=0.0;

    float a11=0.0, a12=0.0, a13=0.0, a14=0.0;//, a15=0.0;
    float a21=0.0, a22=0.0, a23=0.0, a24=0.0, a25=0.0;
    float a31=0.0, a32=0.0, a33=0.0, a34=0.0;

    // Guardando os valores atuais dos motores.
    _last_Out1 = mtr1;
    _last_Out2 = mtr2;
    _last_Out3 = mtr3;
    _last_Out4 = mtr4;

    if(tp==1){

        it = 15;

        for (i=0; i<it; i++) {
            // Será calculado os ângulos dos servos

            ///////////////////////////////////////////////////////////////////////
            /// Segunda parte do processo
            ///////////////////////////////////////////////////////////////////////
            // Obtendo os termos da inversa da matriz de obtenção dos arcossenos de inclinação dos servos.
            // Obtendo os arcossenos de inclinação do servos.

            a11 =  _k1*_last_Out1;
            a12 =  _k1*_last_Out2;
            a13 =  _k1*_last_Out3;
            a14 =  _k1*_last_Out4;
            //a15 =  0.0;

            a21 = -_k1*_last_Out1*_l_arm*sqrt_2_2;
            a22 =  _k1*_last_Out2*_l_arm*sqrt_2_2;
            a23 =  _k1*_last_Out3*_l_arm*sqrt_2_2;
            a24 = -_k1*_last_Out4*_l_arm*sqrt_2_2;
            a25 = _k2*_last_Out1*ag5_c + _k2*_last_Out2*ag6_c - _k2*_last_Out3*ag7_c - _k2*_last_Out4*ag8_c;

            arcsin1 = ((Fx*powf(a11,2)*powf(a22,2))/a11 + (Fx*powf(a11,2)*powf(a23,2))/a11 + (Fx*powf(a11,2)*powf(a24,2))/a11 + (Fx*powf(a11,2)*powf(a25,2))/a11 + (T_yaw*powf(a12,2)*powf(a21,2))/a21 + (T_yaw*powf(a13,2)*powf(a21,2))/a21 + (T_yaw*powf(a14,2)*powf(a21,2))/a21 - (Fx*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (Fx*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (Fx*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (T_yaw*a12*powf(a11,2)*powf(a22,2))/(a11*a22) - (T_yaw*a13*powf(a11,2)*powf(a23,2))/(a11*a23) - (T_yaw*a14*powf(a11,2)*powf(a24,2))/(a11*a24))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a13,2)*powf(a25,2) + powf(a14,2)*powf(a25,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24));
            arcsin2 = ((Fx*powf(a12,2)*powf(a21,2))/a12 + (Fx*powf(a12,2)*powf(a23,2))/a12 + (Fx*powf(a12,2)*powf(a24,2))/a12 + (Fx*powf(a12,2)*powf(a25,2))/a12 + (T_yaw*powf(a11,2)*powf(a22,2))/a22 + (T_yaw*powf(a13,2)*powf(a22,2))/a22 + (T_yaw*powf(a14,2)*powf(a22,2))/a22 - (Fx*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (Fx*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (Fx*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (T_yaw*a11*powf(a12,2)*powf(a21,2))/(a12*a21) - (T_yaw*a13*powf(a12,2)*powf(a23,2))/(a12*a23) - (T_yaw*a14*powf(a12,2)*powf(a24,2))/(a12*a24))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a13,2)*powf(a25,2) + powf(a14,2)*powf(a25,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24));
            arcsin3 = ((Fx*powf(a13,2)*powf(a21,2))/a13 + (Fx*powf(a13,2)*powf(a22,2))/a13 + (Fx*powf(a13,2)*powf(a24,2))/a13 + (Fx*powf(a13,2)*powf(a25,2))/a13 + (T_yaw*powf(a11,2)*powf(a23,2))/a23 + (T_yaw*powf(a12,2)*powf(a23,2))/a23 + (T_yaw*powf(a14,2)*powf(a23,2))/a23 - (Fx*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (Fx*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (Fx*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (T_yaw*a11*powf(a13,2)*powf(a21,2))/(a13*a21) - (T_yaw*a12*powf(a13,2)*powf(a22,2))/(a13*a22) - (T_yaw*a14*powf(a13,2)*powf(a24,2))/(a13*a24))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a13,2)*powf(a25,2) + powf(a14,2)*powf(a25,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24));
            arcsin4 = ((Fx*powf(a14,2)*powf(a21,2))/a14 + (Fx*powf(a14,2)*powf(a22,2))/a14 + (Fx*powf(a14,2)*powf(a23,2))/a14 + (Fx*powf(a14,2)*powf(a25,2))/a14 + (T_yaw*powf(a11,2)*powf(a24,2))/a24 + (T_yaw*powf(a12,2)*powf(a24,2))/a24 + (T_yaw*powf(a13,2)*powf(a24,2))/a24 - (Fx*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (Fx*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (Fx*a23*powf(a13,2)*powf(a24,2))/(a13*a24) - (T_yaw*a11*powf(a14,2)*powf(a21,2))/(a14*a21) - (T_yaw*a12*powf(a14,2)*powf(a22,2))/(a14*a22) - (T_yaw*a13*powf(a14,2)*powf(a23,2))/(a14*a23))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a13,2)*powf(a25,2) + powf(a14,2)*powf(a25,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24));

//            arcsin1 = ((Fx*powf(a11,2)*powf(a22,2))/a11 + (Fx*powf(a11,2)*powf(a23,2))/a11 + (Fx*powf(a11,2)*powf(a24,2))/a11 + (Fx*powf(a11,2)*powf(a25,2))/a11 + (T_yaw*powf(a12,2)*powf(a21,2))/a21 + (T_yaw*powf(a13,2)*powf(a21,2))/a21 + (T_yaw*powf(a14,2)*powf(a21,2))/a21 + (T_yaw*powf(a15,2)*powf(a21,2))/a21 - (Fx*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (Fx*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (Fx*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (Fx*a25*powf(a15,2)*powf(a21,2))/(a15*a21) - (T_yaw*a12*powf(a11,2)*powf(a22,2))/(a11*a22) - (T_yaw*a13*powf(a11,2)*powf(a23,2))/(a11*a23) - (T_yaw*a14*powf(a11,2)*powf(a24,2))/(a11*a24) - (T_yaw*a15*powf(a11,2)*powf(a25,2))/(a11*a25))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a15,2)*powf(a21,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a15,2)*powf(a22,2) + powf(a13,2)*powf(a25,2) + powf(a15,2)*powf(a23,2) + powf(a14,2)*powf(a25,2) + powf(a15,2)*powf(a24,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a11*a25*powf(a15,2)*powf(a21,2))/(a15*a21) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a15*a21*powf(a11,2)*powf(a25,2))/(a11*a25) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a12*a25*powf(a15,2)*powf(a22,2))/(a15*a22) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a15*a22*powf(a12,2)*powf(a25,2))/(a12*a25) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24) - (a13*a25*powf(a15,2)*powf(a23,2))/(a15*a23) - (a15*a23*powf(a13,2)*powf(a25,2))/(a13*a25) - (a14*a25*powf(a15,2)*powf(a24,2))/(a15*a24) - (a15*a24*powf(a14,2)*powf(a25,2))/(a14*a25));
//            arcsin2 = ((Fx*powf(a12,2)*powf(a21,2))/a12 + (Fx*powf(a12,2)*powf(a23,2))/a12 + (Fx*powf(a12,2)*powf(a24,2))/a12 + (Fx*powf(a12,2)*powf(a25,2))/a12 + (T_yaw*powf(a11,2)*powf(a22,2))/a22 + (T_yaw*powf(a13,2)*powf(a22,2))/a22 + (T_yaw*powf(a14,2)*powf(a22,2))/a22 + (T_yaw*powf(a15,2)*powf(a22,2))/a22 - (Fx*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (Fx*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (Fx*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (Fx*a25*powf(a15,2)*powf(a22,2))/(a15*a22) - (T_yaw*a11*powf(a12,2)*powf(a21,2))/(a12*a21) - (T_yaw*a13*powf(a12,2)*powf(a23,2))/(a12*a23) - (T_yaw*a14*powf(a12,2)*powf(a24,2))/(a12*a24) - (T_yaw*a15*powf(a12,2)*powf(a25,2))/(a12*a25))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a15,2)*powf(a21,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a15,2)*powf(a22,2) + powf(a13,2)*powf(a25,2) + powf(a15,2)*powf(a23,2) + powf(a14,2)*powf(a25,2) + powf(a15,2)*powf(a24,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a11*a25*powf(a15,2)*powf(a21,2))/(a15*a21) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a15*a21*powf(a11,2)*powf(a25,2))/(a11*a25) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a12*a25*powf(a15,2)*powf(a22,2))/(a15*a22) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a15*a22*powf(a12,2)*powf(a25,2))/(a12*a25) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24) - (a13*a25*powf(a15,2)*powf(a23,2))/(a15*a23) - (a15*a23*powf(a13,2)*powf(a25,2))/(a13*a25) - (a14*a25*powf(a15,2)*powf(a24,2))/(a15*a24) - (a15*a24*powf(a14,2)*powf(a25,2))/(a14*a25));
//            arcsin3 = ((Fx*powf(a13,2)*powf(a21,2))/a13 + (Fx*powf(a13,2)*powf(a22,2))/a13 + (Fx*powf(a13,2)*powf(a24,2))/a13 + (Fx*powf(a13,2)*powf(a25,2))/a13 + (T_yaw*powf(a11,2)*powf(a23,2))/a23 + (T_yaw*powf(a12,2)*powf(a23,2))/a23 + (T_yaw*powf(a14,2)*powf(a23,2))/a23 + (T_yaw*powf(a15,2)*powf(a23,2))/a23 - (Fx*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (Fx*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (Fx*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (Fx*a25*powf(a15,2)*powf(a23,2))/(a15*a23) - (T_yaw*a11*powf(a13,2)*powf(a21,2))/(a13*a21) - (T_yaw*a12*powf(a13,2)*powf(a22,2))/(a13*a22) - (T_yaw*a14*powf(a13,2)*powf(a24,2))/(a13*a24) - (T_yaw*a15*powf(a13,2)*powf(a25,2))/(a13*a25))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a15,2)*powf(a21,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a15,2)*powf(a22,2) + powf(a13,2)*powf(a25,2) + powf(a15,2)*powf(a23,2) + powf(a14,2)*powf(a25,2) + powf(a15,2)*powf(a24,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a11*a25*powf(a15,2)*powf(a21,2))/(a15*a21) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a15*a21*powf(a11,2)*powf(a25,2))/(a11*a25) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a12*a25*powf(a15,2)*powf(a22,2))/(a15*a22) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a15*a22*powf(a12,2)*powf(a25,2))/(a12*a25) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24) - (a13*a25*powf(a15,2)*powf(a23,2))/(a15*a23) - (a15*a23*powf(a13,2)*powf(a25,2))/(a13*a25) - (a14*a25*powf(a15,2)*powf(a24,2))/(a15*a24) - (a15*a24*powf(a14,2)*powf(a25,2))/(a14*a25));
//            arcsin4 = ((Fx*powf(a14,2)*powf(a21,2))/a14 + (Fx*powf(a14,2)*powf(a22,2))/a14 + (Fx*powf(a14,2)*powf(a23,2))/a14 + (Fx*powf(a14,2)*powf(a25,2))/a14 + (T_yaw*powf(a11,2)*powf(a24,2))/a24 + (T_yaw*powf(a12,2)*powf(a24,2))/a24 + (T_yaw*powf(a13,2)*powf(a24,2))/a24 + (T_yaw*powf(a15,2)*powf(a24,2))/a24 - (Fx*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (Fx*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (Fx*a23*powf(a13,2)*powf(a24,2))/(a13*a24) - (Fx*a25*powf(a15,2)*powf(a24,2))/(a15*a24) - (T_yaw*a11*powf(a14,2)*powf(a21,2))/(a14*a21) - (T_yaw*a12*powf(a14,2)*powf(a22,2))/(a14*a22) - (T_yaw*a13*powf(a14,2)*powf(a23,2))/(a14*a23) - (T_yaw*a15*powf(a14,2)*powf(a25,2))/(a14*a25))/(powf(a11,2)*powf(a22,2) + powf(a12,2)*powf(a21,2) + powf(a11,2)*powf(a23,2) + powf(a13,2)*powf(a21,2) + powf(a11,2)*powf(a24,2) + powf(a12,2)*powf(a23,2) + powf(a13,2)*powf(a22,2) + powf(a14,2)*powf(a21,2) + powf(a11,2)*powf(a25,2) + powf(a12,2)*powf(a24,2) + powf(a14,2)*powf(a22,2) + powf(a15,2)*powf(a21,2) + powf(a12,2)*powf(a25,2) + powf(a13,2)*powf(a24,2) + powf(a14,2)*powf(a23,2) + powf(a15,2)*powf(a22,2) + powf(a13,2)*powf(a25,2) + powf(a15,2)*powf(a23,2) + powf(a14,2)*powf(a25,2) + powf(a15,2)*powf(a24,2) - (a11*a22*powf(a12,2)*powf(a21,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2))/(a11*a23) - (a11*a24*powf(a14,2)*powf(a21,2))/(a14*a21) - (a12*a23*powf(a13,2)*powf(a22,2))/(a13*a22) - (a14*a21*powf(a11,2)*powf(a24,2))/(a11*a24) - (a13*a22*powf(a12,2)*powf(a23,2))/(a12*a23) - (a11*a25*powf(a15,2)*powf(a21,2))/(a15*a21) - (a12*a24*powf(a14,2)*powf(a22,2))/(a14*a22) - (a15*a21*powf(a11,2)*powf(a25,2))/(a11*a25) - (a14*a22*powf(a12,2)*powf(a24,2))/(a12*a24) - (a12*a25*powf(a15,2)*powf(a22,2))/(a15*a22) - (a13*a24*powf(a14,2)*powf(a23,2))/(a14*a23) - (a15*a22*powf(a12,2)*powf(a25,2))/(a12*a25) - (a14*a23*powf(a13,2)*powf(a24,2))/(a13*a24) - (a13*a25*powf(a15,2)*powf(a23,2))/(a15*a23) - (a15*a23*powf(a13,2)*powf(a25,2))/(a13*a25) - (a14*a25*powf(a15,2)*powf(a24,2))/(a15*a24) - (a15*a24*powf(a14,2)*powf(a25,2))/(a14*a25));

//            // Aproximação para pequenos ângulos.
//            srv5    = arcsin1;
//            srv6    = arcsin2;
//            srv7    = arcsin3;
//            srv8    = arcsin4;

//            srv5 = _lastSrv5;
//            srv6 = _lastSrv6;
//            srv7 = _lastSrv7;
//            srv8 = _lastSrv8;

            // Confere se ocorreu problemas na divisão, gerando Infinito e NAN
            if((isinf(arcsin1)==1)||(isinf(arcsin2)==1)||(isinf(arcsin3)==1)||(isinf(arcsin4)==1)||(isnan(arcsin1)==1)||(isnan(arcsin2)==1)||(isnan(arcsin3)==1)||(isnan(arcsin4)==1)){
                arcsin1 = 0.0;
                arcsin2 = 0.0;
                arcsin3 = 0.0;
                arcsin4 = 0.0;
//                Fx      = 0.0;
//                T_yaw   = 0.0;
            }

//            // Confere se ocorreu problemas na divisão, gerando NAN
//            if((isnan(arcsin1)==1)||(isnan(arcsin2)==1)||(isnan(arcsin3)==1)||(isnan(arcsin4)==1)){
//                arcsin1 = 0.0;
//                arcsin2 = 0.0;
//                arcsin3 = 0.0;
//                arcsin4 = 0.0;
//                Fx      = 0.0;
//                T_yaw   = 0.0;
//            }

            // Obtenção dos arcosenos.
            srv5    = asinf(arcsin1);
            srv6    = asinf(arcsin2);
            srv7    = asinf(arcsin3);
            srv8    = asinf(arcsin4);

            // Saturação de inclinação dos servomotores
            if(srv5>=30.0*deg2rad){
                srv5 = 30.0*deg2rad;
            }else if(srv5<=-30.0*deg2rad){
                srv5 = -30.0*deg2rad;
            }
            if(srv6>=30.0*deg2rad){
                srv6 = 30.0*deg2rad;
            }else if(srv6<=-30.0*deg2rad){
                srv6 = -30.0*deg2rad;
            }
            if(srv7>=30.0*deg2rad){
                srv7 = 30.0*deg2rad;
            }else if(srv7<=-30.0*deg2rad){
                srv7 = -30.0*deg2rad;
            }
            if(srv8>=30.0*deg2rad){
                srv8 = 30.0*deg2rad;
            }else if(srv8<=-30.0*deg2rad){
                srv8 = -30.0*deg2rad;
            }

            // Atualizando os ângulos dos servos para a próxima iteração, já transformando em radianos.
            _lastSrv5 = srv5;
            _lastSrv6 = srv6;
            _lastSrv7 = srv7;
            _lastSrv8 = srv8;

            ///////////////////////////////////////////////////////////////////////
            /// Primeira parte do processo
            ///////////////////////////////////////////////////////////////////////
            // Obtendo os cossenos das posições angulares anteriores do servomotores.
            ag5_c = cosf(srv5);
            ag6_c = cosf(srv6);
            ag7_c = cosf(srv7);
            ag8_c = cosf(srv8);

            ag5_s = sinf(srv5);
            ag6_s = sinf(srv6);
            ag7_s = sinf(srv7);
            ag8_s = sinf(srv8);

            // Aproximação para pequneos ângulos.
//            ag5_c = 1.0 - (_lastSrv5*_lastSrv5)/2.0;
//            ag6_c = 1.0 - (_lastSrv6*_lastSrv6)/2.0;
//            ag7_c = 1.0 - (_lastSrv7*_lastSrv7)/2.0;
//            ag8_c = 1.0 - (_lastSrv8*_lastSrv8)/2.0;

            // Termos da matriz
            a11 =  _k1*ag5_c;
            a12 =  _k1*ag6_c;
            a13 =  _k1*ag7_c;
            a14 =  _k1*ag8_c;

            a21 = -_k1*_l_arm*sqrt_2_2*ag5_c - _k2*ag5_s;
            a22 =  _k1*_l_arm*sqrt_2_2*ag6_c - _k2*ag6_s;
            a23 =  _k1*_l_arm*sqrt_2_2*ag7_c + _k2*ag7_s;
            a24 = -_k1*_l_arm*sqrt_2_2*ag8_c + _k2*ag8_s;

            a31 =  _k1*_l_arm*sqrt_2_2*ag5_c;
            a32 = -_k1*_l_arm*sqrt_2_2*ag6_c;
            a33 =  _k1*_l_arm*sqrt_2_2*ag7_c;
            a34 = -_k1*_l_arm*sqrt_2_2*ag8_c;

            //////////////////////////////////////////////////////////////////////
            // Obtendo os PWMs dos motores considerando somente Fz, Torque de Rolagem e Arfagem.
            /////////////
            // Motor 1
            out1 =  ((Fz*powf(a11,2)*powf(a22,2)*powf(a33,2))/a11 + (Fz*powf(a11,2)*powf(a23,2)*powf(a32,2))/a11 + (Fz*powf(a11,2)*powf(a22,2)*powf(a34,2))/a11 + (Fz*powf(a11,2)*powf(a24,2)*powf(a32,2))/a11 + (Fz*powf(a11,2)*powf(a23,2)*powf(a34,2))/a11 + (Fz*powf(a11,2)*powf(a24,2)*powf(a33,2))/a11 + (T_pitch*powf(a12,2)*powf(a23,2)*powf(a31,2))/a31 + (T_pitch*powf(a13,2)*powf(a22,2)*powf(a31,2))/a31 + (T_pitch*powf(a12,2)*powf(a24,2)*powf(a31,2))/a31 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a31,2))/a31 + (T_pitch*powf(a13,2)*powf(a24,2)*powf(a31,2))/a31 + (T_pitch*powf(a14,2)*powf(a23,2)*powf(a31,2))/a31 + (T_roll*powf(a12,2)*powf(a21,2)*powf(a33,2))/a21 + (T_roll*powf(a13,2)*powf(a21,2)*powf(a32,2))/a21 + (T_roll*powf(a12,2)*powf(a21,2)*powf(a34,2))/a21 + (T_roll*powf(a14,2)*powf(a21,2)*powf(a32,2))/a21 + (T_roll*powf(a13,2)*powf(a21,2)*powf(a34,2))/a21 + (T_roll*powf(a14,2)*powf(a21,2)*powf(a33,2))/a21 - (Fz*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (Fz*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (Fz*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (Fz*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (Fz*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (Fz*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (Fz*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (Fz*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (Fz*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (Fz*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (Fz*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (Fz*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (T_pitch*a12*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (T_pitch*a12*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (T_pitch*a13*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (T_pitch*a13*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (T_pitch*a14*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (T_pitch*a14*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (T_pitch*a22*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_pitch*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_pitch*a23*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_pitch*a23*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_pitch*a24*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_pitch*a24*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a12*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (T_roll*a12*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (T_roll*a13*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (T_roll*a13*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (T_roll*a14*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (T_roll*a14*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (T_roll*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_roll*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_roll*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_roll*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_roll*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (Fz*a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) - (Fz*a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (Fz*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (Fz*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (Fz*a22*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a23*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (Fz*a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) - (Fz*a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (Fz*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (Fz*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (Fz*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (Fz*a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) - (Fz*a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (Fz*a23*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (Fz*a24*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (Fz*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a12*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) - (T_pitch*a12*a23*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_pitch*a12*a23*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_pitch*a13*a22*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) - (T_pitch*a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_pitch*a13*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_pitch*a12*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_pitch*a12*a24*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_pitch*a14*a22*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) - (T_pitch*a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_pitch*a14*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_pitch*a13*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) - (T_pitch*a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a13*a24*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_pitch*a14*a23*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) - (T_pitch*a14*a23*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_pitch*a14*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_roll*a12*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_roll*a12*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_roll*a12*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a13*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_roll*a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_roll*a13*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (T_roll*a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_roll*a12*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_roll*a12*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a14*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_roll*a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_roll*a14*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (T_roll*a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_roll*a13*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_roll*a13*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a14*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_roll*a14*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_roll*a14*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a13,2)*powf(a21,2)*powf(a32,2) + powf(a13,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a13,2)*powf(a21,2)*powf(a34,2) + powf(a13,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a13,2)*powf(a22,2)*powf(a34,2) + powf(a13,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            /////////////
            // Motor
            out2 =  ((Fz*powf(a12,2)*powf(a21,2)*powf(a33,2))/a12 + (Fz*powf(a12,2)*powf(a23,2)*powf(a31,2))/a12 + (Fz*powf(a12,2)*powf(a21,2)*powf(a34,2))/a12 + (Fz*powf(a12,2)*powf(a24,2)*powf(a31,2))/a12 + (Fz*powf(a12,2)*powf(a23,2)*powf(a34,2))/a12 + (Fz*powf(a12,2)*powf(a24,2)*powf(a33,2))/a12 + (T_pitch*powf(a11,2)*powf(a23,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a32,2))/a32 + (T_pitch*powf(a11,2)*powf(a24,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a24,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a23,2)*powf(a32,2))/a32 + (T_roll*powf(a11,2)*powf(a22,2)*powf(a33,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a31,2))/a22 + (T_roll*powf(a11,2)*powf(a22,2)*powf(a34,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a31,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a34,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a33,2))/a22 - (Fz*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (Fz*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (Fz*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (Fz*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (Fz*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (Fz*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (Fz*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (Fz*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (Fz*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (Fz*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (Fz*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (Fz*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (T_pitch*a11*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (T_pitch*a11*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (T_pitch*a13*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (T_pitch*a13*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (T_pitch*a14*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (T_pitch*a14*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (T_pitch*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_pitch*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_pitch*a23*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_pitch*a23*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_pitch*a24*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_pitch*a24*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a11*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (T_roll*a11*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (T_roll*a13*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (T_roll*a13*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (T_roll*a14*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (T_roll*a14*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_roll*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_roll*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_roll*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_roll*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) + (Fz*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (Fz*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (Fz*a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (Fz*a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (Fz*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (Fz*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (Fz*a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (Fz*a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (Fz*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (Fz*a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) - (Fz*a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (Fz*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (Fz*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (Fz*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (Fz*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a11*a23*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_pitch*a11*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_pitch*a13*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_pitch*a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_pitch*a13*a21*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_pitch*a11*a24*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_pitch*a11*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_pitch*a14*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_pitch*a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_pitch*a14*a21*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_pitch*a13*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) - (T_pitch*a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a13*a24*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_pitch*a14*a23*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) - (T_pitch*a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_pitch*a14*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_roll*a11*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_roll*a11*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) - (T_roll*a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (T_roll*a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_roll*a13*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a13*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_roll*a11*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_roll*a11*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (T_roll*a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (T_roll*a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_roll*a14*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a14*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_roll*a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_roll*a13*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_roll*a13*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a14*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_roll*a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_roll*a14*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            /////////////
            // Motor 3
            out3 =  ((Fz*powf(a14,2)*powf(a21,2)*powf(a32,2))/a13 + (Fz*powf(a14,2)*powf(a22,2)*powf(a31,2))/a13 + (Fz*powf(a14,2)*powf(a21,2)*powf(a34,2))/a13 + (Fz*powf(a14,2)*powf(a24,2)*powf(a31,2))/a13 + (Fz*powf(a14,2)*powf(a22,2)*powf(a34,2))/a13 + (Fz*powf(a14,2)*powf(a24,2)*powf(a32,2))/a13 + (T_pitch*powf(a11,2)*powf(a22,2)*powf(a33,2))/a33 + (T_pitch*powf(a12,2)*powf(a21,2)*powf(a33,2))/a33 + (T_pitch*powf(a11,2)*powf(a24,2)*powf(a33,2))/a33 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a33,2))/a33 + (T_pitch*powf(a12,2)*powf(a24,2)*powf(a33,2))/a33 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a33,2))/a33 + (T_roll*powf(a11,2)*powf(a23,2)*powf(a32,2))/a23 + (T_roll*powf(a12,2)*powf(a23,2)*powf(a31,2))/a23 + (T_roll*powf(a11,2)*powf(a23,2)*powf(a34,2))/a23 + (T_roll*powf(a14,2)*powf(a23,2)*powf(a31,2))/a23 + (T_roll*powf(a12,2)*powf(a23,2)*powf(a34,2))/a23 + (T_roll*powf(a14,2)*powf(a23,2)*powf(a32,2))/a23 - (Fz*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (Fz*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (Fz*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (Fz*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (Fz*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (Fz*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (Fz*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (Fz*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (Fz*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (Fz*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (Fz*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (Fz*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (T_pitch*a11*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (T_pitch*a11*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (T_pitch*a12*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (T_pitch*a12*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (T_pitch*a14*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (T_pitch*a14*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (T_pitch*a21*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_pitch*a21*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_pitch*a22*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_pitch*a22*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_pitch*a24*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_pitch*a24*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (T_roll*a14*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (T_roll*a14*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (T_roll*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_roll*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_roll*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_roll*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) + (Fz*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (Fz*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (Fz*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (Fz*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (Fz*a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (Fz*a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (Fz*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (Fz*a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) - (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (Fz*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (Fz*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (Fz*a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) - (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (Fz*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a11*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (T_pitch*a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_pitch*a11*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_pitch*a12*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (T_pitch*a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_pitch*a12*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_pitch*a11*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a14*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_pitch*a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_pitch*a14*a21*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a12*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a14*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_pitch*a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_pitch*a14*a22*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a11*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (T_roll*a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_roll*a11*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_roll*a12*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_roll*a12*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_roll*a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a11*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_roll*a11*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) - (T_roll*a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (T_roll*a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_roll*a14*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a14*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_roll*a12*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a12*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) - (T_roll*a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (T_roll*a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_roll*a14*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a14*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            /////////////
            // Motor 4
            out4 = ((Fz*powf(a14,2)*powf(a21,2)*powf(a32,2))/a14 + (Fz*powf(a14,2)*powf(a22,2)*powf(a31,2))/a14 + (Fz*powf(a14,2)*powf(a21,2)*powf(a33,2))/a14 + (Fz*powf(a14,2)*powf(a23,2)*powf(a31,2))/a14 + (Fz*powf(a14,2)*powf(a22,2)*powf(a33,2))/a14 + (Fz*powf(a14,2)*powf(a23,2)*powf(a32,2))/a14 + (T_pitch*powf(a11,2)*powf(a22,2)*powf(a34,2))/a34 + (T_pitch*powf(a12,2)*powf(a21,2)*powf(a34,2))/a34 + (T_pitch*powf(a11,2)*powf(a23,2)*powf(a34,2))/a34 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a34,2))/a34 + (T_pitch*powf(a12,2)*powf(a23,2)*powf(a34,2))/a34 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a34,2))/a34 + (T_roll*powf(a11,2)*powf(a24,2)*powf(a32,2))/a24 + (T_roll*powf(a12,2)*powf(a24,2)*powf(a31,2))/a24 + (T_roll*powf(a11,2)*powf(a24,2)*powf(a33,2))/a24 + (T_roll*powf(a14,2)*powf(a24,2)*powf(a31,2))/a24 + (T_roll*powf(a12,2)*powf(a24,2)*powf(a33,2))/a24 + (T_roll*powf(a14,2)*powf(a24,2)*powf(a32,2))/a24 - (Fz*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (Fz*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (Fz*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (Fz*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (Fz*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (Fz*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (Fz*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (Fz*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (Fz*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (Fz*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (Fz*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (Fz*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (T_pitch*a11*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (T_pitch*a11*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (T_pitch*a12*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (T_pitch*a12*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (T_pitch*a13*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (T_pitch*a13*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (T_pitch*a21*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_pitch*a21*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_pitch*a22*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_pitch*a22*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_pitch*a23*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_pitch*a23*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (T_roll*a13*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (T_roll*a13*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (T_roll*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_roll*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (Fz*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (Fz*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (Fz*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (Fz*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (Fz*a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (Fz*a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (Fz*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (Fz*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (Fz*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (Fz*a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (Fz*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (Fz*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (Fz*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (Fz*a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (Fz*a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a11*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (T_pitch*a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_pitch*a11*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_pitch*a12*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (T_pitch*a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_pitch*a12*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_pitch*a11*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) - (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_pitch*a13*a21*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (T_pitch*a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_pitch*a13*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_pitch*a12*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) - (T_pitch*a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_pitch*a12*a23*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a13*a22*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (T_pitch*a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_pitch*a13*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a11*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_roll*a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_roll*a11*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_roll*a12*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_roll*a12*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_roll*a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a11*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_roll*a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_roll*a11*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_roll*a13*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_roll*a13*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_roll*a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a12*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (T_roll*a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a12*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_roll*a13*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_roll*a13*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_roll*a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            // Confere se ocorreu problemas na divisão, gerando Infinito e NAN
            if((isinf(out1)==1)||(isinf(out2)==1)||(isinf(out3)==1)||(isinf(out4)==1)||(isnan(out1)==1)||(isnan(out2)==1)||(isnan(out3)==1)||(isnan(out4)==1)){

                srv5 = 0.0;
                srv6 = 0.0;
                srv7 = 0.0;
                srv8 = 0.0;

                // Atualizando os ângulos dos servos para a próxima iteração, em radianos.
                _lastSrv5 = srv5;
                _lastSrv6 = srv6;
                _lastSrv7 = srv7;
                _lastSrv8 = srv8;

                ///////////////////////////////////////////////////////////////////////
                /// Primeira parte do processo novamente para garantir segurança
                ///////////////////////////////////////////////////////////////////////
                // Obtendo os cossenos das posições angulares anteriores do servomotores.
                ag5_c = cosf(srv5);
                ag6_c = cosf(srv6);
                ag7_c = cosf(srv7);
                ag8_c = cosf(srv8);

                ag5_s = sinf(srv5);
                ag6_s = sinf(srv6);
                ag7_s = sinf(srv7);
                ag8_s = sinf(srv8);

                // Termos da matriz
                a11 =  _k1*ag5_c;
                a12 =  _k1*ag6_c;
                a13 =  _k1*ag7_c;
                a14 =  _k1*ag8_c;

                a21 = -_k1*_l_arm*sqrt_2_2*ag5_c - _k2*ag5_s;
                a22 =  _k1*_l_arm*sqrt_2_2*ag6_c - _k2*ag6_s;
                a23 =  _k1*_l_arm*sqrt_2_2*ag7_c + _k2*ag7_s;
                a24 = -_k1*_l_arm*sqrt_2_2*ag8_c + _k2*ag8_s;

                a31 =  _k1*_l_arm*sqrt_2_2*ag5_c;
                a32 = -_k1*_l_arm*sqrt_2_2*ag6_c;
                a33 =  _k1*_l_arm*sqrt_2_2*ag7_c;
                a34 = -_k1*_l_arm*sqrt_2_2*ag8_c;

                //////////////////////////////////////////////////////////////////////
                // Obtendo os PWMs dos motores considerando somente Fz, Torque de Rolagem e Arfagem.
                /////////////
                // Motor 1
                out1 =  ((Fz*powf(a11,2)*powf(a22,2)*powf(a33,2))/a11 + (Fz*powf(a11,2)*powf(a23,2)*powf(a32,2))/a11 + (Fz*powf(a11,2)*powf(a22,2)*powf(a34,2))/a11 + (Fz*powf(a11,2)*powf(a24,2)*powf(a32,2))/a11 + (Fz*powf(a11,2)*powf(a23,2)*powf(a34,2))/a11 + (Fz*powf(a11,2)*powf(a24,2)*powf(a33,2))/a11 + (T_pitch*powf(a12,2)*powf(a23,2)*powf(a31,2))/a31 + (T_pitch*powf(a13,2)*powf(a22,2)*powf(a31,2))/a31 + (T_pitch*powf(a12,2)*powf(a24,2)*powf(a31,2))/a31 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a31,2))/a31 + (T_pitch*powf(a13,2)*powf(a24,2)*powf(a31,2))/a31 + (T_pitch*powf(a14,2)*powf(a23,2)*powf(a31,2))/a31 + (T_roll*powf(a12,2)*powf(a21,2)*powf(a33,2))/a21 + (T_roll*powf(a13,2)*powf(a21,2)*powf(a32,2))/a21 + (T_roll*powf(a12,2)*powf(a21,2)*powf(a34,2))/a21 + (T_roll*powf(a14,2)*powf(a21,2)*powf(a32,2))/a21 + (T_roll*powf(a13,2)*powf(a21,2)*powf(a34,2))/a21 + (T_roll*powf(a14,2)*powf(a21,2)*powf(a33,2))/a21 - (Fz*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (Fz*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (Fz*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (Fz*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (Fz*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (Fz*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (Fz*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (Fz*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (Fz*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (Fz*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (Fz*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (Fz*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (T_pitch*a12*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (T_pitch*a12*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (T_pitch*a13*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (T_pitch*a13*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (T_pitch*a14*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (T_pitch*a14*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (T_pitch*a22*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_pitch*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_pitch*a23*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_pitch*a23*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_pitch*a24*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_pitch*a24*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a12*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (T_roll*a12*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (T_roll*a13*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (T_roll*a13*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (T_roll*a14*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (T_roll*a14*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (T_roll*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_roll*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_roll*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_roll*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_roll*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (Fz*a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) - (Fz*a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (Fz*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (Fz*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (Fz*a22*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a23*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (Fz*a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) - (Fz*a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (Fz*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (Fz*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (Fz*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (Fz*a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) - (Fz*a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (Fz*a23*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (Fz*a24*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (Fz*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a12*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) - (T_pitch*a12*a23*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_pitch*a12*a23*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_pitch*a13*a22*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) - (T_pitch*a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_pitch*a13*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_pitch*a12*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_pitch*a12*a24*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_pitch*a14*a22*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) - (T_pitch*a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_pitch*a14*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_pitch*a13*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) - (T_pitch*a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a13*a24*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_pitch*a14*a23*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) - (T_pitch*a14*a23*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_pitch*a14*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_roll*a12*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_roll*a12*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_roll*a12*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a13*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_roll*a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_roll*a13*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (T_roll*a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_roll*a12*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_roll*a12*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a14*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_roll*a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_roll*a14*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (T_roll*a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_roll*a13*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_roll*a13*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a14*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_roll*a14*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_roll*a14*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a13,2)*powf(a21,2)*powf(a32,2) + powf(a13,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a13,2)*powf(a21,2)*powf(a34,2) + powf(a13,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a13,2)*powf(a22,2)*powf(a34,2) + powf(a13,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

                /////////////
                // Motor
                out2 =  ((Fz*powf(a12,2)*powf(a21,2)*powf(a33,2))/a12 + (Fz*powf(a12,2)*powf(a23,2)*powf(a31,2))/a12 + (Fz*powf(a12,2)*powf(a21,2)*powf(a34,2))/a12 + (Fz*powf(a12,2)*powf(a24,2)*powf(a31,2))/a12 + (Fz*powf(a12,2)*powf(a23,2)*powf(a34,2))/a12 + (Fz*powf(a12,2)*powf(a24,2)*powf(a33,2))/a12 + (T_pitch*powf(a11,2)*powf(a23,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a32,2))/a32 + (T_pitch*powf(a11,2)*powf(a24,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a24,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a23,2)*powf(a32,2))/a32 + (T_roll*powf(a11,2)*powf(a22,2)*powf(a33,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a31,2))/a22 + (T_roll*powf(a11,2)*powf(a22,2)*powf(a34,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a31,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a34,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a33,2))/a22 - (Fz*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (Fz*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (Fz*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (Fz*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (Fz*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (Fz*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (Fz*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (Fz*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (Fz*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (Fz*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (Fz*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (Fz*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (T_pitch*a11*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (T_pitch*a11*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (T_pitch*a13*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (T_pitch*a13*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (T_pitch*a14*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (T_pitch*a14*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (T_pitch*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_pitch*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_pitch*a23*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_pitch*a23*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_pitch*a24*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_pitch*a24*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a11*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (T_roll*a11*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (T_roll*a13*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (T_roll*a13*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (T_roll*a14*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (T_roll*a14*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_roll*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_roll*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_roll*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_roll*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) + (Fz*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (Fz*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (Fz*a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (Fz*a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (Fz*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (Fz*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (Fz*a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (Fz*a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (Fz*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (Fz*a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) - (Fz*a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (Fz*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (Fz*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (Fz*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (Fz*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a11*a23*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_pitch*a11*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_pitch*a13*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_pitch*a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_pitch*a13*a21*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_pitch*a11*a24*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_pitch*a11*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_pitch*a14*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_pitch*a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_pitch*a14*a21*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_pitch*a13*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) - (T_pitch*a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a13*a24*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_pitch*a14*a23*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) - (T_pitch*a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_pitch*a14*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_roll*a11*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_roll*a11*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) - (T_roll*a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (T_roll*a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_roll*a13*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a13*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_roll*a11*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_roll*a11*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (T_roll*a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (T_roll*a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_roll*a14*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a14*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_roll*a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_roll*a13*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_roll*a13*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a14*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_roll*a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_roll*a14*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

                /////////////
                // Motor 3
                out3 =  ((Fz*powf(a14,2)*powf(a21,2)*powf(a32,2))/a13 + (Fz*powf(a14,2)*powf(a22,2)*powf(a31,2))/a13 + (Fz*powf(a14,2)*powf(a21,2)*powf(a34,2))/a13 + (Fz*powf(a14,2)*powf(a24,2)*powf(a31,2))/a13 + (Fz*powf(a14,2)*powf(a22,2)*powf(a34,2))/a13 + (Fz*powf(a14,2)*powf(a24,2)*powf(a32,2))/a13 + (T_pitch*powf(a11,2)*powf(a22,2)*powf(a33,2))/a33 + (T_pitch*powf(a12,2)*powf(a21,2)*powf(a33,2))/a33 + (T_pitch*powf(a11,2)*powf(a24,2)*powf(a33,2))/a33 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a33,2))/a33 + (T_pitch*powf(a12,2)*powf(a24,2)*powf(a33,2))/a33 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a33,2))/a33 + (T_roll*powf(a11,2)*powf(a23,2)*powf(a32,2))/a23 + (T_roll*powf(a12,2)*powf(a23,2)*powf(a31,2))/a23 + (T_roll*powf(a11,2)*powf(a23,2)*powf(a34,2))/a23 + (T_roll*powf(a14,2)*powf(a23,2)*powf(a31,2))/a23 + (T_roll*powf(a12,2)*powf(a23,2)*powf(a34,2))/a23 + (T_roll*powf(a14,2)*powf(a23,2)*powf(a32,2))/a23 - (Fz*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (Fz*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (Fz*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (Fz*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (Fz*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (Fz*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (Fz*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (Fz*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (Fz*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (Fz*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (Fz*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (Fz*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (T_pitch*a11*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (T_pitch*a11*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (T_pitch*a12*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (T_pitch*a12*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (T_pitch*a14*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (T_pitch*a14*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (T_pitch*a21*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_pitch*a21*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_pitch*a22*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_pitch*a22*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_pitch*a24*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_pitch*a24*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (T_roll*a14*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (T_roll*a14*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (T_roll*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_roll*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_roll*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_roll*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) + (Fz*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (Fz*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (Fz*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (Fz*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (Fz*a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (Fz*a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (Fz*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (Fz*a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) - (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (Fz*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (Fz*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (Fz*a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) - (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (Fz*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a11*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (T_pitch*a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_pitch*a11*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_pitch*a12*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (T_pitch*a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_pitch*a12*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_pitch*a11*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a14*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_pitch*a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_pitch*a14*a21*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a12*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a14*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_pitch*a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_pitch*a14*a22*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a11*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (T_roll*a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_roll*a11*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_roll*a12*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_roll*a12*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_roll*a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a11*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_roll*a11*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) - (T_roll*a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (T_roll*a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_roll*a14*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a14*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_roll*a12*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a12*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) - (T_roll*a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (T_roll*a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_roll*a14*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a14*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

                /////////////
                // Motor 4
                out4 = ((Fz*powf(a14,2)*powf(a21,2)*powf(a32,2))/a14 + (Fz*powf(a14,2)*powf(a22,2)*powf(a31,2))/a14 + (Fz*powf(a14,2)*powf(a21,2)*powf(a33,2))/a14 + (Fz*powf(a14,2)*powf(a23,2)*powf(a31,2))/a14 + (Fz*powf(a14,2)*powf(a22,2)*powf(a33,2))/a14 + (Fz*powf(a14,2)*powf(a23,2)*powf(a32,2))/a14 + (T_pitch*powf(a11,2)*powf(a22,2)*powf(a34,2))/a34 + (T_pitch*powf(a12,2)*powf(a21,2)*powf(a34,2))/a34 + (T_pitch*powf(a11,2)*powf(a23,2)*powf(a34,2))/a34 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a34,2))/a34 + (T_pitch*powf(a12,2)*powf(a23,2)*powf(a34,2))/a34 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a34,2))/a34 + (T_roll*powf(a11,2)*powf(a24,2)*powf(a32,2))/a24 + (T_roll*powf(a12,2)*powf(a24,2)*powf(a31,2))/a24 + (T_roll*powf(a11,2)*powf(a24,2)*powf(a33,2))/a24 + (T_roll*powf(a14,2)*powf(a24,2)*powf(a31,2))/a24 + (T_roll*powf(a12,2)*powf(a24,2)*powf(a33,2))/a24 + (T_roll*powf(a14,2)*powf(a24,2)*powf(a32,2))/a24 - (Fz*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (Fz*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (Fz*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (Fz*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (Fz*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (Fz*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (Fz*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (Fz*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (Fz*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (Fz*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (Fz*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (Fz*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (T_pitch*a11*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (T_pitch*a11*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (T_pitch*a12*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (T_pitch*a12*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (T_pitch*a13*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (T_pitch*a13*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (T_pitch*a21*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_pitch*a21*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_pitch*a22*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_pitch*a22*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_pitch*a23*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_pitch*a23*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (T_roll*a13*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (T_roll*a13*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (T_roll*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_roll*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (Fz*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (Fz*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (Fz*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (Fz*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (Fz*a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (Fz*a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (Fz*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (Fz*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (Fz*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (Fz*a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (Fz*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (Fz*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (Fz*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (Fz*a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (Fz*a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a11*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (T_pitch*a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_pitch*a11*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_pitch*a12*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (T_pitch*a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_pitch*a12*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_pitch*a11*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) - (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_pitch*a13*a21*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (T_pitch*a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_pitch*a13*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_pitch*a12*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) - (T_pitch*a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_pitch*a12*a23*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a13*a22*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (T_pitch*a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_pitch*a13*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a11*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_roll*a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_roll*a11*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_roll*a12*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_roll*a12*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_roll*a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a11*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_roll*a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_roll*a11*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_roll*a13*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_roll*a13*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_roll*a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a12*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (T_roll*a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a12*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_roll*a13*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_roll*a13*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_roll*a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

                // Guardando os valores atuais dos motores.
                _last_Out1 = mtr1;
                _last_Out2 = mtr2;
                _last_Out3 = mtr3;
                _last_Out4 = mtr4;

            }else{
                // Guardando os valores atuais dos motores.
                _last_Out1 = mtr1;
                _last_Out2 = mtr2;
                _last_Out3 = mtr3;
                _last_Out4 = mtr4;
            }
        }
        }else{
            // Aqui não será calculado os ângulos dos servos.
            ag5_c = cosf(_lastSrv5);
            ag6_c = cosf(_lastSrv6);
            ag7_c = cosf(_lastSrv7);
            ag8_c = cosf(_lastSrv8);

            ag5_s = sinf(_lastSrv5);
            ag6_s = sinf(_lastSrv6);
            ag7_s = sinf(_lastSrv7);
            ag8_s = sinf(_lastSrv8);

            // Aproximação para pequneos ângulos.
    //            ag5_c = 1.0 - (_lastSrv5*_lastSrv5)/2.0;
    //            ag6_c = 1.0 - (_lastSrv6*_lastSrv6)/2.0;
    //            ag7_c = 1.0 - (_lastSrv7*_lastSrv7)/2.0;
    //            ag8_c = 1.0 - (_lastSrv8*_lastSrv8)/2.0;

            // Termos da matriz
            a11 =  _k1*ag5_c;
            a12 =  _k1*ag6_c;
            a13 =  _k1*ag7_c;
            a14 =  _k1*ag8_c;

            a21 = -_k1*_l_arm*sqrt_2_2*ag5_c - _k2*ag5_s;
            a22 =  _k1*_l_arm*sqrt_2_2*ag6_c - _k2*ag6_s;
            a23 =  _k1*_l_arm*sqrt_2_2*ag7_c + _k2*ag7_s;
            a24 = -_k1*_l_arm*sqrt_2_2*ag8_c + _k2*ag8_s;

            a31 =  _k1*_l_arm*sqrt_2_2*ag5_c;
            a32 = -_k1*_l_arm*sqrt_2_2*ag6_c;
            a33 =  _k1*_l_arm*sqrt_2_2*ag7_c;
            a34 = -_k1*_l_arm*sqrt_2_2*ag8_c;

            //////////////////////////////////////////////////////////////////////
            // Obtendo os PWMs dos motores considerando somente Fz, Torque de Rolagem e Arfagem.
            /////////////
            // Motor 1
            out1 =  ((Fz*powf(a11,2)*powf(a22,2)*powf(a33,2))/a11 + (Fz*powf(a11,2)*powf(a23,2)*powf(a32,2))/a11 + (Fz*powf(a11,2)*powf(a22,2)*powf(a34,2))/a11 + (Fz*powf(a11,2)*powf(a24,2)*powf(a32,2))/a11 + (Fz*powf(a11,2)*powf(a23,2)*powf(a34,2))/a11 + (Fz*powf(a11,2)*powf(a24,2)*powf(a33,2))/a11 + (T_pitch*powf(a12,2)*powf(a23,2)*powf(a31,2))/a31 + (T_pitch*powf(a13,2)*powf(a22,2)*powf(a31,2))/a31 + (T_pitch*powf(a12,2)*powf(a24,2)*powf(a31,2))/a31 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a31,2))/a31 + (T_pitch*powf(a13,2)*powf(a24,2)*powf(a31,2))/a31 + (T_pitch*powf(a14,2)*powf(a23,2)*powf(a31,2))/a31 + (T_roll*powf(a12,2)*powf(a21,2)*powf(a33,2))/a21 + (T_roll*powf(a13,2)*powf(a21,2)*powf(a32,2))/a21 + (T_roll*powf(a12,2)*powf(a21,2)*powf(a34,2))/a21 + (T_roll*powf(a14,2)*powf(a21,2)*powf(a32,2))/a21 + (T_roll*powf(a13,2)*powf(a21,2)*powf(a34,2))/a21 + (T_roll*powf(a14,2)*powf(a21,2)*powf(a33,2))/a21 - (Fz*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (Fz*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (Fz*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (Fz*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (Fz*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (Fz*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (Fz*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (Fz*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (Fz*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (Fz*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (Fz*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (Fz*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (T_pitch*a12*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (T_pitch*a12*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (T_pitch*a13*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (T_pitch*a13*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (T_pitch*a14*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (T_pitch*a14*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (T_pitch*a22*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_pitch*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_pitch*a23*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_pitch*a23*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_pitch*a24*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_pitch*a24*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a12*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (T_roll*a12*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (T_roll*a13*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (T_roll*a13*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (T_roll*a14*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (T_roll*a14*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (T_roll*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_roll*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_roll*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_roll*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_roll*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (Fz*a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) - (Fz*a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (Fz*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (Fz*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (Fz*a22*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a23*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (Fz*a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) - (Fz*a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (Fz*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (Fz*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (Fz*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (Fz*a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) - (Fz*a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (Fz*a23*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (Fz*a24*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (Fz*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a12*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) - (T_pitch*a12*a23*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_pitch*a12*a23*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_pitch*a13*a22*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) - (T_pitch*a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_pitch*a13*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_pitch*a12*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_pitch*a12*a24*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_pitch*a14*a22*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) - (T_pitch*a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_pitch*a14*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_pitch*a13*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) - (T_pitch*a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a13*a24*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_pitch*a14*a23*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) - (T_pitch*a14*a23*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_pitch*a14*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_roll*a12*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_roll*a12*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_roll*a12*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a13*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_roll*a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_roll*a13*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (T_roll*a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_roll*a12*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_roll*a12*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a14*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_roll*a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_roll*a14*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (T_roll*a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_roll*a13*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_roll*a13*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a14*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_roll*a14*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_roll*a14*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a13,2)*powf(a21,2)*powf(a32,2) + powf(a13,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a13,2)*powf(a21,2)*powf(a34,2) + powf(a13,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a13,2)*powf(a22,2)*powf(a34,2) + powf(a13,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a13,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a13,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a13,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a13,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a13,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a13,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            /////////////
            // Motor
            out2 =  ((Fz*powf(a12,2)*powf(a21,2)*powf(a33,2))/a12 + (Fz*powf(a12,2)*powf(a23,2)*powf(a31,2))/a12 + (Fz*powf(a12,2)*powf(a21,2)*powf(a34,2))/a12 + (Fz*powf(a12,2)*powf(a24,2)*powf(a31,2))/a12 + (Fz*powf(a12,2)*powf(a23,2)*powf(a34,2))/a12 + (Fz*powf(a12,2)*powf(a24,2)*powf(a33,2))/a12 + (T_pitch*powf(a11,2)*powf(a23,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a32,2))/a32 + (T_pitch*powf(a11,2)*powf(a24,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a24,2)*powf(a32,2))/a32 + (T_pitch*powf(a14,2)*powf(a23,2)*powf(a32,2))/a32 + (T_roll*powf(a11,2)*powf(a22,2)*powf(a33,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a31,2))/a22 + (T_roll*powf(a11,2)*powf(a22,2)*powf(a34,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a31,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a34,2))/a22 + (T_roll*powf(a14,2)*powf(a22,2)*powf(a33,2))/a22 - (Fz*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (Fz*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (Fz*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (Fz*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (Fz*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (Fz*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (Fz*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (Fz*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (Fz*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (Fz*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (Fz*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (Fz*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (T_pitch*a11*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (T_pitch*a11*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (T_pitch*a13*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (T_pitch*a13*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (T_pitch*a14*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (T_pitch*a14*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (T_pitch*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_pitch*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (T_pitch*a23*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_pitch*a23*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_pitch*a24*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_pitch*a24*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a11*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (T_roll*a11*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (T_roll*a13*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (T_roll*a13*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (T_roll*a14*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (T_roll*a14*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (T_roll*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_roll*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_roll*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_roll*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) + (Fz*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (Fz*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (Fz*a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (Fz*a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (Fz*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (Fz*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (Fz*a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (Fz*a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (Fz*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (Fz*a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) - (Fz*a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (Fz*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (Fz*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (Fz*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (Fz*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a11*a23*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_pitch*a11*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_pitch*a13*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_pitch*a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_pitch*a13*a21*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_pitch*a11*a24*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_pitch*a11*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_pitch*a14*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_pitch*a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_pitch*a14*a21*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_pitch*a13*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) - (T_pitch*a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a13*a24*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_pitch*a14*a23*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) - (T_pitch*a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_pitch*a14*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_roll*a11*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_roll*a11*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) - (T_roll*a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (T_roll*a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_roll*a13*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a13*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_roll*a11*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_roll*a11*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) - (T_roll*a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (T_roll*a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_roll*a14*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a14*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_roll*a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_roll*a13*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_roll*a13*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a14*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_roll*a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_roll*a14*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            /////////////
            // Motor 3
            out3 =  ((Fz*powf(a14,2)*powf(a21,2)*powf(a32,2))/a13 + (Fz*powf(a14,2)*powf(a22,2)*powf(a31,2))/a13 + (Fz*powf(a14,2)*powf(a21,2)*powf(a34,2))/a13 + (Fz*powf(a14,2)*powf(a24,2)*powf(a31,2))/a13 + (Fz*powf(a14,2)*powf(a22,2)*powf(a34,2))/a13 + (Fz*powf(a14,2)*powf(a24,2)*powf(a32,2))/a13 + (T_pitch*powf(a11,2)*powf(a22,2)*powf(a33,2))/a33 + (T_pitch*powf(a12,2)*powf(a21,2)*powf(a33,2))/a33 + (T_pitch*powf(a11,2)*powf(a24,2)*powf(a33,2))/a33 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a33,2))/a33 + (T_pitch*powf(a12,2)*powf(a24,2)*powf(a33,2))/a33 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a33,2))/a33 + (T_roll*powf(a11,2)*powf(a23,2)*powf(a32,2))/a23 + (T_roll*powf(a12,2)*powf(a23,2)*powf(a31,2))/a23 + (T_roll*powf(a11,2)*powf(a23,2)*powf(a34,2))/a23 + (T_roll*powf(a14,2)*powf(a23,2)*powf(a31,2))/a23 + (T_roll*powf(a12,2)*powf(a23,2)*powf(a34,2))/a23 + (T_roll*powf(a14,2)*powf(a23,2)*powf(a32,2))/a23 - (Fz*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (Fz*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (Fz*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (Fz*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (Fz*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (Fz*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (Fz*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (Fz*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (Fz*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (Fz*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (Fz*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (Fz*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (T_pitch*a11*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (T_pitch*a11*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (T_pitch*a12*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (T_pitch*a12*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (T_pitch*a14*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (T_pitch*a14*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (T_pitch*a21*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_pitch*a21*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (T_pitch*a22*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_pitch*a22*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (T_pitch*a24*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_pitch*a24*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (T_roll*a14*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (T_roll*a14*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (T_roll*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (T_roll*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (T_roll*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_roll*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) + (Fz*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (Fz*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (Fz*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (Fz*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (Fz*a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (Fz*a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (Fz*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (Fz*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (Fz*a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) - (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (Fz*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (Fz*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (Fz*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (Fz*a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) - (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (Fz*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (Fz*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a11*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) - (T_pitch*a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (T_pitch*a11*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_pitch*a12*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) - (T_pitch*a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (T_pitch*a12*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_pitch*a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_pitch*a11*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_pitch*a14*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_pitch*a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_pitch*a14*a21*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (T_pitch*a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a12*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_pitch*a14*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_pitch*a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_pitch*a14*a22*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a11*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) - (T_roll*a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (T_roll*a11*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (T_roll*a12*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (T_roll*a12*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) - (T_roll*a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (T_roll*a11*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_roll*a11*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) - (T_roll*a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (T_roll*a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_roll*a14*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a14*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_roll*a12*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a12*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) - (T_roll*a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (T_roll*a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_roll*a14*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a14*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            /////////////
            // Motor 4
            out4 = ((Fz*powf(a14,2)*powf(a21,2)*powf(a32,2))/a14 + (Fz*powf(a14,2)*powf(a22,2)*powf(a31,2))/a14 + (Fz*powf(a14,2)*powf(a21,2)*powf(a33,2))/a14 + (Fz*powf(a14,2)*powf(a23,2)*powf(a31,2))/a14 + (Fz*powf(a14,2)*powf(a22,2)*powf(a33,2))/a14 + (Fz*powf(a14,2)*powf(a23,2)*powf(a32,2))/a14 + (T_pitch*powf(a11,2)*powf(a22,2)*powf(a34,2))/a34 + (T_pitch*powf(a12,2)*powf(a21,2)*powf(a34,2))/a34 + (T_pitch*powf(a11,2)*powf(a23,2)*powf(a34,2))/a34 + (T_pitch*powf(a14,2)*powf(a21,2)*powf(a34,2))/a34 + (T_pitch*powf(a12,2)*powf(a23,2)*powf(a34,2))/a34 + (T_pitch*powf(a14,2)*powf(a22,2)*powf(a34,2))/a34 + (T_roll*powf(a11,2)*powf(a24,2)*powf(a32,2))/a24 + (T_roll*powf(a12,2)*powf(a24,2)*powf(a31,2))/a24 + (T_roll*powf(a11,2)*powf(a24,2)*powf(a33,2))/a24 + (T_roll*powf(a14,2)*powf(a24,2)*powf(a31,2))/a24 + (T_roll*powf(a12,2)*powf(a24,2)*powf(a33,2))/a24 + (T_roll*powf(a14,2)*powf(a24,2)*powf(a32,2))/a24 - (Fz*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (Fz*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (Fz*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (Fz*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (Fz*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (Fz*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (Fz*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (Fz*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (Fz*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (Fz*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (Fz*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (Fz*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (T_pitch*a11*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (T_pitch*a11*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (T_pitch*a12*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (T_pitch*a12*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (T_pitch*a13*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (T_pitch*a13*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (T_pitch*a21*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_pitch*a21*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (T_pitch*a22*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_pitch*a22*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (T_pitch*a23*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_pitch*a23*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (T_roll*a11*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (T_roll*a12*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (T_roll*a13*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (T_roll*a13*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (T_roll*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (T_roll*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (T_roll*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (T_roll*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (Fz*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (Fz*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (Fz*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (Fz*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (Fz*a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (Fz*a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (Fz*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (Fz*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (Fz*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (Fz*a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (Fz*a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (Fz*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (Fz*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (Fz*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (Fz*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (Fz*a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (Fz*a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a11*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) - (T_pitch*a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (T_pitch*a11*a22*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_pitch*a12*a21*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) - (T_pitch*a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (T_pitch*a12*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_pitch*a11*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) - (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (T_pitch*a11*a23*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_pitch*a13*a21*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) - (T_pitch*a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (T_pitch*a13*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_pitch*a12*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) - (T_pitch*a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (T_pitch*a12*a23*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_pitch*a13*a22*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) - (T_pitch*a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (T_pitch*a13*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (T_roll*a11*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) - (T_roll*a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (T_roll*a11*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (T_roll*a12*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (T_roll*a12*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) - (T_roll*a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (T_roll*a11*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) - (T_roll*a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (T_roll*a11*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (T_roll*a13*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (T_roll*a13*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) - (T_roll*a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (T_roll*a12*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) - (T_roll*a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (T_roll*a12*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (T_roll*a13*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (T_roll*a13*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) - (T_roll*a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33))/(powf(a11,2)*powf(a22,2)*powf(a33,2) + powf(a11,2)*powf(a23,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a33,2) + powf(a12,2)*powf(a23,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a22,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a32,2) + powf(a12,2)*powf(a21,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a31,2) + powf(a11,2)*powf(a23,2)*powf(a34,2) + powf(a11,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a21,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a31,2) + powf(a14,2)*powf(a21,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a31,2) + powf(a12,2)*powf(a23,2)*powf(a34,2) + powf(a12,2)*powf(a24,2)*powf(a33,2) + powf(a14,2)*powf(a22,2)*powf(a34,2) + powf(a14,2)*powf(a24,2)*powf(a32,2) + powf(a14,2)*powf(a22,2)*powf(a33,2) + powf(a14,2)*powf(a23,2)*powf(a32,2) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22) - (a11*a22*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21) - (a12*a21*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23) - (a11*a23*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22) - (a13*a21*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24) - (a11*a24*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21) - (a14*a21*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24) - (a12*a23*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22) - (a13*a22*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24) - (a12*a24*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22) - (a14*a22*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24) - (a13*a24*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23) - (a14*a23*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24) - (a11*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a32) - (a11*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a31) - (a12*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a32) - (a11*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a31) - (a13*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a33) - (a11*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a31) - (a12*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a32) - (a13*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a33) - (a13*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a33) - (a11*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a34) - (a11*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a31) - (a14*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a34) - (a12*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a32) - (a12*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a32) - (a13*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a33) - (a14*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a34) - (a12*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a32) - (a14*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a34) - (a13*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a34) - (a13*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a33) - (a14*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a34) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a22*a31) - (a22*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a21*a32) - (a21*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a23*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a21*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a23*a31) - (a22*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a23*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a21*a33) - (a23*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a21*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a21*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a24*a31) - (a24*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a21*a34) - (a22*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a23*a32) - (a22*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a23*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a22*a33) - (a24*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a22*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a24*a32) - (a24*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a22*a34) - (a23*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a23*a34) - (a23*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a24*a33) - (a24*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a23*a34) + (a11*a22*a33*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a11*a23*a32*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a33*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a11*a23*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a21*a33*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a13*a22*a31) + (a12*a23*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a13*a21*a32) + (a12*a21*a33*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a12*a23*a31*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a21*a32*powf(a11,2)*powf(a22,2)*powf(a33,2))/(a11*a22*a33) + (a13*a22*a31*powf(a11,2)*powf(a23,2)*powf(a32,2))/(a11*a23*a32) + (a13*a21*a32*powf(a12,2)*powf(a23,2)*powf(a31,2))/(a12*a23*a31) + (a13*a22*a31*powf(a12,2)*powf(a21,2)*powf(a33,2))/(a12*a21*a33) + (a11*a22*a34*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a11*a24*a32*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a22*a34*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a11*a24*a32*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a21*a34*powf(a14,2)*powf(a22,2)*powf(a31,2))/(a14*a22*a31) + (a12*a24*a31*powf(a14,2)*powf(a21,2)*powf(a32,2))/(a14*a21*a32) + (a12*a21*a34*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a12*a24*a31*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a21*a32*powf(a11,2)*powf(a22,2)*powf(a34,2))/(a11*a22*a34) + (a14*a22*a31*powf(a11,2)*powf(a24,2)*powf(a32,2))/(a11*a24*a32) + (a14*a21*a32*powf(a12,2)*powf(a24,2)*powf(a31,2))/(a12*a24*a31) + (a14*a22*a31*powf(a12,2)*powf(a21,2)*powf(a34,2))/(a12*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a11*a24*a33*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a11*a23*a34*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a11*a24*a33*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a21*a34*powf(a14,2)*powf(a23,2)*powf(a31,2))/(a14*a23*a31) + (a13*a24*a31*powf(a14,2)*powf(a21,2)*powf(a33,2))/(a14*a21*a33) + (a13*a21*a34*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a13*a24*a31*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a21*a33*powf(a11,2)*powf(a23,2)*powf(a34,2))/(a11*a23*a34) + (a14*a23*a31*powf(a11,2)*powf(a24,2)*powf(a33,2))/(a11*a24*a33) + (a14*a21*a33*powf(a14,2)*powf(a24,2)*powf(a31,2))/(a13*a24*a31) + (a14*a23*a31*powf(a14,2)*powf(a21,2)*powf(a34,2))/(a13*a21*a34) + (a12*a23*a34*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a12*a24*a33*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34) + (a12*a23*a34*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a12*a24*a33*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a22*a34*powf(a14,2)*powf(a23,2)*powf(a32,2))/(a14*a23*a32) + (a13*a24*a32*powf(a14,2)*powf(a22,2)*powf(a33,2))/(a14*a22*a33) + (a13*a22*a34*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a13*a24*a32*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a22*a33*powf(a12,2)*powf(a23,2)*powf(a34,2))/(a12*a23*a34) + (a14*a23*a32*powf(a12,2)*powf(a24,2)*powf(a33,2))/(a12*a24*a33) + (a14*a22*a33*powf(a14,2)*powf(a24,2)*powf(a32,2))/(a13*a24*a32) + (a14*a23*a32*powf(a14,2)*powf(a22,2)*powf(a34,2))/(a13*a22*a34));

            ////////////////////////////////////////////////////
            // Resgatando os ângulos anteriores dos servomotores
            srv5 = _lastSrv5;
            srv6 = _lastSrv6;
            srv7 = _lastSrv7;
            srv8 = _lastSrv8;
        }

//        ag5_s = sin(_lastSrv5);
//        ag6_s = sin(_lastSrv6);
//        ag7_s = sin(_lastSrv7);
//        ag8_s = sin(_lastSrv8);

    // Delta PWM dos motores em % [0,1].
    out1 = constrain_float(out1, 0, 1.0);
    out2 = constrain_float(out2, 0, 1.0);
    out3 = constrain_float(out3, 0, 1.0);
    out4 = constrain_float(out4, 0, 1.0);

    mtr1 = out1;
    mtr2 = out2;
    mtr3 = out3;
    mtr4 = out4;

    // Conversão de radianos para graus.
    srv5 = srv5*rad2deg;
    srv6 = srv6*rad2deg;
    srv7 = srv7*rad2deg;
    srv8 = srv8*rad2deg;

//    // Arredondando os valores do servomotores para inteiro.
//    srv5 = roundf(srv5);
//    srv6 = roundf(srv6);
//    srv7 = roundf(srv7);
//    srv8 = roundf(srv8);

//    // Filtro para amenizar pequenas alterações no entorno da posição atual do servomotor.
//    _last2_Srv5 = _last2_Srv5*rad2deg*10;
//    _last2_Srv6 = _last2_Srv6*rad2deg*10;
//    _last2_Srv7 = _last2_Srv7*rad2deg*10;
//    _last2_Srv8 = _last2_Srv8*rad2deg*10;

//    srv5 = atenuate_servomtrs(srv5, _last2_Srv5, 1.0);
//    srv6 = atenuate_servomtrs(srv6, _last2_Srv6, 1.0);
//    srv7 = atenuate_servomtrs(srv7, _last2_Srv7, 1.0);
//    srv8 = atenuate_servomtrs(srv8, _last2_Srv8, 1.0);
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // loop through all the possible orders spinning any motors that match that description
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // turn on this motor
            rc_write(i, pwm);
        }
    }
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[motor_num] ) {
            motor_enabled[motor_num] = true;
        }

        // set roll, pitch, thottle factors and opposite motor (for stability patch)
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0;
        _pitch_factor[motor_num] = 0;
        _yaw_factor[motor_num] = 0;
    }
}

void AP_MotorsMatrix::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    bool success = false;

    switch (frame_class) {

        case MOTOR_FRAME_QUAD:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    ///////////////
                    /// MURILLO ///
                    ///////////////
//                    add_motor(AP_MOTORS_MOT_5,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor(AP_MOTORS_MOT_6,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
//                    add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
//                    add_motor(AP_MOTORS_MOT_8,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8);
                    success = true;
                    break;
//                case MOTOR_FRAME_TYPE_V:
//                    add_motor(AP_MOTORS_MOT_1,   45,  0.7981f,  1);
//                    add_motor(AP_MOTORS_MOT_2, -135,  1.0000f,  3);
//                    add_motor(AP_MOTORS_MOT_3,  -45, -0.7981f,  4);
//                    add_motor(AP_MOTORS_MOT_4,  135, -1.0000f,  2);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_H:
//                    // H frame set-up - same as X but motors spin in opposite directiSons
//                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_VTAIL:
//                    /*
//                        Tested with: Lynxmotion Hunter Vtail 400
//                        - inverted rear outward blowing motors (at a 40 degree angle)
//                        - should also work with non-inverted rear outward blowing motors
//                        - no roll in rear motors
//                        - no yaw in front motors
//                        - should fly like some mix between a tricopter and X Quadcopter

//                        Roll control comes only from the front motors, Yaw control only from the rear motors.
//                        Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

//                        Note: if we want the front motors to help with yaw,
//                            motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
//                            motors 3's yaw factor should be changed to -sin(radians(40))
//                    */
//                    add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
//                    add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
//                    add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
//                    add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_ATAIL:
//                    /*
//                        The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
//                        - The Yaw factors are reversed, because the rear motors are facing different directions

//                        With V-Shaped VTails, the props make a V-Shape when spinning, but with
//                        A-Shaped VTails, the props make an A-Shape when spinning.
//                        - Rear thrust on a V-Shaped V-Tail Quad is outward
//                        - Rear thrust on an A-Shaped V-Tail Quad is inward

//                        Still functions the same as the V-Shaped VTail mixing below:
//                        - Yaw control is entirely in the rear motors
//                        - Roll is is entirely in the front motors
//                    */
//                    add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
//                    add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
//                    add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
//                    add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
//                    success = true;
//                    break;
                default:
                    // quad frame class does not support this frame type
                    break;
            }
            break;  // quad

//        case MOTOR_FRAME_HEXA:
//            switch (frame_type) {
//                case MOTOR_FRAME_TYPE_PLUS:
//                    add_motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    add_motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_X:
//                    add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
//                    add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
//                    add_motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
//                    add_motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
//                    add_motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
//                    add_motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
//                    success = true;
//                    break;
//                default:
//                    // hexa frame class does not support this frame type
//                    break;
//            }
//            break;

//        case MOTOR_FRAME_OCTA:
//            switch (frame_type) {
//                case MOTOR_FRAME_TYPE_PLUS:
//                    add_motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
//                    add_motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    add_motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
//                    add_motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_X:
//                    add_motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor(AP_MOTORS_MOT_2, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor(AP_MOTORS_MOT_3,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor(AP_MOTORS_MOT_4,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor(AP_MOTORS_MOT_5,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
//                    add_motor(AP_MOTORS_MOT_6, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    add_motor(AP_MOTORS_MOT_7,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
//                    add_motor(AP_MOTORS_MOT_8,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_V:
//                    add_motor_raw(AP_MOTORS_MOT_1,  1.0f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
//                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    add_motor_raw(AP_MOTORS_MOT_4, -0.5f,  -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,   1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
//                    add_motor_raw(AP_MOTORS_MOT_6, -1.0f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor_raw(AP_MOTORS_MOT_7, -1.0f,   1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor_raw(AP_MOTORS_MOT_8,  0.5f,  -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_H:
//                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor_raw(AP_MOTORS_MOT_2,  1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor_raw(AP_MOTORS_MOT_3, -1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor_raw(AP_MOTORS_MOT_4, -1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
//                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    add_motor_raw(AP_MOTORS_MOT_7,  1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
//                    add_motor_raw(AP_MOTORS_MOT_8, -1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    success = true;
//                    break;
//                default:
//                    // octa frame class does not support this frame type
//                    break;
//            } // octa frame type
//            break;

//        case MOTOR_FRAME_OCTAQUAD:
//            switch (frame_type) {
//                case MOTOR_FRAME_TYPE_PLUS:
//                    add_motor(AP_MOTORS_MOT_1,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
//                    add_motor(AP_MOTORS_MOT_2,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
//                    add_motor(AP_MOTORS_MOT_3,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
//                    add_motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    add_motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
//                    add_motor(AP_MOTORS_MOT_6,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
//                    add_motor(AP_MOTORS_MOT_7,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor(AP_MOTORS_MOT_8,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_X:
//                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
//                    add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
//                    add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
//                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
//                    add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
//                    add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_V:
//                    add_motor(AP_MOTORS_MOT_1,   45,  0.7981f, 1);
//                    add_motor(AP_MOTORS_MOT_2,  -45, -0.7981f, 7);
//                    add_motor(AP_MOTORS_MOT_3, -135,  1.0000f, 5);
//                    add_motor(AP_MOTORS_MOT_4,  135, -1.0000f, 3);
//                    add_motor(AP_MOTORS_MOT_5,  -45,  0.7981f, 8);
//                    add_motor(AP_MOTORS_MOT_6,   45, -0.7981f, 2);
//                    add_motor(AP_MOTORS_MOT_7,  135,  1.0000f, 4);
//                    add_motor(AP_MOTORS_MOT_8, -135, -1.0000f, 6);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_H:
//                    // H frame set-up - same as X but motors spin in opposite directions
//                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7);
//                    add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
//                    add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8);
//                    add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
//                    add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    success = true;
//                    break;
//                default:
//                    // octaquad frame class does not support this frame type
//                    break;
//            }
//            break;

//        case MOTOR_FRAME_DODECAHEXA: {
//            switch (frame_type) {
//                case MOTOR_FRAME_TYPE_PLUS:
//                    add_motor(AP_MOTORS_MOT_1,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);  // forward-top
//                    add_motor(AP_MOTORS_MOT_2,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);  // forward-bottom
//                    add_motor(AP_MOTORS_MOT_3,    60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);  // forward-right-top
//                    add_motor(AP_MOTORS_MOT_4,    60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);  // forward-right-bottom
//                    add_motor(AP_MOTORS_MOT_5,   120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);  // back-right-top
//                    add_motor(AP_MOTORS_MOT_6,   120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);  // back-right-bottom
//                    add_motor(AP_MOTORS_MOT_7,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);  // back-top
//                    add_motor(AP_MOTORS_MOT_8,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);  // back-bottom
//                    add_motor(AP_MOTORS_MOT_9,  -120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 9);  // back-left-top
//                    add_motor(AP_MOTORS_MOT_10, -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10); // back-left-bottom
//                    add_motor(AP_MOTORS_MOT_11,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  11); // forward-left-top
//                    add_motor(AP_MOTORS_MOT_12,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 12); // forward-left-bottom
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_X:
//                    add_motor(AP_MOTORS_MOT_1,    30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1); // forward-right-top
//                    add_motor(AP_MOTORS_MOT_2,    30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2); // forward-right-bottom
//                    add_motor(AP_MOTORS_MOT_3,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3); // right-top
//                    add_motor(AP_MOTORS_MOT_4,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4); // right-bottom
//                    add_motor(AP_MOTORS_MOT_5,   150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5); // back-right-top
//                    add_motor(AP_MOTORS_MOT_6,   150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6); // back-right-bottom
//                    add_motor(AP_MOTORS_MOT_7,  -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7); // back-left-top
//                    add_motor(AP_MOTORS_MOT_8,  -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8); // back-left-bottom
//                    add_motor(AP_MOTORS_MOT_9,   -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9); // left-top
//                    add_motor(AP_MOTORS_MOT_10,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10); // left-bottom
//                    add_motor(AP_MOTORS_MOT_11,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  11); // forward-left-top
//                    add_motor(AP_MOTORS_MOT_12,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 12); // forward-left-bottom
//                    success = true;
//                    break;
//                default:
//                    // dodeca-hexa frame class does not support this frame type
//                    break;
//            }}
//            break;

//        case MOTOR_FRAME_Y6:
//            switch (frame_type) {
//                case MOTOR_FRAME_TYPE_Y6B:
//                    // Y6 motor definition with all top motors spinning clockwise, all bottom motors counter clockwise
//                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor_raw(AP_MOTORS_MOT_3,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
//                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
//                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    success = true;
//                    break;
//                case MOTOR_FRAME_TYPE_Y6F:
//                    // Y6 motor layout for FireFlyY6
//                    add_motor_raw(AP_MOTORS_MOT_1,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
//                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
//                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
//                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
//                    add_motor_raw(AP_MOTORS_MOT_5, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
//                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
//                    success = true;
//                    break;
//                default:
//                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
//                    add_motor_raw(AP_MOTORS_MOT_2,  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
//                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
//                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
//                    add_motor_raw(AP_MOTORS_MOT_5, -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
//                    add_motor_raw(AP_MOTORS_MOT_6,  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
//                    success = true;
//                    break;
//            }
//            break;

        default:
            // matrix doesn't support the configured class
            break;
        } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    _flags.initialised_ok = success;
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
void AP_MotorsMatrix::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (roll_fac < fabsf(_roll_factor[i])) {
                roll_fac = fabsf(_roll_factor[i]);
            }
            if (pitch_fac < fabsf(_pitch_factor[i])) {
                pitch_fac = fabsf(_pitch_factor[i]);
            }
            if (yaw_fac < fabsf(_yaw_factor[i])) {
                yaw_fac = fabsf(_yaw_factor[i]);
            }
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f*_roll_factor[i]/roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f*_pitch_factor[i]/pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f*_yaw_factor[i]/yaw_fac;
            }
        }
    }
}


/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}
