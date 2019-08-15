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
#include <AP_HAL/AP_HAL.h>
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

// MURILLO
void AP_MotorsMatrix::update_srv_action(float srv1, float srv2, float srv3, float srv4)
{
    srv1 = lround(srv1);
    srv2 = lround(srv2);
    srv3 = lround(srv3);
    srv4 = lround(srv4);
    hal.rcout->write(8, uint16_t(srv1));  // Servo 1
    hal.rcout->write(9, uint16_t(srv2));  // Servo 2
    hal.rcout->write(10,uint16_t(srv3));  // Servo 3
    hal.rcout->write(11,uint16_t(srv4));  // Servo 4
}



// MURILLO
void AP_MotorsMatrix::output_to_motors(float &srv1, float &srv2, float &srv3, float &srv4, float &PWM1, float &PWM2, float &PWM3, float &PWM4)
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
//            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
//                if (motor_enabled[i]) {
//                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
//                }
//            }
        // MURILLO
                PWM1 = constrain_float(PWM1,0.0f,1.0f);
                PWM2 = constrain_float(PWM2,0.0f,1.0f);
                PWM3 = constrain_float(PWM3,0.0f,1.0f);
                PWM4 = constrain_float(PWM4,0.0f,1.0f);
                motor_enabled[0] ? motor_out[0]=calc_thrust_to_pwm(PWM1): motor_out[0]=calc_spin_up_to_pwm();
                motor_enabled[1] ? motor_out[1]=calc_thrust_to_pwm(PWM2): motor_out[1]=calc_spin_up_to_pwm();
                motor_enabled[2] ? motor_out[2]=calc_thrust_to_pwm(PWM3): motor_out[2]=calc_spin_up_to_pwm();
                motor_enabled[3] ? motor_out[3]=calc_thrust_to_pwm(PWM4): motor_out[3]=calc_spin_up_to_pwm();
            break;
    }

    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }

    // MURILLO
    //Atualiza a saida dos servos
    update_srv_action(srv1,srv2,srv3,srv4);
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

    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
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

// MURILLO

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing(float &FX,float &FY,float &TN, float &srv1, float &srv2, float &srv3, float &srv4, float &PWM1, float &PWM2, float &PWM3, float &PWM4)
{
    uint8_t i;                          // general purpose counter
//    float   roll_thrust;                // roll thrust input value, +/- 1.0
//    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
//    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
//    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
//    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
//    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
//    float   rpy_low = 0.0f;             // lowest motor value
//    float   rpy_high = 0.0f;            // highest motor value
//    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
//    float   unused_range;               // amount of yaw we can fit in the current channel
//    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
//    roll_thrust = _roll_in * get_compensation_gain();
//    pitch_thrust = _pitch_in * get_compensation_gain();
//    yaw_thrust = _yaw_in * get_compensation_gain();
//    throttle_thrust = get_throttle() * get_compensation_gain();

    // MURILLO

    // Colocar o controle de alocação do Fossen que o Mathaus implementou.

    FOSSEN_alocation_matrix(FX,FY,TN,srv1,srv2,srv3,srv4,PWM1,PWM2,PWM3,PWM4);
    pwm_servo_angle(srv1,srv2,srv3,srv4);

    _thrust_rpyt_out[0] = PWM1;
    _thrust_rpyt_out[1] = PWM2;
    _thrust_rpyt_out[2] = PWM3;
    _thrust_rpyt_out[3] = PWM4;


    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);
        }
    }
}

// MURILLO
void AP_MotorsMatrix::FOSSEN_alocation_matrix(float &FX,float &FY,float &TN,float &Theta1,float &Theta2,float &Theta3,float &Theta4,float &PWM1,float &PWM2,float &PWM3,float &PWM4)
{
    /// TRABALHA COM RADIANOS
    /// Fx = força no eixo X - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// Fy = força no eixo y - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// tN = torque de guinada - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// Função para alocar as forças do barco a partir da metodologia descrita em FOSSEN
    //Tratamento para o stick do throttle estar sempre acima da zona morta

    FX = constrain_float(FX,-1.0f,1.0f);
    FY = constrain_float(FY,-1.0f,1.0f);
    TN = constrain_float(TN,-1.0f,1.0f);
    TN = TN * Nmax;
    FX = FX * Fmax;
    FY = FY * Fmax;

    FT = sqrt(sq(TN/L) + sq(FX) + sq(FY));
    FT = constrain_float(FT,0.0f,Fmax);

    // Converte o valor normalizado de 0  a 1 para PWM
    PWM1 = NormtoPWM(PWM1);
    PWM2 = NormtoPWM(PWM2);
    PWM3 = NormtoPWM(PWM3);
    PWM4 = NormtoPWM(PWM4);
    // Convertendo de grau para Radianos
    Theta1 = Theta1 * DEG_TO_RAD;
    Theta2 = Theta2 * DEG_TO_RAD;
    Theta3 = Theta3 * DEG_TO_RAD;
    Theta4 = Theta4 * DEG_TO_RAD;

    if(FT<0.02*Fmax)
    {
        //Se as forças são muito pequenas(proximas a zero) nao executa a matriz de alocação Envia os angulos  0
        Theta1 = 0.0f;
        Theta2 = 0.0f;
        Theta3 = 0.0f;
        Theta4 = 0.0f;
        //Envia todos os PWMs muito pequenos (Nulos-Na prática) Os valores aqui, não estão normalizados entre 0 e 1
        PWM1 = NormtoPWM(0.0f);
        PWM2 = NormtoPWM(0.0f);
        PWM3 = NormtoPWM(0.0f);
        PWM4 = NormtoPWM(0.0f);
    }else
    {
        // ============ Angulo calculado a partir da força
        Theta1 = atan2f(((TN*(Lx*k1*sq(k2) + Lx*k1*sq(k4)))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FY*(sq(Lx)*k1*powf(k2,4) + sq(Lx)*k1*powf(k4,4) + sq(Lx)*powf(k1,3)*sq(k2) + sq(Lx)*powf(k1,3)*sq(k4) + 2*sq(Ly)*powf(k1,3)*sq(k2) + 2*sq(Ly)*powf(k1,3)*sq(k3) + sq(Lx)*k1*sq(k2)*sq(k3) + 2*sq(Lx)*k1*sq(k2)*sq(k4) + sq(Lx)*k1*sq(k3)*sq(k4) + 2*sq(Ly)*k1*sq(k2)*sq(k4) + 2*sq(Ly)*k1*sq(k3)*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FX*(Lx*k1*sq(k2) + Lx*k1*sq(k4))*(Ly*sq(k1) - Ly*sq(k2) - Ly*sq(k3) + Ly*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))),((FX*(sq(Ly)*k1*powf(k2,4) + sq(Ly)*k1*powf(k3,4) + 2*sq(Lx)*powf(k1,3)*sq(k2) + 2*sq(Lx)*powf(k1,3)*sq(k4) + sq(Ly)*powf(k1,3)*sq(k2) + sq(Ly)*powf(k1,3)*sq(k3) + 2*sq(Lx)*k1*sq(k2)*sq(k3) + 2*sq(Lx)*k1*sq(k3)*sq(k4) + 2*sq(Ly)*k1*sq(k2)*sq(k3) + sq(Ly)*k1*sq(k2)*sq(k4) + sq(Ly)*k1*sq(k3)*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (TN*(Ly*k1*sq(k2) + Ly*k1*sq(k3)))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FY*(Ly*k1*sq(k2) + Ly*k1*sq(k3))*(Lx*sq(k1) - Lx*sq(k2) + Lx*sq(k3) - Lx*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));
        Theta2 = atan2f(((FY*(sq(Lx)*powf(k1,4)*k2 + sq(Lx)*k2*powf(k3,4) + sq(Lx)*sq(k1)*powf(k2,3) + sq(Lx)*powf(k2,3)*sq(k3) + 2*sq(Ly)*sq(k1)*powf(k2,3) + 2*sq(Ly)*powf(k2,3)*sq(k4) + 2*sq(Lx)*sq(k1)*k2*sq(k3) + sq(Lx)*sq(k1)*k2*sq(k4) + sq(Lx)*k2*sq(k3)*sq(k4) + 2*sq(Ly)*sq(k1)*k2*sq(k3) + 2*sq(Ly)*k2*sq(k3)*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (TN*(Lx*sq(k1)*k2 + Lx*k2*sq(k3)))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (FX*(Lx*sq(k1)*k2 + Lx*k2*sq(k3))*(Ly*sq(k1) - Ly*sq(k2) - Ly*sq(k3) + Ly*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))),((TN*(Ly*sq(k1)*k2 + Ly*k2*sq(k4)))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FX*(sq(Ly)*powf(k1,4)*k2 + sq(Ly)*k2*powf(k4,4) + 2*sq(Lx)*sq(k1)*powf(k2,3) + 2*sq(Lx)*powf(k2,3)*sq(k3) + sq(Ly)*sq(k1)*powf(k2,3) + sq(Ly)*powf(k2,3)*sq(k4) + 2*sq(Lx)*sq(k1)*k2*sq(k4) + 2*sq(Lx)*k2*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*k2*sq(k3) + 2*sq(Ly)*sq(k1)*k2*sq(k4) + sq(Ly)*k2*sq(k3)*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (FY*(Ly*sq(k1)*k2 + Ly*k2*sq(k4))*(Lx*sq(k1) - Lx*sq(k2) + Lx*sq(k3) - Lx*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));
        Theta3 = atan2f(((TN*(Lx*sq(k2)*k3 + Lx*k3*sq(k4)))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FY*(sq(Lx)*powf(k2,4)*k3 + sq(Lx)*k3*powf(k4,4) + sq(Lx)*sq(k2)*powf(k3,3) + sq(Lx)*powf(k3,3)*sq(k4) + 2*sq(Ly)*sq(k1)*powf(k3,3) + 2*sq(Ly)*powf(k3,3)*sq(k4) + sq(Lx)*sq(k1)*sq(k2)*k3 + sq(Lx)*sq(k1)*k3*sq(k4) + 2*sq(Lx)*sq(k2)*k3*sq(k4) + 2*sq(Ly)*sq(k1)*sq(k2)*k3 + 2*sq(Ly)*sq(k2)*k3*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FX*(Lx*sq(k2)*k3 + Lx*k3*sq(k4))*(Ly*sq(k1) - Ly*sq(k2) - Ly*sq(k3) + Ly*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))),((TN*(Ly*sq(k1)*k3 + Ly*k3*sq(k4)))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FX*(sq(Ly)*powf(k1,4)*k3 + sq(Ly)*k3*powf(k4,4) + 2*sq(Lx)*sq(k2)*powf(k3,3) + 2*sq(Lx)*powf(k3,3)*sq(k4) + sq(Ly)*sq(k1)*powf(k3,3) + sq(Ly)*powf(k3,3)*sq(k4) + 2*sq(Lx)*sq(k1)*sq(k2)*k3 + 2*sq(Lx)*sq(k1)*k3*sq(k4) + sq(Ly)*sq(k1)*sq(k2)*k3 + 2*sq(Ly)*sq(k1)*k3*sq(k4) + sq(Ly)*sq(k2)*k3*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (FY*(Ly*sq(k1)*k3 + Ly*k3*sq(k4))*(Lx*sq(k1) - Lx*sq(k2) + Lx*sq(k3) - Lx*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));
        Theta4 = atan2f(((FY*(sq(Lx)*powf(k1,4)*k4 + sq(Lx)*powf(k3,4)*k4 + sq(Lx)*sq(k1)*powf(k4,3) + sq(Lx)*sq(k3)*powf(k4,3) + 2*sq(Ly)*sq(k2)*powf(k4,3) + 2*sq(Ly)*sq(k3)*powf(k4,3) + sq(Lx)*sq(k1)*sq(k2)*k4 + 2*sq(Lx)*sq(k1)*sq(k3)*k4 + sq(Lx)*sq(k2)*sq(k3)*k4 + 2*sq(Ly)*sq(k1)*sq(k2)*k4 + 2*sq(Ly)*sq(k1)*sq(k3)*k4))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (TN*(Lx*sq(k1)*k4 + Lx*sq(k3)*k4))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (FX*(Lx*sq(k1)*k4 + Lx*sq(k3)*k4)*(Ly*sq(k1) - Ly*sq(k2) - Ly*sq(k3) + Ly*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))),((FX*(sq(Ly)*powf(k2,4)*k4 + sq(Ly)*powf(k3,4)*k4 + 2*sq(Lx)*sq(k1)*powf(k4,3) + 2*sq(Lx)*sq(k3)*powf(k4,3) + sq(Ly)*sq(k2)*powf(k4,3) + sq(Ly)*sq(k3)*powf(k4,3) + 2*sq(Lx)*sq(k1)*sq(k2)*k4 + 2*sq(Lx)*sq(k2)*sq(k3)*k4 + sq(Ly)*sq(k1)*sq(k2)*k4 + sq(Ly)*sq(k1)*sq(k3)*k4 + 2*sq(Ly)*sq(k2)*sq(k3)*k4))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) - (TN*(Ly*sq(k2)*k4 + Ly*sq(k3)*k4))/(2*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (FY*(Ly*sq(k2)*k4 + Ly*sq(k3)*k4)*(Lx*sq(k1) - Lx*sq(k2) + Lx*sq(k3) - Lx*sq(k4)))/(2*(sq(k1) + sq(k2) + sq(k3) + sq(k4))*(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));

        // Saturação
        Theta1 = constrain_float(Theta1,-M_PI,M_PI);
        Theta2 = constrain_float(Theta2,-M_PI,M_PI);
        Theta3 = constrain_float(Theta3,-M_PI,M_PI);
        Theta4 = constrain_float(Theta4,-M_PI,M_PI);

        // ============================ PWM calculado a partir da força
        PWM1 = (sqrt((sq(k1)*sq(FY*sq(Lx)*powf(k2,4) + FY*sq(Lx)*powf(k4,4) + Lx*TN*powf(k2,4) + Lx*TN*powf(k4,4) - FX*Lx*Ly*powf(k2,4) + FX*Lx*Ly*powf(k4,4) + Lx*TN*sq(k1)*sq(k2) + Lx*TN*sq(k1)*sq(k4) + Lx*TN*sq(k2)*sq(k3) + 2*Lx*TN*sq(k2)*sq(k4) + Lx*TN*sq(k3)*sq(k4) + FY*sq(Lx)*sq(k1)*sq(k2) + FY*sq(Lx)*sq(k1)*sq(k4) + FY*sq(Lx)*sq(k2)*sq(k3) + 2*FY*sq(Lx)*sq(k2)*sq(k4) + FY*sq(Lx)*sq(k3)*sq(k4) + 2*FY*sq(Ly)*sq(k1)*sq(k2) + 2*FY*sq(Ly)*sq(k1)*sq(k3) + 2*FY*sq(Ly)*sq(k2)*sq(k4) + 2*FY*sq(Ly)*sq(k3)*sq(k4) + FX*Lx*Ly*sq(k1)*sq(k2) + FX*Lx*Ly*sq(k1)*sq(k4) - FX*Lx*Ly*sq(k2)*sq(k3) - FX*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (sq(k1)*sq(FX*sq(Ly)*powf(k2,4) + FX*sq(Ly)*powf(k3,4) - Ly*TN*powf(k2,4) - Ly*TN*powf(k3,4) - FY*Lx*Ly*powf(k2,4) + FY*Lx*Ly*powf(k3,4) - Ly*TN*sq(k1)*sq(k2) - Ly*TN*sq(k1)*sq(k3) - 2*Ly*TN*sq(k2)*sq(k3) - Ly*TN*sq(k2)*sq(k4) - Ly*TN*sq(k3)*sq(k4) + 2*FX*sq(Lx)*sq(k1)*sq(k2) + 2*FX*sq(Lx)*sq(k1)*sq(k4) + 2*FX*sq(Lx)*sq(k2)*sq(k3) + 2*FX*sq(Lx)*sq(k3)*sq(k4) + FX*sq(Ly)*sq(k1)*sq(k2) + FX*sq(Ly)*sq(k1)*sq(k3) + 2*FX*sq(Ly)*sq(k2)*sq(k3) + FX*sq(Ly)*sq(k2)*sq(k4) + FX*sq(Ly)*sq(k3)*sq(k4) + FY*Lx*Ly*sq(k1)*sq(k2) + FY*Lx*Ly*sq(k1)*sq(k3) - FY*Lx*Ly*sq(k2)*sq(k4) - FY*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));
        PWM2 = (sqrt((sq(k2)*sq(FY*sq(Lx)*powf(k1,4) + FY*sq(Lx)*powf(k3,4) - Lx*TN*powf(k1,4) - Lx*TN*powf(k3,4) - FX*Lx*Ly*powf(k1,4) + FX*Lx*Ly*powf(k3,4) - Lx*TN*sq(k1)*sq(k2) - 2*Lx*TN*sq(k1)*sq(k3) - Lx*TN*sq(k1)*sq(k4) - Lx*TN*sq(k2)*sq(k3) - Lx*TN*sq(k3)*sq(k4) + FY*sq(Lx)*sq(k1)*sq(k2) + 2*FY*sq(Lx)*sq(k1)*sq(k3) + FY*sq(Lx)*sq(k1)*sq(k4) + FY*sq(Lx)*sq(k2)*sq(k3) + FY*sq(Lx)*sq(k3)*sq(k4) + 2*FY*sq(Ly)*sq(k1)*sq(k2) + 2*FY*sq(Ly)*sq(k1)*sq(k3) + 2*FY*sq(Ly)*sq(k2)*sq(k4) + 2*FY*sq(Ly)*sq(k3)*sq(k4) + FX*Lx*Ly*sq(k1)*sq(k2) - FX*Lx*Ly*sq(k1)*sq(k4) + FX*Lx*Ly*sq(k2)*sq(k3) - FX*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (sq(k2)*sq(FX*sq(Ly)*powf(k1,4) + FX*sq(Ly)*powf(k4,4) + Ly*TN*powf(k1,4) + Ly*TN*powf(k4,4) - FY*Lx*Ly*powf(k1,4) + FY*Lx*Ly*powf(k4,4) + Ly*TN*sq(k1)*sq(k2) + Ly*TN*sq(k1)*sq(k3) + 2*Ly*TN*sq(k1)*sq(k4) + Ly*TN*sq(k2)*sq(k4) + Ly*TN*sq(k3)*sq(k4) + 2*FX*sq(Lx)*sq(k1)*sq(k2) + 2*FX*sq(Lx)*sq(k1)*sq(k4) + 2*FX*sq(Lx)*sq(k2)*sq(k3) + 2*FX*sq(Lx)*sq(k3)*sq(k4) + FX*sq(Ly)*sq(k1)*sq(k2) + FX*sq(Ly)*sq(k1)*sq(k3) + 2*FX*sq(Ly)*sq(k1)*sq(k4) + FX*sq(Ly)*sq(k2)*sq(k4) + FX*sq(Ly)*sq(k3)*sq(k4) + FY*Lx*Ly*sq(k1)*sq(k2) - FY*Lx*Ly*sq(k1)*sq(k3) + FY*Lx*Ly*sq(k2)*sq(k4) - FY*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));
        PWM3 = (sqrt((sq(k3)*sq(FY*sq(Lx)*powf(k2,4) + FY*sq(Lx)*powf(k4,4) + Lx*TN*powf(k2,4) + Lx*TN*powf(k4,4) - FX*Lx*Ly*powf(k2,4) + FX*Lx*Ly*powf(k4,4) + Lx*TN*sq(k1)*sq(k2) + Lx*TN*sq(k1)*sq(k4) + Lx*TN*sq(k2)*sq(k3) + 2*Lx*TN*sq(k2)*sq(k4) + Lx*TN*sq(k3)*sq(k4) + FY*sq(Lx)*sq(k1)*sq(k2) + FY*sq(Lx)*sq(k1)*sq(k4) + FY*sq(Lx)*sq(k2)*sq(k3) + 2*FY*sq(Lx)*sq(k2)*sq(k4) + FY*sq(Lx)*sq(k3)*sq(k4) + 2*FY*sq(Ly)*sq(k1)*sq(k2) + 2*FY*sq(Ly)*sq(k1)*sq(k3) + 2*FY*sq(Ly)*sq(k2)*sq(k4) + 2*FY*sq(Ly)*sq(k3)*sq(k4) + FX*Lx*Ly*sq(k1)*sq(k2) + FX*Lx*Ly*sq(k1)*sq(k4) - FX*Lx*Ly*sq(k2)*sq(k3) - FX*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (sq(k3)*sq(FX*sq(Ly)*powf(k1,4) + FX*sq(Ly)*powf(k4,4) + Ly*TN*powf(k1,4) + Ly*TN*powf(k4,4) - FY*Lx*Ly*powf(k1,4) + FY*Lx*Ly*powf(k4,4) + Ly*TN*sq(k1)*sq(k2) + Ly*TN*sq(k1)*sq(k3) + 2*Ly*TN*sq(k1)*sq(k4) + Ly*TN*sq(k2)*sq(k4) + Ly*TN*sq(k3)*sq(k4) + 2*FX*sq(Lx)*sq(k1)*sq(k2) + 2*FX*sq(Lx)*sq(k1)*sq(k4) + 2*FX*sq(Lx)*sq(k2)*sq(k3) + 2*FX*sq(Lx)*sq(k3)*sq(k4) + FX*sq(Ly)*sq(k1)*sq(k2) + FX*sq(Ly)*sq(k1)*sq(k3) + 2*FX*sq(Ly)*sq(k1)*sq(k4) + FX*sq(Ly)*sq(k2)*sq(k4) + FX*sq(Ly)*sq(k3)*sq(k4) + FY*Lx*Ly*sq(k1)*sq(k2) - FY*Lx*Ly*sq(k1)*sq(k3) + FY*Lx*Ly*sq(k2)*sq(k4) - FY*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));
        PWM4 = (sqrt((sq(k4)*sq(FY*sq(Lx)*powf(k1,4) + FY*sq(Lx)*powf(k3,4) - Lx*TN*powf(k1,4) - Lx*TN*powf(k3,4) - FX*Lx*Ly*powf(k1,4) + FX*Lx*Ly*powf(k3,4) - Lx*TN*sq(k1)*sq(k2) - 2*Lx*TN*sq(k1)*sq(k3) - Lx*TN*sq(k1)*sq(k4) - Lx*TN*sq(k2)*sq(k3) - Lx*TN*sq(k3)*sq(k4) + FY*sq(Lx)*sq(k1)*sq(k2) + 2*FY*sq(Lx)*sq(k1)*sq(k3) + FY*sq(Lx)*sq(k1)*sq(k4) + FY*sq(Lx)*sq(k2)*sq(k3) + FY*sq(Lx)*sq(k3)*sq(k4) + 2*FY*sq(Ly)*sq(k1)*sq(k2) + 2*FY*sq(Ly)*sq(k1)*sq(k3) + 2*FY*sq(Ly)*sq(k2)*sq(k4) + 2*FY*sq(Ly)*sq(k3)*sq(k4) + FX*Lx*Ly*sq(k1)*sq(k2) - FX*Lx*Ly*sq(k1)*sq(k4) + FX*Lx*Ly*sq(k2)*sq(k3) - FX*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4))) + (sq(k4)*sq(FX*sq(Ly)*powf(k2,4) + FX*sq(Ly)*powf(k3,4) - Ly*TN*powf(k2,4) - Ly*TN*powf(k3,4) - FY*Lx*Ly*powf(k2,4) + FY*Lx*Ly*powf(k3,4) - Ly*TN*sq(k1)*sq(k2) - Ly*TN*sq(k1)*sq(k3) - 2*Ly*TN*sq(k2)*sq(k3) - Ly*TN*sq(k2)*sq(k4) - Ly*TN*sq(k3)*sq(k4) + 2*FX*sq(Lx)*sq(k1)*sq(k2) + 2*FX*sq(Lx)*sq(k1)*sq(k4) + 2*FX*sq(Lx)*sq(k2)*sq(k3) + 2*FX*sq(Lx)*sq(k3)*sq(k4) + FX*sq(Ly)*sq(k1)*sq(k2) + FX*sq(Ly)*sq(k1)*sq(k3) + 2*FX*sq(Ly)*sq(k2)*sq(k3) + FX*sq(Ly)*sq(k2)*sq(k4) + FX*sq(Ly)*sq(k3)*sq(k4) + FY*Lx*Ly*sq(k1)*sq(k2) + FY*Lx*Ly*sq(k1)*sq(k3) - FY*Lx*Ly*sq(k2)*sq(k4) - FY*Lx*Ly*sq(k3)*sq(k4)))/(4*sq(sq(k1) + sq(k2) + sq(k3) + sq(k4))*sq(sq(Lx)*sq(k1)*sq(k2) + sq(Lx)*sq(k1)*sq(k4) + sq(Lx)*sq(k2)*sq(k3) + sq(Lx)*sq(k3)*sq(k4) + sq(Ly)*sq(k1)*sq(k2) + sq(Ly)*sq(k1)*sq(k3) + sq(Ly)*sq(k2)*sq(k4) + sq(Ly)*sq(k3)*sq(k4)))));

        // Saturação
        PWM1 = constrain_float(PWM1,Pwmmin,Pwmmax);
        PWM2 = constrain_float(PWM2,Pwmmin,Pwmmax);
        PWM3 = constrain_float(PWM3,Pwmmin,Pwmmax);
        PWM4 = constrain_float(PWM4,Pwmmin,Pwmmax);

        // Normaliza o valor de PWM encontrado entre 0 e 1 para ativar a saida entre mínima e maxima potência
        PWM1 = PWMtoNorm(PWM1);
        PWM2 = PWMtoNorm(PWM2);
        PWM3 = PWMtoNorm(PWM3);
        PWM4 = PWMtoNorm(PWM4);

        // Conveter o valor de Theta para Graus
        Theta1 = Theta1 * RAD_TO_DEG;
        Theta2 = Theta2 * RAD_TO_DEG;
        Theta3 = Theta3 * RAD_TO_DEG;
        Theta4 = Theta4 * RAD_TO_DEG;
    }
}

// MURILLO
void AP_MotorsMatrix::pwm_servo_angle(float &servo_m1, float &servo_m2, float &servo_m3, float &servo_m4)
{
    /// todos os angulos devem estar em graus nesta função

    //Linha utilizada para medir valores de pwm min e max
    //    servo_m4 = (channel_throttle->get_radio_in()-channel_throttle->get_radio_min()) + 1.5*(canalservo->get_radio_in()-canalservo->get_radio_min());
//    servo_m1 = servo_angle_to_pwm(servo_m1,675.0,2329.0);
//    servo_m2 = servo_angle_to_pwm(servo_m2,664.0,2144.0);
//    servo_m3 = servo_angle_to_pwm(servo_m3,656.0,2400.0);
//    servo_m4 = servo_angle_to_pwm(servo_m4,700.0,2345.0);

    servo_m1 = servo_angle_to_pwm(servo_m1,675.0,2329.0);
    servo_m2 = servo_angle_to_pwm(servo_m2,664.0,2144.0);
    servo_m3 = servo_angle_to_pwm(servo_m3,575.0,2257.0);
    servo_m4 = servo_angle_to_pwm(servo_m4,700.0,2345.0);

}

// MURILLO
float AP_MotorsMatrix::servo_angle_to_pwm(float angle,float srv_min_pwm, float srv_max_pwm)
{
    /// Nessa função pode-se inserir os valores mínimos e maxímos do pwm  considerando 0 a 180 como angulos mínimos e máximos
    //Entrada de angulo deve ser  de -90 a 90 ELE CHEGARÁ A 180 DEVIDO A ENGRENAGEM
    angle = constrain_float(angle,-180.0,180.0);
    angle = angle + 180.0;

    //valor que o servo entende como 0 graus
    float srv_min_angle = 0.0;

    //valor de pwm que o servo entende como 180
    float srv_max_angle = 360.0;
    float pwm           =  srv_min_pwm + angle * (srv_max_pwm - srv_min_pwm)/(srv_max_angle - srv_min_angle);
    return pwm;
}

// MURILLO
float AP_MotorsMatrix::NormtoPWM(float val)
{
    /// Entra um valor de 0 a 1 e sai um PWM
    return val*(Pwmmax-Pwmmin) + Pwmmin;
}

// MURILLO
float AP_MotorsMatrix::PWMtoNorm(float pwm)
{
    /// Entra um valor de PWM e sai de 0 a 1
    float V;
    V = float(pwm - Pwmmin)/float(Pwmmax-Pwmmin);
    return constrain_float(V,0.0f,1.0f);
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
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_V:
                    add_motor(AP_MOTORS_MOT_1,   45,  0.7981f,  1);
                    add_motor(AP_MOTORS_MOT_2, -135,  1.0000f,  3);
                    add_motor(AP_MOTORS_MOT_3,  -45, -0.7981f,  4);
                    add_motor(AP_MOTORS_MOT_4,  135, -1.0000f,  2);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    // H frame set-up - same as X but motors spin in opposite directiSons
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_VTAIL:
                    /*
                        Tested with: Lynxmotion Hunter Vtail 400
                        - inverted rear outward blowing motors (at a 40 degree angle)
                        - should also work with non-inverted rear outward blowing motors
                        - no roll in rear motors
                        - no yaw in front motors
                        - should fly like some mix between a tricopter and X Quadcopter

                        Roll control comes only from the front motors, Yaw control only from the rear motors.
                        Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

                        Note: if we want the front motors to help with yaw,
                            motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
                            motors 3's yaw factor should be changed to -sin(radians(40))
                    */
                    add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
                    add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
                    add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
                    add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_ATAIL:
                    /*
                        The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
                        - The Yaw factors are reversed, because the rear motors are facing different directions

                        With V-Shaped VTails, the props make a V-Shape when spinning, but with
                        A-Shaped VTails, the props make an A-Shape when spinning.
                        - Rear thrust on a V-Shaped V-Tail Quad is outward
                        - Rear thrust on an A-Shaped V-Tail Quad is inward

                        Still functions the same as the V-Shaped VTail mixing below:
                        - Yaw control is entirely in the rear motors
                        - Roll is is entirely in the front motors
                    */
                    add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
                    add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
                    add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
                    success = true;
                    break;
                default:
                    // quad frame class does not support this frame type
                    break;
            }
            break;  // quad

        case MOTOR_FRAME_HEXA:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    add_motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    success = true;
                    break;
                default:
                    // hexa frame class does not support this frame type
                    break;
            }
            break;

        case MOTOR_FRAME_OCTA:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_3,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_4,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_5,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor(AP_MOTORS_MOT_7,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_8,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_V:
                    add_motor_raw(AP_MOTORS_MOT_1,  1.0f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor_raw(AP_MOTORS_MOT_4, -0.5f,  -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,   1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor_raw(AP_MOTORS_MOT_6, -1.0f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_7, -1.0f,   1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_8,  0.5f,  -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_2,  1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor_raw(AP_MOTORS_MOT_3, -1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_4, -1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor_raw(AP_MOTORS_MOT_7,  1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor_raw(AP_MOTORS_MOT_8, -1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    success = true;
                    break;
                default:
                    // octa frame class does not support this frame type
                    break;
            } // octa frame type
            break;

        case MOTOR_FRAME_OCTAQUAD:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_2,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_3,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6,    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor(AP_MOTORS_MOT_7,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_8,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
                    add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
                    add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_V:
                    add_motor(AP_MOTORS_MOT_1,   45,  0.7981f, 1);
                    add_motor(AP_MOTORS_MOT_2,  -45, -0.7981f, 7);
                    add_motor(AP_MOTORS_MOT_3, -135,  1.0000f, 5);
                    add_motor(AP_MOTORS_MOT_4,  135, -1.0000f, 3);
                    add_motor(AP_MOTORS_MOT_5,  -45,  0.7981f, 8);
                    add_motor(AP_MOTORS_MOT_6,   45, -0.7981f, 2);
                    add_motor(AP_MOTORS_MOT_7,  135,  1.0000f, 4);
                    add_motor(AP_MOTORS_MOT_8, -135, -1.0000f, 6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_H:
                    // H frame set-up - same as X but motors spin in opposite directions
                    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7);
                    add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8);
                    add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    success = true;
                    break;
                default:
                    // octaquad frame class does not support this frame type
                    break;
            }
            break;

        case MOTOR_FRAME_DODECAHEXA: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);  // forward-top
                    add_motor(AP_MOTORS_MOT_2,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);  // forward-bottom
                    add_motor(AP_MOTORS_MOT_3,    60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);  // forward-right-top
                    add_motor(AP_MOTORS_MOT_4,    60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);  // forward-right-bottom
                    add_motor(AP_MOTORS_MOT_5,   120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);  // back-right-top
                    add_motor(AP_MOTORS_MOT_6,   120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);  // back-right-bottom
                    add_motor(AP_MOTORS_MOT_7,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);  // back-top
                    add_motor(AP_MOTORS_MOT_8,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);  // back-bottom
                    add_motor(AP_MOTORS_MOT_9,  -120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 9);  // back-left-top
                    add_motor(AP_MOTORS_MOT_10, -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10); // back-left-bottom
                    add_motor(AP_MOTORS_MOT_11,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  11); // forward-left-top
                    add_motor(AP_MOTORS_MOT_12,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 12); // forward-left-bottom
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    add_motor(AP_MOTORS_MOT_1,    30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1); // forward-right-top
                    add_motor(AP_MOTORS_MOT_2,    30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2); // forward-right-bottom
                    add_motor(AP_MOTORS_MOT_3,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3); // right-top
                    add_motor(AP_MOTORS_MOT_4,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4); // right-bottom
                    add_motor(AP_MOTORS_MOT_5,   150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5); // back-right-top
                    add_motor(AP_MOTORS_MOT_6,   150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6); // back-right-bottom
                    add_motor(AP_MOTORS_MOT_7,  -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7); // back-left-top
                    add_motor(AP_MOTORS_MOT_8,  -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8); // back-left-bottom
                    add_motor(AP_MOTORS_MOT_9,   -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9); // left-top
                    add_motor(AP_MOTORS_MOT_10,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10); // left-bottom
                    add_motor(AP_MOTORS_MOT_11,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  11); // forward-left-top
                    add_motor(AP_MOTORS_MOT_12,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 12); // forward-left-bottom
                    success = true;
                    break;
                default:
                    // dodeca-hexa frame class does not support this frame type
                    break;
            }}
            break;

        case MOTOR_FRAME_Y6:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_Y6B:
                    // Y6 motor definition with all top motors spinning clockwise, all bottom motors counter clockwise
                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_3,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
                    add_motor_raw(AP_MOTORS_MOT_5,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_Y6F:
                    // Y6 motor layout for FireFlyY6
                    add_motor_raw(AP_MOTORS_MOT_1,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    add_motor_raw(AP_MOTORS_MOT_2, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor_raw(AP_MOTORS_MOT_5, -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
                    add_motor_raw(AP_MOTORS_MOT_6,  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6);
                    success = true;
                    break;
                default:
                    add_motor_raw(AP_MOTORS_MOT_1, -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
                    add_motor_raw(AP_MOTORS_MOT_2,  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
                    add_motor_raw(AP_MOTORS_MOT_3,  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
                    add_motor_raw(AP_MOTORS_MOT_4,  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
                    add_motor_raw(AP_MOTORS_MOT_5, -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
                    add_motor_raw(AP_MOTORS_MOT_6,  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
                    success = true;
                    break;
            }
            break;

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
