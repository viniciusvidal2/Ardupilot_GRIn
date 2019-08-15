/// @file	AP_MotorsMatrix.h
/// @brief	Motor control class for Matrixcopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsMatrix(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
    {};

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type);

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type);

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    void                set_update_rate(uint16_t speed_hz);

    // enable - starts allowing signals to be sent to motors
    void                enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    void                output_test(uint8_t motor_seq, int16_t pwm);

    // output_to_motors - sends minimum values out to the motors
    void                output_to_motors();

    // MURILLO
    void                output_to_motors(float &srv1, float &srv2, float &srv3, float &srv4, float &PWM1, float &PWM2, float &PWM3, float &PWM4);
    void                update_srv_action(float srv1, float srv2, float srv3, float srv4);
    void                pwm_servo_angle(float &servo_m1, float &servo_m2, float &servo_m3, float &servo_m4);
    float               servo_angle_to_pwm(float angle,float srv_min_pwm, float srv_max_pwm);

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint16_t            get_motor_mask();

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    // MURILLO
    void                output_armed_stabilizing(float&FX, float &FY, float &TN, float &srv1, float &srv2, float &srv3, float &srv4, float &PWM1, float &PWM2, float &PWM3, float &PWM4);
    void                FOSSEN_alocation_matrix(float &FX,float &FY,float &TN,float &Theta1,float &Theta2,float &Theta3,float &Theta4,float &PWM1,float &PWM2,float &PWM3,float &PWM4);
    float               NormtoPWM(float val);
    float               PWMtoNorm(float pwm);

    // add_motor using raw roll, pitch, throttle and yaw factors
    void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order);

    // add_motor using just position and yaw_factor (or prop direction)
    void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    // add_motor using separate roll and pitch factors (for asymmetrical frames) and prop direction
    void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

    // remove_motor
    void                remove_motor(int8_t motor_num);

    // configures the motors for the defined frame_class and frame_type
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type);

    // normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
    void                normalise_rpy_factors();

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;
    
    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
    float               _thrust_rpyt_out[AP_MOTORS_MAX_NUM_MOTORS]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
    uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence
    motor_frame_class   _last_frame_class; // most recently requested frame class (i.e. quad, hexa, octa, etc)
    motor_frame_type    _last_frame_type; // most recently requested frame type (i.e. plus, x, v, etc)

    // MURILLO
    // Propriedade Física do Barco
        float FT  = 0.0f;
        float FM1 = 10*0.86;
        float FM2 = 10*2.60;
        float FM3 = 10*0.86;
        float FM4 = 10*2.60;

        float Fmax = FM1 + FM2 + FM3 + FM4;       // Força e torque maximos do barco

        float L  = 0.586f;          // Tamanho do braço do barco
        float Lx = L*cosf(M_PI/4.0f);
        float Ly = L*cosf(M_PI/4.0f);

        float Pwmmax = 1001.0f; // Esse valor será a faixa de pwm que eu vou escolher para trabalhar --------------- // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
        float Pwmmin = 1.0f;    // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
        float Nmax   = L*Fmax;

        float k1 = (FM1)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
        float k2 = (FM2)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
        float k3 = (FM3)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória
        float k4 = (FM4)/(Pwmmax-Pwmmin); // Esse valor é atualizado no AduCopter.cpp para corresponder aos valores de memória

    //    float k2 = k1;
    //    float k3 = k1;
    //    float k4 = k1;
};
