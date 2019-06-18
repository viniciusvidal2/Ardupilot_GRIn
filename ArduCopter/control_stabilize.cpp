#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::stabilize_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    //channel_pitch->get_control_in();

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
//    get_pilot_desired_lean_angles(0,0, target_roll, target_pitch, aparm.angle_max);
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
//    target_yaw_rate = 0;
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // Mathaus
    get_pilot_desired_force_to_boat_M();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}


//  MATHAUS
void Copter::get_pilot_desired_force_to_boat_M()
{
    //Essa abordagem considera que o stick direito controla a força em X e Y.
    //a posição do stick determina a intensidade da foça nos eixos onde, o ponto médio é o (0,0).
    //O Yaw é controlado da mesma maneira que em um quadrotor, contudo, o código foi construido de forma empirica.

    // Calcula o valor médio dos sticks do controle para que seja possível dividir em forças positivas e negativas

    float_t med_roll  = (channel_roll->get_radio_min() + ((channel_roll->get_radio_max() - channel_roll->get_radio_min())/2.0));
    float_t med_pitch = (channel_pitch->get_radio_min()+ ((channel_pitch->get_radio_max()- channel_pitch->get_radio_min())/2.0));
    float_t med_yaw   = (channel_yaw->get_radio_min()  + ((channel_yaw->get_radio_max()  - channel_yaw->get_radio_min())/2.0));

    //Calcula a força em Y a partir do stick de Rolagem
    Fy = float(channel_roll->get_radio_in()- med_roll)/float(channel_roll->get_radio_max() - med_roll);
    //Calcula a força em X a partir do stick de Arfagem
    Fx = float(channel_pitch->get_radio_in()-med_pitch)/float(channel_pitch->get_radio_max()- med_pitch);
    //Calcula o torque em Z a partir do stick de Guinada
    tN = float(channel_yaw->get_radio_in()-  med_yaw)/float(channel_yaw->get_radio_max() - med_yaw);

    Fx = constrain_float(Fx,-1.0f,1.0f);
    Fy = constrain_float(Fy,-1.0f,1.0f);

    Fx = map(Fx,Fy);
    Fy = map(Fy,Fx);

    // Envia os valores para a matriz de alocação de forças
//    FOSSEN_alocation_matrix(Fx,Fy,tN,theta_m1,theta_m2,theta_m3,theta_m4,Pwm1,Pwm2,Pwm3,Pwm4);
}
