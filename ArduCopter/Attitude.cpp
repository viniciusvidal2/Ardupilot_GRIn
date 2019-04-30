#include "Copter.h"
#include "math.h"
#pragma GCC diagnostic ignored "-Wframe-larger-than="

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float Copter::get_smoothing_gain()
{
    return (2.0f + (float)g.rc_feel_rp/10.0f);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Copter::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // sanity check angle max parameter
    aparm.angle_max = constrain_int16(aparm.angle_max,1000,8000);

    // limit max lean angle
    angle_max = constrain_float(angle_max, 1000, aparm.angle_max);

    // scale roll_in, pitch_in to ANGLE_MAX parameter range
    float scaler = aparm.angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = norm(pitch_in, roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI) * atanf(cosf(pitch_in*(M_PI/18000))*tanf(roll_in*(M_PI/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;

    // Mathaus ( zerando a atitude de rollagem e arfagem)
    roll_out = roll_out*0;
    pitch_out = pitch_out*0;

    // Mathaus -- onde deve ser chamada a função
    // get_pilot_desired_force_to_boat_M();

}

// Mathaus
//Função para transformar os comandos de roll pitch e yaw em comandos de força para o barco.

void Copter:: get_pilot_desired_force_to_boat()
{
    // TRABALHA COM ANGULOS EM GRAUS
    //A intensidade da força total é controlada pelo stick to throtle.
    //Essa abordagem considera que o stick direito controla a distribuição da Força ( através do ângulo theta).
    //O ângulo theta é alterado na dinâmica correspondente a rolagem.
    //O Yaw é controlado da mesma maneira que em um quadrotor, contudo, o código foi construido de forma empirica.

    float_t med_roll  = round(channel_roll->get_radio_min()+((channel_roll->get_radio_max()-channel_roll->get_radio_min())/2));

    theta_motor =  (channel_roll->get_radio_in() - med_roll)*(-90.0/(med_roll - channel_roll->get_radio_min()));


    float_t med_yaw = round(channel_yaw->get_radio_min()+((channel_yaw->get_radio_max()-channel_yaw->get_radio_min())/2));
    float_t theta_yaw = (channel_yaw->get_radio_in() - med_yaw)*(-90.0/(med_yaw - channel_yaw->get_radio_min()));


    theta_m1 = theta_motor + theta_yaw;
    theta_m2 = theta_motor - theta_yaw;
    theta_m3 = theta_motor + theta_yaw;
    theta_m4 = theta_motor - theta_yaw;

}

void Copter:: get_pilot_desired_force_to_boat(float roll, float pitch, float yaw)
{
    // TRABALHA COM ANGULOS EM GRAUS
    //implementação que LEO quer!
    //Essa abordagem considera que o stick direito controla a força em X e Y.
    //a posição do stick determina a intensidade da foça nos eixos onde, o ponto médio é o (0,0).
    //O Yaw é controlado da mesma maneira que em um quadrotor, contudo, o código foi construido de forma empirica.

    float_t med_roll  = round(channel_roll->get_radio_min()+((channel_roll->get_radio_max()-channel_roll->get_radio_min())/2));
    float_t med_pitch = round(channel_pitch->get_radio_min()+((channel_pitch->get_radio_max()-channel_pitch->get_radio_min())/2));

    theta_motor = atan2f(-(channel_roll->get_radio_in() - med_roll),(channel_pitch->get_radio_in()-med_pitch)) *(18000/M_PI);


    float_t med_yaw = round(channel_yaw->get_radio_min()+((channel_yaw->get_radio_max()-channel_yaw->get_radio_min())/2));
    float_t theta_yaw = (channel_yaw->get_radio_in() - med_yaw)*(90.0/(med_yaw - channel_yaw->get_radio_min()));


    servo_m1 = theta_motor + theta_yaw;
    servo_m2 = theta_motor - theta_yaw;
    servo_m3 = theta_motor + theta_yaw;
    servo_m4 = theta_motor - theta_yaw;

    //Saturações
    if(servo_m1>90) servo_m1 = 90.0;
    if(servo_m1<-90) servo_m1 = -90.0;

    if(servo_m2>90) servo_m2 = 90.0;
    if(servo_m2<-90) servo_m2 = -90.0;

    if(servo_m3>90) servo_m3 = 90.0;
    if(servo_m3<-90) servo_m3 = -90.0;

    if(servo_m4>90) servo_m4 = 90.0;
    if(servo_m4<-90) servo_m4 = -90.0;
}

float Copter::max(float *vet)
{
    int i =0;
    int t = sizeof(vet);
    float_t max_val = 0;
    while(i<t)
    {
        if(vet[i]>max_val){
            max_val = vet[i];
        }
        i++;
    }
    return max_val;
}

float Copter::Saturacao(float Var , float Sup, float Infe)
{
    /// Var = variavel a ser saturada
    /// Sup = Limite superior de saturação
    /// Infe = Limite inferior de saturação
    // Saturação Superior
    Var > Sup? Var = Sup : Var = Var;

    //Saturação Inferior
    Var < Infe? Var = Infe : Var = Var;

    return Var;
}

float Copter::PWMtoNorm(float pwm)
{
    /// Retorna um valor de 0 a 1 para o PWM considerando os valores maximos e mínimos do GCS
    return Saturacao(pwm/(Pwmmax-Pwmmin),1.0f,0.0f);
}


float Copter::NormtoPWM(float pwm)
{
    /// Retorna um valor de 0 a DELTA_PWM para o valor normalizado considerando os valores maximos e mínimos do GCS
    return pwm*(Pwmmax-Pwmmin);
}

void Copter::alocation_matrix(float &FT,float &FX,float &FY,float &N,float &theta_motor1,float &theta_motor2,float &theta_motor3,float &theta_motor4,float &PWM1,float &PWM2,float &PWM3,float &PWM4)
{
    /// TRABALHA COM RADIANOS
    /// Fx = força no eixo X - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// Fy = força no eixo y - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// N  = tork de guinada - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// Função para alocar as forças do barco a partir da metodologia desenvolvida na  Tese de Doutorado do Murillo,
    /// a Matriz A = [Fx;Fy;N] foi subdividida em A1 = [Ft;Fx,N] e A2 = [Ft;Fy;N]
    /// A linha Ft foi adicionada a tecnica para evitar linhas de 0 na primeira matriz, fazendo com que a solução nao seja possivel///
    float ARC_seno[5] = {};

    float M = 0;

    float Num_p1;
    float Den_p1;
    float Num_p2;
    float Den_p2;
    float Num_p3;
    float Den_p3;
    float Num_p4;
    float Den_p4;

    FX = Saturacao(FX,Fmax,-Fmax);
    FY = Saturacao(FY,Fmax,-Fmax);

    //Calculo da Força total
    FT = sqrt(sq(FX)+sq(FY));

    if(FT<0.05 && FT>-0.05){
        FT = FT + N;
    }

    // Converte o valor normalizado para PWM
    PWM1 = NormtoPWM(PWM1);
    PWM2 = NormtoPWM(PWM2);
    PWM3 = NormtoPWM(PWM3);
    PWM4 = NormtoPWM(PWM4);

    // Convertendo para Radianos
    theta_motor1 = theta_motor1 * DEG_TO_RAD;
    theta_motor2 = theta_motor2 * DEG_TO_RAD;
    theta_motor3 = theta_motor3 * DEG_TO_RAD;
    theta_motor4 = theta_motor4 * DEG_TO_RAD;


    if((FT<0.05*Fmax && FT>-0.05*Fmax)&&(N<0.05*Fmax && N>-0.05*Fmax)){
        // Se as forças são muito pequenas ( proximas a zero) nao executa a matriz de alocação
        // Envia todos os angulos  nulos
        theta_motor1 = 0.0;
        theta_motor2 = 0.0;
        theta_motor3 = 0.0;
        theta_motor4 = 0.0;

        //Envia todos os PWMs nulos
        PWM1 = 1.0f;
        PWM2 = 1.0f;
        PWM3 = 1.0f;
        PWM4 = 1.0f;
    }else{
        for(int v=0;v<1;v++){
            M = 0.0;

            //Seno do angulo atual
            s_th_m1 = sinf(theta_motor1);
            s_th_m2 = sinf(theta_motor2);
            s_th_m3 = sinf(theta_motor3);
            s_th_m4 = sinf(theta_motor4);

            //Cosseno do angulo atual
            c_th_m1 = cosf(theta_motor1);
            c_th_m2 = cosf(theta_motor2);
            c_th_m3 = cosf(theta_motor3);
            c_th_m4 = cosf(theta_motor4);

            // ========================================= PWM calculado a partir da força e dos angulos ==========================================================================
            //quebrando o PWM em várias equações para diminuir o espaço do codigo

            Num_p1 = (2*M1_Lx*N*c_th_m1*sq(c_th_m2) + 2*M1_Lx*N*c_th_m1*sq(c_th_m3) + 2*M2_Lx*N*c_th_m1*sq(c_th_m2) + 2*M1_Lx*N*c_th_m1*sq(c_th_m4) + M2_Lx*N*c_th_m2*sq(c_th_m3) - M2_Lx*N*sq(c_th_m2)*c_th_m3 + 2*M3_Lx*N*c_th_m1*sq(c_th_m3) + M2_Lx*N*c_th_m2*sq(c_th_m4) - M2_Lx*N*sq(c_th_m2)*c_th_m4 - M3_Lx*N*c_th_m2*sq(c_th_m3) + M3_Lx*N*sq(c_th_m2)*c_th_m3 - 2*M4_Lx*N*c_th_m1*sq(c_th_m4) + M3_Lx*N*c_th_m3*sq(c_th_m4) - M3_Lx*N*sq(c_th_m3)*c_th_m4 + M4_Lx*N*c_th_m2*sq(c_th_m4) - M4_Lx*N*sq(c_th_m2)*c_th_m4 + M4_Lx*N*c_th_m3*sq(c_th_m4) - M4_Lx*N*sq(c_th_m3)*c_th_m4 - 2*M1_Ly*N*sq(c_th_m2)*s_th_m1 - 2*M1_Ly*N*sq(c_th_m3)*s_th_m1 - 2*M1_Ly*N*sq(c_th_m4)*s_th_m1 - M2_Ly*N*sq(c_th_m3)*s_th_m2 - M2_Ly*N*sq(c_th_m4)*s_th_m2 + M3_Ly*N*sq(c_th_m2)*s_th_m3 + M3_Ly*N*sq(c_th_m4)*s_th_m3 - M4_Ly*N*sq(c_th_m2)*s_th_m4 - M4_Ly*N*sq(c_th_m3)*s_th_m4 - 2*FX*sq(M2_Lx)*c_th_m1*sq(c_th_m2) + FX*sq(M2_Lx)*sq(c_th_m2)*c_th_m3 - 2*FX*sq(M3_Lx)*c_th_m1*sq(c_th_m3) + FX*sq(M2_Lx)*sq(c_th_m2)*c_th_m4 + FX*sq(M3_Lx)*c_th_m2*sq(c_th_m3) - 2*FX*sq(M4_Lx)*c_th_m1*sq(c_th_m4) + FX*sq(M3_Lx)*sq(c_th_m3)*c_th_m4 + FX*sq(M4_Lx)*c_th_m2*sq(c_th_m4) + FX*sq(M4_Lx)*c_th_m3*sq(c_th_m4) - 2*FX*sq(M2_Ly)*c_th_m1*sq(s_th_m2) + FX*sq(M2_Ly)*c_th_m3*sq(s_th_m2) - 2*FX*sq(M3_Ly)*c_th_m1*sq(s_th_m3) + FX*sq(M2_Ly)*c_th_m4*sq(s_th_m2) + FX*sq(M3_Ly)*c_th_m2*sq(s_th_m3) - 2*FX*sq(M4_Ly)*c_th_m1*sq(s_th_m4) + FX*sq(M3_Ly)*c_th_m4*sq(s_th_m3) + FX*sq(M4_Ly)*c_th_m2*sq(s_th_m4) + FX*sq(M4_Ly)*c_th_m3*sq(s_th_m4) - FT*sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m3) - FT*sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m4) - FT*sq(M3_Lx)*sq(c_th_m2)*sq(c_th_m3) - FT*sq(M3_Lx)*sq(c_th_m3)*sq(c_th_m4) - FT*sq(M4_Lx)*sq(c_th_m2)*sq(c_th_m4) - FT*sq(M4_Lx)*sq(c_th_m3)*sq(c_th_m4) - FT*sq(M2_Ly)*sq(c_th_m3)*sq(s_th_m2) - FT*sq(M2_Ly)*sq(c_th_m4)*sq(s_th_m2) - FT*sq(M3_Ly)*sq(c_th_m2)*sq(s_th_m3) - FT*sq(M3_Ly)*sq(c_th_m4)*sq(s_th_m3) - FT*sq(M4_Ly)*sq(c_th_m2)*sq(s_th_m4) - FT*sq(M4_Ly)*sq(c_th_m3)*sq(s_th_m4) - 2*M1_Lx*N*c_th_m1*c_th_m2*c_th_m3 - 2*M1_Lx*N*c_th_m1*c_th_m2*c_th_m4 - M2_Lx*N*c_th_m1*c_th_m2*c_th_m3 - 2*M1_Lx*N*c_th_m1*c_th_m3*c_th_m4 - M2_Lx*N*c_th_m1*c_th_m2*c_th_m4 - M3_Lx*N*c_th_m1*c_th_m2*c_th_m3 - M3_Lx*N*c_th_m1*c_th_m3*c_th_m4 + M4_Lx*N*c_th_m1*c_th_m2*c_th_m4 + M4_Lx*N*c_th_m1*c_th_m3*c_th_m4 + 2*M1_Ly*N*c_th_m2*c_th_m3*s_th_m1 - 2*M2_Ly*N*c_th_m1*c_th_m2*s_th_m2 + 2*M1_Ly*N*c_th_m2*c_th_m4*s_th_m1 + M2_Ly*N*c_th_m1*c_th_m3*s_th_m2 + 2*M1_Ly*N*c_th_m3*c_th_m4*s_th_m1 + M2_Ly*N*c_th_m1*c_th_m4*s_th_m2 + M2_Ly*N*c_th_m2*c_th_m3*s_th_m2 - M3_Ly*N*c_th_m1*c_th_m2*s_th_m3 + M2_Ly*N*c_th_m2*c_th_m4*s_th_m2 + 2*M3_Ly*N*c_th_m1*c_th_m3*s_th_m3 - M3_Ly*N*c_th_m1*c_th_m4*s_th_m3 - M3_Ly*N*c_th_m2*c_th_m3*s_th_m3 + M4_Ly*N*c_th_m1*c_th_m2*s_th_m4 + M4_Ly*N*c_th_m1*c_th_m3*s_th_m4 - M3_Ly*N*c_th_m3*c_th_m4*s_th_m3 - 2*M4_Ly*N*c_th_m1*c_th_m4*s_th_m4 + M4_Ly*N*c_th_m2*c_th_m4*s_th_m4 + M4_Ly*N*c_th_m3*c_th_m4*s_th_m4 - 2*FX*M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2) - 2*FX*M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3) + 2*FX*M1_Lx*M4_Lx*c_th_m1*sq(c_th_m4) - FX*M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3) - FX*M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3 + FX*M2_Lx*M4_Lx*c_th_m2*sq(c_th_m4) + FX*M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m4 + FX*M3_Lx*M4_Lx*c_th_m3*sq(c_th_m4) + FX*M3_Lx*M4_Lx*sq(c_th_m3)*c_th_m4 + 2*FX*M1_Ly*M2_Lx*sq(c_th_m2)*s_th_m1 + 2*FX*M1_Ly*M3_Lx*sq(c_th_m3)*s_th_m1 - 2*FX*M1_Ly*M4_Lx*sq(c_th_m4)*s_th_m1 - FX*M2_Lx*M3_Ly*sq(c_th_m2)*s_th_m3 + FX*M2_Ly*M3_Lx*sq(c_th_m3)*s_th_m2 + FX*M2_Lx*M4_Ly*sq(c_th_m2)*s_th_m4 - FX*M2_Ly*M4_Lx*sq(c_th_m4)*s_th_m2 + FX*M3_Lx*M4_Ly*sq(c_th_m3)*s_th_m4 + FX*M3_Ly*M4_Lx*sq(c_th_m4)*s_th_m3 + 2*FT*M2_Lx*M3_Lx*sq(c_th_m2)*sq(c_th_m3) - 2*FT*M2_Lx*M4_Lx*sq(c_th_m2)*sq(c_th_m4) - 2*FT*M3_Lx*M4_Lx*sq(c_th_m3)*sq(c_th_m4) + FT*sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m3 + FT*sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m4 + FT*sq(M3_Lx)*c_th_m1*c_th_m2*sq(c_th_m3) + FT*sq(M3_Lx)*c_th_m1*sq(c_th_m3)*c_th_m4 + FT*sq(M4_Lx)*c_th_m1*c_th_m2*sq(c_th_m4) + FT*sq(M4_Lx)*c_th_m1*c_th_m3*sq(c_th_m4) + FT*sq(M2_Ly)*c_th_m1*c_th_m3*sq(s_th_m2) + FT*sq(M2_Ly)*c_th_m1*c_th_m4*sq(s_th_m2) + FT*sq(M3_Ly)*c_th_m1*c_th_m2*sq(s_th_m3) + FT*sq(M3_Ly)*c_th_m1*c_th_m4*sq(s_th_m3) + FT*sq(M4_Ly)*c_th_m1*c_th_m2*sq(s_th_m4) + FT*sq(M4_Ly)*c_th_m1*c_th_m3*sq(s_th_m4) + FX*M1_Lx*M2_Lx*c_th_m1*c_th_m2*c_th_m3 + FX*M1_Lx*M2_Lx*c_th_m1*c_th_m2*c_th_m4 + FX*M1_Lx*M3_Lx*c_th_m1*c_th_m2*c_th_m3 + 2*FX*M2_Lx*M3_Lx*c_th_m1*c_th_m2*c_th_m3 + FX*M1_Lx*M3_Lx*c_th_m1*c_th_m3*c_th_m4 - FX*M1_Lx*M4_Lx*c_th_m1*c_th_m2*c_th_m4 - FX*M1_Lx*M4_Lx*c_th_m1*c_th_m3*c_th_m4 - 2*FX*M2_Lx*M4_Lx*c_th_m1*c_th_m2*c_th_m4 - 2*FX*M3_Lx*M4_Lx*c_th_m1*c_th_m3*c_th_m4 + 2*FX*M1_Lx*M2_Ly*c_th_m1*c_th_m2*s_th_m2 - FX*M1_Lx*M2_Ly*c_th_m1*c_th_m3*s_th_m2 - FX*M1_Ly*M2_Lx*c_th_m2*c_th_m3*s_th_m1 + 4*FX*M2_Lx*M2_Ly*c_th_m1*c_th_m2*s_th_m2 - FX*M1_Lx*M2_Ly*c_th_m1*c_th_m4*s_th_m2 + FX*M1_Lx*M3_Ly*c_th_m1*c_th_m2*s_th_m3 - FX*M1_Ly*M2_Lx*c_th_m2*c_th_m4*s_th_m1 - FX*M1_Ly*M3_Lx*c_th_m2*c_th_m3*s_th_m1 - 2*FX*M1_Lx*M3_Ly*c_th_m1*c_th_m3*s_th_m3 - 2*FX*M2_Lx*M2_Ly*c_th_m2*c_th_m3*s_th_m2 + 2*FX*M2_Lx*M3_Ly*c_th_m1*c_th_m2*s_th_m3 - 2*FX*M2_Ly*M3_Lx*c_th_m1*c_th_m3*s_th_m2 + FX*M1_Lx*M3_Ly*c_th_m1*c_th_m4*s_th_m3 - FX*M1_Lx*M4_Ly*c_th_m1*c_th_m2*s_th_m4 - FX*M1_Ly*M3_Lx*c_th_m3*c_th_m4*s_th_m1 + FX*M1_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m1 - 2*FX*M2_Lx*M2_Ly*c_th_m2*c_th_m4*s_th_m2 + FX*M2_Ly*M3_Lx*c_th_m2*c_th_m3*s_th_m2 - FX*M1_Lx*M4_Ly*c_th_m1*c_th_m3*s_th_m4 + FX*M1_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m1 - FX*M2_Lx*M3_Ly*c_th_m2*c_th_m3*s_th_m3 - 2*FX*M2_Lx*M4_Ly*c_th_m1*c_th_m2*s_th_m4 + 2*FX*M2_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m2 - 4*FX*M3_Lx*M3_Ly*c_th_m1*c_th_m3*s_th_m3 + 2*FX*M1_Lx*M4_Ly*c_th_m1*c_th_m4*s_th_m4 - FX*M2_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m2 + 2*FX*M3_Lx*M3_Ly*c_th_m2*c_th_m3*s_th_m3 - 2*FX*M3_Lx*M4_Ly*c_th_m1*c_th_m3*s_th_m4 - 2*FX*M3_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m3 + FX*M2_Lx*M4_Ly*c_th_m2*c_th_m4*s_th_m4 + 2*FX*M3_Lx*M3_Ly*c_th_m3*c_th_m4*s_th_m3 + FX*M3_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m3 - 4*FX*M4_Lx*M4_Ly*c_th_m1*c_th_m4*s_th_m4 + FX*M3_Lx*M4_Ly*c_th_m3*c_th_m4*s_th_m4 + 2*FX*M4_Lx*M4_Ly*c_th_m2*c_th_m4*s_th_m4 + 2*FX*M4_Lx*M4_Ly*c_th_m3*c_th_m4*s_th_m4 - 2*FX*M1_Ly*M2_Ly*c_th_m2*s_th_m1*s_th_m2 + FX*M1_Ly*M2_Ly*c_th_m3*s_th_m1*s_th_m2 + FX*M1_Ly*M2_Ly*c_th_m4*s_th_m1*s_th_m2 - FX*M1_Ly*M3_Ly*c_th_m2*s_th_m1*s_th_m3 + 2*FX*M1_Ly*M3_Ly*c_th_m3*s_th_m1*s_th_m3 - 2*FX*M2_Ly*M3_Ly*c_th_m1*s_th_m2*s_th_m3 - FX*M1_Ly*M3_Ly*c_th_m4*s_th_m1*s_th_m3 + FX*M1_Ly*M4_Ly*c_th_m2*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m2*s_th_m2*s_th_m3 + FX*M1_Ly*M4_Ly*c_th_m3*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m3*s_th_m2*s_th_m3 + 2*FX*M2_Ly*M4_Ly*c_th_m1*s_th_m2*s_th_m4 - 2*FX*M1_Ly*M4_Ly*c_th_m4*s_th_m1*s_th_m4 - FX*M2_Ly*M4_Ly*c_th_m2*s_th_m2*s_th_m4 - 2*FX*M3_Ly*M4_Ly*c_th_m1*s_th_m3*s_th_m4 - FX*M2_Ly*M4_Ly*c_th_m4*s_th_m2*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m3*s_th_m3*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m4*s_th_m3*s_th_m4 - FT*M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + FT*M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 - FT*M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + FT*M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 + FT*M1_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - FT*M1_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 - FT*M2_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - FT*M2_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 - FT*M1_Lx*M3_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + FT*M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - FT*M1_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + FT*M1_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - FT*M1_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + FT*M1_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 + FT*M2_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + FT*M2_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 + FT*M3_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + FT*M3_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 + FT*M1_Lx*M2_Ly*c_th_m1*sq(c_th_m3)*s_th_m2 + FT*M1_Ly*M2_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 - FT*M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 + FT*M1_Lx*M2_Ly*c_th_m1*sq(c_th_m4)*s_th_m2 - FT*M1_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + FT*M1_Ly*M2_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - FT*M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 - FT*M1_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 + FT*M1_Ly*M3_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 + 2*FT*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m3)*s_th_m2 - FT*M2_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + FT*M2_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m2 - FT*M1_Lx*M3_Ly*c_th_m1*sq(c_th_m4)*s_th_m3 + FT*M1_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 + FT*M1_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - FT*M1_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + FT*M1_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - FT*M1_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 + 2*FT*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*FT*M2_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m2 + FT*M1_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 + FT*M1_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - FT*M1_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + 2*FT*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 + FT*M2_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - FT*M2_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m2 + 2*FT*M2_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*FT*M3_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 + FT*M3_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 + FT*M3_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m3 - 2*FT*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*FT*M3_Lx*M3_Ly*c_th_m3*sq(c_th_m4)*s_th_m3 - 2*FT*M3_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m3 - 2*FT*M3_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - 2*FT*M4_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*FT*M4_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - FT*M1_Ly*M2_Ly*sq(c_th_m3)*s_th_m1*s_th_m2 - FT*M1_Ly*M2_Ly*sq(c_th_m4)*s_th_m1*s_th_m2 + FT*M1_Ly*M3_Ly*sq(c_th_m2)*s_th_m1*s_th_m3 + FT*M1_Ly*M3_Ly*sq(c_th_m4)*s_th_m1*s_th_m3 - FT*M1_Ly*M4_Ly*sq(c_th_m2)*s_th_m1*s_th_m4 - FT*M1_Ly*M4_Ly*sq(c_th_m3)*s_th_m1*s_th_m4 - FT*M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - FT*M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 - 2*FT*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + FT*M1_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - 2*FT*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + FT*M2_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - FT*M2_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 + FT*M1_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 - FT*M2_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + 2*FT*M3_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - FT*M1_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 - FT*M1_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + FT*M2_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + 2*FT*M3_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + FT*M3_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + FT*M3_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + 2*FT*M4_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + 2*FT*M4_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + FT*M1_Ly*M2_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m2 + FT*M1_Ly*M2_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m2 - FT*M1_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m3 + FT*M2_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m3 + FT*M2_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m2*s_th_m3 - FT*M1_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m3 - 2*FT*M2_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m3 - FT*M2_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m4 + FT*M1_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m4 + FT*M1_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m4 - FT*M2_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m2*s_th_m4 + 2*FT*M2_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m3*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m3*s_th_m4 - 2*FT*M3_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m3*s_th_m4);
            Den_p1 = (2*k1*(2*M1_Lx*M4_Lx*sq(c_th_m1)*sq(c_th_m4) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m1)*sq(c_th_m2) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M3_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M4_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M1_Ly)*sq(c_th_m2)*sq(s_th_m1) - sq(M1_Ly)*sq(c_th_m3)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m1)*sq(s_th_m2) - sq(M1_Ly)*sq(c_th_m4)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m3)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m1)*sq(s_th_m3) - sq(M2_Ly)*sq(c_th_m4)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m2)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m1)*sq(s_th_m4) - sq(M3_Ly)*sq(c_th_m4)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m2)*sq(s_th_m4) - sq(M4_Ly)*sq(c_th_m3)*sq(s_th_m4) - 2*M1_Lx*M2_Lx*sq(c_th_m1)*sq(c_th_m2) - 2*M1_Lx*M3_Lx*sq(c_th_m1)*sq(c_th_m3) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m2) + 2*M2_Lx*M3_Lx*sq(c_th_m2)*sq(c_th_m3) - 2*M2_Lx*M4_Lx*sq(c_th_m2)*sq(c_th_m4) - 2*M3_Lx*M4_Lx*sq(c_th_m3)*sq(c_th_m4) + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m3*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m4 + sq(M3_Lx)*c_th_m1*c_th_m2*sq(c_th_m3) + sq(M2_Lx)*sq(c_th_m2)*c_th_m3*c_th_m4 + sq(M3_Lx)*c_th_m1*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m2*sq(c_th_m4) + sq(M3_Lx)*c_th_m2*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m3*sq(c_th_m4) + sq(M4_Lx)*c_th_m2*c_th_m3*sq(c_th_m4) + sq(M1_Ly)*c_th_m2*c_th_m3*sq(s_th_m1) + sq(M1_Ly)*c_th_m2*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m3*sq(s_th_m2) + sq(M1_Ly)*c_th_m3*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m2*sq(s_th_m3) + sq(M2_Ly)*c_th_m3*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m2*sq(s_th_m4) + sq(M3_Ly)*c_th_m2*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m3*sq(s_th_m4) + sq(M4_Ly)*c_th_m2*c_th_m3*sq(s_th_m4) - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M1_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M1_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M2_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M2_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M2_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M3_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M2_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M2_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M2_Lx*M3_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M1_Ly*M2_Lx*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m3)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m3*s_th_m2 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 + 2*M1_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M2_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m4)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m4*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 + M1_Ly*M3_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 - 2*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 - 2*M1_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m3)*s_th_m2 - M2_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M2_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M2_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m2 - M2_Ly*M3_Lx*sq(c_th_m1)*c_th_m3*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m4)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m4*s_th_m3 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 + M1_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M2_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m2 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M1_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + 2*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M2_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 - M2_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 + 2*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 + M2_Lx*M3_Ly*c_th_m2*sq(c_th_m4)*s_th_m3 - M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m4*s_th_m3 - M2_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + 2*M2_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 - M2_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 + M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 - M2_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + M3_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m3 - 2*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M3_Lx*M3_Ly*c_th_m3*sq(c_th_m4)*s_th_m3 + M3_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m3 - 2*M3_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m3 - 2*M4_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 - 2*M3_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - M1_Ly*M2_Ly*sq(c_th_m3)*s_th_m1*s_th_m2 - M1_Ly*M2_Ly*sq(c_th_m4)*s_th_m1*s_th_m2 + M1_Ly*M3_Ly*sq(c_th_m2)*s_th_m1*s_th_m3 - M2_Ly*M3_Ly*sq(c_th_m1)*s_th_m2*s_th_m3 + M1_Ly*M3_Ly*sq(c_th_m4)*s_th_m1*s_th_m3 - M1_Ly*M4_Ly*sq(c_th_m2)*s_th_m1*s_th_m4 - M1_Ly*M4_Ly*sq(c_th_m3)*s_th_m1*s_th_m4 + M2_Ly*M4_Ly*sq(c_th_m1)*s_th_m2*s_th_m4 - M2_Ly*M3_Ly*sq(c_th_m4)*s_th_m2*s_th_m3 + M2_Ly*M4_Ly*sq(c_th_m3)*s_th_m2*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m1)*s_th_m3*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m2)*s_th_m3*s_th_m4 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Ly*M3_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + M2_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M2_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 + M1_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 - 2*M2_Lx*M2_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M2_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M2_Ly*M3_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M1_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 - M2_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 - M2_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + 2*M3_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M3_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 - 2*M1_Ly*M2_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m2 - M1_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m3 + M1_Ly*M2_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m2 + 2*M1_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m2*s_th_m3 - M1_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m3 - 2*M1_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m4 - 2*M2_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m4 + M1_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m2*s_th_m4 - M2_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m4 + 2*M2_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m3*s_th_m4 - M2_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m3*s_th_m4 - 2*M3_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m3*s_th_m4));

            Num_p2 = (M1_Lx*N*sq(c_th_m1)*c_th_m3 - M1_Lx*N*c_th_m1*sq(c_th_m3) - 2*M1_Lx*N*sq(c_th_m1)*c_th_m2 - 2*M2_Lx*N*sq(c_th_m1)*c_th_m2 - M1_Lx*N*c_th_m1*sq(c_th_m4) + M1_Lx*N*sq(c_th_m1)*c_th_m4 - 2*M2_Lx*N*c_th_m2*sq(c_th_m3) - M3_Lx*N*c_th_m1*sq(c_th_m3) + M3_Lx*N*sq(c_th_m1)*c_th_m3 - 2*M2_Lx*N*c_th_m2*sq(c_th_m4) + 2*M3_Lx*N*c_th_m2*sq(c_th_m3) + M4_Lx*N*c_th_m1*sq(c_th_m4) - M4_Lx*N*sq(c_th_m1)*c_th_m4 + M3_Lx*N*c_th_m3*sq(c_th_m4) - M3_Lx*N*sq(c_th_m3)*c_th_m4 - 2*M4_Lx*N*c_th_m2*sq(c_th_m4) + M4_Lx*N*c_th_m3*sq(c_th_m4) - M4_Lx*N*sq(c_th_m3)*c_th_m4 + M1_Ly*N*sq(c_th_m3)*s_th_m1 + 2*M2_Ly*N*sq(c_th_m1)*s_th_m2 + M1_Ly*N*sq(c_th_m4)*s_th_m1 + 2*M2_Ly*N*sq(c_th_m3)*s_th_m2 + M3_Ly*N*sq(c_th_m1)*s_th_m3 + 2*M2_Ly*N*sq(c_th_m4)*s_th_m2 - M4_Ly*N*sq(c_th_m1)*s_th_m4 + M3_Ly*N*sq(c_th_m4)*s_th_m3 - M4_Ly*N*sq(c_th_m3)*s_th_m4 - 2*FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m2 + FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m3 + FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m4 + FX*sq(M3_Lx)*c_th_m1*sq(c_th_m3) - 2*FX*sq(M3_Lx)*c_th_m2*sq(c_th_m3) + FX*sq(M4_Lx)*c_th_m1*sq(c_th_m4) + FX*sq(M3_Lx)*sq(c_th_m3)*c_th_m4 - 2*FX*sq(M4_Lx)*c_th_m2*sq(c_th_m4) + FX*sq(M4_Lx)*c_th_m3*sq(c_th_m4) - 2*FX*sq(M1_Ly)*c_th_m2*sq(s_th_m1) + FX*sq(M1_Ly)*c_th_m3*sq(s_th_m1) + FX*sq(M1_Ly)*c_th_m4*sq(s_th_m1) + FX*sq(M3_Ly)*c_th_m1*sq(s_th_m3) - 2*FX*sq(M3_Ly)*c_th_m2*sq(s_th_m3) + FX*sq(M4_Ly)*c_th_m1*sq(s_th_m4) + FX*sq(M3_Ly)*c_th_m4*sq(s_th_m3) - 2*FX*sq(M4_Ly)*c_th_m2*sq(s_th_m4) + FX*sq(M4_Ly)*c_th_m3*sq(s_th_m4) - FT*sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m3) - FT*sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m4) - FT*sq(M3_Lx)*sq(c_th_m1)*sq(c_th_m3) - FT*sq(M4_Lx)*sq(c_th_m1)*sq(c_th_m4) - FT*sq(M3_Lx)*sq(c_th_m3)*sq(c_th_m4) - FT*sq(M4_Lx)*sq(c_th_m3)*sq(c_th_m4) - FT*sq(M1_Ly)*sq(c_th_m3)*sq(s_th_m1) - FT*sq(M1_Ly)*sq(c_th_m4)*sq(s_th_m1) - FT*sq(M3_Ly)*sq(c_th_m1)*sq(s_th_m3) - FT*sq(M4_Ly)*sq(c_th_m1)*sq(s_th_m4) - FT*sq(M3_Ly)*sq(c_th_m4)*sq(s_th_m3) - FT*sq(M4_Ly)*sq(c_th_m3)*sq(s_th_m4) + M1_Lx*N*c_th_m1*c_th_m2*c_th_m3 + M1_Lx*N*c_th_m1*c_th_m2*c_th_m4 + 2*M2_Lx*N*c_th_m1*c_th_m2*c_th_m3 + 2*M2_Lx*N*c_th_m1*c_th_m2*c_th_m4 - M3_Lx*N*c_th_m1*c_th_m2*c_th_m3 + 2*M2_Lx*N*c_th_m2*c_th_m3*c_th_m4 + M4_Lx*N*c_th_m1*c_th_m2*c_th_m4 - M3_Lx*N*c_th_m2*c_th_m3*c_th_m4 + M4_Lx*N*c_th_m2*c_th_m3*c_th_m4 + 2*M1_Ly*N*c_th_m1*c_th_m2*s_th_m1 - M1_Ly*N*c_th_m1*c_th_m3*s_th_m1 - M1_Ly*N*c_th_m1*c_th_m4*s_th_m1 - M1_Ly*N*c_th_m2*c_th_m3*s_th_m1 - M1_Ly*N*c_th_m2*c_th_m4*s_th_m1 - 2*M2_Ly*N*c_th_m1*c_th_m3*s_th_m2 - 2*M2_Ly*N*c_th_m1*c_th_m4*s_th_m2 - M3_Ly*N*c_th_m1*c_th_m2*s_th_m3 - M3_Ly*N*c_th_m1*c_th_m3*s_th_m3 - 2*M2_Ly*N*c_th_m3*c_th_m4*s_th_m2 + 2*M3_Ly*N*c_th_m2*c_th_m3*s_th_m3 + M4_Ly*N*c_th_m1*c_th_m2*s_th_m4 - M3_Ly*N*c_th_m2*c_th_m4*s_th_m3 - M3_Ly*N*c_th_m3*c_th_m4*s_th_m3 + M4_Ly*N*c_th_m1*c_th_m4*s_th_m4 + M4_Ly*N*c_th_m2*c_th_m3*s_th_m4 - 2*M4_Ly*N*c_th_m2*c_th_m4*s_th_m4 + M4_Ly*N*c_th_m3*c_th_m4*s_th_m4 - 2*FX*M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2 + FX*M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3) + FX*M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3 - FX*M1_Lx*M4_Lx*c_th_m1*sq(c_th_m4) - FX*M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m4 + 2*FX*M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3) - 2*FX*M2_Lx*M4_Lx*c_th_m2*sq(c_th_m4) + FX*M3_Lx*M4_Lx*c_th_m3*sq(c_th_m4) + FX*M3_Lx*M4_Lx*sq(c_th_m3)*c_th_m4 + 2*FX*M1_Lx*M2_Ly*sq(c_th_m1)*s_th_m2 + FX*M1_Lx*M3_Ly*sq(c_th_m1)*s_th_m3 - FX*M1_Ly*M3_Lx*sq(c_th_m3)*s_th_m1 - FX*M1_Lx*M4_Ly*sq(c_th_m1)*s_th_m4 + FX*M1_Ly*M4_Lx*sq(c_th_m4)*s_th_m1 - 2*FX*M2_Ly*M3_Lx*sq(c_th_m3)*s_th_m2 + 2*FX*M2_Ly*M4_Lx*sq(c_th_m4)*s_th_m2 + FX*M3_Lx*M4_Ly*sq(c_th_m3)*s_th_m4 + FX*M3_Ly*M4_Lx*sq(c_th_m4)*s_th_m3 - 2*FT*M1_Lx*M3_Lx*sq(c_th_m1)*sq(c_th_m3) + 2*FT*M1_Lx*M4_Lx*sq(c_th_m1)*sq(c_th_m4) - 2*FT*M3_Lx*M4_Lx*sq(c_th_m3)*sq(c_th_m4) + FT*sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m3 + FT*sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m4 + FT*sq(M3_Lx)*c_th_m1*c_th_m2*sq(c_th_m3) + FT*sq(M4_Lx)*c_th_m1*c_th_m2*sq(c_th_m4) + FT*sq(M3_Lx)*c_th_m2*sq(c_th_m3)*c_th_m4 + FT*sq(M4_Lx)*c_th_m2*c_th_m3*sq(c_th_m4) + FT*sq(M1_Ly)*c_th_m2*c_th_m3*sq(s_th_m1) + FT*sq(M1_Ly)*c_th_m2*c_th_m4*sq(s_th_m1) + FT*sq(M3_Ly)*c_th_m1*c_th_m2*sq(s_th_m3) + FT*sq(M4_Ly)*c_th_m1*c_th_m2*sq(s_th_m4) + FT*sq(M3_Ly)*c_th_m2*c_th_m4*sq(s_th_m3) + FT*sq(M4_Ly)*c_th_m2*c_th_m3*sq(s_th_m4) + FX*M1_Lx*M2_Lx*c_th_m1*c_th_m2*c_th_m3 + FX*M1_Lx*M2_Lx*c_th_m1*c_th_m2*c_th_m4 - 2*FX*M1_Lx*M3_Lx*c_th_m1*c_th_m2*c_th_m3 - FX*M2_Lx*M3_Lx*c_th_m1*c_th_m2*c_th_m3 + 2*FX*M1_Lx*M4_Lx*c_th_m1*c_th_m2*c_th_m4 + FX*M2_Lx*M4_Lx*c_th_m1*c_th_m2*c_th_m4 - FX*M2_Lx*M3_Lx*c_th_m2*c_th_m3*c_th_m4 + FX*M2_Lx*M4_Lx*c_th_m2*c_th_m3*c_th_m4 - 2*FX*M3_Lx*M4_Lx*c_th_m2*c_th_m3*c_th_m4 + 4*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m2*s_th_m1 - 2*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m3*s_th_m1 + 2*FX*M1_Ly*M2_Lx*c_th_m1*c_th_m2*s_th_m1 - 2*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m4*s_th_m1 - FX*M1_Lx*M2_Ly*c_th_m1*c_th_m3*s_th_m2 - FX*M1_Ly*M2_Lx*c_th_m2*c_th_m3*s_th_m1 - FX*M1_Ly*M3_Lx*c_th_m1*c_th_m3*s_th_m1 - FX*M1_Lx*M2_Ly*c_th_m1*c_th_m4*s_th_m2 - 2*FX*M1_Lx*M3_Ly*c_th_m1*c_th_m2*s_th_m3 - FX*M1_Ly*M2_Lx*c_th_m2*c_th_m4*s_th_m1 + 2*FX*M1_Ly*M3_Lx*c_th_m2*c_th_m3*s_th_m1 + FX*M1_Lx*M3_Ly*c_th_m1*c_th_m3*s_th_m3 + FX*M1_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m1 - FX*M2_Lx*M3_Ly*c_th_m1*c_th_m2*s_th_m3 + FX*M2_Ly*M3_Lx*c_th_m1*c_th_m3*s_th_m2 + 2*FX*M1_Lx*M4_Ly*c_th_m1*c_th_m2*s_th_m4 - 2*FX*M1_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m1 + 2*FX*M2_Lx*M3_Ly*c_th_m2*c_th_m3*s_th_m3 + FX*M2_Lx*M4_Ly*c_th_m1*c_th_m2*s_th_m4 - FX*M2_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m2 + 2*FX*M3_Lx*M3_Ly*c_th_m1*c_th_m3*s_th_m3 - FX*M1_Lx*M4_Ly*c_th_m1*c_th_m4*s_th_m4 - FX*M2_Lx*M3_Ly*c_th_m2*c_th_m4*s_th_m3 + FX*M2_Ly*M3_Lx*c_th_m3*c_th_m4*s_th_m2 - 4*FX*M3_Lx*M3_Ly*c_th_m2*c_th_m3*s_th_m3 + FX*M2_Lx*M4_Ly*c_th_m2*c_th_m3*s_th_m4 - FX*M2_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m2 - 2*FX*M2_Lx*M4_Ly*c_th_m2*c_th_m4*s_th_m4 + 2*FX*M3_Lx*M3_Ly*c_th_m3*c_th_m4*s_th_m3 - 2*FX*M3_Lx*M4_Ly*c_th_m2*c_th_m3*s_th_m4 - 2*FX*M3_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m3 + FX*M3_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m3 + 2*FX*M4_Lx*M4_Ly*c_th_m1*c_th_m4*s_th_m4 + FX*M3_Lx*M4_Ly*c_th_m3*c_th_m4*s_th_m4 - 4*FX*M4_Lx*M4_Ly*c_th_m2*c_th_m4*s_th_m4 + 2*FX*M4_Lx*M4_Ly*c_th_m3*c_th_m4*s_th_m4 - 2*FX*M1_Ly*M2_Ly*c_th_m1*s_th_m1*s_th_m2 + FX*M1_Ly*M2_Ly*c_th_m3*s_th_m1*s_th_m2 - FX*M1_Ly*M3_Ly*c_th_m1*s_th_m1*s_th_m3 + FX*M1_Ly*M2_Ly*c_th_m4*s_th_m1*s_th_m2 + 2*FX*M1_Ly*M3_Ly*c_th_m2*s_th_m1*s_th_m3 - FX*M1_Ly*M3_Ly*c_th_m3*s_th_m1*s_th_m3 + FX*M1_Ly*M4_Ly*c_th_m1*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m1*s_th_m2*s_th_m3 - 2*FX*M1_Ly*M4_Ly*c_th_m2*s_th_m1*s_th_m4 - 2*FX*M2_Ly*M3_Ly*c_th_m3*s_th_m2*s_th_m3 - FX*M2_Ly*M4_Ly*c_th_m1*s_th_m2*s_th_m4 + FX*M1_Ly*M4_Ly*c_th_m4*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m4*s_th_m2*s_th_m3 - FX*M2_Ly*M4_Ly*c_th_m3*s_th_m2*s_th_m4 + 2*FX*M2_Ly*M4_Ly*c_th_m4*s_th_m2*s_th_m4 - 2*FX*M3_Ly*M4_Ly*c_th_m2*s_th_m3*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m3*s_th_m3*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m4*s_th_m3*s_th_m4 - FT*M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + FT*M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - FT*M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + FT*M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + FT*M1_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + FT*M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - FT*M2_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + FT*M2_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - FT*M1_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) - FT*M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + FT*M2_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) - FT*M2_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + FT*M2_Lx*M3_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - FT*M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + FT*M2_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - FT*M2_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + FT*M3_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) + FT*M3_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + 2*FT*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*FT*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m4)*s_th_m1 + FT*M1_Lx*M2_Ly*c_th_m1*sq(c_th_m3)*s_th_m2 - FT*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m3*s_th_m2 + FT*M1_Ly*M2_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 + 2*FT*M1_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m1 + FT*M1_Lx*M2_Ly*c_th_m1*sq(c_th_m4)*s_th_m2 - FT*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m4*s_th_m2 + FT*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + FT*M1_Ly*M2_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - FT*M1_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 - 2*FT*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 - 2*FT*M1_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m1 + FT*M2_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + FT*M2_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m2 - FT*M2_Ly*M3_Lx*sq(c_th_m1)*c_th_m3*s_th_m2 - FT*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 + FT*M1_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - FT*M2_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 - FT*M2_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m2 + FT*M2_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m2 - 2*FT*M3_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 + 2*FT*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 + FT*M2_Lx*M3_Ly*c_th_m2*sq(c_th_m4)*s_th_m3 - FT*M2_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + FT*M2_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 - FT*M2_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 - FT*M2_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + FT*M2_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 - 2*FT*M3_Lx*M3_Ly*c_th_m3*sq(c_th_m4)*s_th_m3 + FT*M3_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 + FT*M3_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m3 - 2*FT*M3_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m3 - 2*FT*M4_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 - 2*FT*M3_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - 2*FT*M4_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - FT*M1_Ly*M2_Ly*sq(c_th_m3)*s_th_m1*s_th_m2 - FT*M1_Ly*M2_Ly*sq(c_th_m4)*s_th_m1*s_th_m2 - FT*M2_Ly*M3_Ly*sq(c_th_m1)*s_th_m2*s_th_m3 + FT*M2_Ly*M4_Ly*sq(c_th_m1)*s_th_m2*s_th_m4 - FT*M2_Ly*M3_Ly*sq(c_th_m4)*s_th_m2*s_th_m3 + FT*M2_Ly*M4_Ly*sq(c_th_m3)*s_th_m2*s_th_m4 - 2*FT*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*FT*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - FT*M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - FT*M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - FT*M1_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 + FT*M1_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 + FT*M1_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - FT*M2_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 + 2*FT*M3_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - FT*M1_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 - FT*M2_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + FT*M2_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + 2*FT*M3_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + FT*M2_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + FT*M3_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + 2*FT*M4_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + FT*M3_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + 2*FT*M4_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + FT*M1_Ly*M2_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m2 + FT*M1_Ly*M2_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m2 - FT*M1_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m3 + 2*FT*M1_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m3 - FT*M1_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m3 + FT*M1_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m4 + FT*M2_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m2*s_th_m3 - 2*FT*M1_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m4 + FT*M1_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m4 + FT*M2_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m3 - FT*M2_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m2*s_th_m4 - FT*M2_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m3*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m3*s_th_m4 - 2*FT*M3_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m3*s_th_m4);
            Den_p2 = (2*k1*(2*M1_Lx*M4_Lx*sq(c_th_m1)*sq(c_th_m4) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m1)*sq(c_th_m2) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M3_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M4_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M1_Ly)*sq(c_th_m2)*sq(s_th_m1) - sq(M1_Ly)*sq(c_th_m3)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m1)*sq(s_th_m2) - sq(M1_Ly)*sq(c_th_m4)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m3)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m1)*sq(s_th_m3) - sq(M2_Ly)*sq(c_th_m4)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m2)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m1)*sq(s_th_m4) - sq(M3_Ly)*sq(c_th_m4)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m2)*sq(s_th_m4) - sq(M4_Ly)*sq(c_th_m3)*sq(s_th_m4) - 2*M1_Lx*M2_Lx*sq(c_th_m1)*sq(c_th_m2) - 2*M1_Lx*M3_Lx*sq(c_th_m1)*sq(c_th_m3) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m2) + 2*M2_Lx*M3_Lx*sq(c_th_m2)*sq(c_th_m3) - 2*M2_Lx*M4_Lx*sq(c_th_m2)*sq(c_th_m4) - 2*M3_Lx*M4_Lx*sq(c_th_m3)*sq(c_th_m4) + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m3*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m4 + sq(M3_Lx)*c_th_m1*c_th_m2*sq(c_th_m3) + sq(M2_Lx)*sq(c_th_m2)*c_th_m3*c_th_m4 + sq(M3_Lx)*c_th_m1*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m2*sq(c_th_m4) + sq(M3_Lx)*c_th_m2*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m3*sq(c_th_m4) + sq(M4_Lx)*c_th_m2*c_th_m3*sq(c_th_m4) + sq(M1_Ly)*c_th_m2*c_th_m3*sq(s_th_m1) + sq(M1_Ly)*c_th_m2*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m3*sq(s_th_m2) + sq(M1_Ly)*c_th_m3*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m2*sq(s_th_m3) + sq(M2_Ly)*c_th_m3*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m2*sq(s_th_m4) + sq(M3_Ly)*c_th_m2*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m3*sq(s_th_m4) + sq(M4_Ly)*c_th_m2*c_th_m3*sq(s_th_m4) - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M1_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M1_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M2_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M2_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M2_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M3_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M2_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M2_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M2_Lx*M3_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M1_Ly*M2_Lx*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m3)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m3*s_th_m2 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 + 2*M1_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M2_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m4)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m4*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 + M1_Ly*M3_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 - 2*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 - 2*M1_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m3)*s_th_m2 - M2_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M2_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M2_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m2 - M2_Ly*M3_Lx*sq(c_th_m1)*c_th_m3*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m4)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m4*s_th_m3 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 + M1_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M2_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m2 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M1_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + 2*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M2_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 - M2_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 + 2*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 + M2_Lx*M3_Ly*c_th_m2*sq(c_th_m4)*s_th_m3 - M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m4*s_th_m3 - M2_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + 2*M2_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 - M2_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 + M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 - M2_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + M3_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m3 - 2*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M3_Lx*M3_Ly*c_th_m3*sq(c_th_m4)*s_th_m3 + M3_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m3 - 2*M3_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m3 - 2*M4_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 - 2*M3_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - M1_Ly*M2_Ly*sq(c_th_m3)*s_th_m1*s_th_m2 - M1_Ly*M2_Ly*sq(c_th_m4)*s_th_m1*s_th_m2 + M1_Ly*M3_Ly*sq(c_th_m2)*s_th_m1*s_th_m3 - M2_Ly*M3_Ly*sq(c_th_m1)*s_th_m2*s_th_m3 + M1_Ly*M3_Ly*sq(c_th_m4)*s_th_m1*s_th_m3 - M1_Ly*M4_Ly*sq(c_th_m2)*s_th_m1*s_th_m4 - M1_Ly*M4_Ly*sq(c_th_m3)*s_th_m1*s_th_m4 + M2_Ly*M4_Ly*sq(c_th_m1)*s_th_m2*s_th_m4 - M2_Ly*M3_Ly*sq(c_th_m4)*s_th_m2*s_th_m3 + M2_Ly*M4_Ly*sq(c_th_m3)*s_th_m2*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m1)*s_th_m3*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m2)*s_th_m3*s_th_m4 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Ly*M3_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + M2_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M2_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 + M1_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 - 2*M2_Lx*M2_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M2_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M2_Ly*M3_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M1_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 - M2_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 - M2_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + 2*M3_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M3_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 - 2*M1_Ly*M2_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m2 - M1_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m3 + M1_Ly*M2_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m2 + 2*M1_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m2*s_th_m3 - M1_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m3 - 2*M1_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m4 - 2*M2_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m4 + M1_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m2*s_th_m4 - M2_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m4 + 2*M2_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m3*s_th_m4 - M2_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m3*s_th_m4 - 2*M3_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m3*s_th_m4));

            Num_p3 = (M1_Lx*N*sq(c_th_m1)*c_th_m2 - M1_Lx*N*c_th_m1*sq(c_th_m2) - 2*M1_Lx*N*sq(c_th_m1)*c_th_m3 - M2_Lx*N*c_th_m1*sq(c_th_m2) + M2_Lx*N*sq(c_th_m1)*c_th_m2 - M1_Lx*N*c_th_m1*sq(c_th_m4) + M1_Lx*N*sq(c_th_m1)*c_th_m4 + 2*M2_Lx*N*sq(c_th_m2)*c_th_m3 - 2*M3_Lx*N*sq(c_th_m1)*c_th_m3 + M2_Lx*N*c_th_m2*sq(c_th_m4) - M2_Lx*N*sq(c_th_m2)*c_th_m4 - 2*M3_Lx*N*sq(c_th_m2)*c_th_m3 + M4_Lx*N*c_th_m1*sq(c_th_m4) - M4_Lx*N*sq(c_th_m1)*c_th_m4 - 2*M3_Lx*N*c_th_m3*sq(c_th_m4) + M4_Lx*N*c_th_m2*sq(c_th_m4) - M4_Lx*N*sq(c_th_m2)*c_th_m4 - 2*M4_Lx*N*c_th_m3*sq(c_th_m4) + M1_Ly*N*sq(c_th_m2)*s_th_m1 - M2_Ly*N*sq(c_th_m1)*s_th_m2 + M1_Ly*N*sq(c_th_m4)*s_th_m1 - 2*M3_Ly*N*sq(c_th_m1)*s_th_m3 - M2_Ly*N*sq(c_th_m4)*s_th_m2 - 2*M3_Ly*N*sq(c_th_m2)*s_th_m3 - M4_Ly*N*sq(c_th_m1)*s_th_m4 - 2*M3_Ly*N*sq(c_th_m4)*s_th_m3 - M4_Ly*N*sq(c_th_m2)*s_th_m4 + FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m2 - 2*FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m3 + FX*sq(M2_Lx)*c_th_m1*sq(c_th_m2) + FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m4 - 2*FX*sq(M2_Lx)*sq(c_th_m2)*c_th_m3 + FX*sq(M2_Lx)*sq(c_th_m2)*c_th_m4 + FX*sq(M4_Lx)*c_th_m1*sq(c_th_m4) + FX*sq(M4_Lx)*c_th_m2*sq(c_th_m4) - 2*FX*sq(M4_Lx)*c_th_m3*sq(c_th_m4) + FX*sq(M1_Ly)*c_th_m2*sq(s_th_m1) - 2*FX*sq(M1_Ly)*c_th_m3*sq(s_th_m1) + FX*sq(M2_Ly)*c_th_m1*sq(s_th_m2) + FX*sq(M1_Ly)*c_th_m4*sq(s_th_m1) - 2*FX*sq(M2_Ly)*c_th_m3*sq(s_th_m2) + FX*sq(M2_Ly)*c_th_m4*sq(s_th_m2) + FX*sq(M4_Ly)*c_th_m1*sq(s_th_m4) + FX*sq(M4_Ly)*c_th_m2*sq(s_th_m4) - 2*FX*sq(M4_Ly)*c_th_m3*sq(s_th_m4) - FT*sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m2) - FT*sq(M2_Lx)*sq(c_th_m1)*sq(c_th_m2) - FT*sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m4) - FT*sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m4) - FT*sq(M4_Lx)*sq(c_th_m1)*sq(c_th_m4) - FT*sq(M4_Lx)*sq(c_th_m2)*sq(c_th_m4) - FT*sq(M1_Ly)*sq(c_th_m2)*sq(s_th_m1) - FT*sq(M2_Ly)*sq(c_th_m1)*sq(s_th_m2) - FT*sq(M1_Ly)*sq(c_th_m4)*sq(s_th_m1) - FT*sq(M2_Ly)*sq(c_th_m4)*sq(s_th_m2) - FT*sq(M4_Ly)*sq(c_th_m1)*sq(s_th_m4) - FT*sq(M4_Ly)*sq(c_th_m2)*sq(s_th_m4) + M1_Lx*N*c_th_m1*c_th_m2*c_th_m3 - M2_Lx*N*c_th_m1*c_th_m2*c_th_m3 + M1_Lx*N*c_th_m1*c_th_m3*c_th_m4 + 2*M3_Lx*N*c_th_m1*c_th_m2*c_th_m3 - M2_Lx*N*c_th_m2*c_th_m3*c_th_m4 + 2*M3_Lx*N*c_th_m1*c_th_m3*c_th_m4 + 2*M3_Lx*N*c_th_m2*c_th_m3*c_th_m4 + M4_Lx*N*c_th_m1*c_th_m3*c_th_m4 + M4_Lx*N*c_th_m2*c_th_m3*c_th_m4 - M1_Ly*N*c_th_m1*c_th_m2*s_th_m1 + 2*M1_Ly*N*c_th_m1*c_th_m3*s_th_m1 - M1_Ly*N*c_th_m1*c_th_m4*s_th_m1 - M1_Ly*N*c_th_m2*c_th_m3*s_th_m1 + M2_Ly*N*c_th_m1*c_th_m2*s_th_m2 + M2_Ly*N*c_th_m1*c_th_m3*s_th_m2 - M1_Ly*N*c_th_m3*c_th_m4*s_th_m1 - 2*M2_Ly*N*c_th_m2*c_th_m3*s_th_m2 + 2*M3_Ly*N*c_th_m1*c_th_m2*s_th_m3 + M2_Ly*N*c_th_m2*c_th_m4*s_th_m2 + M2_Ly*N*c_th_m3*c_th_m4*s_th_m2 + 2*M3_Ly*N*c_th_m1*c_th_m4*s_th_m3 + 2*M3_Ly*N*c_th_m2*c_th_m4*s_th_m3 + M4_Ly*N*c_th_m1*c_th_m3*s_th_m4 + M4_Ly*N*c_th_m1*c_th_m4*s_th_m4 + M4_Ly*N*c_th_m2*c_th_m3*s_th_m4 + M4_Ly*N*c_th_m2*c_th_m4*s_th_m4 - 2*M4_Ly*N*c_th_m3*c_th_m4*s_th_m4 + FX*M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2) + FX*M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2 - 2*FX*M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3 - FX*M1_Lx*M4_Lx*c_th_m1*sq(c_th_m4) - FX*M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m4 + 2*FX*M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3 + FX*M2_Lx*M4_Lx*c_th_m2*sq(c_th_m4) + FX*M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m4 - 2*FX*M3_Lx*M4_Lx*c_th_m3*sq(c_th_m4) - FX*M1_Lx*M2_Ly*sq(c_th_m1)*s_th_m2 - FX*M1_Ly*M2_Lx*sq(c_th_m2)*s_th_m1 - 2*FX*M1_Lx*M3_Ly*sq(c_th_m1)*s_th_m3 - FX*M1_Lx*M4_Ly*sq(c_th_m1)*s_th_m4 + FX*M1_Ly*M4_Lx*sq(c_th_m4)*s_th_m1 + 2*FX*M2_Lx*M3_Ly*sq(c_th_m2)*s_th_m3 + FX*M2_Lx*M4_Ly*sq(c_th_m2)*s_th_m4 - FX*M2_Ly*M4_Lx*sq(c_th_m4)*s_th_m2 - 2*FX*M3_Ly*M4_Lx*sq(c_th_m4)*s_th_m3 - 2*FT*M1_Lx*M2_Lx*sq(c_th_m1)*sq(c_th_m2) + 2*FT*M1_Lx*M4_Lx*sq(c_th_m1)*sq(c_th_m4) - 2*FT*M2_Lx*M4_Lx*sq(c_th_m2)*sq(c_th_m4) + FT*sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m3 + FT*sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m3 + FT*sq(M1_Lx)*sq(c_th_m1)*c_th_m3*c_th_m4 + FT*sq(M2_Lx)*sq(c_th_m2)*c_th_m3*c_th_m4 + FT*sq(M4_Lx)*c_th_m1*c_th_m3*sq(c_th_m4) + FT*sq(M4_Lx)*c_th_m2*c_th_m3*sq(c_th_m4) + FT*sq(M1_Ly)*c_th_m2*c_th_m3*sq(s_th_m1) + FT*sq(M2_Ly)*c_th_m1*c_th_m3*sq(s_th_m2) + FT*sq(M1_Ly)*c_th_m3*c_th_m4*sq(s_th_m1) + FT*sq(M2_Ly)*c_th_m3*c_th_m4*sq(s_th_m2) + FT*sq(M4_Ly)*c_th_m1*c_th_m3*sq(s_th_m4) + FT*sq(M4_Ly)*c_th_m2*c_th_m3*sq(s_th_m4) - 2*FX*M1_Lx*M2_Lx*c_th_m1*c_th_m2*c_th_m3 + FX*M1_Lx*M3_Lx*c_th_m1*c_th_m2*c_th_m3 - FX*M2_Lx*M3_Lx*c_th_m1*c_th_m2*c_th_m3 + FX*M1_Lx*M3_Lx*c_th_m1*c_th_m3*c_th_m4 + 2*FX*M1_Lx*M4_Lx*c_th_m1*c_th_m3*c_th_m4 - FX*M2_Lx*M3_Lx*c_th_m2*c_th_m3*c_th_m4 - 2*FX*M2_Lx*M4_Lx*c_th_m2*c_th_m3*c_th_m4 + FX*M3_Lx*M4_Lx*c_th_m1*c_th_m3*c_th_m4 + FX*M3_Lx*M4_Lx*c_th_m2*c_th_m3*c_th_m4 - 2*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m2*s_th_m1 + 4*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m3*s_th_m1 - FX*M1_Ly*M2_Lx*c_th_m1*c_th_m2*s_th_m1 - 2*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m4*s_th_m1 - FX*M1_Lx*M2_Ly*c_th_m1*c_th_m2*s_th_m2 + 2*FX*M1_Lx*M2_Ly*c_th_m1*c_th_m3*s_th_m2 + 2*FX*M1_Ly*M2_Lx*c_th_m2*c_th_m3*s_th_m1 + 2*FX*M1_Ly*M3_Lx*c_th_m1*c_th_m3*s_th_m1 - 2*FX*M2_Lx*M2_Ly*c_th_m1*c_th_m2*s_th_m2 + FX*M1_Lx*M3_Ly*c_th_m1*c_th_m2*s_th_m3 - FX*M1_Ly*M3_Lx*c_th_m2*c_th_m3*s_th_m1 + FX*M1_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m1 + 4*FX*M2_Lx*M2_Ly*c_th_m2*c_th_m3*s_th_m2 - FX*M2_Lx*M3_Ly*c_th_m1*c_th_m2*s_th_m3 + FX*M2_Ly*M3_Lx*c_th_m1*c_th_m3*s_th_m2 + FX*M1_Lx*M3_Ly*c_th_m1*c_th_m4*s_th_m3 - FX*M1_Ly*M3_Lx*c_th_m3*c_th_m4*s_th_m1 - 2*FX*M2_Lx*M2_Ly*c_th_m2*c_th_m4*s_th_m2 - 2*FX*M2_Ly*M3_Lx*c_th_m2*c_th_m3*s_th_m2 + 2*FX*M1_Lx*M4_Ly*c_th_m1*c_th_m3*s_th_m4 - 2*FX*M1_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m1 - FX*M1_Lx*M4_Ly*c_th_m1*c_th_m4*s_th_m4 - FX*M2_Lx*M3_Ly*c_th_m2*c_th_m4*s_th_m3 + FX*M2_Ly*M3_Lx*c_th_m3*c_th_m4*s_th_m2 - FX*M2_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m2 - 2*FX*M2_Lx*M4_Ly*c_th_m2*c_th_m3*s_th_m4 + 2*FX*M2_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m2 + FX*M3_Lx*M4_Ly*c_th_m1*c_th_m3*s_th_m4 + FX*M3_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m3 + FX*M2_Lx*M4_Ly*c_th_m2*c_th_m4*s_th_m4 + FX*M3_Lx*M4_Ly*c_th_m2*c_th_m3*s_th_m4 + FX*M3_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m3 + 2*FX*M4_Lx*M4_Ly*c_th_m1*c_th_m4*s_th_m4 - 2*FX*M3_Lx*M4_Ly*c_th_m3*c_th_m4*s_th_m4 + 2*FX*M4_Lx*M4_Ly*c_th_m2*c_th_m4*s_th_m4 - 4*FX*M4_Lx*M4_Ly*c_th_m3*c_th_m4*s_th_m4 + FX*M1_Ly*M2_Ly*c_th_m1*s_th_m1*s_th_m2 + FX*M1_Ly*M2_Ly*c_th_m2*s_th_m1*s_th_m2 - 2*FX*M1_Ly*M2_Ly*c_th_m3*s_th_m1*s_th_m2 + 2*FX*M1_Ly*M3_Ly*c_th_m1*s_th_m1*s_th_m3 - FX*M1_Ly*M3_Ly*c_th_m2*s_th_m1*s_th_m3 + FX*M1_Ly*M4_Ly*c_th_m1*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m1*s_th_m2*s_th_m3 - FX*M1_Ly*M3_Ly*c_th_m4*s_th_m1*s_th_m3 - 2*FX*M2_Ly*M3_Ly*c_th_m2*s_th_m2*s_th_m3 - 2*FX*M1_Ly*M4_Ly*c_th_m3*s_th_m1*s_th_m4 + FX*M1_Ly*M4_Ly*c_th_m4*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m4*s_th_m2*s_th_m3 - FX*M2_Ly*M4_Ly*c_th_m2*s_th_m2*s_th_m4 + 2*FX*M2_Ly*M4_Ly*c_th_m3*s_th_m2*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m1*s_th_m3*s_th_m4 - FX*M2_Ly*M4_Ly*c_th_m4*s_th_m2*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m2*s_th_m3*s_th_m4 - 2*FX*M3_Ly*M4_Ly*c_th_m4*s_th_m3*s_th_m4 + FT*M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + FT*M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - FT*M1_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + FT*M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - FT*M2_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + FT*M2_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - FT*M1_Lx*M3_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + FT*M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 - FT*M1_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) - FT*M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + FT*M2_Lx*M3_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - FT*M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + FT*M2_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) + FT*M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + FT*M3_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) - FT*M3_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + FT*M3_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - FT*M3_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + 2*FT*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*FT*M1_Ly*M2_Lx*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*FT*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*FT*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 - FT*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m3*s_th_m2 - FT*M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 + 2*FT*M2_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 - FT*M1_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + FT*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + FT*M1_Ly*M3_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 - 2*FT*M1_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m1 - FT*M2_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + FT*M2_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 - FT*M2_Ly*M3_Lx*sq(c_th_m1)*c_th_m3*s_th_m2 - FT*M1_Lx*M3_Ly*c_th_m1*sq(c_th_m4)*s_th_m3 + FT*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m4*s_th_m3 + FT*M1_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 + 2*FT*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m4)*s_th_m2 - FT*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + FT*M1_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 + 2*FT*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 + FT*M2_Lx*M3_Ly*c_th_m2*sq(c_th_m4)*s_th_m3 - FT*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m4*s_th_m3 - FT*M2_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + 2*FT*M2_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m2 + FT*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 - FT*M2_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 - FT*M3_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + FT*M3_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m3 - FT*M3_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m3 - 2*FT*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - FT*M3_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 + FT*M3_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m3 - FT*M3_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m3 - 2*FT*M4_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 - 2*FT*M4_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 + FT*M1_Ly*M3_Ly*sq(c_th_m2)*s_th_m1*s_th_m3 - FT*M2_Ly*M3_Ly*sq(c_th_m1)*s_th_m2*s_th_m3 + FT*M1_Ly*M3_Ly*sq(c_th_m4)*s_th_m1*s_th_m3 - FT*M2_Ly*M3_Ly*sq(c_th_m4)*s_th_m2*s_th_m3 - FT*M3_Ly*M4_Ly*sq(c_th_m1)*s_th_m3*s_th_m4 - FT*M3_Ly*M4_Ly*sq(c_th_m2)*s_th_m3*s_th_m4 - 2*FT*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - FT*M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*FT*M1_Lx*M1_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - FT*M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - FT*M1_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*FT*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - FT*M1_Ly*M3_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 + FT*M2_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + FT*M1_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - 2*FT*M2_Lx*M2_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m2 + FT*M2_Ly*M3_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - FT*M1_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 - FT*M2_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 + FT*M2_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + FT*M3_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + FT*M3_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + 2*FT*M4_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + 2*FT*M4_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 - 2*FT*M1_Ly*M2_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m2 + FT*M1_Ly*M2_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m2 + FT*M1_Ly*M2_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m2 - FT*M1_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m3 - FT*M1_Ly*M3_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m3 + FT*M2_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m3 + FT*M1_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m4 - 2*FT*M1_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m4 + FT*M2_Ly*M3_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m3 + FT*M1_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m4 - FT*M2_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m4 + 2*FT*M2_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m4 - FT*M2_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m3*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m3*s_th_m4);
            Den_p3 = (2*k1*(2*M1_Lx*M4_Lx*sq(c_th_m1)*sq(c_th_m4) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m1)*sq(c_th_m2) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M3_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M4_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M1_Ly)*sq(c_th_m2)*sq(s_th_m1) - sq(M1_Ly)*sq(c_th_m3)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m1)*sq(s_th_m2) - sq(M1_Ly)*sq(c_th_m4)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m3)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m1)*sq(s_th_m3) - sq(M2_Ly)*sq(c_th_m4)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m2)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m1)*sq(s_th_m4) - sq(M3_Ly)*sq(c_th_m4)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m2)*sq(s_th_m4) - sq(M4_Ly)*sq(c_th_m3)*sq(s_th_m4) - 2*M1_Lx*M2_Lx*sq(c_th_m1)*sq(c_th_m2) - 2*M1_Lx*M3_Lx*sq(c_th_m1)*sq(c_th_m3) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m2) + 2*M2_Lx*M3_Lx*sq(c_th_m2)*sq(c_th_m3) - 2*M2_Lx*M4_Lx*sq(c_th_m2)*sq(c_th_m4) - 2*M3_Lx*M4_Lx*sq(c_th_m3)*sq(c_th_m4) + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m3*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m4 + sq(M3_Lx)*c_th_m1*c_th_m2*sq(c_th_m3) + sq(M2_Lx)*sq(c_th_m2)*c_th_m3*c_th_m4 + sq(M3_Lx)*c_th_m1*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m2*sq(c_th_m4) + sq(M3_Lx)*c_th_m2*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m3*sq(c_th_m4) + sq(M4_Lx)*c_th_m2*c_th_m3*sq(c_th_m4) + sq(M1_Ly)*c_th_m2*c_th_m3*sq(s_th_m1) + sq(M1_Ly)*c_th_m2*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m3*sq(s_th_m2) + sq(M1_Ly)*c_th_m3*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m2*sq(s_th_m3) + sq(M2_Ly)*c_th_m3*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m2*sq(s_th_m4) + sq(M3_Ly)*c_th_m2*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m3*sq(s_th_m4) + sq(M4_Ly)*c_th_m2*c_th_m3*sq(s_th_m4) - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M1_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M1_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M2_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M2_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M2_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M3_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M2_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M2_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M2_Lx*M3_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M1_Ly*M2_Lx*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m3)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m3*s_th_m2 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 + 2*M1_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M2_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m4)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m4*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 + M1_Ly*M3_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 - 2*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 - 2*M1_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m3)*s_th_m2 - M2_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M2_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M2_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m2 - M2_Ly*M3_Lx*sq(c_th_m1)*c_th_m3*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m4)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m4*s_th_m3 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 + M1_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M2_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m2 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M1_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + 2*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M2_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 - M2_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 + 2*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 + M2_Lx*M3_Ly*c_th_m2*sq(c_th_m4)*s_th_m3 - M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m4*s_th_m3 - M2_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + 2*M2_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 - M2_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 + M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 - M2_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + M3_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m3 - 2*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M3_Lx*M3_Ly*c_th_m3*sq(c_th_m4)*s_th_m3 + M3_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m3 - 2*M3_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m3 - 2*M4_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 - 2*M3_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - M1_Ly*M2_Ly*sq(c_th_m3)*s_th_m1*s_th_m2 - M1_Ly*M2_Ly*sq(c_th_m4)*s_th_m1*s_th_m2 + M1_Ly*M3_Ly*sq(c_th_m2)*s_th_m1*s_th_m3 - M2_Ly*M3_Ly*sq(c_th_m1)*s_th_m2*s_th_m3 + M1_Ly*M3_Ly*sq(c_th_m4)*s_th_m1*s_th_m3 - M1_Ly*M4_Ly*sq(c_th_m2)*s_th_m1*s_th_m4 - M1_Ly*M4_Ly*sq(c_th_m3)*s_th_m1*s_th_m4 + M2_Ly*M4_Ly*sq(c_th_m1)*s_th_m2*s_th_m4 - M2_Ly*M3_Ly*sq(c_th_m4)*s_th_m2*s_th_m3 + M2_Ly*M4_Ly*sq(c_th_m3)*s_th_m2*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m1)*s_th_m3*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m2)*s_th_m3*s_th_m4 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Ly*M3_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + M2_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M2_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 + M1_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 - 2*M2_Lx*M2_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M2_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M2_Ly*M3_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M1_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 - M2_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 - M2_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + 2*M3_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M3_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 - 2*M1_Ly*M2_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m2 - M1_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m3 + M1_Ly*M2_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m2 + 2*M1_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m2*s_th_m3 - M1_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m3 - 2*M1_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m4 - 2*M2_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m4 + M1_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m2*s_th_m4 - M2_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m4 + 2*M2_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m3*s_th_m4 - M2_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m3*s_th_m4 - 2*M3_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m3*s_th_m4));

            Num_p4 = (M1_Lx*N*sq(c_th_m1)*c_th_m2 - M1_Lx*N*c_th_m1*sq(c_th_m2) - M1_Lx*N*c_th_m1*sq(c_th_m3) + M1_Lx*N*sq(c_th_m1)*c_th_m3 - M2_Lx*N*c_th_m1*sq(c_th_m2) + M2_Lx*N*sq(c_th_m1)*c_th_m2 - 2*M1_Lx*N*sq(c_th_m1)*c_th_m4 + M2_Lx*N*c_th_m2*sq(c_th_m3) - M2_Lx*N*sq(c_th_m2)*c_th_m3 - M3_Lx*N*c_th_m1*sq(c_th_m3) + M3_Lx*N*sq(c_th_m1)*c_th_m3 + 2*M2_Lx*N*sq(c_th_m2)*c_th_m4 - M3_Lx*N*c_th_m2*sq(c_th_m3) + M3_Lx*N*sq(c_th_m2)*c_th_m3 + 2*M4_Lx*N*sq(c_th_m1)*c_th_m4 + 2*M3_Lx*N*sq(c_th_m3)*c_th_m4 + 2*M4_Lx*N*sq(c_th_m2)*c_th_m4 + 2*M4_Lx*N*sq(c_th_m3)*c_th_m4 + M1_Ly*N*sq(c_th_m2)*s_th_m1 + M1_Ly*N*sq(c_th_m3)*s_th_m1 - M2_Ly*N*sq(c_th_m1)*s_th_m2 - M2_Ly*N*sq(c_th_m3)*s_th_m2 + M3_Ly*N*sq(c_th_m1)*s_th_m3 + M3_Ly*N*sq(c_th_m2)*s_th_m3 + 2*M4_Ly*N*sq(c_th_m1)*s_th_m4 + 2*M4_Ly*N*sq(c_th_m2)*s_th_m4 + 2*M4_Ly*N*sq(c_th_m3)*s_th_m4 + FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m2 + FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m3 + FX*sq(M2_Lx)*c_th_m1*sq(c_th_m2) - 2*FX*sq(M1_Lx)*sq(c_th_m1)*c_th_m4 + FX*sq(M2_Lx)*sq(c_th_m2)*c_th_m3 + FX*sq(M3_Lx)*c_th_m1*sq(c_th_m3) - 2*FX*sq(M2_Lx)*sq(c_th_m2)*c_th_m4 + FX*sq(M3_Lx)*c_th_m2*sq(c_th_m3) - 2*FX*sq(M3_Lx)*sq(c_th_m3)*c_th_m4 + FX*sq(M1_Ly)*c_th_m2*sq(s_th_m1) + FX*sq(M1_Ly)*c_th_m3*sq(s_th_m1) + FX*sq(M2_Ly)*c_th_m1*sq(s_th_m2) - 2*FX*sq(M1_Ly)*c_th_m4*sq(s_th_m1) + FX*sq(M2_Ly)*c_th_m3*sq(s_th_m2) + FX*sq(M3_Ly)*c_th_m1*sq(s_th_m3) - 2*FX*sq(M2_Ly)*c_th_m4*sq(s_th_m2) + FX*sq(M3_Ly)*c_th_m2*sq(s_th_m3) - 2*FX*sq(M3_Ly)*c_th_m4*sq(s_th_m3) - FT*sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m2) - FT*sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m3) - FT*sq(M2_Lx)*sq(c_th_m1)*sq(c_th_m2) - FT*sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m3) - FT*sq(M3_Lx)*sq(c_th_m1)*sq(c_th_m3) - FT*sq(M3_Lx)*sq(c_th_m2)*sq(c_th_m3) - FT*sq(M1_Ly)*sq(c_th_m2)*sq(s_th_m1) - FT*sq(M1_Ly)*sq(c_th_m3)*sq(s_th_m1) - FT*sq(M2_Ly)*sq(c_th_m1)*sq(s_th_m2) - FT*sq(M2_Ly)*sq(c_th_m3)*sq(s_th_m2) - FT*sq(M3_Ly)*sq(c_th_m1)*sq(s_th_m3) - FT*sq(M3_Ly)*sq(c_th_m2)*sq(s_th_m3) + M1_Lx*N*c_th_m1*c_th_m2*c_th_m4 + M1_Lx*N*c_th_m1*c_th_m3*c_th_m4 - M2_Lx*N*c_th_m1*c_th_m2*c_th_m4 - M2_Lx*N*c_th_m2*c_th_m3*c_th_m4 - M3_Lx*N*c_th_m1*c_th_m3*c_th_m4 - 2*M4_Lx*N*c_th_m1*c_th_m2*c_th_m4 - M3_Lx*N*c_th_m2*c_th_m3*c_th_m4 - 2*M4_Lx*N*c_th_m1*c_th_m3*c_th_m4 - 2*M4_Lx*N*c_th_m2*c_th_m3*c_th_m4 - M1_Ly*N*c_th_m1*c_th_m2*s_th_m1 - M1_Ly*N*c_th_m1*c_th_m3*s_th_m1 + 2*M1_Ly*N*c_th_m1*c_th_m4*s_th_m1 + M2_Ly*N*c_th_m1*c_th_m2*s_th_m2 - M1_Ly*N*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*N*c_th_m3*c_th_m4*s_th_m1 + M2_Ly*N*c_th_m1*c_th_m4*s_th_m2 + M2_Ly*N*c_th_m2*c_th_m3*s_th_m2 - 2*M2_Ly*N*c_th_m2*c_th_m4*s_th_m2 - M3_Ly*N*c_th_m1*c_th_m3*s_th_m3 + M2_Ly*N*c_th_m3*c_th_m4*s_th_m2 - M3_Ly*N*c_th_m1*c_th_m4*s_th_m3 - M3_Ly*N*c_th_m2*c_th_m3*s_th_m3 - 2*M4_Ly*N*c_th_m1*c_th_m2*s_th_m4 - M3_Ly*N*c_th_m2*c_th_m4*s_th_m3 - 2*M4_Ly*N*c_th_m1*c_th_m3*s_th_m4 + 2*M3_Ly*N*c_th_m3*c_th_m4*s_th_m3 - 2*M4_Ly*N*c_th_m2*c_th_m3*s_th_m4 + FX*M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2) + FX*M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2 + FX*M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3) + FX*M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3 + 2*FX*M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m4 - FX*M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3) - FX*M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3 - 2*FX*M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m4 - 2*FX*M3_Lx*M4_Lx*sq(c_th_m3)*c_th_m4 - FX*M1_Lx*M2_Ly*sq(c_th_m1)*s_th_m2 - FX*M1_Ly*M2_Lx*sq(c_th_m2)*s_th_m1 + FX*M1_Lx*M3_Ly*sq(c_th_m1)*s_th_m3 - FX*M1_Ly*M3_Lx*sq(c_th_m3)*s_th_m1 + 2*FX*M1_Lx*M4_Ly*sq(c_th_m1)*s_th_m4 - FX*M2_Lx*M3_Ly*sq(c_th_m2)*s_th_m3 + FX*M2_Ly*M3_Lx*sq(c_th_m3)*s_th_m2 - 2*FX*M2_Lx*M4_Ly*sq(c_th_m2)*s_th_m4 - 2*FX*M3_Lx*M4_Ly*sq(c_th_m3)*s_th_m4 - 2*FT*M1_Lx*M2_Lx*sq(c_th_m1)*sq(c_th_m2) - 2*FT*M1_Lx*M3_Lx*sq(c_th_m1)*sq(c_th_m3) + 2*FT*M2_Lx*M3_Lx*sq(c_th_m2)*sq(c_th_m3) + FT*sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m4 + FT*sq(M1_Lx)*sq(c_th_m1)*c_th_m3*c_th_m4 + FT*sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m4 + FT*sq(M2_Lx)*sq(c_th_m2)*c_th_m3*c_th_m4 + FT*sq(M3_Lx)*c_th_m1*sq(c_th_m3)*c_th_m4 + FT*sq(M3_Lx)*c_th_m2*sq(c_th_m3)*c_th_m4 + FT*sq(M1_Ly)*c_th_m2*c_th_m4*sq(s_th_m1) + FT*sq(M1_Ly)*c_th_m3*c_th_m4*sq(s_th_m1) + FT*sq(M2_Ly)*c_th_m1*c_th_m4*sq(s_th_m2) + FT*sq(M2_Ly)*c_th_m3*c_th_m4*sq(s_th_m2) + FT*sq(M3_Ly)*c_th_m1*c_th_m4*sq(s_th_m3) + FT*sq(M3_Ly)*c_th_m2*c_th_m4*sq(s_th_m3) - 2*FX*M1_Lx*M2_Lx*c_th_m1*c_th_m2*c_th_m4 - 2*FX*M1_Lx*M3_Lx*c_th_m1*c_th_m3*c_th_m4 - FX*M1_Lx*M4_Lx*c_th_m1*c_th_m2*c_th_m4 - FX*M1_Lx*M4_Lx*c_th_m1*c_th_m3*c_th_m4 + FX*M2_Lx*M4_Lx*c_th_m1*c_th_m2*c_th_m4 + 2*FX*M2_Lx*M3_Lx*c_th_m2*c_th_m3*c_th_m4 + FX*M2_Lx*M4_Lx*c_th_m2*c_th_m3*c_th_m4 + FX*M3_Lx*M4_Lx*c_th_m1*c_th_m3*c_th_m4 + FX*M3_Lx*M4_Lx*c_th_m2*c_th_m3*c_th_m4 - 2*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m2*s_th_m1 - 2*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m3*s_th_m1 - FX*M1_Ly*M2_Lx*c_th_m1*c_th_m2*s_th_m1 + 4*FX*M1_Lx*M1_Ly*c_th_m1*c_th_m4*s_th_m1 - FX*M1_Lx*M2_Ly*c_th_m1*c_th_m2*s_th_m2 - FX*M1_Ly*M3_Lx*c_th_m1*c_th_m3*s_th_m1 - 2*FX*M2_Lx*M2_Ly*c_th_m1*c_th_m2*s_th_m2 + 2*FX*M1_Lx*M2_Ly*c_th_m1*c_th_m4*s_th_m2 + 2*FX*M1_Ly*M2_Lx*c_th_m2*c_th_m4*s_th_m1 + FX*M1_Lx*M3_Ly*c_th_m1*c_th_m3*s_th_m3 - 2*FX*M1_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m1 - 2*FX*M2_Lx*M2_Ly*c_th_m2*c_th_m3*s_th_m2 - 2*FX*M1_Lx*M3_Ly*c_th_m1*c_th_m4*s_th_m3 - FX*M1_Lx*M4_Ly*c_th_m1*c_th_m2*s_th_m4 + 2*FX*M1_Ly*M3_Lx*c_th_m3*c_th_m4*s_th_m1 + FX*M1_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m1 + 4*FX*M2_Lx*M2_Ly*c_th_m2*c_th_m4*s_th_m2 + FX*M2_Ly*M3_Lx*c_th_m2*c_th_m3*s_th_m2 - FX*M1_Lx*M4_Ly*c_th_m1*c_th_m3*s_th_m4 + FX*M1_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m1 - FX*M2_Lx*M3_Ly*c_th_m2*c_th_m3*s_th_m3 + FX*M2_Lx*M4_Ly*c_th_m1*c_th_m2*s_th_m4 - FX*M2_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m2 + 2*FX*M3_Lx*M3_Ly*c_th_m1*c_th_m3*s_th_m3 + 2*FX*M2_Lx*M3_Ly*c_th_m2*c_th_m4*s_th_m3 - 2*FX*M2_Ly*M3_Lx*c_th_m3*c_th_m4*s_th_m2 + 2*FX*M2_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m2 + 2*FX*M3_Lx*M3_Ly*c_th_m2*c_th_m3*s_th_m3 + FX*M2_Lx*M4_Ly*c_th_m2*c_th_m3*s_th_m4 - FX*M2_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m2 + FX*M3_Lx*M4_Ly*c_th_m1*c_th_m3*s_th_m4 + FX*M3_Ly*M4_Lx*c_th_m1*c_th_m4*s_th_m3 - 4*FX*M3_Lx*M3_Ly*c_th_m3*c_th_m4*s_th_m3 + FX*M3_Lx*M4_Ly*c_th_m2*c_th_m3*s_th_m4 + FX*M3_Ly*M4_Lx*c_th_m2*c_th_m4*s_th_m3 - 2*FX*M3_Ly*M4_Lx*c_th_m3*c_th_m4*s_th_m3 + FX*M1_Ly*M2_Ly*c_th_m1*s_th_m1*s_th_m2 + FX*M1_Ly*M2_Ly*c_th_m2*s_th_m1*s_th_m2 - FX*M1_Ly*M3_Ly*c_th_m1*s_th_m1*s_th_m3 - 2*FX*M1_Ly*M2_Ly*c_th_m4*s_th_m1*s_th_m2 - FX*M1_Ly*M3_Ly*c_th_m3*s_th_m1*s_th_m3 - 2*FX*M1_Ly*M4_Ly*c_th_m1*s_th_m1*s_th_m4 + 2*FX*M1_Ly*M3_Ly*c_th_m4*s_th_m1*s_th_m3 + FX*M1_Ly*M4_Ly*c_th_m2*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m2*s_th_m2*s_th_m3 + FX*M1_Ly*M4_Ly*c_th_m3*s_th_m1*s_th_m4 + FX*M2_Ly*M3_Ly*c_th_m3*s_th_m2*s_th_m3 - FX*M2_Ly*M4_Ly*c_th_m1*s_th_m2*s_th_m4 - 2*FX*M2_Ly*M3_Ly*c_th_m4*s_th_m2*s_th_m3 + 2*FX*M2_Ly*M4_Ly*c_th_m2*s_th_m2*s_th_m4 - FX*M2_Ly*M4_Ly*c_th_m3*s_th_m2*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m1*s_th_m3*s_th_m4 + FX*M3_Ly*M4_Ly*c_th_m2*s_th_m3*s_th_m4 - 2*FX*M3_Ly*M4_Ly*c_th_m3*s_th_m3*s_th_m4 + FT*M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 + FT*M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + FT*M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 + FT*M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + FT*M1_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - FT*M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + FT*M1_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - FT*M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + FT*M2_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - FT*M2_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 - FT*M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - FT*M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 - FT*M2_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + FT*M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + FT*M3_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - FT*M3_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + FT*M3_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - FT*M3_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + 2*FT*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*FT*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*FT*M1_Ly*M2_Lx*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*FT*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + 2*FT*M1_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*FT*M2_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 - FT*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m4*s_th_m2 - FT*M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 - 2*FT*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 + 2*FT*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m3)*s_th_m2 + FT*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m4*s_th_m3 + FT*M1_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - FT*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 - FT*M1_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 - FT*M1_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 - 2*FT*M2_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m2 + FT*M1_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - FT*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 - FT*M1_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + 2*FT*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 + FT*M2_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - FT*M2_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 + FT*M2_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m2 - 2*FT*M3_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 - FT*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m4*s_th_m3 + FT*M2_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 - 2*FT*M3_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 - FT*M2_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 + FT*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 + FT*M2_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + FT*M3_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - FT*M3_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 - FT*M3_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m3 + FT*M3_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 - FT*M3_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 - FT*M3_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m3 - FT*M1_Ly*M4_Ly*sq(c_th_m2)*s_th_m1*s_th_m4 - FT*M1_Ly*M4_Ly*sq(c_th_m3)*s_th_m1*s_th_m4 + FT*M2_Ly*M4_Ly*sq(c_th_m1)*s_th_m2*s_th_m4 + FT*M2_Ly*M4_Ly*sq(c_th_m3)*s_th_m2*s_th_m4 - FT*M3_Ly*M4_Ly*sq(c_th_m1)*s_th_m3*s_th_m4 - FT*M3_Ly*M4_Ly*sq(c_th_m2)*s_th_m3*s_th_m4 - 2*FT*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - 2*FT*M1_Lx*M1_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - FT*M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - FT*M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 - FT*M1_Ly*M3_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 + FT*M1_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - 2*FT*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + FT*M1_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 + FT*M1_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 - 2*FT*M2_Lx*M2_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - FT*M2_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + FT*M2_Ly*M3_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - FT*M2_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 - FT*M2_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 + 2*FT*M3_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + 2*FT*M3_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + FT*M3_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + FT*M3_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m3 - 2*FT*M1_Ly*M2_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m2 + FT*M1_Ly*M2_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m2 + FT*M1_Ly*M2_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m2 + 2*FT*M1_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m3 - FT*M1_Ly*M3_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m3 + FT*M1_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m4 + FT*M1_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m4 - FT*M1_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m3 - 2*FT*M2_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m3 - FT*M2_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m4 + FT*M2_Ly*M3_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m3 + FT*M2_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m3 - FT*M2_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m3*s_th_m4 + FT*M3_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m3*s_th_m4);
            Den_p4 = (2*k1*(2*M1_Lx*M4_Lx*sq(c_th_m1)*sq(c_th_m4) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m1)*sq(c_th_m2) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M3_Lx)*sq(c_th_m1)*sq(c_th_m3) - sq(M2_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m2)*sq(c_th_m3) - sq(M4_Lx)*sq(c_th_m1)*sq(c_th_m4) - sq(M3_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m2)*sq(c_th_m4) - sq(M4_Lx)*sq(c_th_m3)*sq(c_th_m4) - sq(M1_Ly)*sq(c_th_m2)*sq(s_th_m1) - sq(M1_Ly)*sq(c_th_m3)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m1)*sq(s_th_m2) - sq(M1_Ly)*sq(c_th_m4)*sq(s_th_m1) - sq(M2_Ly)*sq(c_th_m3)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m1)*sq(s_th_m3) - sq(M2_Ly)*sq(c_th_m4)*sq(s_th_m2) - sq(M3_Ly)*sq(c_th_m2)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m1)*sq(s_th_m4) - sq(M3_Ly)*sq(c_th_m4)*sq(s_th_m3) - sq(M4_Ly)*sq(c_th_m2)*sq(s_th_m4) - sq(M4_Ly)*sq(c_th_m3)*sq(s_th_m4) - 2*M1_Lx*M2_Lx*sq(c_th_m1)*sq(c_th_m2) - 2*M1_Lx*M3_Lx*sq(c_th_m1)*sq(c_th_m3) - sq(M1_Lx)*sq(c_th_m1)*sq(c_th_m2) + 2*M2_Lx*M3_Lx*sq(c_th_m2)*sq(c_th_m3) - 2*M2_Lx*M4_Lx*sq(c_th_m2)*sq(c_th_m4) - 2*M3_Lx*M4_Lx*sq(c_th_m3)*sq(c_th_m4) + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m2*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m3 + sq(M1_Lx)*sq(c_th_m1)*c_th_m3*c_th_m4 + sq(M2_Lx)*c_th_m1*sq(c_th_m2)*c_th_m4 + sq(M3_Lx)*c_th_m1*c_th_m2*sq(c_th_m3) + sq(M2_Lx)*sq(c_th_m2)*c_th_m3*c_th_m4 + sq(M3_Lx)*c_th_m1*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m2*sq(c_th_m4) + sq(M3_Lx)*c_th_m2*sq(c_th_m3)*c_th_m4 + sq(M4_Lx)*c_th_m1*c_th_m3*sq(c_th_m4) + sq(M4_Lx)*c_th_m2*c_th_m3*sq(c_th_m4) + sq(M1_Ly)*c_th_m2*c_th_m3*sq(s_th_m1) + sq(M1_Ly)*c_th_m2*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m3*sq(s_th_m2) + sq(M1_Ly)*c_th_m3*c_th_m4*sq(s_th_m1) + sq(M2_Ly)*c_th_m1*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m2*sq(s_th_m3) + sq(M2_Ly)*c_th_m3*c_th_m4*sq(s_th_m2) + sq(M3_Ly)*c_th_m1*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m2*sq(s_th_m4) + sq(M3_Ly)*c_th_m2*c_th_m4*sq(s_th_m3) + sq(M4_Ly)*c_th_m1*c_th_m3*sq(s_th_m4) + sq(M4_Ly)*c_th_m2*c_th_m3*sq(s_th_m4) - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m3) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M2_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M2_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 + M1_Lx*M2_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M1_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M1_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M2_Lx*M3_Lx*c_th_m1*c_th_m2*sq(c_th_m3) - M2_Lx*M3_Lx*c_th_m1*sq(c_th_m2)*c_th_m3 + M2_Lx*M3_Lx*sq(c_th_m1)*c_th_m2*c_th_m3 - M1_Lx*M3_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M3_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 + M1_Lx*M3_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 - M1_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M1_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M1_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m1*c_th_m2*sq(c_th_m4) + M2_Lx*M4_Lx*c_th_m1*sq(c_th_m2)*c_th_m4 - M2_Lx*M4_Lx*sq(c_th_m1)*c_th_m2*c_th_m4 + M2_Lx*M3_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M3_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M2_Lx*M3_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M2_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) - M2_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 + M2_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m1*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m1*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m1)*c_th_m3*c_th_m4 + M3_Lx*M4_Lx*c_th_m2*c_th_m3*sq(c_th_m4) + M3_Lx*M4_Lx*c_th_m2*sq(c_th_m3)*c_th_m4 - M3_Lx*M4_Lx*sq(c_th_m2)*c_th_m3*c_th_m4 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M1_Ly*M2_Lx*c_th_m1*sq(c_th_m2)*s_th_m1 + 2*M1_Lx*M1_Ly*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m3)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m3*s_th_m2 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 + 2*M1_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m1 + 2*M2_Lx*M2_Ly*sq(c_th_m1)*c_th_m2*s_th_m2 + M1_Lx*M2_Ly*c_th_m1*sq(c_th_m4)*s_th_m2 - M1_Lx*M2_Ly*sq(c_th_m1)*c_th_m4*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M1_Ly*M2_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M2_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m1 + M1_Ly*M3_Lx*sq(c_th_m2)*c_th_m3*s_th_m1 - 2*M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 - 2*M1_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m3)*s_th_m2 - M2_Lx*M3_Ly*c_th_m1*sq(c_th_m2)*s_th_m3 + M2_Lx*M3_Ly*sq(c_th_m1)*c_th_m2*s_th_m3 + M2_Ly*M3_Lx*c_th_m1*sq(c_th_m3)*s_th_m2 - M2_Ly*M3_Lx*sq(c_th_m1)*c_th_m3*s_th_m2 - M1_Lx*M3_Ly*c_th_m1*sq(c_th_m4)*s_th_m3 + M1_Lx*M3_Ly*sq(c_th_m1)*c_th_m4*s_th_m3 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 + M1_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m1 + 2*M2_Lx*M2_Ly*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M2_Ly*M3_Lx*c_th_m2*sq(c_th_m3)*s_th_m2 + M1_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M1_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m1 - M1_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m1 + 2*M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*sq(c_th_m2)*s_th_m4 - M2_Lx*M4_Ly*sq(c_th_m1)*c_th_m2*s_th_m4 - M2_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m1)*c_th_m3*s_th_m3 + 2*M1_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 + M2_Lx*M3_Ly*c_th_m2*sq(c_th_m4)*s_th_m3 - M2_Lx*M3_Ly*sq(c_th_m2)*c_th_m4*s_th_m3 - M2_Ly*M3_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M3_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + 2*M2_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m2 - 2*M3_Lx*M3_Ly*sq(c_th_m2)*c_th_m3*s_th_m3 - M2_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 + M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 - M2_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m2 + M2_Ly*M4_Lx*sq(c_th_m3)*c_th_m4*s_th_m2 + M3_Lx*M4_Ly*c_th_m1*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m1)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m1*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m1)*c_th_m4*s_th_m3 - 2*M2_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M3_Lx*M3_Ly*c_th_m3*sq(c_th_m4)*s_th_m3 + M3_Lx*M4_Ly*c_th_m2*sq(c_th_m3)*s_th_m4 - M3_Lx*M4_Ly*sq(c_th_m2)*c_th_m3*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*sq(c_th_m4)*s_th_m3 - M3_Ly*M4_Lx*sq(c_th_m2)*c_th_m4*s_th_m3 - 2*M3_Ly*M4_Lx*c_th_m3*sq(c_th_m4)*s_th_m3 - 2*M4_Lx*M4_Ly*sq(c_th_m1)*c_th_m4*s_th_m4 - 2*M3_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m2)*c_th_m4*s_th_m4 - 2*M4_Lx*M4_Ly*sq(c_th_m3)*c_th_m4*s_th_m4 - M1_Ly*M2_Ly*sq(c_th_m3)*s_th_m1*s_th_m2 - M1_Ly*M2_Ly*sq(c_th_m4)*s_th_m1*s_th_m2 + M1_Ly*M3_Ly*sq(c_th_m2)*s_th_m1*s_th_m3 - M2_Ly*M3_Ly*sq(c_th_m1)*s_th_m2*s_th_m3 + M1_Ly*M3_Ly*sq(c_th_m4)*s_th_m1*s_th_m3 - M1_Ly*M4_Ly*sq(c_th_m2)*s_th_m1*s_th_m4 - M1_Ly*M4_Ly*sq(c_th_m3)*s_th_m1*s_th_m4 + M2_Ly*M4_Ly*sq(c_th_m1)*s_th_m2*s_th_m4 - M2_Ly*M3_Ly*sq(c_th_m4)*s_th_m2*s_th_m3 + M2_Ly*M4_Ly*sq(c_th_m3)*s_th_m2*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m1)*s_th_m3*s_th_m4 - M3_Ly*M4_Ly*sq(c_th_m2)*s_th_m3*s_th_m4 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - 2*M1_Lx*M1_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 - M1_Ly*M2_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - M1_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m1 - M1_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Ly*M3_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 + M1_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m1 - 2*M2_Lx*M2_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + M2_Ly*M3_Lx*c_th_m1*c_th_m2*c_th_m3*s_th_m2 + M1_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m1 - M2_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 + M1_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 - 2*M2_Lx*M2_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M2_Ly*M4_Lx*c_th_m1*c_th_m2*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m2*c_th_m3*s_th_m3 - M1_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M2_Ly*M3_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 - M1_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 - M2_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 - M2_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m2 + 2*M3_Lx*M3_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + 2*M3_Lx*M3_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + M3_Ly*M4_Lx*c_th_m1*c_th_m3*c_th_m4*s_th_m3 + M2_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + M3_Ly*M4_Lx*c_th_m2*c_th_m3*c_th_m4*s_th_m3 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m2*c_th_m4*s_th_m4 + M3_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m1*c_th_m3*c_th_m4*s_th_m4 + 2*M4_Lx*M4_Ly*c_th_m2*c_th_m3*c_th_m4*s_th_m4 - 2*M1_Ly*M2_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m2 + M1_Ly*M2_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m2 - M1_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m3 + M1_Ly*M2_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m2 + 2*M1_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m3 - M1_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m1*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m1*c_th_m3*s_th_m2*s_th_m3 - M1_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m3 - 2*M1_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m1*s_th_m4 - 2*M2_Ly*M3_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m2*s_th_m2*s_th_m4 + M1_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m3 + M1_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m1*s_th_m4 + M2_Ly*M3_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m3 - M2_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m2*s_th_m4 - M2_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m2*s_th_m4 + 2*M2_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m3*s_th_m3*s_th_m4 - M2_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m2*s_th_m4 + M3_Ly*M4_Ly*c_th_m1*c_th_m4*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m3*s_th_m3*s_th_m4 + M3_Ly*M4_Ly*c_th_m2*c_th_m4*s_th_m3*s_th_m4 - 2*M3_Ly*M4_Ly*c_th_m3*c_th_m4*s_th_m3*s_th_m4));

            PWM1 = Num_p1/Den_p1;
            PWM2 = Num_p2/Den_p2;
            PWM3 = Num_p3/Den_p3;
            PWM4 = Num_p4/Den_p4;

            // Aqui é feita a saturação do PWM para que caso ele saia fora dos valores aceitaveis o método irá tentar convergir através das demais variáveis
            // Saturacao
            PWM1 = Saturacao(PWM1,Pwmmax-Pwmmin,1.0f);
            PWM2 = Saturacao(PWM2,Pwmmax-Pwmmin,1.0f);
            PWM3 = Saturacao(PWM3,Pwmmax-Pwmmin,1.0f);
            PWM4 = Saturacao(PWM4,Pwmmax-Pwmmin,1.0f);

            //         Arco seno do angulo calculado a partir da força e do novo PWM
            ARC_seno[0] = (PWM1*(FY*sq(M2_Ly)*powf(PWM2,3.0) + FY*sq(M3_Ly)*powf(PWM3,3.0) + FY*sq(M4_Ly)*powf(PWM4,3.0) + M1_Ly*N*powf(PWM2,3.0) + M1_Ly*N*powf(PWM3,3.0) + M2_Ly*N*powf(PWM2,3.0) + M1_Ly*N*powf(PWM4,3.0) - M3_Ly*N*powf(PWM3,3.0) + M4_Ly*N*powf(PWM4,3.0) + FY*sq(M2_Ly)*PWM1*sq(PWM2) + FY*sq(M2_Ly)*sq(PWM2)*PWM3 + FY*sq(M3_Ly)*PWM1*sq(PWM3) + FY*sq(M2_Ly)*sq(PWM2)*PWM4 + FY*sq(M3_Ly)*PWM2*sq(PWM3) + FY*sq(M4_Ly)*PWM1*sq(PWM4) + FY*sq(M3_Ly)*sq(PWM3)*PWM4 + FY*sq(M4_Ly)*PWM2*sq(PWM4) + FY*sq(M4_Ly)*PWM3*sq(PWM4) + FY*M1_Ly*M2_Ly*powf(PWM2,3.0) - FY*M1_Ly*M3_Ly*powf(PWM3,3.0) + FY*M1_Ly*M4_Ly*powf(PWM4,3.0) + M1_Ly*N*PWM1*sq(PWM2) + M1_Ly*N*PWM1*sq(PWM3) + M2_Ly*N*PWM1*sq(PWM2) + M1_Ly*N*PWM1*sq(PWM4) + M1_Ly*N*PWM2*sq(PWM3) + M1_Ly*N*sq(PWM2)*PWM3 + M1_Ly*N*PWM2*sq(PWM4) + M1_Ly*N*sq(PWM2)*PWM4 + M2_Ly*N*sq(PWM2)*PWM3 - M3_Ly*N*PWM1*sq(PWM3) + M1_Ly*N*PWM3*sq(PWM4) + M1_Ly*N*sq(PWM3)*PWM4 + M2_Ly*N*sq(PWM2)*PWM4 - M3_Ly*N*PWM2*sq(PWM3) + M4_Ly*N*PWM1*sq(PWM4) - M3_Ly*N*sq(PWM3)*PWM4 + M4_Ly*N*PWM2*sq(PWM4) + M4_Ly*N*PWM3*sq(PWM4) + FY*M1_Ly*M2_Ly*PWM1*sq(PWM2) + FY*M1_Ly*M2_Ly*sq(PWM2)*PWM3 - FY*M1_Ly*M3_Ly*PWM1*sq(PWM3) + FY*M1_Ly*M2_Ly*sq(PWM2)*PWM4 - FY*M1_Ly*M3_Ly*PWM2*sq(PWM3) + FY*M1_Ly*M4_Ly*PWM1*sq(PWM4) - FY*M1_Ly*M3_Ly*sq(PWM3)*PWM4 + FY*M1_Ly*M4_Ly*PWM2*sq(PWM4) + FY*M1_Ly*M4_Ly*PWM3*sq(PWM4) - FT*M1_Ly*M2_Lx*powf(PWM2,3.0)*c_th_m2 - FT*M2_Lx*M2_Ly*powf(PWM2,3.0)*c_th_m2 - FT*M1_Ly*M3_Lx*powf(PWM3,3.0)*c_th_m3 + FT*M3_Lx*M3_Ly*powf(PWM3,3.0)*c_th_m3 + FT*M1_Ly*M4_Lx*powf(PWM4,3.0)*c_th_m4 + FT*M4_Lx*M4_Ly*powf(PWM4,3.0)*c_th_m4 + FT*M1_Lx*M1_Ly*PWM1*sq(PWM2)*c_th_m1 + FT*M1_Lx*M1_Ly*PWM1*sq(PWM3)*c_th_m1 + FT*M1_Lx*M2_Ly*PWM1*sq(PWM2)*c_th_m1 + FT*M1_Lx*M1_Ly*PWM1*sq(PWM4)*c_th_m1 - FT*M1_Lx*M3_Ly*PWM1*sq(PWM3)*c_th_m1 - FT*M1_Ly*M2_Lx*PWM2*sq(PWM3)*c_th_m2 + FT*M1_Lx*M4_Ly*PWM1*sq(PWM4)*c_th_m1 - FT*M1_Ly*M2_Lx*PWM2*sq(PWM4)*c_th_m2 - FT*M1_Ly*M3_Lx*sq(PWM2)*PWM3*c_th_m3 + FT*M2_Lx*M3_Ly*PWM2*sq(PWM3)*c_th_m2 - FT*M2_Ly*M3_Lx*sq(PWM2)*PWM3*c_th_m3 - FT*M1_Ly*M3_Lx*PWM3*sq(PWM4)*c_th_m3 - FT*M2_Lx*M4_Ly*PWM2*sq(PWM4)*c_th_m2 + FT*M1_Ly*M4_Lx*sq(PWM2)*PWM4*c_th_m4 + FT*M1_Ly*M4_Lx*sq(PWM3)*PWM4*c_th_m4 + FT*M2_Ly*M4_Lx*sq(PWM2)*PWM4*c_th_m4 - FT*M3_Lx*M4_Ly*PWM3*sq(PWM4)*c_th_m3 - FT*M3_Ly*M4_Lx*sq(PWM3)*PWM4*c_th_m4))/(k1*(PWM1 + PWM2 + PWM3 + PWM4)*(sq(M1_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM4) + sq(M2_Ly)*sq(PWM2)*sq(PWM3) + sq(M3_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM2)*sq(PWM4) + sq(M3_Ly)*sq(PWM2)*sq(PWM3) + sq(M4_Ly)*sq(PWM1)*sq(PWM4) + sq(M3_Ly)*sq(PWM3)*sq(PWM4) + sq(M4_Ly)*sq(PWM2)*sq(PWM4) + sq(M4_Ly)*sq(PWM3)*sq(PWM4) + 2*M1_Ly*M2_Ly*sq(PWM1)*sq(PWM2) - 2*M1_Ly*M3_Ly*sq(PWM1)*sq(PWM3) + 2*M1_Ly*M4_Ly*sq(PWM1)*sq(PWM4) + 2*M2_Ly*M3_Ly*sq(PWM2)*sq(PWM3) - 2*M2_Ly*M4_Ly*sq(PWM2)*sq(PWM4) + 2*M3_Ly*M4_Ly*sq(PWM3)*sq(PWM4)));
            ARC_seno[1] = (PWM2*(FY*sq(M1_Ly)*powf(PWM1,3.0) + FY*sq(M3_Ly)*powf(PWM3,3.0) + FY*sq(M4_Ly)*powf(PWM4,3.0) - M1_Ly*N*powf(PWM1,3.0) - M2_Ly*N*powf(PWM1,3.0) - M2_Ly*N*powf(PWM3,3.0) - M2_Ly*N*powf(PWM4,3.0) - M3_Ly*N*powf(PWM3,3.0) + M4_Ly*N*powf(PWM4,3.0) + FY*sq(M1_Ly)*sq(PWM1)*PWM2 + FY*sq(M1_Ly)*sq(PWM1)*PWM3 + FY*sq(M1_Ly)*sq(PWM1)*PWM4 + FY*sq(M3_Ly)*PWM1*sq(PWM3) + FY*sq(M3_Ly)*PWM2*sq(PWM3) + FY*sq(M4_Ly)*PWM1*sq(PWM4) + FY*sq(M3_Ly)*sq(PWM3)*PWM4 + FY*sq(M4_Ly)*PWM2*sq(PWM4) + FY*sq(M4_Ly)*PWM3*sq(PWM4) + FY*M1_Ly*M2_Ly*powf(PWM1,3.0) + FY*M2_Ly*M3_Ly*powf(PWM3,3.0) - FY*M2_Ly*M4_Ly*powf(PWM4,3.0) - M1_Ly*N*sq(PWM1)*PWM2 - M1_Ly*N*sq(PWM1)*PWM3 - M2_Ly*N*sq(PWM1)*PWM2 - M1_Ly*N*sq(PWM1)*PWM4 - M2_Ly*N*PWM1*sq(PWM3) - M2_Ly*N*sq(PWM1)*PWM3 - M2_Ly*N*PWM1*sq(PWM4) - M2_Ly*N*PWM2*sq(PWM3) - M2_Ly*N*sq(PWM1)*PWM4 - M3_Ly*N*PWM1*sq(PWM3) - M2_Ly*N*PWM2*sq(PWM4) - M3_Ly*N*PWM2*sq(PWM3) - M2_Ly*N*PWM3*sq(PWM4) - M2_Ly*N*sq(PWM3)*PWM4 + M4_Ly*N*PWM1*sq(PWM4) - M3_Ly*N*sq(PWM3)*PWM4 + M4_Ly*N*PWM2*sq(PWM4) + M4_Ly*N*PWM3*sq(PWM4) + FY*M1_Ly*M2_Ly*sq(PWM1)*PWM2 + FY*M1_Ly*M2_Ly*sq(PWM1)*PWM3 + FY*M1_Ly*M2_Ly*sq(PWM1)*PWM4 + FY*M2_Ly*M3_Ly*PWM1*sq(PWM3) + FY*M2_Ly*M3_Ly*PWM2*sq(PWM3) - FY*M2_Ly*M4_Ly*PWM1*sq(PWM4) + FY*M2_Ly*M3_Ly*sq(PWM3)*PWM4 - FY*M2_Ly*M4_Ly*PWM2*sq(PWM4) - FY*M2_Ly*M4_Ly*PWM3*sq(PWM4) - FT*M1_Lx*M1_Ly*powf(PWM1,3.0)*c_th_m1 - FT*M1_Lx*M2_Ly*powf(PWM1,3.0)*c_th_m1 + FT*M2_Ly*M3_Lx*powf(PWM3,3.0)*c_th_m3 + FT*M3_Lx*M3_Ly*powf(PWM3,3.0)*c_th_m3 - FT*M2_Ly*M4_Lx*powf(PWM4,3.0)*c_th_m4 + FT*M4_Lx*M4_Ly*powf(PWM4,3.0)*c_th_m4 - FT*M1_Lx*M2_Ly*PWM1*sq(PWM3)*c_th_m1 + FT*M1_Ly*M2_Lx*sq(PWM1)*PWM2*c_th_m2 - FT*M1_Lx*M2_Ly*PWM1*sq(PWM4)*c_th_m1 - FT*M1_Lx*M3_Ly*PWM1*sq(PWM3)*c_th_m1 + FT*M2_Lx*M2_Ly*sq(PWM1)*PWM2*c_th_m2 + FT*M1_Lx*M4_Ly*PWM1*sq(PWM4)*c_th_m1 + FT*M1_Ly*M3_Lx*sq(PWM1)*PWM3*c_th_m3 + FT*M2_Lx*M2_Ly*PWM2*sq(PWM3)*c_th_m2 + FT*M2_Lx*M2_Ly*PWM2*sq(PWM4)*c_th_m2 + FT*M2_Lx*M3_Ly*PWM2*sq(PWM3)*c_th_m2 + FT*M2_Ly*M3_Lx*sq(PWM1)*PWM3*c_th_m3 - FT*M1_Ly*M4_Lx*sq(PWM1)*PWM4*c_th_m4 - FT*M2_Lx*M4_Ly*PWM2*sq(PWM4)*c_th_m2 + FT*M2_Ly*M3_Lx*PWM3*sq(PWM4)*c_th_m3 - FT*M2_Ly*M4_Lx*sq(PWM1)*PWM4*c_th_m4 - FT*M2_Ly*M4_Lx*sq(PWM3)*PWM4*c_th_m4 - FT*M3_Lx*M4_Ly*PWM3*sq(PWM4)*c_th_m3 - FT*M3_Ly*M4_Lx*sq(PWM3)*PWM4*c_th_m4))/(k1*(PWM1 + PWM2 + PWM3 + PWM4)*(sq(M1_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM4) + sq(M2_Ly)*sq(PWM2)*sq(PWM3) + sq(M3_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM2)*sq(PWM4) + sq(M3_Ly)*sq(PWM2)*sq(PWM3) + sq(M4_Ly)*sq(PWM1)*sq(PWM4) + sq(M3_Ly)*sq(PWM3)*sq(PWM4) + sq(M4_Ly)*sq(PWM2)*sq(PWM4) + sq(M4_Ly)*sq(PWM3)*sq(PWM4) + 2*M1_Ly*M2_Ly*sq(PWM1)*sq(PWM2) - 2*M1_Ly*M3_Ly*sq(PWM1)*sq(PWM3) + 2*M1_Ly*M4_Ly*sq(PWM1)*sq(PWM4) + 2*M2_Ly*M3_Ly*sq(PWM2)*sq(PWM3) - 2*M2_Ly*M4_Ly*sq(PWM2)*sq(PWM4) + 2*M3_Ly*M4_Ly*sq(PWM3)*sq(PWM4)));
            ARC_seno[2] = (PWM3*(FY*sq(M1_Ly)*powf(PWM1,3.0) + FY*sq(M2_Ly)*powf(PWM2,3.0) + FY*sq(M4_Ly)*powf(PWM4,3.0) - M1_Ly*N*powf(PWM1,3.0) + M2_Ly*N*powf(PWM2,3.0) + M3_Ly*N*powf(PWM1,3.0) + M3_Ly*N*powf(PWM2,3.0) + M3_Ly*N*powf(PWM4,3.0) + M4_Ly*N*powf(PWM4,3.0) + FY*sq(M1_Ly)*sq(PWM1)*PWM2 + FY*sq(M1_Ly)*sq(PWM1)*PWM3 + FY*sq(M2_Ly)*PWM1*sq(PWM2) + FY*sq(M1_Ly)*sq(PWM1)*PWM4 + FY*sq(M2_Ly)*sq(PWM2)*PWM3 + FY*sq(M2_Ly)*sq(PWM2)*PWM4 + FY*sq(M4_Ly)*PWM1*sq(PWM4) + FY*sq(M4_Ly)*PWM2*sq(PWM4) + FY*sq(M4_Ly)*PWM3*sq(PWM4) - FY*M1_Ly*M3_Ly*powf(PWM1,3.0) + FY*M2_Ly*M3_Ly*powf(PWM2,3.0) + FY*M3_Ly*M4_Ly*powf(PWM4,3.0) - M1_Ly*N*sq(PWM1)*PWM2 - M1_Ly*N*sq(PWM1)*PWM3 + M2_Ly*N*PWM1*sq(PWM2) - M1_Ly*N*sq(PWM1)*PWM4 + M3_Ly*N*PWM1*sq(PWM2) + M3_Ly*N*sq(PWM1)*PWM2 + M2_Ly*N*sq(PWM2)*PWM3 + M3_Ly*N*sq(PWM1)*PWM3 + M2_Ly*N*sq(PWM2)*PWM4 + M3_Ly*N*PWM1*sq(PWM4) + M3_Ly*N*sq(PWM1)*PWM4 + M3_Ly*N*sq(PWM2)*PWM3 + M3_Ly*N*PWM2*sq(PWM4) + M3_Ly*N*sq(PWM2)*PWM4 + M4_Ly*N*PWM1*sq(PWM4) + M3_Ly*N*PWM3*sq(PWM4) + M4_Ly*N*PWM2*sq(PWM4) + M4_Ly*N*PWM3*sq(PWM4) - FY*M1_Ly*M3_Ly*sq(PWM1)*PWM2 - FY*M1_Ly*M3_Ly*sq(PWM1)*PWM3 + FY*M2_Ly*M3_Ly*PWM1*sq(PWM2) - FY*M1_Ly*M3_Ly*sq(PWM1)*PWM4 + FY*M2_Ly*M3_Ly*sq(PWM2)*PWM3 + FY*M2_Ly*M3_Ly*sq(PWM2)*PWM4 + FY*M3_Ly*M4_Ly*PWM1*sq(PWM4) + FY*M3_Ly*M4_Ly*PWM2*sq(PWM4) + FY*M3_Ly*M4_Ly*PWM3*sq(PWM4) - FT*M1_Lx*M1_Ly*powf(PWM1,3.0)*c_th_m1 + FT*M1_Lx*M3_Ly*powf(PWM1,3.0)*c_th_m1 - FT*M2_Lx*M2_Ly*powf(PWM2,3.0)*c_th_m2 - FT*M2_Lx*M3_Ly*powf(PWM2,3.0)*c_th_m2 + FT*M3_Ly*M4_Lx*powf(PWM4,3.0)*c_th_m4 + FT*M4_Lx*M4_Ly*powf(PWM4,3.0)*c_th_m4 + FT*M1_Lx*M2_Ly*PWM1*sq(PWM2)*c_th_m1 + FT*M1_Lx*M3_Ly*PWM1*sq(PWM2)*c_th_m1 + FT*M1_Ly*M2_Lx*sq(PWM1)*PWM2*c_th_m2 + FT*M1_Lx*M3_Ly*PWM1*sq(PWM4)*c_th_m1 - FT*M2_Lx*M3_Ly*sq(PWM1)*PWM2*c_th_m2 + FT*M1_Lx*M4_Ly*PWM1*sq(PWM4)*c_th_m1 + FT*M1_Ly*M3_Lx*sq(PWM1)*PWM3*c_th_m3 - FT*M2_Lx*M3_Ly*PWM2*sq(PWM4)*c_th_m2 - FT*M2_Ly*M3_Lx*sq(PWM2)*PWM3*c_th_m3 - FT*M3_Lx*M3_Ly*sq(PWM1)*PWM3*c_th_m3 - FT*M1_Ly*M4_Lx*sq(PWM1)*PWM4*c_th_m4 - FT*M2_Lx*M4_Ly*PWM2*sq(PWM4)*c_th_m2 - FT*M3_Lx*M3_Ly*sq(PWM2)*PWM3*c_th_m3 + FT*M2_Ly*M4_Lx*sq(PWM2)*PWM4*c_th_m4 - FT*M3_Lx*M3_Ly*PWM3*sq(PWM4)*c_th_m3 + FT*M3_Ly*M4_Lx*sq(PWM1)*PWM4*c_th_m4 - FT*M3_Lx*M4_Ly*PWM3*sq(PWM4)*c_th_m3 + FT*M3_Ly*M4_Lx*sq(PWM2)*PWM4*c_th_m4))/(k1*(PWM1 + PWM2 + PWM3 + PWM4)*(sq(M1_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM4) + sq(M2_Ly)*sq(PWM2)*sq(PWM3) + sq(M3_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM2)*sq(PWM4) + sq(M3_Ly)*sq(PWM2)*sq(PWM3) + sq(M4_Ly)*sq(PWM1)*sq(PWM4) + sq(M3_Ly)*sq(PWM3)*sq(PWM4) + sq(M4_Ly)*sq(PWM2)*sq(PWM4) + sq(M4_Ly)*sq(PWM3)*sq(PWM4) + 2*M1_Ly*M2_Ly*sq(PWM1)*sq(PWM2) - 2*M1_Ly*M3_Ly*sq(PWM1)*sq(PWM3) + 2*M1_Ly*M4_Ly*sq(PWM1)*sq(PWM4) + 2*M2_Ly*M3_Ly*sq(PWM2)*sq(PWM3) - 2*M2_Ly*M4_Ly*sq(PWM2)*sq(PWM4) + 2*M3_Ly*M4_Ly*sq(PWM3)*sq(PWM4)));
            ARC_seno[3] = (PWM4*(FY*sq(M1_Ly)*powf(PWM1,3.0) + FY*sq(M2_Ly)*powf(PWM2,3.0) + FY*sq(M3_Ly)*powf(PWM3,3.0) - M1_Ly*N*powf(PWM1,3.0) + M2_Ly*N*powf(PWM2,3.0) - M4_Ly*N*powf(PWM1,3.0) - M3_Ly*N*powf(PWM3,3.0) - M4_Ly*N*powf(PWM2,3.0) - M4_Ly*N*powf(PWM3,3.0) + FY*sq(M1_Ly)*sq(PWM1)*PWM2 + FY*sq(M1_Ly)*sq(PWM1)*PWM3 + FY*sq(M2_Ly)*PWM1*sq(PWM2) + FY*sq(M1_Ly)*sq(PWM1)*PWM4 + FY*sq(M2_Ly)*sq(PWM2)*PWM3 + FY*sq(M3_Ly)*PWM1*sq(PWM3) + FY*sq(M2_Ly)*sq(PWM2)*PWM4 + FY*sq(M3_Ly)*PWM2*sq(PWM3) + FY*sq(M3_Ly)*sq(PWM3)*PWM4 + FY*M1_Ly*M4_Ly*powf(PWM1,3.0) - FY*M2_Ly*M4_Ly*powf(PWM2,3.0) + FY*M3_Ly*M4_Ly*powf(PWM3,3.0) - M1_Ly*N*sq(PWM1)*PWM2 - M1_Ly*N*sq(PWM1)*PWM3 + M2_Ly*N*PWM1*sq(PWM2) - M1_Ly*N*sq(PWM1)*PWM4 + M2_Ly*N*sq(PWM2)*PWM3 - M3_Ly*N*PWM1*sq(PWM3) - M4_Ly*N*PWM1*sq(PWM2) - M4_Ly*N*sq(PWM1)*PWM2 + M2_Ly*N*sq(PWM2)*PWM4 - M3_Ly*N*PWM2*sq(PWM3) - M4_Ly*N*PWM1*sq(PWM3) - M4_Ly*N*sq(PWM1)*PWM3 - M4_Ly*N*PWM2*sq(PWM3) - M4_Ly*N*sq(PWM1)*PWM4 - M4_Ly*N*sq(PWM2)*PWM3 - M3_Ly*N*sq(PWM3)*PWM4 - M4_Ly*N*sq(PWM2)*PWM4 - M4_Ly*N*sq(PWM3)*PWM4 + FY*M1_Ly*M4_Ly*sq(PWM1)*PWM2 + FY*M1_Ly*M4_Ly*sq(PWM1)*PWM3 - FY*M2_Ly*M4_Ly*PWM1*sq(PWM2) + FY*M1_Ly*M4_Ly*sq(PWM1)*PWM4 - FY*M2_Ly*M4_Ly*sq(PWM2)*PWM3 + FY*M3_Ly*M4_Ly*PWM1*sq(PWM3) - FY*M2_Ly*M4_Ly*sq(PWM2)*PWM4 + FY*M3_Ly*M4_Ly*PWM2*sq(PWM3) + FY*M3_Ly*M4_Ly*sq(PWM3)*PWM4 - FT*M1_Lx*M1_Ly*powf(PWM1,3.0)*c_th_m1 - FT*M1_Lx*M4_Ly*powf(PWM1,3.0)*c_th_m1 - FT*M2_Lx*M2_Ly*powf(PWM2,3.0)*c_th_m2 + FT*M2_Lx*M4_Ly*powf(PWM2,3.0)*c_th_m2 + FT*M3_Lx*M3_Ly*powf(PWM3,3.0)*c_th_m3 + FT*M3_Lx*M4_Ly*powf(PWM3,3.0)*c_th_m3 + FT*M1_Lx*M2_Ly*PWM1*sq(PWM2)*c_th_m1 + FT*M1_Ly*M2_Lx*sq(PWM1)*PWM2*c_th_m2 - FT*M1_Lx*M3_Ly*PWM1*sq(PWM3)*c_th_m1 - FT*M1_Lx*M4_Ly*PWM1*sq(PWM2)*c_th_m1 - FT*M1_Lx*M4_Ly*PWM1*sq(PWM3)*c_th_m1 + FT*M1_Ly*M3_Lx*sq(PWM1)*PWM3*c_th_m3 + FT*M2_Lx*M4_Ly*sq(PWM1)*PWM2*c_th_m2 + FT*M2_Lx*M3_Ly*PWM2*sq(PWM3)*c_th_m2 + FT*M2_Lx*M4_Ly*PWM2*sq(PWM3)*c_th_m2 - FT*M2_Ly*M3_Lx*sq(PWM2)*PWM3*c_th_m3 - FT*M1_Ly*M4_Lx*sq(PWM1)*PWM4*c_th_m4 + FT*M3_Lx*M4_Ly*sq(PWM1)*PWM3*c_th_m3 + FT*M3_Lx*M4_Ly*sq(PWM2)*PWM3*c_th_m3 + FT*M2_Ly*M4_Lx*sq(PWM2)*PWM4*c_th_m4 - FT*M4_Lx*M4_Ly*sq(PWM1)*PWM4*c_th_m4 - FT*M3_Ly*M4_Lx*sq(PWM3)*PWM4*c_th_m4 - FT*M4_Lx*M4_Ly*sq(PWM2)*PWM4*c_th_m4 - FT*M4_Lx*M4_Ly*sq(PWM3)*PWM4*c_th_m4))/(k1*(PWM1 + PWM2 + PWM3 + PWM4)*(sq(M1_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM1)*sq(PWM2) + sq(M1_Ly)*sq(PWM1)*sq(PWM4) + sq(M2_Ly)*sq(PWM2)*sq(PWM3) + sq(M3_Ly)*sq(PWM1)*sq(PWM3) + sq(M2_Ly)*sq(PWM2)*sq(PWM4) + sq(M3_Ly)*sq(PWM2)*sq(PWM3) + sq(M4_Ly)*sq(PWM1)*sq(PWM4) + sq(M3_Ly)*sq(PWM3)*sq(PWM4) + sq(M4_Ly)*sq(PWM2)*sq(PWM4) + sq(M4_Ly)*sq(PWM3)*sq(PWM4) + 2*M1_Ly*M2_Ly*sq(PWM1)*sq(PWM2) - 2*M1_Ly*M3_Ly*sq(PWM1)*sq(PWM3) + 2*M1_Ly*M4_Ly*sq(PWM1)*sq(PWM4) + 2*M2_Ly*M3_Ly*sq(PWM2)*sq(PWM3) - 2*M2_Ly*M4_Ly*sq(PWM2)*sq(PWM4) + 2*M3_Ly*M4_Ly*sq(PWM3)*sq(PWM4)));
            ARC_seno[4] = FT/(k1*(PWM1 + PWM2 + PWM3 + PWM4));

            // O maior valor de ArcSeno pode ser 1, caso algum erro matemático faça com que este valor ultrapasse 1,faz-se a saturação

            int T =sizeof(ARC_seno)/sizeof(ARC_seno[0]);

            for(int i = 0; i < T;i++){
                if((float)fabs(ARC_seno[i])>M){
                    M = (float)fabs(ARC_seno[i]);
                }
            }

            // Normaliza o valor que vem em ARC_seno[4] que deve sempre ser 1
            ARC_seno[0] = ARC_seno[0]/M;
            ARC_seno[1] = ARC_seno[1]/M;
            ARC_seno[2] = ARC_seno[2]/M;
            ARC_seno[3] = ARC_seno[3]/M;
            ARC_seno[4] = ARC_seno[4]/M;

            theta_motor1 = asinf(ARC_seno[0]);
            theta_motor2 = asinf(ARC_seno[1]);
            theta_motor3 = asinf(ARC_seno[2]);
            theta_motor4 = asinf(ARC_seno[3]);

            // Aqui é feita a saturação do Angulo para que caso ele saia fora dos valores aceitaveis o método irá tentar convergir através das demais variáveis
            theta_motor1 = Saturacao(theta_motor1,1.0f,-1.0f);
            theta_motor2 = Saturacao(theta_motor2,1.0f,-1.0f);
            theta_motor3 = Saturacao(theta_motor3,1.0f,-1.0f);
            theta_motor4 = Saturacao(theta_motor4,1.0f,-1.0f);
        }
    }
    // aplica um ganho ao valor de pwm para diminuir a potencia
    PWM1 = PWM1 * ganho;
    PWM2 = PWM2 * ganho;
    PWM3 = PWM3 * ganho;
    PWM4 = PWM4 * ganho;

    // Normaliza o valor de PWM encontrado entre 0 e 1 para ativar a saida entre mínima e maxima potência
    PWM1 = PWMtoNorm(PWM1);
    PWM2 = PWMtoNorm(PWM2);
    PWM3 = PWMtoNorm(PWM3);
    PWM4 = PWMtoNorm(PWM4);

    // Conveter o valor de Theta para Graus
    theta_motor1 = theta_motor1 * RAD_TO_DEG;
    theta_motor2 = theta_motor2 * RAD_TO_DEG;
    theta_motor3 = theta_motor3 * RAD_TO_DEG;
    theta_motor4 = theta_motor4 * RAD_TO_DEG;
}

void Copter::get_pilot_desired_force_to_boat_M()
{
    //Essa abordagem considera que o stick direito controla a força em X e Y.
    //a posição do stick determina a intensidade da foça nos eixos onde, o ponto médio é o (0,0).
    //O Yaw é controlado da mesma maneira que em um quadrotor, contudo, o código foi construido de forma empirica.

    float_t med_roll  = (channel_roll->get_radio_min() + ((channel_roll->get_radio_max() - channel_roll->get_radio_min())/2.0));
    float_t med_pitch = (channel_pitch->get_radio_min()+ ((channel_pitch->get_radio_max()- channel_pitch->get_radio_min())/2.0));
    float_t med_yaw   = (channel_yaw->get_radio_min()  + ((channel_yaw->get_radio_max()  - channel_yaw->get_radio_min())/2.0));

    Fy = (channel_roll->get_radio_in()  - med_roll) * Fmax/((channel_roll->get_radio_max() -channel_roll->get_radio_min())/2);
    Fx = (channel_pitch->get_radio_in() - med_pitch)* Fmax/((channel_pitch->get_radio_max()-channel_pitch->get_radio_min())/2);
    tN = (channel_yaw->get_radio_in()   - med_yaw)  * Nmax/((channel_yaw->get_radio_max()  -channel_yaw->get_radio_min())/2);

    ganho  = (float)fabs(((canalservo->get_radio_in()*1.0 - canalservo->get_radio_min())/(canalservo->get_radio_max() - canalservo->get_radio_min())));

////         Controlar a sensibilidade dos comandos através do canal 5
//        Fx = Fx * ganho;
//        Fy = Fy * ganho;
//        tN = tN * ganho;

    alocation_matrix(Ft,Fx,Fy,tN,theta_m1,theta_m2,theta_m3,theta_m4,Pwm1,Pwm2,Pwm3,Pwm4);

}

// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Copter::get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    float yaw_request;

    // calculate yaw rate request
    if (g2.acro_y_expo <= 0) {
        yaw_request = stick_angle * g.acro_yaw_p;
    } else {
        // expo variables
        float y_in, y_in3, y_out;

        // range check expo
        if (g2.acro_y_expo > 1.0f || g2.acro_y_expo < 0.5f) {
            g2.acro_y_expo = 1.0f;
        }

        // yaw expo
        y_in = float(stick_angle)/ROLL_PITCH_YAW_INPUT_MAX;
        y_in3 = y_in*y_in*y_in;
        y_out = (g2.acro_y_expo * y_in3) + ((1.0f - g2.acro_y_expo) * y_in);
        yaw_request = ROLL_PITCH_YAW_INPUT_MAX * y_out * g.acro_yaw_p;
    }
    // convert pilot input to the desired yaw rate
    yaw_request = 0* yaw_request;  //mathaus - Revomendo a dinamica de guinada
    return yaw_request;
}

/*************************************************************
 * yaw controllers
 *************************************************************/

// get_roi_yaw - returns heading towards location held in roi_WP
// should be called at 100hz
float Copter::get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // used to reduce update rate to 100hz

    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        roi_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), roi_WP);
    }

    return yaw_look_at_WP_bearing;
}

float Copter::get_look_ahead_yaw()
{
    const Vector3f& vel = inertial_nav.get_velocity();
    float speed = norm(vel.x,vel.y);
    // Commanded Yaw to automatically look ahead.
    if (position_ok() && (speed > YAW_LOOK_AHEAD_MIN_SPEED)) {
        yaw_look_ahead_bearing = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void Copter::update_throttle_hover()
{
#if FRAME_CONFIG != HELI_FRAME
    // if not armed or landed exit
    if (!motors->armed() || ap.land_complete) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (mode_has_manual_throttle(control_mode) || (control_mode == DRIFT)) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_desired_velocity().z)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // calc average throttle if we are in a level hover
    if (throttle > 0.0f && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
    }
#endif
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
void Copter::set_throttle_takeoff()
{
    return; // Mathaus - Barco obviamente nao deve ter controle de takeoff
    // tell position controller to reset alt target and reset I terms
    // pos_control->init_takeoff();
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1
// returns throttle output 0 to 1
float Copter::get_pilot_desired_throttle(int16_t throttle_control, float thr_mid)
{
    if (thr_mid <= 0.0f) {
        thr_mid = motors->get_throttle_hover();
    }

    int16_t mid_stick = channel_throttle->get_control_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 500;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        // below the deadband
        throttle_in = ((float)throttle_control)*0.5f/(float)mid_stick;
    }else if(throttle_control > mid_stick) {
        // above the deadband
        throttle_in = 0.5f + ((float)(throttle_control-mid_stick)) * 0.5f / (float)(1000-mid_stick);
    }else{
        // must be in the deadband
        throttle_in = 0.5f;
    }

    float expo = constrain_float(-(thr_mid-0.5)/0.375, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if( failsafe.radio ) {
        return 0.0f;
    }

    float desired_rate = 0.0f;
    float mid_stick = channel_throttle->get_control_mid();
    float deadband_top = mid_stick + g.throttle_deadzone;
    float deadband_bottom = mid_stick - g.throttle_deadzone;

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = g.pilot_velocity_z_max * (throttle_control-deadband_bottom) / deadband_bottom;
    }else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_velocity_z_max * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    }else{
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// get_surface_tracking_climb_rate - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
float Copter::get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt)
{
#if RANGEFINDER_ENABLED == ENABLED
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;
    float current_alt = inertial_nav.get_altitude();

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if (now - last_call_ms > RANGEFINDER_TIMEOUT_MS) {
        target_rangefinder_alt = rangefinder_state.alt_cm + current_alt_target - current_alt;
    }
    last_call_ms = now;

    // adjust rangefinder target alt if motors have not hit their limits
    if ((target_rate<0 && !motors->limit.throttle_lower) || (target_rate>0 && !motors->limit.throttle_upper)) {
        target_rangefinder_alt += target_rate * dt;
    }

    /*
      handle rangefinder glitches. When we get a rangefinder reading
      more than RANGEFINDER_GLITCH_ALT_CM different from the current
      rangefinder reading then we consider it a glitch and reject
      until we get RANGEFINDER_GLITCH_NUM_SAMPLES samples in a
      row. When that happens we reset the target altitude to the new
      reading
     */
    int32_t glitch_cm = rangefinder_state.alt_cm - target_rangefinder_alt;
    if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
        rangefinder_state.glitch_count = MAX(rangefinder_state.glitch_count+1,1);
    } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
        rangefinder_state.glitch_count = MIN(rangefinder_state.glitch_count-1,-1);
    } else {
        rangefinder_state.glitch_count = 0;
    }
    if (abs(rangefinder_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // shift to the new rangefinder reading
        target_rangefinder_alt = rangefinder_state.alt_cm;
        rangefinder_state.glitch_count = 0;
    }
    if (rangefinder_state.glitch_count != 0) {
        // we are currently glitching, just use the target rate
        return target_rate;
    }

    // calc desired velocity correction from target rangefinder alt vs actual rangefinder alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_rangefinder_alt - rangefinder_state.alt_cm) - (current_alt_target - current_alt);
    velocity_correction = distance_error * g.rangefinder_gain;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct rangefinder alt error
    return (target_rate + velocity_correction);
#else
    return (float)target_rate;
#endif
}

// get target climb rate reduced to avoid obstacles and altitude fence
float Copter::get_avoidance_adjusted_climbrate(float target_rate)
{
#if AC_AVOID_ENABLED == ENABLED
    avoid.adjust_velocity_z(pos_control->get_pos_z_kP(), pos_control->get_accel_z(), target_rate);
    return target_rate;
#else
    return target_rate;
#endif
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_accel_z.set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}
