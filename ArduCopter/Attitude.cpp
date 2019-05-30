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
    roll_out  = roll_out *0;
    pitch_out = pitch_out*0;

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

float Copter::PWMtoNorm(float pwm)
{
    /// Entra um valor de PWM e sai de 0 a 1

    float V;
    V = float(pwm - Pwmmin)/float(Pwmmax-Pwmmin);

    return constrain_float(V,0.0f,1.0f);
}

float Copter::NormtoPWM(float val)
{
    /// Entra um valor de 0 a 1 e sai um PWM
    return val*(Pwmmax-Pwmmin) + Pwmmin;

}

void Copter::alocation_matrix(float &FX,float &FY,float &TN,float &Theta1,float &Theta2,float &Theta3,float &Theta4,float &PWM1,float &PWM2,float &PWM3,float &PWM4)
{
    /// TRABALHA COM RADIANOS
    /// Fx = força no eixo X - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// Fy = força no eixo y - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// N  = tork de guinada - A principio, seu valor deve variar de 0 a 1 deve-se fazer alterações para -1 a 1
    /// Função para alocar as forças do barco a partir da metodologia desenvolvida na  Tese de Doutorado do Murillo,
    /// a Matriz A = [Fx;Fy;N] foi subdividida em A1 = [Ft;Fx,N] e A2 = [Ft;Fy;N]
    /// A linha Ft foi adicionada a tecnica para evitar linhas de 0 na primeira matriz, fazendo com que a solução nao seja possivel///

    //    int i = 0 ;
    //    int iteracao = 1;

    FX = constrain_float(FX,-Fmax,Fmax);
    FY = constrain_float(FY,-Fmax,Fmax);
    TN = constrain_float(TN,-Nmax,Nmax);

    //Calculo da Força total
    FT = sqrt(sq(FX)+ sq(FY) + sq(TN/L));

    FT = constrain_float(FT,-Fmax,Fmax);

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

    if(FT<0.10*Fmax)
    {
        // Se as forças são muito pequenas ( proximas a zero) nao executa a matriz de alocação
        // Envia todos os angulos  nulos
        Theta1 = 0.0;
        Theta2 = 0.0;
        Theta3 = 0.0;
        Theta4 = 0.0;

        //Envia todos os PWMs muito pequenos (Nulos-Na prática) Os valores aqui, não estão normalizados entre 0 e 1
        PWM1 = NormtoPWM(0.0);
        PWM2 = NormtoPWM(0.0);
        PWM3 = NormtoPWM(0.0);
        PWM4 = NormtoPWM(0.0);

    }else
    {
        // ==================================================================== Arco seno do angulo calculado a partir da força e do novo PWM ====================================================================

        Theta1 = atan2f(((PWM1*(PWM1 + PWM2 + PWM3 + PWM4)*(FY*sq(Ly)*powf(PWM2,4) + FY*sq(Ly)*powf(PWM4,4) + Ly*powf(PWM2,4)*TN + Ly*powf(PWM4,4)*TN + Ly*sq(PWM1)*sq(PWM2)*TN + Ly*sq(PWM1)*sq(PWM4)*TN + Ly*sq(PWM2)*sq(PWM3)*TN + 2*Ly*sq(PWM2)*sq(PWM4)*TN + Ly*sq(PWM3)*sq(PWM4)*TN + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM2) + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FY*sq(Lx)*sq(PWM2)*sq(PWM4) + 2*FY*sq(Lx)*sq(PWM3)*sq(PWM4) + FY*sq(Ly)*sq(PWM1)*sq(PWM2) + FY*sq(Ly)*sq(PWM1)*sq(PWM4) + FY*sq(Ly)*sq(PWM2)*sq(PWM3) + 2*FY*sq(Ly)*sq(PWM2)*sq(PWM4) + FY*sq(Ly)*sq(PWM3)*sq(PWM4) - FX*Lx*Ly*powf(PWM2,4) + FX*Lx*Ly*powf(PWM4,4) + FX*Lx*Ly*sq(PWM1)*sq(PWM2) + FX*Lx*Ly*sq(PWM1)*sq(PWM4) - FX*Lx*Ly*sq(PWM2)*sq(PWM3) - FX*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))),((PWM1*(PWM1 + PWM2 + PWM3 + PWM4)*(FX*sq(Lx)*powf(PWM2,4) + FX*sq(Lx)*powf(PWM3,4) - Lx*powf(PWM2,4)*TN - Lx*powf(PWM3,4)*TN - Lx*sq(PWM1)*sq(PWM2)*TN - Lx*sq(PWM1)*sq(PWM3)*TN - 2*Lx*sq(PWM2)*sq(PWM3)*TN - Lx*sq(PWM2)*sq(PWM4)*TN - Lx*sq(PWM3)*sq(PWM4)*TN + FX*sq(Lx)*sq(PWM1)*sq(PWM2) + FX*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FX*sq(Lx)*sq(PWM2)*sq(PWM3) + FX*sq(Lx)*sq(PWM2)*sq(PWM4) + FX*sq(Lx)*sq(PWM3)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM2) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM2)*sq(PWM3) + 2*FX*sq(Ly)*sq(PWM3)*sq(PWM4) - FY*Lx*Ly*powf(PWM2,4) + FY*Lx*Ly*powf(PWM3,4) + FY*Lx*Ly*sq(PWM1)*sq(PWM2) + FY*Lx*Ly*sq(PWM1)*sq(PWM3) - FY*Lx*Ly*sq(PWM2)*sq(PWM4) - FY*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))));
        Theta2 = atan2f(((PWM2*(PWM1 + PWM2 + PWM3 + PWM4)*(FY*sq(Ly)*powf(PWM1,4) + FY*sq(Ly)*powf(PWM3,4) - Ly*powf(PWM1,4)*TN - Ly*powf(PWM3,4)*TN - Ly*sq(PWM1)*sq(PWM2)*TN - 2*Ly*sq(PWM1)*sq(PWM3)*TN - Ly*sq(PWM1)*sq(PWM4)*TN - Ly*sq(PWM2)*sq(PWM3)*TN - Ly*sq(PWM3)*sq(PWM4)*TN + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM2) + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FY*sq(Lx)*sq(PWM2)*sq(PWM4) + 2*FY*sq(Lx)*sq(PWM3)*sq(PWM4) + FY*sq(Ly)*sq(PWM1)*sq(PWM2) + 2*FY*sq(Ly)*sq(PWM1)*sq(PWM3) + FY*sq(Ly)*sq(PWM1)*sq(PWM4) + FY*sq(Ly)*sq(PWM2)*sq(PWM3) + FY*sq(Ly)*sq(PWM3)*sq(PWM4) - FX*Lx*Ly*powf(PWM1,4) + FX*Lx*Ly*powf(PWM3,4) + FX*Lx*Ly*sq(PWM1)*sq(PWM2) - FX*Lx*Ly*sq(PWM1)*sq(PWM4) + FX*Lx*Ly*sq(PWM2)*sq(PWM3) - FX*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))),((PWM2*(PWM1 + PWM2 + PWM3 + PWM4)*(FX*sq(Lx)*powf(PWM1,4) + FX*sq(Lx)*powf(PWM4,4) + Lx*powf(PWM1,4)*TN + Lx*powf(PWM4,4)*TN + Lx*sq(PWM1)*sq(PWM2)*TN + Lx*sq(PWM1)*sq(PWM3)*TN + 2*Lx*sq(PWM1)*sq(PWM4)*TN + Lx*sq(PWM2)*sq(PWM4)*TN + Lx*sq(PWM3)*sq(PWM4)*TN + FX*sq(Lx)*sq(PWM1)*sq(PWM2) + FX*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FX*sq(Lx)*sq(PWM1)*sq(PWM4) + FX*sq(Lx)*sq(PWM2)*sq(PWM4) + FX*sq(Lx)*sq(PWM3)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM2) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM2)*sq(PWM3) + 2*FX*sq(Ly)*sq(PWM3)*sq(PWM4) - FY*Lx*Ly*powf(PWM1,4) + FY*Lx*Ly*powf(PWM4,4) + FY*Lx*Ly*sq(PWM1)*sq(PWM2) - FY*Lx*Ly*sq(PWM1)*sq(PWM3) + FY*Lx*Ly*sq(PWM2)*sq(PWM4) - FY*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))));
        Theta3 = atan2f(((PWM3*(PWM1 + PWM2 + PWM3 + PWM4)*(FY*sq(Ly)*powf(PWM2,4) + FY*sq(Ly)*powf(PWM4,4) + Ly*powf(PWM2,4)*TN + Ly*powf(PWM4,4)*TN + Ly*sq(PWM1)*sq(PWM2)*TN + Ly*sq(PWM1)*sq(PWM4)*TN + Ly*sq(PWM2)*sq(PWM3)*TN + 2*Ly*sq(PWM2)*sq(PWM4)*TN + Ly*sq(PWM3)*sq(PWM4)*TN + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM2) + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FY*sq(Lx)*sq(PWM2)*sq(PWM4) + 2*FY*sq(Lx)*sq(PWM3)*sq(PWM4) + FY*sq(Ly)*sq(PWM1)*sq(PWM2) + FY*sq(Ly)*sq(PWM1)*sq(PWM4) + FY*sq(Ly)*sq(PWM2)*sq(PWM3) + 2*FY*sq(Ly)*sq(PWM2)*sq(PWM4) + FY*sq(Ly)*sq(PWM3)*sq(PWM4) - FX*Lx*Ly*powf(PWM2,4) + FX*Lx*Ly*powf(PWM4,4) + FX*Lx*Ly*sq(PWM1)*sq(PWM2) + FX*Lx*Ly*sq(PWM1)*sq(PWM4) - FX*Lx*Ly*sq(PWM2)*sq(PWM3) - FX*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))),((PWM3*(PWM1 + PWM2 + PWM3 + PWM4)*(FX*sq(Lx)*powf(PWM1,4) + FX*sq(Lx)*powf(PWM4,4) + Lx*powf(PWM1,4)*TN + Lx*powf(PWM4,4)*TN + Lx*sq(PWM1)*sq(PWM2)*TN + Lx*sq(PWM1)*sq(PWM3)*TN + 2*Lx*sq(PWM1)*sq(PWM4)*TN + Lx*sq(PWM2)*sq(PWM4)*TN + Lx*sq(PWM3)*sq(PWM4)*TN + FX*sq(Lx)*sq(PWM1)*sq(PWM2) + FX*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FX*sq(Lx)*sq(PWM1)*sq(PWM4) + FX*sq(Lx)*sq(PWM2)*sq(PWM4) + FX*sq(Lx)*sq(PWM3)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM2) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM2)*sq(PWM3) + 2*FX*sq(Ly)*sq(PWM3)*sq(PWM4) - FY*Lx*Ly*powf(PWM1,4) + FY*Lx*Ly*powf(PWM4,4) + FY*Lx*Ly*sq(PWM1)*sq(PWM2) - FY*Lx*Ly*sq(PWM1)*sq(PWM3) + FY*Lx*Ly*sq(PWM2)*sq(PWM4) - FY*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))));
        Theta4 = atan2f(((PWM4*(PWM1 + PWM2 + PWM3 + PWM4)*(FY*sq(Ly)*powf(PWM1,4) + FY*sq(Ly)*powf(PWM3,4) - Ly*powf(PWM1,4)*TN - Ly*powf(PWM3,4)*TN - Ly*sq(PWM1)*sq(PWM2)*TN - 2*Ly*sq(PWM1)*sq(PWM3)*TN - Ly*sq(PWM1)*sq(PWM4)*TN - Ly*sq(PWM2)*sq(PWM3)*TN - Ly*sq(PWM3)*sq(PWM4)*TN + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM2) + 2*FY*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FY*sq(Lx)*sq(PWM2)*sq(PWM4) + 2*FY*sq(Lx)*sq(PWM3)*sq(PWM4) + FY*sq(Ly)*sq(PWM1)*sq(PWM2) + 2*FY*sq(Ly)*sq(PWM1)*sq(PWM3) + FY*sq(Ly)*sq(PWM1)*sq(PWM4) + FY*sq(Ly)*sq(PWM2)*sq(PWM3) + FY*sq(Ly)*sq(PWM3)*sq(PWM4) - FX*Lx*Ly*powf(PWM1,4) + FX*Lx*Ly*powf(PWM3,4) + FX*Lx*Ly*sq(PWM1)*sq(PWM2) - FX*Lx*Ly*sq(PWM1)*sq(PWM4) + FX*Lx*Ly*sq(PWM2)*sq(PWM3) - FX*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))),((PWM4*(PWM1 + PWM2 + PWM3 + PWM4)*(FX*sq(Lx)*powf(PWM2,4) + FX*sq(Lx)*powf(PWM3,4) - Lx*powf(PWM2,4)*TN - Lx*powf(PWM3,4)*TN - Lx*sq(PWM1)*sq(PWM2)*TN - Lx*sq(PWM1)*sq(PWM3)*TN - 2*Lx*sq(PWM2)*sq(PWM3)*TN - Lx*sq(PWM2)*sq(PWM4)*TN - Lx*sq(PWM3)*sq(PWM4)*TN + FX*sq(Lx)*sq(PWM1)*sq(PWM2) + FX*sq(Lx)*sq(PWM1)*sq(PWM3) + 2*FX*sq(Lx)*sq(PWM2)*sq(PWM3) + FX*sq(Lx)*sq(PWM2)*sq(PWM4) + FX*sq(Lx)*sq(PWM3)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM2) + 2*FX*sq(Ly)*sq(PWM1)*sq(PWM4) + 2*FX*sq(Ly)*sq(PWM2)*sq(PWM3) + 2*FX*sq(Ly)*sq(PWM3)*sq(PWM4) - FY*Lx*Ly*powf(PWM2,4) + FY*Lx*Ly*powf(PWM3,4) + FY*Lx*Ly*sq(PWM1)*sq(PWM2) + FY*Lx*Ly*sq(PWM1)*sq(PWM3) - FY*Lx*Ly*sq(PWM2)*sq(PWM4) - FY*Lx*Ly*sq(PWM3)*sq(PWM4)))/(FT*(sq(PWM1) + sq(PWM2) + sq(PWM3) + sq(PWM4))*(sq(Lx)*sq(PWM1)*sq(PWM2) + sq(Lx)*sq(PWM1)*sq(PWM3) + sq(Lx)*sq(PWM2)*sq(PWM4) + sq(Lx)*sq(PWM3)*sq(PWM4) + sq(Ly)*sq(PWM1)*sq(PWM2) + sq(Ly)*sq(PWM1)*sq(PWM4) + sq(Ly)*sq(PWM2)*sq(PWM3) + sq(Ly)*sq(PWM3)*sq(PWM4)))));

        // Saturação
        Theta1 = constrain_float(Theta1,-M_PI,M_PI);
        Theta2 = constrain_float(Theta2,-M_PI,M_PI);
        Theta3 = constrain_float(Theta3,-M_PI,M_PI);
        Theta4 = constrain_float(Theta4,-M_PI,M_PI);

        // ========================================================================== PWM calculado a partir da força e dos angulos ==========================================================================
        PWM1 = (2*FX*sq(Lx)*cosf(Theta1 - 2*Theta4) - 4*FX*sq(Ly)*cosf(Theta1) - 4*FY*sq(Lx)*sinf(Theta1) - 4*FY*sq(Ly)*sinf(Theta1) - FX*sq(Lx)*cosf(Theta1 + 2*Theta2) - FX*sq(Lx)*cosf(Theta1 + 2*Theta3) - 4*FX*sq(Lx)*cosf(Theta1) + FX*sq(Ly)*cosf(Theta1 - 2*Theta2) + FX*sq(Ly)*cosf(Theta1 + 2*Theta2) + 4*FX*sq(Ly)*cosf(Theta1 - 2*Theta3) + FX*sq(Ly)*cosf(Theta1 - 2*Theta4) + FX*sq(Ly)*cosf(Theta1 + 2*Theta4) - FY*sq(Lx)*sinf(Theta1 - 2*Theta2) - FY*sq(Lx)*sinf(Theta1 + 2*Theta2) - FY*sq(Lx)*sinf(Theta1 - 2*Theta3) - FY*sq(Lx)*sinf(Theta1 + 2*Theta3) - 4*FY*sq(Lx)*sinf(Theta1 - 2*Theta4) + FY*sq(Ly)*sinf(Theta1 + 2*Theta2) - 2*FY*sq(Ly)*sinf(Theta1 - 2*Theta3) + FY*sq(Ly)*sinf(Theta1 + 2*Theta4) + FX*sq(Lx)*cosf(2*Theta2 - Theta1 + 2*Theta4) + FX*sq(Lx)*cosf(2*Theta3 - Theta1 + 2*Theta4) - FX*sq(Ly)*cosf(2*Theta2 - Theta1 + 2*Theta3) - FX*sq(Ly)*cosf(2*Theta3 - Theta1 + 2*Theta4) + FY*sq(Lx)*sinf(2*Theta2 - Theta1 + 2*Theta4) + FY*sq(Lx)*sinf(2*Theta3 - Theta1 + 2*Theta4) - FY*sq(Ly)*sinf(2*Theta2 - Theta1 + 2*Theta3) - FY*sq(Ly)*sinf(2*Theta3 - Theta1 + 2*Theta4) - Lx*TN*cosf(Theta1 - 2*Theta2 + 2*Theta3) - Lx*TN*cosf(Theta1 + 2*Theta2 - 2*Theta3) - Lx*TN*cosf(Theta1 + 2*Theta2 - 2*Theta4) - Lx*TN*cosf(Theta1 + 2*Theta3 - 2*Theta4) + Ly*TN*sinf(Theta1 + 2*Theta2 - 2*Theta3) + Ly*TN*sinf(Theta1 - 2*Theta2 + 2*Theta4) + Ly*TN*sinf(Theta1 + 2*Theta2 - 2*Theta4) + Ly*TN*sinf(Theta1 - 2*Theta3 + 2*Theta4) + 4*Lx*TN*cosf(Theta1) - 4*Ly*TN*sinf(Theta1) + Lx*TN*cosf(Theta1 - 2*Theta2) + Lx*TN*cosf(Theta1 - 2*Theta3) - 2*Lx*TN*cosf(Theta1 - 2*Theta4) + FX*sq(Lx)*cosf(Theta1 - 2*Theta2 + 2*Theta3) + FX*sq(Lx)*cosf(Theta1 + 2*Theta2 - 2*Theta3) - FX*sq(Ly)*cosf(Theta1 + 2*Theta2 - 2*Theta3) - FX*sq(Ly)*cosf(Theta1 - 2*Theta3 + 2*Theta4) + Ly*TN*sinf(Theta1 - 2*Theta2) - 2*Ly*TN*sinf(Theta1 - 2*Theta3) + Ly*TN*sinf(Theta1 - 2*Theta4) - FY*sq(Lx)*sinf(Theta1 + 2*Theta2 - 2*Theta4) - FY*sq(Lx)*sinf(Theta1 + 2*Theta3 - 2*Theta4) + FY*sq(Ly)*sinf(Theta1 - 2*Theta2 + 2*Theta4) + FY*sq(Ly)*sinf(Theta1 + 2*Theta2 - 2*Theta4) + 2*FX*Lx*Ly*sinf(Theta1) + FY*Lx*Ly*cosf(Theta1 - 2*Theta2) - 2*FY*Lx*Ly*cosf(Theta1 + 2*Theta2) + FY*Lx*Ly*cosf(Theta1 - 2*Theta3) + 2*FY*Lx*Ly*cosf(Theta1 + 2*Theta3) - 4*FY*Lx*Ly*cosf(Theta1 - 2*Theta4) + 2*FY*Lx*Ly*cosf(Theta1 + 2*Theta4) - FX*Lx*Ly*sinf(Theta1 - 2*Theta2) + 2*FX*Lx*Ly*sinf(Theta1 + 2*Theta2) + 4*FX*Lx*Ly*sinf(Theta1 - 2*Theta3) - 2*FX*Lx*Ly*sinf(Theta1 + 2*Theta3) - FX*Lx*Ly*sinf(Theta1 - 2*Theta4) - 2*FX*Lx*Ly*sinf(Theta1 + 2*Theta4) - 2*FY*Lx*Ly*cosf(2*Theta3 - Theta1 + 2*Theta4) + 2*FX*Lx*Ly*sinf(2*Theta3 - Theta1 + 2*Theta4) + FY*Lx*Ly*cosf(Theta1 - 2*Theta2 + 2*Theta4) - 2*FY*Lx*Ly*cosf(Theta1 + 2*Theta2 - 2*Theta4) + FY*Lx*Ly*cosf(Theta1 - 2*Theta3 + 2*Theta4) + FX*Lx*Ly*sinf(Theta1 - 2*Theta2 + 2*Theta3) - 2*FX*Lx*Ly*sinf(Theta1 + 2*Theta2 - 2*Theta3) + FX*Lx*Ly*sinf(Theta1 + 2*Theta3 - 2*Theta4) + 2*FY*Lx*Ly*cosf(Theta1))/(k1*(2*sq(Ly)*cosf(2*Theta1) - 2*sq(Lx)*cosf(2*Theta2) - 2*sq(Lx)*cosf(2*Theta3) - 2*sq(Lx)*cosf(2*Theta4) - 2*sq(Lx)*cosf(2*Theta1) + 2*sq(Ly)*cosf(2*Theta2) + 2*sq(Ly)*cosf(2*Theta3) + 2*sq(Ly)*cosf(2*Theta4) + 4*sq(Lx)*cosf(2*Theta1 - 2*Theta4) + 4*sq(Lx)*cosf(2*Theta2 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta1 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta2 - 2*Theta4) - 8*sq(Lx) - 8*sq(Ly) + sq(Lx)*cosf(2*Theta1 - 2*Theta2 + 2*Theta3) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) + sq(Lx)*cosf(2*Theta2 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta3 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) + sq(Lx)*cosf(2*Theta2 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 - 2*Theta1 + 2*Theta3) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) - sq(Ly)*cosf(2*Theta1 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 + 2*Theta3 - 2*Theta4) + 6*Lx*Ly*sinf(2*Theta1) + 6*Lx*Ly*sinf(2*Theta2) - 6*Lx*Ly*sinf(2*Theta3) - 6*Lx*Ly*sinf(2*Theta4) + 4*Lx*Ly*sinf(2*Theta1 - 2*Theta3) - 4*Lx*Ly*sinf(2*Theta1 - 2*Theta4) - 4*Lx*Ly*sinf(2*Theta2 - 2*Theta3) + 4*Lx*Ly*sinf(2*Theta2 - 2*Theta4) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta3) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta1 + 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta2 + 2*Theta4)));
        PWM2 = (2*FX*sq(Lx)*cosf(Theta2 - 2*Theta3) - 4*FX*sq(Ly)*cosf(Theta2) - 4*FY*sq(Lx)*sinf(Theta2) - 4*FY*sq(Ly)*sinf(Theta2) - FX*sq(Lx)*cosf(2*Theta1 + Theta2) - 4*FX*sq(Lx)*cosf(Theta2) - FX*sq(Lx)*cosf(Theta2 + 2*Theta4) + FX*sq(Ly)*cosf(2*Theta1 + Theta2) + FX*sq(Ly)*cosf(Theta2 - 2*Theta3) + FX*sq(Ly)*cosf(Theta2 + 2*Theta3) + 4*FX*sq(Ly)*cosf(Theta2 - 2*Theta4) - Lx*TN*cosf(2*Theta1 - Theta2) - FY*sq(Lx)*sinf(2*Theta1 + Theta2) - 4*FY*sq(Lx)*sinf(Theta2 - 2*Theta3) - FY*sq(Lx)*sinf(Theta2 - 2*Theta4) - FY*sq(Lx)*sinf(Theta2 + 2*Theta4) + FY*sq(Ly)*sinf(2*Theta1 + Theta2) + FY*sq(Ly)*sinf(Theta2 + 2*Theta3) - 2*FY*sq(Ly)*sinf(Theta2 - 2*Theta4) + FX*sq(Lx)*cosf(2*Theta1 - Theta2 + 2*Theta3) + FX*sq(Lx)*cosf(2*Theta3 - Theta2 + 2*Theta4) - FX*sq(Ly)*cosf(2*Theta1 - Theta2 + 2*Theta4) - FX*sq(Ly)*cosf(2*Theta3 - Theta2 + 2*Theta4) + Ly*TN*sinf(2*Theta1 - Theta2) + FY*sq(Lx)*sinf(2*Theta1 - Theta2 + 2*Theta3) + FY*sq(Lx)*sinf(2*Theta3 - Theta2 + 2*Theta4) - FY*sq(Ly)*sinf(2*Theta1 - Theta2 + 2*Theta4) - FY*sq(Ly)*sinf(2*Theta3 - Theta2 + 2*Theta4) + FX*sq(Ly)*cosf(2*Theta1 - Theta2) + Lx*TN*cosf(2*Theta1 + Theta2 - 2*Theta3) + Lx*TN*cosf(Theta2 - 2*Theta1 + 2*Theta4) + Lx*TN*cosf(2*Theta1 + Theta2 - 2*Theta4) + Lx*TN*cosf(Theta2 - 2*Theta3 + 2*Theta4) + FY*sq(Lx)*sinf(2*Theta1 - Theta2) - Ly*TN*sinf(Theta2 - 2*Theta1 + 2*Theta3) - Ly*TN*sinf(2*Theta1 + Theta2 - 2*Theta3) - Ly*TN*sinf(2*Theta1 + Theta2 - 2*Theta4) - Ly*TN*sinf(Theta2 + 2*Theta3 - 2*Theta4) - 4*Lx*TN*cosf(Theta2) + 4*Ly*TN*sinf(Theta2) + 2*Lx*TN*cosf(Theta2 - 2*Theta3) - Lx*TN*cosf(Theta2 - 2*Theta4) + FX*sq(Lx)*cosf(Theta2 - 2*Theta1 + 2*Theta4) + FX*sq(Lx)*cosf(2*Theta1 + Theta2 - 2*Theta4) - FX*sq(Ly)*cosf(2*Theta1 + Theta2 - 2*Theta4) - FX*sq(Ly)*cosf(Theta2 + 2*Theta3 - 2*Theta4) - Ly*TN*sinf(Theta2 - 2*Theta3) + 2*Ly*TN*sinf(Theta2 - 2*Theta4) - FY*sq(Lx)*sinf(2*Theta1 + Theta2 - 2*Theta3) - FY*sq(Lx)*sinf(Theta2 - 2*Theta3 + 2*Theta4) + FY*sq(Ly)*sinf(Theta2 - 2*Theta1 + 2*Theta3) + FY*sq(Ly)*sinf(2*Theta1 + Theta2 - 2*Theta3) + 2*FX*Lx*Ly*sinf(Theta2) - 2*FY*Lx*Ly*cosf(2*Theta1 + Theta2) - 4*FY*Lx*Ly*cosf(Theta2 - 2*Theta3) + 2*FY*Lx*Ly*cosf(Theta2 + 2*Theta3) + FY*Lx*Ly*cosf(Theta2 - 2*Theta4) + 2*FY*Lx*Ly*cosf(Theta2 + 2*Theta4) + 2*FX*Lx*Ly*sinf(2*Theta1 + Theta2) - FX*Lx*Ly*sinf(Theta2 - 2*Theta3) - 2*FX*Lx*Ly*sinf(Theta2 + 2*Theta3) + 4*FX*Lx*Ly*sinf(Theta2 - 2*Theta4) - 2*FX*Lx*Ly*sinf(Theta2 + 2*Theta4) - 2*FY*Lx*Ly*cosf(2*Theta3 - Theta2 + 2*Theta4) + 2*FX*Lx*Ly*sinf(2*Theta3 - Theta2 + 2*Theta4) + FY*Lx*Ly*cosf(2*Theta1 - Theta2) + FX*Lx*Ly*sinf(2*Theta1 - Theta2) + FY*Lx*Ly*cosf(Theta2 - 2*Theta1 + 2*Theta3) - 2*FY*Lx*Ly*cosf(2*Theta1 + Theta2 - 2*Theta3) + FY*Lx*Ly*cosf(Theta2 + 2*Theta3 - 2*Theta4) + FX*Lx*Ly*sinf(Theta2 - 2*Theta1 + 2*Theta4) - 2*FX*Lx*Ly*sinf(2*Theta1 + Theta2 - 2*Theta4) + FX*Lx*Ly*sinf(Theta2 - 2*Theta3 + 2*Theta4) + 2*FY*Lx*Ly*cosf(Theta2))/(k1*(2*sq(Ly)*cosf(2*Theta1) - 2*sq(Lx)*cosf(2*Theta2) - 2*sq(Lx)*cosf(2*Theta3) - 2*sq(Lx)*cosf(2*Theta4) - 2*sq(Lx)*cosf(2*Theta1) + 2*sq(Ly)*cosf(2*Theta2) + 2*sq(Ly)*cosf(2*Theta3) + 2*sq(Ly)*cosf(2*Theta4) + 4*sq(Lx)*cosf(2*Theta1 - 2*Theta4) + 4*sq(Lx)*cosf(2*Theta2 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta1 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta2 - 2*Theta4) - 8*sq(Lx) - 8*sq(Ly) + sq(Lx)*cosf(2*Theta1 - 2*Theta2 + 2*Theta3) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) + sq(Lx)*cosf(2*Theta2 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta3 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) + sq(Lx)*cosf(2*Theta2 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 - 2*Theta1 + 2*Theta3) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) - sq(Ly)*cosf(2*Theta1 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 + 2*Theta3 - 2*Theta4) + 6*Lx*Ly*sinf(2*Theta1) + 6*Lx*Ly*sinf(2*Theta2) - 6*Lx*Ly*sinf(2*Theta3) - 6*Lx*Ly*sinf(2*Theta4) + 4*Lx*Ly*sinf(2*Theta1 - 2*Theta3) - 4*Lx*Ly*sinf(2*Theta1 - 2*Theta4) - 4*Lx*Ly*sinf(2*Theta2 - 2*Theta3) + 4*Lx*Ly*sinf(2*Theta2 - 2*Theta4) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta3) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta1 + 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta2 + 2*Theta4)));
        PWM3 = (FX*sq(Ly)*cosf(2*Theta2 + Theta3) - 4*FX*sq(Ly)*cosf(Theta3) - 4*FY*sq(Lx)*sinf(Theta3) - 4*FY*sq(Ly)*sinf(Theta3) - FX*sq(Lx)*cosf(2*Theta1 + Theta3) - FX*sq(Lx)*cosf(Theta3 + 2*Theta4) - 4*FX*sq(Lx)*cosf(Theta3) + FX*sq(Ly)*cosf(Theta3 - 2*Theta4) + FX*sq(Ly)*cosf(Theta3 + 2*Theta4) - Lx*TN*cosf(2*Theta1 - Theta3) + 2*Lx*TN*cosf(2*Theta2 - Theta3) - FY*sq(Lx)*sinf(2*Theta1 + Theta3) - FY*sq(Lx)*sinf(Theta3 - 2*Theta4) - FY*sq(Lx)*sinf(Theta3 + 2*Theta4) + FY*sq(Ly)*sinf(2*Theta2 + Theta3) + FY*sq(Ly)*sinf(Theta3 + 2*Theta4) + FX*sq(Lx)*cosf(2*Theta1 + 2*Theta2 - Theta3) + FX*sq(Lx)*cosf(2*Theta2 - Theta3 + 2*Theta4) - FX*sq(Ly)*cosf(2*Theta1 + 2*Theta2 - Theta3) - FX*sq(Ly)*cosf(2*Theta1 - Theta3 + 2*Theta4) + 2*Ly*TN*sinf(2*Theta1 - Theta3) - Ly*TN*sinf(2*Theta2 - Theta3) + FY*sq(Lx)*sinf(2*Theta1 + 2*Theta2 - Theta3) + FY*sq(Lx)*sinf(2*Theta2 - Theta3 + 2*Theta4) - FY*sq(Ly)*sinf(2*Theta1 + 2*Theta2 - Theta3) - FY*sq(Ly)*sinf(2*Theta1 - Theta3 + 2*Theta4) + 2*FX*sq(Lx)*cosf(2*Theta2 - Theta3) + 4*FX*sq(Ly)*cosf(2*Theta1 - Theta3) + FX*sq(Ly)*cosf(2*Theta2 - Theta3) + Lx*TN*cosf(2*Theta1 - 2*Theta2 + Theta3) + Lx*TN*cosf(Theta3 - 2*Theta1 + 2*Theta4) + Lx*TN*cosf(2*Theta1 + Theta3 - 2*Theta4) + Lx*TN*cosf(Theta3 - 2*Theta2 + 2*Theta4) + FY*sq(Lx)*sinf(2*Theta1 - Theta3) + 4*FY*sq(Lx)*sinf(2*Theta2 - Theta3) + 2*FY*sq(Ly)*sinf(2*Theta1 - Theta3) + Ly*TN*sinf(2*Theta2 - 2*Theta1 + Theta3) + Ly*TN*sinf(Theta3 - 2*Theta1 + 2*Theta4) + Ly*TN*sinf(Theta3 - 2*Theta2 + 2*Theta4) + Ly*TN*sinf(2*Theta2 + Theta3 - 2*Theta4) - 4*Lx*TN*cosf(Theta3) - 4*Ly*TN*sinf(Theta3) - Lx*TN*cosf(Theta3 - 2*Theta4) + FX*sq(Lx)*cosf(Theta3 - 2*Theta1 + 2*Theta4) + FX*sq(Lx)*cosf(2*Theta1 + Theta3 - 2*Theta4) - FX*sq(Ly)*cosf(2*Theta2 - 2*Theta1 + Theta3) - FX*sq(Ly)*cosf(Theta3 - 2*Theta1 + 2*Theta4) + Ly*TN*sinf(Theta3 - 2*Theta4) - FY*sq(Lx)*sinf(2*Theta1 - 2*Theta2 + Theta3) - FY*sq(Lx)*sinf(Theta3 - 2*Theta2 + 2*Theta4) + FY*sq(Ly)*sinf(Theta3 - 2*Theta2 + 2*Theta4) + FY*sq(Ly)*sinf(2*Theta2 + Theta3 - 2*Theta4) - 2*FX*Lx*Ly*sinf(Theta3) - 2*FY*Lx*Ly*cosf(2*Theta1 + Theta3) - 2*FY*Lx*Ly*cosf(2*Theta2 + Theta3) - FY*Lx*Ly*cosf(Theta3 - 2*Theta4) + 2*FY*Lx*Ly*cosf(Theta3 + 2*Theta4) + 2*FX*Lx*Ly*sinf(2*Theta1 + Theta3) + 2*FX*Lx*Ly*sinf(2*Theta2 + Theta3) + FX*Lx*Ly*sinf(Theta3 - 2*Theta4) - 2*FX*Lx*Ly*sinf(Theta3 + 2*Theta4) + 2*FY*Lx*Ly*cosf(2*Theta1 + 2*Theta2 - Theta3) - 2*FX*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - Theta3) - FY*Lx*Ly*cosf(2*Theta1 - Theta3) + 4*FY*Lx*Ly*cosf(2*Theta2 - Theta3) + 4*FX*Lx*Ly*sinf(2*Theta1 - Theta3) - FX*Lx*Ly*sinf(2*Theta2 - Theta3) - FY*Lx*Ly*cosf(2*Theta2 - 2*Theta1 + Theta3) + 2*FY*Lx*Ly*cosf(Theta3 - 2*Theta2 + 2*Theta4) - FY*Lx*Ly*cosf(2*Theta2 + Theta3 - 2*Theta4) - FX*Lx*Ly*sinf(2*Theta1 - 2*Theta2 + Theta3) + 2*FX*Lx*Ly*sinf(Theta3 - 2*Theta1 + 2*Theta4) - FX*Lx*Ly*sinf(2*Theta1 + Theta3 - 2*Theta4) - 2*FY*Lx*Ly*cosf(Theta3))/(k1*(2*sq(Ly)*cosf(2*Theta1) - 2*sq(Lx)*cosf(2*Theta2) - 2*sq(Lx)*cosf(2*Theta3) - 2*sq(Lx)*cosf(2*Theta4) - 2*sq(Lx)*cosf(2*Theta1) + 2*sq(Ly)*cosf(2*Theta2) + 2*sq(Ly)*cosf(2*Theta3) + 2*sq(Ly)*cosf(2*Theta4) + 4*sq(Lx)*cosf(2*Theta1 - 2*Theta4) + 4*sq(Lx)*cosf(2*Theta2 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta1 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta2 - 2*Theta4) - 8*sq(Lx) - 8*sq(Ly) + sq(Lx)*cosf(2*Theta1 - 2*Theta2 + 2*Theta3) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) + sq(Lx)*cosf(2*Theta2 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta3 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) + sq(Lx)*cosf(2*Theta2 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 - 2*Theta1 + 2*Theta3) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) - sq(Ly)*cosf(2*Theta1 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 + 2*Theta3 - 2*Theta4) + 6*Lx*Ly*sinf(2*Theta1) + 6*Lx*Ly*sinf(2*Theta2) - 6*Lx*Ly*sinf(2*Theta3) - 6*Lx*Ly*sinf(2*Theta4) + 4*Lx*Ly*sinf(2*Theta1 - 2*Theta3) - 4*Lx*Ly*sinf(2*Theta1 - 2*Theta4) - 4*Lx*Ly*sinf(2*Theta2 - 2*Theta3) + 4*Lx*Ly*sinf(2*Theta2 - 2*Theta4) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta3) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta1 + 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta2 + 2*Theta4)));
        PWM4 = -(4*FX*sq(Lx)*cosf(Theta4) + 4*FX*sq(Ly)*cosf(Theta4) + 4*FY*sq(Lx)*sinf(Theta4) + 4*FY*sq(Ly)*sinf(Theta4) + FX*sq(Lx)*cosf(2*Theta2 + Theta4) + FX*sq(Lx)*cosf(2*Theta3 + Theta4) - FX*sq(Ly)*cosf(2*Theta1 + Theta4) - FX*sq(Ly)*cosf(2*Theta3 + Theta4) + 2*Lx*TN*cosf(2*Theta1 - Theta4) - Lx*TN*cosf(2*Theta2 - Theta4) - Lx*TN*cosf(2*Theta3 - Theta4) + FY*sq(Lx)*sinf(2*Theta2 + Theta4) + FY*sq(Lx)*sinf(2*Theta3 + Theta4) - FY*sq(Ly)*sinf(2*Theta1 + Theta4) - FY*sq(Ly)*sinf(2*Theta3 + Theta4) - FX*sq(Lx)*cosf(2*Theta1 + 2*Theta2 - Theta4) - FX*sq(Lx)*cosf(2*Theta1 + 2*Theta3 - Theta4) + FX*sq(Ly)*cosf(2*Theta1 + 2*Theta2 - Theta4) + FX*sq(Ly)*cosf(2*Theta2 + 2*Theta3 - Theta4) - Ly*TN*sinf(2*Theta1 - Theta4) + 2*Ly*TN*sinf(2*Theta2 - Theta4) - Ly*TN*sinf(2*Theta3 - Theta4) - FY*sq(Lx)*sinf(2*Theta1 + 2*Theta2 - Theta4) - FY*sq(Lx)*sinf(2*Theta1 + 2*Theta3 - Theta4) + FY*sq(Ly)*sinf(2*Theta1 + 2*Theta2 - Theta4) + FY*sq(Ly)*sinf(2*Theta2 + 2*Theta3 - Theta4) - 2*FX*sq(Lx)*cosf(2*Theta1 - Theta4) - FX*sq(Ly)*cosf(2*Theta1 - Theta4) - 4*FX*sq(Ly)*cosf(2*Theta2 - Theta4) - FX*sq(Ly)*cosf(2*Theta3 - Theta4) + Lx*TN*cosf(2*Theta2 - 2*Theta1 + Theta4) + Lx*TN*cosf(2*Theta3 - 2*Theta1 + Theta4) + Lx*TN*cosf(2*Theta3 - 2*Theta2 + Theta4) + Lx*TN*cosf(2*Theta2 - 2*Theta3 + Theta4) - 4*FY*sq(Lx)*sinf(2*Theta1 - Theta4) - FY*sq(Lx)*sinf(2*Theta2 - Theta4) - FY*sq(Lx)*sinf(2*Theta3 - Theta4) - 2*FY*sq(Ly)*sinf(2*Theta2 - Theta4) + Ly*TN*sinf(2*Theta1 - 2*Theta2 + Theta4) + Ly*TN*sinf(2*Theta3 - 2*Theta1 + Theta4) + Ly*TN*sinf(2*Theta1 - 2*Theta3 + Theta4) + Ly*TN*sinf(2*Theta3 - 2*Theta2 + Theta4) - 4*Lx*TN*cosf(Theta4) - 4*Ly*TN*sinf(Theta4) - FX*sq(Lx)*cosf(2*Theta3 - 2*Theta2 + Theta4) - FX*sq(Lx)*cosf(2*Theta2 - 2*Theta3 + Theta4) + FX*sq(Ly)*cosf(2*Theta1 - 2*Theta2 + Theta4) + FX*sq(Ly)*cosf(2*Theta3 - 2*Theta2 + Theta4) + FY*sq(Lx)*sinf(2*Theta2 - 2*Theta1 + Theta4) + FY*sq(Lx)*sinf(2*Theta3 - 2*Theta1 + Theta4) - FY*sq(Ly)*sinf(2*Theta3 - 2*Theta1 + Theta4) - FY*sq(Ly)*sinf(2*Theta1 - 2*Theta3 + Theta4) + 2*FX*Lx*Ly*sinf(Theta4) + 2*FY*Lx*Ly*cosf(2*Theta1 + Theta4) + 2*FY*Lx*Ly*cosf(2*Theta2 + Theta4) - 2*FY*Lx*Ly*cosf(2*Theta3 + Theta4) - 2*FX*Lx*Ly*sinf(2*Theta1 + Theta4) - 2*FX*Lx*Ly*sinf(2*Theta2 + Theta4) + 2*FX*Lx*Ly*sinf(2*Theta3 + Theta4) - 2*FY*Lx*Ly*cosf(2*Theta1 + 2*Theta2 - Theta4) + 2*FX*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - Theta4) - 4*FY*Lx*Ly*cosf(2*Theta1 - Theta4) + FY*Lx*Ly*cosf(2*Theta2 - Theta4) + FY*Lx*Ly*cosf(2*Theta3 - Theta4) + FX*Lx*Ly*sinf(2*Theta1 - Theta4) - 4*FX*Lx*Ly*sinf(2*Theta2 - Theta4) + FX*Lx*Ly*sinf(2*Theta3 - Theta4) + FY*Lx*Ly*cosf(2*Theta1 - 2*Theta2 + Theta4) - 2*FY*Lx*Ly*cosf(2*Theta3 - 2*Theta1 + Theta4) + FY*Lx*Ly*cosf(2*Theta1 - 2*Theta3 + Theta4) + FX*Lx*Ly*sinf(2*Theta2 - 2*Theta1 + Theta4) - 2*FX*Lx*Ly*sinf(2*Theta3 - 2*Theta2 + Theta4) + FX*Lx*Ly*sinf(2*Theta2 - 2*Theta3 + Theta4) + 2*FY*Lx*Ly*cosf(Theta4))/(k1*(2*sq(Ly)*cosf(2*Theta1) - 2*sq(Lx)*cosf(2*Theta2) - 2*sq(Lx)*cosf(2*Theta3) - 2*sq(Lx)*cosf(2*Theta4) - 2*sq(Lx)*cosf(2*Theta1) + 2*sq(Ly)*cosf(2*Theta2) + 2*sq(Ly)*cosf(2*Theta3) + 2*sq(Ly)*cosf(2*Theta4) + 4*sq(Lx)*cosf(2*Theta1 - 2*Theta4) + 4*sq(Lx)*cosf(2*Theta2 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta1 - 2*Theta3) + 4*sq(Ly)*cosf(2*Theta2 - 2*Theta4) - 8*sq(Lx) - 8*sq(Ly) + sq(Lx)*cosf(2*Theta1 - 2*Theta2 + 2*Theta3) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) + sq(Lx)*cosf(2*Theta2 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) + sq(Lx)*cosf(2*Theta1 + 2*Theta3 - 2*Theta4) + sq(Lx)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) + sq(Lx)*cosf(2*Theta2 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 - 2*Theta1 + 2*Theta3) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta3) - sq(Ly)*cosf(2*Theta1 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 + 2*Theta2 - 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta1 + 2*Theta4) - sq(Ly)*cosf(2*Theta1 - 2*Theta3 + 2*Theta4) - sq(Ly)*cosf(2*Theta3 - 2*Theta2 + 2*Theta4) - sq(Ly)*cosf(2*Theta2 + 2*Theta3 - 2*Theta4) + 6*Lx*Ly*sinf(2*Theta1) + 6*Lx*Ly*sinf(2*Theta2) - 6*Lx*Ly*sinf(2*Theta3) - 6*Lx*Ly*sinf(2*Theta4) + 4*Lx*Ly*sinf(2*Theta1 - 2*Theta3) - 4*Lx*Ly*sinf(2*Theta1 - 2*Theta4) - 4*Lx*Ly*sinf(2*Theta2 - 2*Theta3) + 4*Lx*Ly*sinf(2*Theta2 - 2*Theta4) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta3) - 2*Lx*Ly*sinf(2*Theta1 + 2*Theta2 - 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta1 + 2*Theta4) + 2*Lx*Ly*sinf(2*Theta3 - 2*Theta2 + 2*Theta4)));

        // Aqui é feita a saturação do PWM para que caso ele saia fora dos valores aceitaveis- o método irá tentar convergir através das demais variáveis
        PWM1 = constrain_float(PWM1,Pwmmin,Pwmmax);
        PWM2 = constrain_float(PWM2,Pwmmin,Pwmmax);
        PWM3 = constrain_float(PWM3,Pwmmin,Pwmmax);
        PWM4 = constrain_float(PWM4,Pwmmin,Pwmmax);

    }
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
    Fy = Fmax * float(channel_roll->get_radio_in()- med_roll)/float(channel_roll->get_radio_max() - med_roll);
    //Calcula a força em X a partir do stick de Arfagem
    Fx = Fmax * float(channel_pitch->get_radio_in()-med_pitch)/float(channel_pitch->get_radio_max()- med_pitch);
    //Calcula o torque em Z a partir do stick de Guinada
    tN = Nmax * float(channel_yaw->get_radio_in()-  med_yaw)/float(channel_yaw->get_radio_max() - med_yaw);
    // Envia os valores para a matriz de alocação de forças
    alocation_matrix(Fx,Fy,tN,theta_m1,theta_m2,theta_m3,theta_m4,Pwm1,Pwm2,Pwm3,Pwm4);
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
