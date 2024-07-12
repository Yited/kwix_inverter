#include <math.h>
#include "Variable.h"
#include "Input_Output.h"
#include "Function_.h"

void DCLink_VoltageControl_PI();
void DCLink_VoltageControl_IP();
void DCLink_VoltageControl_ip();
void Switching_two_level();
void Switching_off();
void HarmoniCompensation();
void DSOGI_PLL();

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
{
    t_cnt += dt;
    Simulation_time += t;
    Input(in);


    /* 지령 전류 */
    Iqe_ref = 15; 
    Ide_ref = 0.;
    
    if (t > 0.2)
    {
        Iqe_ref = 30;
        Ide_ref = 0.;
    }
   
    /*DC Link Voltage*/
    DCLink_Voltage_ref = 500;

    if (t_cnt > T_samp)
    {

       ADC_Sensing();

       //PLL();
       DSOGI_PLL();

       if (t > 0.1)
       {
          
          // DCLink_VoltageControl_IP();   


          CurrentControl();

         
          //HarmoniCompensation();
         

          Modulation();


          normalization();        
       }
       

       
         t_cnt -= T_samp;
    }

    if (DCLink_Voltage_ref * (1 - 0.05) <= Vdc )
    {
           Switching_two_level();
    }
    else
    {
           Switching_off();
    }
    

    Measure();
    Output(out);

}

void DSOGI_PLL()
{
    abc_to_dqs(Vas_grid, Vbs_grid, Vcs_grid, &Vds_grid, &Vqs_grid);
    dqe_to_abc(Vde_grid, Vqe_grid, theta_grid, &VA, &VB, &VC);

    Damping_K = 1.414;

    //SOGI 연산 //
    Alpha = Vds_grid;
    Beta = Vqs_grid;

    //////////////////////////////////////////////////////////////////
    //alpha 연산//
    Alpha_error_1 = Alpha - Alpha_prime;

    Alpha_error_2 = (Damping_K * Alpha_error_1) - q_Alpha_prime;

    Alpha_error_3 = (Wde_ref * Alpha_error_2);

    Alpha_prime += Alpha_error_3 * T_samp;

    Alpha_error_4 += Alpha_prime * T_samp;

    q_Alpha_prime = Wde_ref * Alpha_error_4;
    /////////////////////////////////////////////////////////////////


    //beta 연산//
    Beta_error_1 = Beta - Beta_prime;

    Beta_error_2 = (Damping_K * Beta_error_1) - q_Beta_prime;

    Beta_error_3 = (Wde_ref * Beta_error_2);

    Beta_prime += Beta_error_3 * T_samp;

    Beta_error_4 += Beta_prime * T_samp;

    q_Beta_prime = Wde_ref * Beta_error_4;
    ///////////////////////////////////////////////////////////////////

    //Positive Sequence 연산// 
    Alpha_plus = 0.5 * (Alpha_prime - q_Beta_prime);
    Beta_plus = 0.5 * (q_Alpha_prime + Beta_prime);

    //negative Sequence 연산// 
    Alpha_negative = 0.5 * (Alpha_prime + q_Beta_prime);
    Beta_negative = 0.5 * (-q_Alpha_prime + Beta_prime);

    dqs_to_dqe(Alpha_plus, Beta_plus, theta_grid, &Vde_grid, &Vqe_grid);

    // SRF-PLL 상수값//
    Em = 179.629;
    Zeta = 0.707, Wn = 50;
    Wc = 1. + 2. * Zeta * Wn;


    Kp_PLL = (2. * Zeta * Wn) / (Em);
    Ki_PLL = (Wn * Wn) / (Em);


    // Kp_PLL = 0.273;
    // Ki_PLL = 0.135;

    // Kp_PLL = 2.22;
    // Ki_PLL = 61.69;

    // SRF_PLL 입력 //
    Vde_err = 0. - Vde_grid;
    Vde_err_int += (Vde_err)*T_samp;
    Wde_ref_fb = Kp_PLL * Vde_err + Ki_PLL * Vde_err_int;
    W_rated = 2. * PI * Freq;
    Wde_ref = Wde_ref_fb + W_rated;


    // LPF //
     //La = (2. - Wc * T_samp) / (2. + Wc * T_samp);
     //Lb = (Wc * T_samp) / (2. + Wc * T_samp);
     //Vde_LPF = La * Vde_LPF + Lb * (Vde_grid + Vde_old);
     //Vde_old = Vde_grid;

    // 정상분 theta 계산 //
    theta_grid += Wde_ref * T_samp;
    if (theta_grid > PI) { theta_grid -= 2. * PI; }
    else if (theta_grid < -PI) { theta_grid += 2. * PI; }

    // 역상분 5차 theta 계산 //
    Grid_Angle_N_5th += -5 * Wde_ref * T_samp;
    if (Grid_Angle_N_5th > PI) { Grid_Angle_N_5th -= 2. * PI; }
    else if (Grid_Angle_N_5th < -PI) { Grid_Angle_N_5th += 2. * PI; }
    
    // 정상분 7차 theta 계산 //
    Grid_Angle_P_7th += 7 * Wde_ref * T_samp;
    if (Grid_Angle_P_7th > PI) { Grid_Angle_P_7th -= 2. * PI; }
    else if (Grid_Angle_P_7th < -PI) { Grid_Angle_P_7th += 2. * PI; }


    theta_voltage = theta_grid + (PI / 2);
    if (theta_voltage > PI) { theta_voltage -= 2. * PI; }
    else if (theta_voltage < -PI) { theta_voltage += 2. * PI; }


    //   V_out_region_old = V_out_region;
    //  if (theta_voltage >= 0) V_out_region = floor(theta_voltage / (PI / 6));
    //   else V_out_region = floor((theta_voltage + 2. * PI) / (PI / 6));
}






void PLL()
{
    abc_to_dqs(Vas_grid, Vbs_grid, Vcs_grid, &Vds_grid, &Vqs_grid);
    abc_to_dqe(Vas_grid, Vbs_grid, Vcs_grid, V_out_theta, &Vde_grid, &Vqe_grid);

    Em = sqrt(Vds_grid * Vds_grid + Vqs_grid * Vqs_grid) + 0.00001;
    Zeta = 0.707, Wn = 50., Wc = 1. + 2. * Zeta * Wn;


    La = (2. - Wc * T_samp) / (2. + Wc * T_samp);
    Lb = (Wc * T_samp) / (2. + Wc * T_samp);


    Vde_LPF = La * Vde_LPF + Lb * (Vde_grid + Vde_old);
    Vde_old = Vde_grid;

    Vde_err = 0. - Vde_LPF;

    Kp_PLL = (2. * Zeta * Wn) / (Em + 0.00001);
    Ki_PLL = (Wn * Wn) / ((Em + 0.00001) * Wc);

    Vde_err_int += (Vde_err)*T_samp;

    Wde_ref_fb = Kp_PLL * Vde_err + Ki_PLL * Vde_err_int;
    W_rated = 2. * PI * Freq;

    //전향 보상//
    //Wde_ref = Wde_ref_fb;


    Wde_ref = Wde_ref_fb + W_rated;

    theta_grid += Wde_ref * T_samp;
    if (theta_grid > PI) { theta_grid -= 2. * PI; }
    else if (theta_grid < -PI) { theta_grid += 2. * PI; }


    Grid_Angle_N_5th += -5 * Wde_ref * T_samp;
    if (Grid_Angle_N_5th > PI) { Grid_Angle_N_5th -= 2. * PI; }
    else if (Grid_Angle_N_5th < -PI) { Grid_Angle_N_5th += 2. * PI; }


    Grid_Angle_P_7th += 7 * Wde_ref * T_samp;
    if (Grid_Angle_P_7th > PI) { Grid_Angle_P_7th -= 2. * PI; }
    else if (Grid_Angle_P_7th < -PI) { Grid_Angle_P_7th += 2. * PI; }


    theta_voltage = theta_grid + (PI / 2);
    if (theta_voltage > PI) { theta_voltage -= 2. * PI; }
    else if (theta_voltage < -PI) { theta_voltage += 2. * PI; }

    V_out_region_old = V_out_region;


    if (theta_voltage >= 0) V_out_region = floor(theta_voltage / (PI / 6));
    else V_out_region = floor((theta_voltage + 2. * PI) / (PI / 6));

}



void DCLink_VoltageControl_IP()
{

    DCLINK_Kp = -((3.0 * Vqe_grid) / (2.0 * DCLink_Voltage_ref)) * BandWidth_DCLink_Voltage * Cdc;
    DCLINK_Ki = DCLINK_Kp * BandWidth_DCLink_Voltage / 5.0;

    DCLink_Voltage_err = DCLink_Voltage_ref - Vdc;
    DCLink_Voltage_err_int += DCLINK_Ki* DCLink_Voltage_err * T_samp;

    I_DC_ref = (DCLink_Voltage_err_int - DCLINK_Kp * Vdc);
    Iqe_ref = (I_DC_ref * (2.0 * Vdc) / (3.0 * Vqe_grid));

    if (Iqe_ref < -100)
    {
        Iqe_ref = -100;
    }
    else if (Iqe_ref > 100)
    {
       Iqe_ref = 100;
    }
}



void ADC_Sensing(void)
{
    // 계통 위상각 //
    V_out_theta = theta_grid;

    // 상단 + 하단 커패시터 전압 */
    Vdc = Vdc1 + Vdc2;

    // 전류 센서 //
    abc_to_dqs(Ia, Ib, Ic, &Ids, &Iqs);
    dqs_to_dqe(Ids, Iqs, V_out_theta, &Ide, &Iqe);

    // 전압 센서 //
    // 상전압 <- 선간 전압 //
    Vas_grid = (Vab_grid - Vca_grid) / 3.;
    Vbs_grid = (Vbc_grid - Vab_grid) / 3.;
    Vcs_grid = (Vca_grid - Vbc_grid) / 3.;
}


void CurrentControl()
{
    Ki_cc = Rs * BandWidth_current;
    Kp_cc = L * BandWidth_current;

    id_err = Ide_ref - Ide;
    iq_err = Iqe_ref - Iqe;

    id_err_int += id_err * T_samp;
    iq_err_int += iq_err * T_samp;

    Vde_fb = Ki_cc * id_err + Ki_cc * id_err_int;
    Vqe_fb = Kp_cc * iq_err + Ki_cc * iq_err_int;

    Vde_ff = -2. * PI * 60. * L * Iqe + Vde_grid; // 계통 전압 보상항
    Vqe_ff = 2. * PI * 60. * L * Ide + Vqe_grid;

    Vde_ref = (Vde_fb + Vde_ff);
    Vqe_ref = (Vqe_fb + Vqe_ff);
}

void HarmoniCompensation() 
{

    abc_to_dqe(Ia, Ib, Ic, Grid_Angle_N_5th, &Ide_5th_n, &Iqe_5th_n);
    

    Wc_I_5th = 2*PI;

    La_I_5th = (2. - Wc_I_5th * T_samp) / (2 + Wc_I_5th * T_samp);
    Lb_I_5th = (Wc_I_5th * T_samp) / (2. + Wc_I_5th * T_samp);
    
    Ide_5th_flt_n = La_I_5th * Ide_5th_flt_n + Lb_I_5th * (Ide_5th_n + Ide_5th_old_n);
    Iqe_5th_flt_n = La_I_5th * Iqe_5th_flt_n + Lb_I_5th * (Iqe_5th_n + Iqe_5th_old_n);

    Ide_5th_old_n = Ide_5th_n;
    Iqe_5th_old_n = Iqe_5th_n;
    

    //제어기 지령//
    Ide_5th_err_n = (0 - Ide_5th_flt_n);
    Iqe_5th_err_n = (0 - Iqe_5th_flt_n);

    Ki_CC_5th = Rs * BandWidth_current*1;
    KP_CC_5th = L * BandWidth_current*24;

    Vde_5th_integ_n += Ki_CC_5th * Ide_5th_err_n * T_samp;
    Vde_5th_ref_fb_n = Vde_5th_integ_n + KP_CC_5th * Ide_5th_err_n;

    Vqe_5th_integ_n += Ki_CC_5th * Iqe_5th_err_n * T_samp;
    Vqe_5th_ref_fb_n = Vde_5th_integ_n + KP_CC_5th * Iqe_5th_err_n;

    Vde_5th_ref_n = Vde_5th_ref_fb_n;
    Vqe_5th_ref_n = Vqe_5th_ref_fb_n;

    dqe_to_dqs(Vde_5th_ref_n, Vqe_5th_ref_n, Grid_Angle_N_5th, &Vds_5th_ref_n, &Vqs_5th_ref_n);

    //////////////////////////////////////////////////////////////////////////////////////////////

    abc_to_dqe(Ia, Ib, Ic, Grid_Angle_P_7th, &Ide_7th_p, &Iqe_7th_p);
    
    Wc_I_7th = 2*PI;

    La_I_7th = (2. - Wc_I_7th * T_samp) / (2 + Wc_I_7th * T_samp);
    Lb_I_7th = (Wc_I_7th * T_samp) / (2. + Wc_I_7th * T_samp);

    Ide_7th_flt_p = La_I_7th * Ide_7th_flt_p + Lb_I_7th * (Ide_7th_p + Ide_7th_old_p);
    Iqe_7th_flt_p = La_I_7th * Iqe_7th_flt_p + Lb_I_7th * (Iqe_7th_p + Iqe_7th_old_p);

    Ide_7th_old_p = Ide_7th_p;
    Iqe_7th_old_p = Iqe_7th_p;

    //제어기 지령//
    Ide_7th_err_p = (0 - Ide_7th_flt_p);
    Iqe_7th_err_p = (0 - Iqe_7th_flt_p);

    Ki_CC_7th = Rs * BandWidth_current*1.0;
    KP_CC_7th = L * BandWidth_current*30.0;

     Vde_7th_integ_p += Ki_CC_7th * Ide_7th_err_p * T_samp;
     Vde_7th_ref_fb_p = Vde_7th_integ_p + KP_CC_7th * Ide_7th_err_p;

     Vqe_7th_integ_p += Ki_CC_7th * Iqe_7th_err_p * T_samp;
     Vqe_7th_ref_fb_p = Vde_7th_integ_p + KP_CC_7th * Iqe_7th_err_p;

     Vde_7th_ref_p = Vde_7th_ref_fb_p;
     Vqe_7th_ref_p = Vqe_7th_ref_fb_p;

     dqe_to_dqs(Vde_7th_ref_p, Vqe_7th_ref_p, Grid_Angle_P_7th, &Vds_7th_ref_p, &Vqs_7th_ref_p);

}


void Modulation()
{

    // dq 축 전압지령 -> 상전압 지령 변환 // 
    dqe_to_dqs(Vde_ref, Vqe_ref, V_out_theta, &Vds_ref, &Vqs_ref);
    dqe_to_dqs(Vde_5th_ref_n, Vqe_5th_ref_n, Grid_Angle_N_5th, &Vds_5th_ref_n, &Vqs_5th_ref_n);
    dqe_to_dqs(Vde_7th_ref_p, Vqe_7th_ref_p, Grid_Angle_P_7th, &Vds_7th_ref_p, &Vqs_7th_ref_p);
    
    Vds_ref = Vds_ref + Vds_5th_ref_n + Vds_7th_ref_p;
    Vqs_ref = Vqs_ref + Vqs_5th_ref_n + Vqs_7th_ref_p;
       
   // Vds_ref = Vds_ref ;
   // Vqs_ref = Vqs_ref ;

    dqs_to_abc(Vds_ref, Vqs_ref, &Vas_ref, &Vbs_ref, &Vcs_ref);
    
    // A 상전압 지령 제한 //
    if ((Vdc * 0.57735) < Vas_ref)
    {
        Vas_ref = Vdc * 0.57735;
    }
    else if (-(Vdc * 0.57735) > Vas_ref)
    {
        Vas_ref = -Vdc * 0.57735;
    }

    // B 상전압 지령 제한 //
    if ((Vdc * 0.57735) < Vbs_ref)
    {
        Vbs_ref = Vdc * 0.57735;
    }
    else if (-(Vdc * 0.57735) > Vbs_ref)
    {
        Vbs_ref = -Vdc * 0.57735;
    }


    // C 상전압 지령 제한 //
    if ((Vdc * 0.57735) < Vcs_ref)
    {
        Vcs_ref = Vdc * 0.57735;
    }
    else if (-(Vdc * 0.57735) > Vcs_ref)
    {
        Vcs_ref = -Vdc * 0.57735;
    }

    // Offset 도출 //
    Max_Mid_Min(&Vas_ref, &Vbs_ref, &Vcs_ref, &max, &mid, &min);
 
    
    // 옵셋변조 //
    Vsn = -0.5 * (max + min);
    Van_ref = Vas_ref + Vsn, Vbn_ref = Vbs_ref + Vsn, Vcn_ref = Vcs_ref + Vsn; 
}



void normalization()
{

   // 크기 정규화 //
  San = Van_ref / (Vdc/2);
  Sbn = Vbn_ref / (Vdc/2);
  Scn = Vcn_ref / (Vdc/2);

}


 
void Switching_two_level()
{

  if (San > Vtri1_L) { T1a = 1;  T3a = 0;}
  else { T1a = 0;  T3a = 1; }
   
    
  if (Sbn > Vtri1_L) { T1b = 1;  T3b = 0; }
  else { T1b = 0;  T3b = 1; }


  if (Scn > Vtri1_L) { T1c = 1;  T3c = 0; }
  else { T1c = 0;  T3c = 1; }

}


void Switching_off()
{
    T1a = 0;  
    T3a = 0;
    
    T1b = 0;  
    T3b = 0;

    T1c = 0;
    T3c = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


void Measure()
{
    /* MI */
    MI = sqrt(Vde_ref * Vde_ref + Vqe_ref * Vqe_ref) / (Vdc / 2. + 0.000001);
}

//void CurrentControl()
//{
//    //지역 변수 선언 //
//    double Vde_fb_intg = 0.;
//    double Vqe_fb_intg = 0.;
//    double Vde_fb_proportion = 0.;
//    double Vqe_fb_proportion = 0.;
//    double Vde_ref_limit = 0.;
//    double Vqe_ref_limit = 0.;
//
//    // 이득 계산 //
//    Ki_cc = Rs * BandWidth_current;
//    Kp_cc = L * BandWidth_current;
//    Ka_cc = 1 / Kp_cc; 
//    
//
//    // 오차 계산 //
//    id_err = Ide_ref - Ide; 
//    iq_err = Iqe_ref - Iqe; 
//    
//
//    // Pgain 오차  계산 //
//    Vde_fb_proportion = Kp_cc* id_err;
//    Vqe_fb_proportion = Kp_cc* iq_err;
//
//
//    // 안티 와인드 업  // 
//    Vde_fb_intg += Ki_cc * (id_err - Ka_cc*(Vde_ref_limit - Vde_ref))* T_samp;
//    Vqe_fb_intg += Ki_cc * (iq_err - Ka_cc*(Vqe_ref_limit - Vqe_ref))* T_samp;
//    
//    //Vde_fb_intg += Ki_cc * id_err * T_samp;
//    //Vqe_fb_intg += Ki_cc * iq_err * T_samp;
//
//    //피드백 값 계산 //
//    Vde_fb = Vde_fb_proportion + Vde_fb_intg;
//    Vqe_fb = Vqe_fb_proportion + Vqe_fb_intg;
//
//
//    // 상호간섭항 제거 + 초기값 더하기 //
//    Vde_ff = -2. * PI * 60. * L * Iqe + Vde_grid;
//    Vqe_ff =  2. * PI * 60. * L * Ide + Vqe_grid;
//
//
//    // 전압 지령 출력 // 
//    Vde_ref_limit = (Vde_fb + Vde_ff); 
//    Vqe_ref_limit = (Vqe_fb + Vqe_ff); 
//
//
//    // d축 제한 //
//    if ( Vdc/2 < Vde_ref_limit )
//    {
//        Vde_ref_limit =  Vdc/2 ;
//    }
//    else if (- Vdc / 2 > Vde_ref_limit)
//    {
//        Vde_ref_limit = -Vdc/2;
//    }
//
//    // q축 제한 //
//    if (Vdc / 2 < Vqe_ref_limit)
//    {
//        Vqe_ref_limit = Vdc / 2;
//    }
//    else if (-Vdc / 2 > Vqe_ref_limit)
//    {
//        Vqe_ref_limit = -Vdc / 2;
//    }
//
//    Vde_ref = Vde_ref_limit;
//    Vqe_ref = Vqe_ref_limit;
//
//}

void DCLink_VoltageControl_ip()
{
    if (DCLink_Voltage_ref < 500)
    {
        if (Simulation_time > 0.1)
        {
            Slope = 40;
            Y_Intercept = 0;
            TimeInterval = 0.01;

            DCLink_Voltage_ref = LinearFunction(time, Slope, Y_Intercept);
            time += TimeInterval;

        }
    }
    else
    {
        DCLink_Voltage_ref = 500;
    }

    // 상수값 입 력//
    double zeta = 0.90;
    double DCLink_VoltageControl_Wn = 302.;



    DCLink_VoltageController_Kp = (2. * zeta * DCLink_VoltageControl_Wn * Cdc * DCLink_Voltage_ref) / (1.5 * Vqe_grid);
    DCLink_VoltageController_Ki = ((DCLink_VoltageControl_Wn * DCLink_VoltageControl_Wn) * Cdc * DCLink_Voltage_ref) / (1.5 * Vqe_grid);
    DCLink_VoltageController_Ka = 1. / DCLink_VoltageController_Kp;


    //error 연산 //
    DCLink_Voltage_err = DCLink_Voltage_ref - Vdc;

    DCLink_Voltage_err_int += DCLink_VoltageController_Ki * T_samp * DCLink_Voltage_err;

    Iqe_feedback = -(DCLink_Voltage_err_int - (Vdc * DCLink_VoltageController_Kp));


    if (Iqe_feedback < -100)
    {
        Iqe_ref = -100;
    }
    else if (Iqe_feedback > 100)
    {
        Iqe_ref = 100;
    }
    else
    {
        Iqe_ref = Iqe_feedback;
    }

}

void DCLink_VoltageControl_PI()
{

    DCLINK_Kp = -((3.0 * Vqe_grid) / (2.0 * DCLink_Voltage_ref)) * BandWidth_DCLink_Voltage * Cdc;
    DCLINK_Ki = DCLINK_Kp * BandWidth_DCLink_Voltage / 5.0;


    DCLink_Voltage_err = DCLink_Voltage_ref - Vdc;
    DCLink_Voltage_err_int += DCLink_Voltage_err * T_samp;


    I_DC_ref = (DCLINK_Kp * DCLink_Voltage_err + DCLINK_Ki * DCLink_Voltage_err_int);
    Iqe_ref = (I_DC_ref * (2.0 * Vdc) / (3.0 * Vqe_grid));

    if (Iqe_ref < -100)
    {
        Iqe_ref = -100;
    }
    else if (Iqe_ref > 100)
    {
        Iqe_ref = 100;
    }

}
