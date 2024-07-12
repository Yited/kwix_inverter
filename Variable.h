#define T_samp            1.25e-4   //제어주기   //제어주기 20kHZ                  
#define PI			      3.141592
#define Freq	          60.
#define PwmPeriod	      8000.
#define BandWidth_current 2513.274                // 250Hz : 1570 rad/sec    // 500 Hz  : 3141rad/sec  // 750 Hz  : 4712rad/sec     // 1000Hz : 6283rad/sec
#define BandWidth_NtrVoltage 503.               
#define BandWidth_DCLink_Voltage 314.           // 400Hz : 2516 rad/sec  // 200 Hz : 1208    // 100Hz : 604
#define BandWidth_Wpi 


//  #define Rs    40e-3
#define Rs    15e-3
  //#define L     3.5e-3
#define L     4.6e-3
#define Cdc   1120.e-006
#define CN    2*(2240.e-006)/4 


#define Plus  5 
#define OU1   4 
#define OU2   3 
#define OL1   2 
#define OL2   1 
#define Minus 0


/* 삼각파 */
double Vtri1_P, Vtri2_P, Vtri1_L, Vtri2_L = 0.;
double t_cnt, t_cnt2 = 0.;
double Vdc = 0.;
double Vdc1 = 0.;
double Vdc2 = 0.;

/* CurrentControl */
double Ia, Ib, Ic, Ids, Iqs, Ide, Iqe = 0.;
double Ki_cc, Kp_cc, Ka_cc = 0.;
double Ide_ref, Iqe_ref, id_err, iq_err, id_err_int, iq_err_int = 0.;
double Vde_fb, Vqe_fb, Vde_ff, Vqe_ff = 0.;
double Wc_I_5th=0.;
double Fc_I_5th=0.;
double La_I_5th = 0.;
double Lb_I_5th = 0.;
double Ide_5th_flt_n = 0.0;
double Iqe_5th_flt_n = 0.0;
double Ide_5th_n = 0.0;
double Iqe_5th_n = 0.0;
double Ide_5th_old_n = 0.0;
double Iqe_5th_old_n = 0.0;
double Ide_5th_err_n = 0.0;
double Iqe_5th_err_n = 0.0;
double Ki_CC_5th = 0.0;
double KP_CC_5th = 0.0;
double Vde_5th_integ_n = 0.0;
double Vde_5th_ref_fb_n = 0.0;
double Vqe_5th_integ_n = 0.0;
double Vqe_5th_ref_fb_n = 0.0;
double Vde_5th_ref_n = 0.0;
double Vqe_5th_ref_n = 0.0;
double Vds_5th_ref_n =0.0;
double Vqs_5th_ref_n =0.0;

double Wc_I_7th, La_I_7th, Lb_I_7th, Ide_7th_flt_p, Iqe_7th_flt_p, Ide_7th_p, Iqe_7th_p = 0.0;
double Ide_7th_old_p, Iqe_7th_old_p = 0.0;
double Ide_7th_err_p, Iqe_7th_err_p, Ki_CC_7th, KP_CC_7th, Vde_7th_integ_p, Vde_7th_ref_fb_p = 0.0;
double Vqe_7th_integ_p, Vqe_7th_ref_fb_p, Vde_7th_ref_p, Vqe_7th_ref_p = 0.0;
double Vds_7th_ref_p, Vqs_7th_ref_p = 0.0;


/* MakeTheta */
double Wr, theta, V_out_theta = 0.;
double V_out_region, V_out_region_old = 0;

/*DSOGI PLL*/
double Damping_K = 0.0001;
double Alpha = 0.0001;
double Alpha_prime = 0.0001;
double q_Alpha_prime = 0.0001;
double Alpha_prime_old = 0.0001;
double Alpha_error_1 = 0.0001;
double Alpha_error_2 = 0.0001;
double Alpha_error_3 = 0.0001;
double Alpha_error_4 = 0.0001;
double Alpha_error_5 = 0.0001;
double Alpha_plus = 0.0001;


double Beta = 0.0001;
double Beta_prime = 0.0001;
double q_Beta_prime = 0.0001;
double Beta_prime_old = 0.0001;
double Beta_error_1 = 0.0001;
double Beta_error_2 = 0.0001;
double Beta_error_3 = 0.0001;
double Beta_error_4 = 0.0001;
double Beta_error_5 = 0.0001;
double Beta_plus = 0.0001;
double Alpha_negative = 0.;
double Beta_negative = 0.;
double VA, VB, VC = 0.;

double Vas_APF, Vbs_APF, Vcs_APF = 0.;
double Vas_old, Vbs_old, Vcs_old = 0.;
double Vap, Vbp, Vcp = 0.;
double APF_W, KT, La_APF = 0.;
double Eap, Ebp, Ecp = 0.;
double Wde_real = 0.;
double theta_grid_real = 0.;




/* PLL */
double Vab_grid, Vbc_grid, Vca_grid = 0.;
double Vas_grid, Vbs_grid, Vcs_grid = 0.;
double Vds_grid, Vqs_grid, Vde_grid, Vqe_grid = 0.;
double theta_grid, theta_voltage = 0.;
double Em, Zeta, Wn, Wc, La, Lb = 0.;
double Vde_LPF, Vde_old, Vde_err = 0.;
double Kp_PLL, Ki_PLL, Vde_err_int, Wde_ref = 0.;
double W_rated, Wde_ref_fb = 0.;
double Grid_Angle_N_5th = 0.;
double Grid_Angle_P_7th = 0.;

/* Modulation */
double Vde_ref, Vqe_ref, Vds_ref, Vqs_ref = 0.;
double Vas_ref, Vbs_ref, Vcs_ref = 0;
double Van_ref, Vbn_ref, Vcn_ref = 0.;
double max, mid, min, Vsn = 0.;

/* normalization */
double San, Sbn, Scn = 0.;
int Sign_A, Sign_B, Sign_C = 0;

/* NeutralVoltageComp */
double I_ntr = 0.;
int Avr_cnt = 0;
double I_ntr_avr = 0;
double I_ntr_maf[11] = { 0, };
double del_Vdc, del_Vdc_err, del_Vdc_ref = 0.;
double Ki_nvc, Kp_nvc = 0.;
double Voffset_Pgain_ref, Voffset_IP_ref, Voffset_Igain_ref, Voffset_ref_fb, Voffset_ref = 0;

/* DCLink_VoltageControl*/

double DCLink_VoltageController_Ki = 0.;
double DCLink_VoltageController_Kp = 0.;
double DCLink_VoltageController_Ka = 0.;
double Iqe_feedback = 0.;
double Voltage_ref = 0.;
double DCLink_Voltage_ref = 0.;
double DCLink_Voltage_err = 0.; 
double DCLink_Voltage_err_int = -130.;
double DCLink_Voltage = 0.;

double DCLINK_Ki = 0.;
double DCLINK_Kp = 0.;
double I_DC_ref = 0.;
double I_qe_ref = 0.;
double TimeInterval;
double Slope;
double Y_Intercept;
double time = 0.0;
double Simulation_time = 0.0;

/* Switching_Ps */
int T1a, T1ca, T3a, T3ca, T2a, T2ca = 0;
int T1b, T1cb, T3b, T3cb, T2b, T2cb = 0;
int T1c, T1cc, T3c, T3cc, T2c, T2cc = 0;

/* Measure */
int state_A, state_B, state_C = 0;
double MI, MI_ref = 0.;

int i = 0;
double j = 0;
