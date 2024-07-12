//입-출력
void Input(double* in)
{
	Vdc1 = in[0];
	Vdc2 = in[1];

	Vtri1_L = in[2];
	Vtri2_L = in[3];

	Ia = in[4];
	Ib = in[5];
	Ic = in[6];

	Vab_grid = in[7];
	Vbc_grid = in[8];
	Vca_grid = in[9];
}
void Output(double* out)
{
	out[0] = San;
	out[1] = Sbn;
	out[2] = Scn;

	out[3] = T1a;
	out[4] = T3a;
	
	out[5] = T1b;
	out[6] = T3b;
	
	out[7] = T1c;
	out[8] = T3c;


	out[9]  = Ide_5th_err_n;
	out[10] = Iqe_5th_err_n;

	out[11] = Ide_7th_err_p;
	out[12] = Iqe_7th_err_p;

	out[13] = Iqe;
	out[14] = Iqe_ref;


	out[15] = Ide;
	out[16] = Ide_ref;

	out[17] = Ide_5th_flt_n;
	out[18] = Iqe_5th_flt_n;


	out[19] = Ide_7th_flt_p;
	out[20] = Iqe_7th_flt_p;

	out[21] = Ide_7th_p;
	out[22] = Iqe_7th_p;

	out[23] = DCLink_Voltage_err_int;
	out[24] = DCLink_Voltage_ref;
	
}
