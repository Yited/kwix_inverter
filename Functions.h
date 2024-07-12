//정변환
void abc_to_dqs(double input_A, double input_B, double input_C, double* output_DS, double* output_QS)
{
	(*output_DS) = (2. * input_A - input_B - input_C) / 3.;
	(*output_QS) = (input_B - input_C) / sqrt(3.);
}
void dqs_to_dqe(double input_DS, double input_QS, double input_theta, double* output_DR, double* output_QR)
{
	*output_DR = cos(input_theta) * input_DS + sin(input_theta) * input_QS;
	*output_QR = -sin(input_theta) * input_DS + cos(input_theta) * input_QS;
}
void abc_to_dqe(double input_A, double input_B, double input_C, double input_theta, double* output_DR, double* output_QR)
{
	*output_DR = 2. / 3. * (input_A * cos(input_theta) + input_B * cos(input_theta - 2. * PI / 3.) + input_C * cos(input_theta - 4. * PI / 3.));
	*output_QR = 2. / 3. * ((-1.) * input_A * sin(input_theta) + (-1.) * input_B * sin(input_theta - 2. * PI / 3.) + (-1.) * input_C * sin(input_theta - 4. * PI / 3.));

}

void abc_to_dqze(double input_A, double input_B, double input_C, double input_theta, double* output_DR, double* output_QR, double* output_ZR)
{
	*output_DR = 2. / 3. * (input_A * cos(input_theta) + input_B * cos(input_theta - 2. * PI / 3.) + input_C * cos(input_theta - 4. * PI / 3.));
	*output_QR = 2. / 3. * ((-1.) * input_A * sin(input_theta) + (-1.) * input_B * sin(input_theta - 2. * PI / 3.) + (-1.) * input_C * sin(input_theta - 4. * PI / 3.));
	*output_ZR = (input_A + input_B + input_C) / 3.;
}

//역변환
void dqe_to_dqs(double input_DR, double input_QR, double input_theta, double* output_DS, double* output_QS)
{
	*output_DS = input_DR * cos(input_theta) - input_QR * sin(input_theta);
	*output_QS = input_DR * sin(input_theta) + input_QR * cos(input_theta);
}
void dqs_to_abc(double input_DS, double input_QS, double* output_A, double* output_B, double* output_C)
{
	*output_A = input_DS;
	*output_B = -0.5 * input_DS + sqrt(3.) / 2. * input_QS;
	*output_C = -0.5 * input_DS - sqrt(3.) / 2. * input_QS;
}
void dqe_to_abc(double input_DR, double input_QR, double input_theta, double* output_A, double* output_B, double* output_C)
{
	*output_A = cos(input_theta) * input_DR - sin(input_theta) * input_QR;
	*output_B = cos(input_theta - 2. * PI / 3.) * input_DR - sin(input_theta - 2. * PI / 3.) * input_QR;
	*output_C = cos(input_theta + 2. * PI / 3.) * input_DR - sin(input_theta + 2. * PI / 3.) * input_QR;
}

void dqze_to_abc(double input_DR, double input_QR, double input_ZR, double input_theta, double* output_A, double* output_B, double* output_C)
{
	*output_A = cos(input_theta) * input_DR - sin(input_theta) * input_QR + input_ZR * 0.5;
	*output_B = cos(input_theta - 2. * PI / 3.) * input_DR - sin(input_theta - 2. * PI / 3.) * input_QR + input_ZR * 0.5;
	*output_C = cos(input_theta + 2. * PI / 3.) * input_DR - sin(input_theta + 2. * PI / 3.) * input_QR + input_ZR * 0.5;
}


void Max_Mid_Min(double* input_A, double* input_B, double* input_C, double* max, double* mid, double* min)
{
	if (*input_A <= *input_B)
	{
		if (*input_B <= *input_C) { *max = *input_C, * mid = *input_B, * min = *input_A; }
		else if (*input_C <= *input_B)
		{
			*max = *input_B;
			if (*input_C <= *input_A) { *mid = *input_A, * min = *input_C; }
			else { *mid = *input_C, * min = *input_A; }
		}

	}

	else if (*input_B <= *input_A)
	{
		if (*input_A <= *input_C) { *max = *input_C, * mid = *input_A, * min = *input_B; }
		else if (*input_C <= *input_A)
		{
			*max = *input_A;
			if (*input_C <= *input_B) { *mid = *input_B, * min = *input_C; }
			else { *mid = *input_C, * min = *input_B; }
		}
	}
}


void Neutral_Current(int region, int region_old, double* Ia, double* Ib, double* Ic, double* I_ntr) {
	
	if (region == 11 || region == 0) *I_ntr = -*Ia;
	else if (region == 1 || region == 2)	*I_ntr = *Ic;
	else if (region == 3 || region == 4)	*I_ntr = -*Ib;
	else if (region == 5 || region == 6)	*I_ntr = *Ia;
	else if (region == 7 || region == 8)	*I_ntr = -*Ic;
	else if (region == 9 || region == 10)	*I_ntr = *Ib;
}



double LinearFunction(double x, double a, double b)
{
	return a * x + b;
}



void Modulation();

void normalization();

void Measure();

void ADC_Sensing();

void CurrentControl();

void PLL();
