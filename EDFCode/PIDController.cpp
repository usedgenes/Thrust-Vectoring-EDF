
PID::PID()
{
	kp = 0;
	ki = 0;
	kd = 0;
	err = 0;
	sum_err = 0;
	dif_err = 0;
	lastInput = 0;
	outmax = 400;
	outmin = -400;
}

PID::PID(float kp_, float ki_, float kd_)
{
	kp = kp_;
	ki = ki_;
	kd = kd_;

	err = 0;
	sum_err = 0;
	dif_err = 0;
	lastInput = 0;
	outmax = 400;
	outmin = -400;
}

float PID::update_pid_std(float setpoint, float input, float dt)
{
	err = setpoint-input;
	sum_err += err*ki*dt;
	dif_err = -kd/dt * (input - lastInput);
	output = kp*err + sum_err + dif_err;
	if (output > outmax) {
		sum_err = 0.0;
		output = outmax;
	}else if (output < outmin) {
		sum_err =0.0;
		output = outmin;
	}
	lastInput = input;
	return output;

}


void PID::reset()
{
	sum_err = 0;
	dif_err = 0;
	lastInput = 0;
}


void PID::set_Kpid(float KP, float KI, float KD)
{
	kp = KP;
	ki = KI;
	kd = KD;
}
void PID::set_windup_bounds(float min, float max)
{
	outmax = max;
	outmin = min;
}


	