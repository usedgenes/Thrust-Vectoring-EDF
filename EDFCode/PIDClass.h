#ifndef pid_controller
#define pid_controller


#include <iostream>
#include <stdlib.h>
#include <fstream>


class PID {

	private:
		float kp;
		float ki;
		float kd;

		float err;
		float sum_err;
		float dif_err;
		float lastInput;
		float max;
		float min;
		float output;

	public:
		PID();
		PID(float,float,float);
		float update_pid_std(float setpt, float input, float dt);
		void setKpid(float, float, float);
		void set_windup_bounds(float, float);
		void reset();
		float setpoint;

};


#endif