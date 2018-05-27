#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;

    // Twiddle
    enable_twiddle = false;
    // select adjustment width
    dp = { 0.05*Kp, 
    	   0.05*Kd,
    	   0.05*Ki};
    step = 1;
    param_index = 2;
    per_lap_steps = 600;
    lap_error = 0;
    best_error = std::numeric_limits<double>::max();
    try_positive = false; 
    try_negative = false;
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    lap_error += pow(cte,2);

    // every lap
    if (enable_twiddle && step % (per_lap_steps) == 0)
    {
    	if(step ==  per_lap_steps)
    	{
    		cout << "reference lap error: " << lap_error << endl;
    	}
    	else
    	{
	        cout << "step: " << step << endl;
	        cout << "total error: " << lap_error << endl;
	        cout << "best error: " << best_error << endl;
    	}
        if (lap_error < best_error)
        {
        	cout << "Improve!" << endl;
            best_error = lap_error;
            // skip fist reference lap
            if (step !=  per_lap_steps)
            {
                dp[param_index] *= 1.1;
            }
            // change to next parameter
            param_index = (param_index + 1) % 3;
            try_positive = false;
            try_negative = false;
        }
        else
        {
        	cout << "No improvement" << endl;
        }

        if (!try_positive && !try_negative)
        {
        	cout << "new parameters" << endl;
            // try increase dp[i] to params[i]
            UpdateParameter(param_index, dp[param_index]);
            try_positive = true;
        }
        else if (try_positive && !try_negative)
        {
        	cout << "new parameters" << endl;
            // try decrease dp[i] from params[i]
            UpdateParameter(param_index, -2 * dp[param_index]);     
            try_negative = true;
        }
        else
        {
            // neither increase or decrease help
            // reset parameter
            cout << "reset parameters" << endl;
            UpdateParameter(param_index, dp[param_index]);
            // reduce adjustment by mulitply 0.9    
            dp[param_index] *= 0.9;
            // change to next parameter
            param_index = (param_index + 1) % 3;
            try_positive = false;
            try_negative = false;
        }
        lap_error = 0;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
    }
    // cout << "step: " << step << endl;
    step++;
}

double PID::TotalError() {
	double total_error = - Kp * p_error 
                		 - Kd * d_error 
              			 - Ki * i_error;
    return total_error;
}

void PID::UpdateParameter(int index, double value) {
    if (index == 0)
    {
        Kp += value;
    }
    else if (index == 1)
    {
        Kd += value;
    }
    else if (index == 2)
    {
        Ki += value;
    }
    else {
        std::cout << "UpdateParameter: index out of bounds";
    }
}
