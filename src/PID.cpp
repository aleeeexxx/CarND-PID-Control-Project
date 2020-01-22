#include "PID.h"
#include <limits>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <array>

using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
	/**
	 * TODO: Initialize PID coefficients (and errors, if needed)
	 */
    this->need_optimize = true;

	 // coefficients
	this->Kp = Kp_;
	this->Ki = Ki_;
	this->Kd = Kd_;

	// error
	this->p_error = this->i_error = this->d_error = this->total_error = 0;

}

void PID::UpdateError(double cte) {
	/**
	 * TODO: Update PID errors based on cte.
	 */
	this->d_error = cte - this->p_error;
	this->p_error = cte;
	this->i_error += cte;

}

double PID::TotalError() {
	/**
	 * TODO: Calculate and return the total error
	 */
	this->total_error = this->p_error*this->Kp + this->i_error*this->Ki + this->d_error*this->Kd;
	return this->total_error;
}

void PID::Optimal_PID(double tol, double cte) {
  if (this->need_optimize){
    
	double Kp = this->Kp;
	double Ki = this->Ki;
	double Kd = this->Kd;
	double p[] = { Kp, Ki, Kd };
	double dp[] = { 0.1*Kp, 0.1*Ki, 0.1*Kd};
	
    
	double sum_of_dp = std::numeric_limits<double>::max();
   
	int step = 0;
	double best_error = std::numeric_limits<double>::max();

	while (sum_of_dp > tol) {
		
      //reset sum of dp
      sum_of_dp = 0;
      for (int i = 0; i<3 ; i++) {
			p[i] += dp[i];
            
            // update PID para
            this->Kp = p[0];
            this->Ki = p[1];
            this->Kd = p[2];
			this->UpdateError(cte);
			this->total_error = this->TotalError();

			if (this->total_error < best_error) {
				best_error = this->total_error;
				dp[i] *= 1.1;
			}
			else {
				p[i] -= 2 * dp[i];
                // update PID para
                this->Kp = p[0];
                this->Ki = p[1];
                this->Kd = p[2];
				this->UpdateError(this->total_error);
				this->total_error = this->TotalError();

				if (this->total_error < best_error) {
					best_error = this->total_error;
					dp[i] *= 1.1;
				}
				else {
					p[i] += dp[i];
					dp[i] *= 0.9;
				}
			}
			
			step += 1;
          
          
            sum_of_dp += dp[i];
            
          
            
           
		}
		
     
	}
	this->Kp = p[0];
	this->Ki = p[1];
	this->Kd = p[2];
    this->need_optimize = false;
    this->p_error = this->i_error = this->d_error = this->total_error = 0;
    
  
  }

}
