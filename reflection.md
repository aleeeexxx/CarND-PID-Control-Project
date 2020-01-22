# PID_controller
## Introduction
This project is one project of Udacity Self-driving Car Nanodegree. A PID controller was made for a car in simulator to drive along the road. In this project, for now, only steering angle was controlled leaving the throttle unchanged at 0.3. The steering angle is between -1 to 1. The hyperparameters are tuned by twiddling, which is described in the member function "Optimal_PID".
## Describe the effect each of the P, I, D components.
### P component
In PID, P stands for "proportional". In this project, this means a part of my controller's output(steering angle) is proportional to the error(CTE), which may lead the car to steer a larger angle when the error is larger. This component could quickly react to the large error and eliminate it. However, it can never bring the car to the perfect condition with 0 error alone. Because when the car approaches 0 error position, the steering angle would not be 0. That's also the reason why the D component exists. 
### D component
D is "differential" that outputs a steering angle proportional to the differential of the error. Since it can detect the changing rate of the error, it can solve the overshot issue very well and reduce the oscillating of the car.
### I component
I is "integral", taking accumulative error in consideration to eliminate the bias. It outputs a steering angle changing as the integral of the CTE.

## Describe how the final hyperparameters were chosen.
The parameters at initialization were chosen randomly at first, after several tries, 0.13, 0.001 and 3 works for P, I, D as initial guess respectively. Then those parameters were tuned in the function Optimal_PID and finally became 0.160308, 0.00117479 and 3.03199. With these as the parameters of the PID controller, the car can stay on track for the whole loop. 
