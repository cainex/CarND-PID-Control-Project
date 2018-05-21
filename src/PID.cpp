#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    K[p] = Kp;
    K[i] = Ki;
    K[d] = Kd;

    dK[p] = 0.005;
    dK[i] = 0.00002;
    dK[d] = 0.01;

    twiddleInterval = 1500;
    twiddleStep = 0;
    twiddleThreshold = 0.00001;
    enableTwiddle = false;
    twiddleBestError = 0.0;
    twiddleTotalError = 0.0;
    twiddleState = twiddleAdd;
    twiddleParam = 0;

    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    if (enableTwiddle && dK[p]+dK[i]+dK[p] > twiddleThreshold) {
        // increase total error
        twiddleTotalError += pow(cte,2);

        if (twiddleStep > twiddleInterval) {
            if (twiddleStep%twiddleInterval == 0) {

                cout << "---Perform Twiddle on Param [" << twiddleParam << "] ---" << endl;
                cout << "BestError:" << twiddleBestError << "  TotalError:" << twiddleTotalError << endl;
                // perform twiddle for every 100 steps
                switch(twiddleState){
                    case twiddleAdd: 
                        K[twiddleParam] += dK[twiddleParam];
                        twiddleState = twiddleSubtract;
                    break;
                    case twiddleSubtract:
                        if (twiddleTotalError < twiddleBestError) {
                            cout << "   Twiddle Update" << endl;
                            cout << "   >>> Kp[" << K[p] << "] Ki[" << K[i] << "] Kd[" << K[d] << "]" << endl;
                            twiddleBestError = twiddleTotalError;
                            dK[twiddleParam] *= 1.1;
                            twiddleParam = (twiddleParam + 1)%3;
                            twiddleState = twiddleAdd;
                        } else {
                            K[twiddleParam] -= 2*dK[twiddleParam];
                            twiddleState = twiddleAdjust;
                        }
                    break;
                    case twiddleAdjust:
                        if (twiddleTotalError < twiddleBestError) {
                            cout << "   Twiddle Update" << endl;
                            cout << "   >>> Kp[" << K[p] << "] Ki[" << K[i] << "] Kd[" << K[d] << "]" << endl;
                            twiddleBestError = twiddleTotalError;
                            dK[twiddleParam] *= 1.1;
                            twiddleParam = (twiddleParam + 1)%3;
                            twiddleState = twiddleAdd;
                        } else {
                            K[twiddleParam] += dK[twiddleParam];
                            dK[twiddleParam] *= 0.9;
                            twiddleParam = (twiddleParam + 1)%3;
                            twiddleState = twiddleAdd;
                        }
                    break;
                }
                twiddleTotalError = 0.0;
                cout << "Kp[" << K[p] << "] Ki[" << K[i] << "] Kd[" << K[d] << "]" << endl;
                cout << "dKp[" << dK[p] << "] dKi[" << dK[i] << "] dKd[" << dK[d] << "]" << endl;
            }
            
        }
        else if (twiddleStep%(twiddleInterval) == 0 && twiddleStep != 0) {
            twiddleBestError = twiddleTotalError;
            twiddleTotalError = 0.0;
            cout << "Initial Best Error: " << twiddleBestError << endl;
            cout << "---Perform Initial Twiddle on Param [" << twiddleParam << "] ---" << endl;
            K[twiddleParam] += dK[twiddleParam];
            twiddleState = twiddleSubtract;
        }

        twiddleStep++;
    } else {
//        cout << "Kp[" << K[p] << "] Ki[" << K[i] << "] Kd[" << K[d] << "]" << endl;
    }
}

double PID::TotalError() {
    return -K[p]*p_error - K[d]*d_error - K[i]*i_error;
}

