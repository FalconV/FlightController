/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 Contact information
 -------------------
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
#include "common.h"

class Kalman {
  public:
    Kalman();
    
    //angle in degrees, rate in degrees/sec, delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);
    
    void setAngle(float angle);//used to set the starting angle
    float getRate(); //return the unbiased rate
    
    //these are used to tune the Kalman filter
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);
    
    float getQangle();
    float getQbias();
    float getRmeasure();
    
  private:
    //Kalman filter variables
    float Q_angle; //process noise variance for the accelerometer
    float Q_bias; //process noise variance for the gyro bias
    float R_measure; //measurement noise variance
    
    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

Kalman::Kalman() {
    Q_angle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.03;
    

    angle = 0.0; // Reset the angle
    bias = 0.0; // Reset bias
    rate = 0.0;

    P[0][0] = 0.0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
//    hal.console->printf_P(PSTR("in constructor: angle=%.2f bias=%.2f rate=%.2f\n"),angle, bias, rate);
  
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float Kalman::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle(float newAngle, float newRate, float dt){
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
    
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
//    bias = 0.0;
//    rate = 0.0;
//    angle = 0.0;
    rate = newRate - bias;
    angle = angle + (dt * rate);
//    hal.console->printf_P(PSTR("rate=%.2f angle=%.2f"),rate, angle);
    
    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
//    hal.console->printf_P(PSTR("(bias=%.2f rate=%.2f newRate=%.2f newAngle=%.2f dt=%.2f y=%.2f S=%.2f K[0]=%.2f K[0]*y=%.2f angle=%.2f)"),bias,rate,newRate,newAngle,dt,y,S,K[0],K[0]*y,angle);
    
    /* Step 6 */
    this->angle += K[0] * y;
    this->bias += K[1] * y;
    
    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
//    hal.console->printf_P(PSTR("(bias=%.2f rate=%.2f newRate=%.2f newAngle=%.2f dt=%.2f y=%.2f S=%.2f K[0]=%.2f K[0]*y=%.2f angle=%.2f)"),bias,rate,newRate,newAngle,dt,y,S,K[0],K[0]*y,angle);
    
    return angle;
  
}
    
    
