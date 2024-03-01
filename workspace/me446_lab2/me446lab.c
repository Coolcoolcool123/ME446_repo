#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.45; //-0.37;
float offset_Enc3_rad = 0.23; //0.27;


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;
float something = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];
float theta2array[100];

long arrayindex = 0;
int UARTprint = 0;

//print variables
float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// Assign length  values
float L1 = 0.254;
float L2 = 0.254;
float L3 = 0.254;

//values that we need to print (part 2 of lab1)
float x = 0;
float y = 0;
float z = 0;

float theta1 = 0;
float theta2 = 0;
float theta3 = 0;

float theta1IK_DH = 0;
float theta2IK_DH = 0;
float theta3IK_DH = 0;

float theta1IK_motor = 0;
float theta2IK_motor = 0;
float theta3IK_motor = 0;

//lab 2 global variables
float theta_d1 = 0;
float theta_d2 = 0;
float theta_d3 = 0;

float Kp1 = 12; //12 
float Kp2 = 100; //100
float Kp3 = 120; //120
float KD1 = 0.9; // 0.9
float KD2 = 4; //4
float KD3 = 6; //6

//variables for 2nd method of filtering velocity
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

float thresh = 0.01;
float ek_1 = 0;
float ek_1old= 0;
float ek_2 = 0;
float ek_2old = 0;
float ek_3 = 0;
float ek_3old = 0;

float IK1 = 0;
float IK1_old = 0;
float IK2 = 0;
float IK2_old = 0;
float IK3 = 0;
float IK3_old = 0;

float KI1 = 0.2; // 0.2
float KI2 = 4; //4
float KI3 = 6; //6

float dt = 0.001;

float a0 = 0;
float a1 = 0;
float a2 = 3/2;
float a3 = -1;
float b0 = -2;
float b1 = 6;
float b2 = -9/2;
float b3 = 1;
float t_in[100];

struct thetas {
    float Theta[100];
    float Theta_dot[100];
    float Theta_ddot[100];
};

void mains_code(void);

//
// Main
//
void main(void)
{
    mains_code();
}



struct thetas theta(float time) {
    struct thetas values;
    t_in[0] = 0;
    int i;
    for (i = 1; i < 100; i++) {
        t_in[i] = i*time/100 + t_in[i-1];
    }
    int ii;
    for (ii = 0; ii<100; ii++) {
        if(t_in[ii] <= 1) {
            values.Theta[ii] = a0 + a1 * t_in[ii] + a2 * t_in[ii] * t_in[ii] + a3 * t_in[ii] * t_in[ii] * t_in[ii];
            values.Theta_dot[ii] = a1 + 2 * a2 * t_in[ii] + 3 * a3 * t_in[ii] * t_in[ii];
             values.Theta_ddot[ii] = 2*a2+ 6*a3*t_in[ii];
        }
        else if(t_in[ii] > 1 && t_in[ii] <= 2) {
            values.Theta[ii] = b0 + b1 * t_in[ii] + b2 * t_in[ii] * t_in[ii] + b3 * t_in[ii] * t_in[ii] * t_in[ii];;
            values.Theta_dot[ii] = b1 + 2 * b2 * t_in[ii] + 3 * b3 * t_in[ii] * t_in[ii];
            values.Theta_ddot[ii] = 2*b2+ 6*b3*t_in[ii];
        } else {
            values.Theta[ii] = 0;
            values.Theta_dot[ii] = 0;
            values.Theta_ddot[ii] = 0;
        }
    }

    return values;
}

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


    *tau1 = 0;
    *tau2 = 0;
    *tau3 = -1;

    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    //Lab 2 part 5 
    if ((mycount%2000)<1000) {

        theta_d1 = 0;
        theta_d2 = 0;
        theta_d3 = 0;
        
    } else {
            
       theta_d1 = PI/6;
       theta_d2 = PI/6;
       theta_d3 = PI/6;
       
   }
    //implementing the 2nd method of filtering velocity
   Omega1 = (theta1motor-Theta1_old)/0.001;
   Omega1 = (Omega1+Omega1_old1 + Omega1_old2)/3.0;
   
   Theta1_old = theta1motor;
   
   Omega1_old2 = Omega1_old1;
   Omega1_old1 = Omega1;
   
   Omega2 = (theta2motor-Theta2_old)/0.001;
   Omega2 = (Omega2+Omega2_old1 + Omega2_old2)/3.0;
   
   Theta2_old = theta2motor;
   
   Omega2_old2 = Omega2_old1;
   Omega2_old1 = Omega2;
   
   Omega3 = (theta3motor-Theta3_old)/0.001;
   Omega3 = (Omega3+Omega3_old1 + Omega3_old2)/3.0;
   
   Theta3_old = theta3motor;
   
   Omega3_old2 = Omega3_old1;
   Omega3_old1 = Omega3;
   
   //implementing the integral control
   //solving for errors 
   ek_1old = ek_1;
   ek_1 = theta_d1 - theta1motor;
   
   ek_2old = ek_2;
   ek_2 = theta_d2 - theta1motor;
   
   ek_3old = ek_3;
   ek_3 = theta_d3 - theta1motor;
   
   //solving for intregral tracking error
   IK1_old = IK1;

   
   IK2_old = IK2;

   IK3_old = IK3;

   
   if(fabs(ek_1)>thresh) {
       IK1 = 0;
   } if(fabs(*tau1)>5) {
       IK1 = 0;
   } else {
       IK1 = IK1_old + ((ek_1+ek_1old)/2*dt);
  }
   
   if(fabs(ek_2)>thresh) {
      IK2 = 0;
   } if(fabs(*tau1)>5) {
       IK2 = 0;
   } else {
       IK2 = IK2_old + ((ek_2+ek_2old)/2*dt);
  }
   
   if(fabs(ek_3)>thresh) {
       IK3 = 0;
   } if(fabs(*tau1)>5) {
       IK3 = 0;
   } else {
       IK3 = IK3_old + ((ek_3+ek_3old)/2*dt);
       
  }
   
   
    //equations to convert measured thetas in terms of theta motors
    theta1 = theta1motor;
    theta2 = theta2motor - PI/2;
    theta3 = -theta2motor + theta3motor + PI/2;

    //Foward Kinematics equations in terms of thetamotors (part1 Lab1, calcuations done in MATLAB)
    x = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    y = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    z = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;

    //measured thetas calcuated from geometric Inverse Kinematics (part2 of Lab 1)
    theta1IK_DH = atan(y/x);
    theta2IK_DH = -atan( (z-L1)/sqrt(pow(x,2)+pow(y,2)) ) - acos( (pow(L2,2)+pow(x,2)+pow(y,2)+pow((z-L1),2)-pow(L3,2)) / (2*L2*sqrt(pow(x,2)+pow(y,2)+pow((z-L1),2))) );
    theta3IK_DH =  PI - acos((pow(L2,2)+pow(L3,2)-(pow(x,2)+pow(y,2)+pow(z-L1,2)))/(2*L2*L3));

    //converting IK thetas to theta motors instead
    theta1IK_motor = theta1IK_DH;
    theta2IK_motor = theta2IK_DH + PI/2;
    theta3IK_motor = theta3IK_DH + theta2IK_motor - PI/2;

    
    *tau1 = Kp1*(theta_d1-theta1motor)-KD1*Omega1 + IK1*KI1;
    *tau2 = Kp2*(theta_d2-theta2motor)-KD2*Omega2 + IK2*KI2;
    *tau3 = Kp3*(theta_d3-theta3motor)-KD3*Omega3 + IK3*KI3;
    
    
   if(*tau1>5) {
       *tau1 =5;
   } if (*tau1<-5){
       *tau1 = -5;
  }
   
   if(*tau2>5) {
       *tau2 =5;
   } if (*tau2<-5){
       *tau2 = -5;
   }
   
   if(*tau3>5) {
       *tau3 =5;
   } if (*tau3<-5){
       *tau3 = -5;
   }
    
    //sets theta motors to printable variables. print variables are global variables
    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = theta_d3;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "Motor Thetas:(%.2f, %.2f, %.2f), FK (xyz): (%.2f, %.2f, %.2f)  \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI, x, y,z);
        serial_printf(&SerialA,"IK Motor Thetas: (%.2f, %.2f, %.2f), DH Thetas:  (%.2f, %.2f, %.2f)  \n\r",theta1IK_motor*180/PI,theta2IK_motor*180/PI,theta3IK_motor*180/PI,theta1IK_DH*180/PI,theta2IK_DH*180/PI,theta3IK_DH*180/PI);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

