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
float KI1 = 0.2; // 0.2
float KI2 = 4; //4
float KI3 = 6; //6

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


float dt = 0.001;

//
float a0 = 0;
float a1 = 0;
float a2 = 1.5;
float a3 = -1;
float b0 = -2;
float b1 = 6;
float b2 = -4.5;
float b3 = 1;


float Theta1 = 0;
float Theta2 = 0;
float Theta3 = 0;
float Theta_dot1 = 0;
float Theta_dot2 = 0;
float Theta_dot3 = 0;
float Theta_ddot1 = 0;
float Theta_ddot2 = 0;
float Theta_ddot3 = 0;

float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

float x1 = 0.18;
float x2 = 0.16;
float x3 = 0.25;
float y1 = -0.08;
float y2 = 0.11;
float y3 = -0.10;
float z1 = 0.63;
float z2 = 0.63;
float z3 = 0.20;


void mains_code(void);

//
// Main
//
void main(void)
{
    mains_code();
}

void custom(float time){
    x = .30+.1*cos(PI*time);
    y = .1*sin(PI*time);
    z = .35;

    //taking inverse kinematic code from lab 1
    theta1IK_DH = atan(y/x);
    theta2IK_DH = -atan( (z-L1)/sqrt(pow(x,2)+pow(y,2)) ) - acos((pow(L2,2)+pow(x,2)+pow(y,2)+pow((z-L1),2)-pow(L3,2)) / (2*L2*sqrt(pow(x,2)+pow(y,2)+pow((z-L1),2))) );
    theta3IK_DH =  PI - acos((pow(L2,2)+pow(L3,2)-(pow(x,2)+pow(y,2)+pow(z-L1,2)))/(2*L2*L3));

    Theta1 = theta1IK_DH;
    Theta2 = theta2IK_DH + PI/2;
    Theta3 = theta3IK_DH + Theta2 - PI/2;

    }

//void custom(float time) {
//    if (time < 1) {
//        x = x1 + time*((x2-x1)/0.001);
//        y = y1 + time*((y2-y1)/0.001);
//        z = z1 + time*((z2-z1)/0.001);
//        //taking inverse kinematic code from lab 1
//        theta1IK_DH = atan(y/x);
//        theta2IK_DH = -atan( (z-L1)/sqrt(pow(x,2)+pow(y,2)) ) - acos((pow(L2,2)+pow(x,2)+pow(y,2)+pow((z-L1),2)-pow(L3,2)) / (2*L2*sqrt(pow(x,2)+pow(y,2)+pow((z-L1),2))) );
//        theta3IK_DH =  PI - acos((pow(L2,2)+pow(L3,2)-(pow(x,2)+pow(y,2)+pow(z-L1,2)))/(2*L2*L3));
//
//        //converting IK thetas to theta motors instead
//        Theta1 = theta1IK_DH;
//        Theta2 = theta2IK_DH + PI/2;
//        Theta3 = theta3IK_DH + theta2IK_motor - PI/2;
//    } else if(time < 2) {
//        x = x2 + time*((x3-x2)/0.001);
//        y = y2 + time*((y3-y2)/0.001);
//        z = z2 + time*((z3-z2)/0.001);
//        //taking inverse kinematic code from lab 1
//        theta1IK_DH = atan(y/x);
//        theta2IK_DH = -atan( (z-L1)/sqrt(pow(x,2)+pow(y,2)) ) - acos((pow(L2,2)+pow(x,2)+pow(y,2)+pow((z-L1),2)-pow(L3,2)) / (2*L2*sqrt(pow(x,2)+pow(y,2)+pow((z-L1),2))) );
//        theta3IK_DH =  PI - acos((pow(L2,2)+pow(L3,2)-(pow(x,2)+pow(y,2)+pow(z-L1,2)))/(2*L2*L3));
//        //converting IK thetas to theta motors instead
//        Theta1 = theta1IK_DH;
//        Theta2 = theta2IK_DH + PI/2;
//        Theta3 = theta3IK_DH + theta2IK_motor - PI/2;
//    } else if (time < 3) {
//        x = x3 + time*((x1-x3)/0.001);
//        y = y3 + time*((y1-y3)/0.001);
//        z = z3 + time*((z1-z3)/0.001);
//        //taking inverse kinematic code from lab 1
//        theta1IK_DH = atan(y/x);
//        theta2IK_DH = -atan( (z-L1)/sqrt(pow(x,2)+pow(y,2)) ) - acos((pow(L2,2)+pow(x,2)+pow(y,2)+pow((z-L1),2)-pow(L3,2)) / (2*L2*sqrt(pow(x,2)+pow(y,2)+pow((z-L1),2))) );
//        theta3IK_DH =  PI - acos((pow(L2,2)+pow(L3,2)-(pow(x,2)+pow(y,2)+pow(z-L1,2)))/(2*L2*L3));
//        //converting IK thetas to theta motors instead
//        Theta1 = theta1IK_DH;
//        Theta2 = theta2IK_DH + PI/2;
//        Theta3 = theta3IK_DH + theta2IK_motor - PI/2;
//    }
//}

//implementing the cubic polynomial trajectory
void theta(float time) {
    if (time < 1) {
        Theta1 = a0 + a1*time +a2*pow(time,2)+a3*pow(time,3);
        Theta2 = Theta1;
        Theta3 = Theta1;
        Theta_dot1 = a1 + 2 *a2*time+ 3*a3*pow(time,2);
        Theta_dot2 = Theta_dot1;
        Theta_dot3 = Theta_dot1;
        Theta_ddot1 = 2*a2+6*a3*time;
        Theta_ddot2 = Theta_ddot1;
        Theta_ddot3 = Theta_ddot1;

    } else if (time < 2) {
        Theta1 = b0 + b1*time +b2*pow(time,2)+b3*pow(time,3);
        Theta2 = Theta1;
        Theta3 = Theta1;
        Theta_dot1 = b1 + 2 *b2*time + 3*b3*pow(time,2);
        Theta_dot2 = Theta_dot1;
        Theta_dot3 = Theta_dot1;
        Theta_ddot1 = 2*b2+6*b3*time;
        Theta_ddot2 = Theta_ddot1;
        Theta_ddot3 = Theta_ddot1;
//    } else {
//        Theta1 = 0;
//        Theta2 = 0;
//        Theta3 = 0;
//        Theta_dot1 = 0;
//        Theta_dot2 = 0;
//        Theta_dot3 = 0;
//        Theta_ddot1 = 0;
//        Theta_ddot2 = 0;
//        Theta_ddot3 = 0;
    }
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

//    //Lab 2 part 5
//    if ((mycount%2000)<1000) {
//
//        theta_d1 = 0;
//        theta_d2 = 0;
//        theta_d3 = 0;
//
//    } else {
//
//       theta_d1 = PI/6;
//       theta_d2 = PI/6;
//       theta_d3 = PI/6;
//
//   }

//    theta((mycount % 2000)/1000.0);
    custom((mycount % 4000)/1000.0);
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
   
   
//    //equations to convert measured thetas in terms of theta motors
//    theta1 = theta1motor;
//    theta2 = theta2motor - PI/2;
//    theta3 = -theta2motor + theta3motor + PI/2;
//
//    //Foward Kinematics equations in terms of thetamotors (part1 Lab1, calcuations done in MATLAB)
//    x = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
//    y = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
//    z = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;
//
//    //measured thetas calcuated from geometric Inverse Kinematics (part2 of Lab 1)
//    theta1IK_DH = atan(y/x);
//    theta2IK_DH = -atan( (z-L1)/sqrt(pow(x,2)+pow(y,2)) ) - acos( (pow(L2,2)+pow(x,2)+pow(y,2)+pow((z-L1),2)-pow(L3,2)) / (2*L2*sqrt(pow(x,2)+pow(y,2)+pow((z-L1),2))) );
//    theta3IK_DH =  PI - acos((pow(L2,2)+pow(L3,2)-(pow(x,2)+pow(y,2)+pow(z-L1,2)))/(2*L2*L3));
//
//    //converting IK thetas to theta motors instead
//    theta1IK_motor = theta1IK_DH;
//    theta2IK_motor = theta2IK_DH + PI/2;
//    theta3IK_motor = theta3IK_DH + theta2IK_motor - PI/2;

    
    *tau1 = Kp1*(Theta1-theta1motor)-KD1*Omega1 + IK1*KI1;
    *tau2 = Kp2*(Theta2-theta2motor)-KD2*Omega2 + IK2*KI2;
    *tau3 = Kp3*(Theta3-theta3motor)-KD3*Omega3 + IK3*KI3;
//
//    *tau1 = J1*Theta_ddot1+Kp1*(Theta1-theta1motor)+ KI1*IK1+KD1*(Theta_dot1-Omega1);
//    *tau2 = J2*Theta_ddot2+Kp2*(Theta2-theta2motor)+ KI2*IK2+KD2*(Theta_dot2-Omega2);
//    *tau3 = J3*Theta_ddot3+Kp3*(Theta3-theta3motor)+ KI3*IK3+KD3*(Theta_dot3-Omega3);

    
    
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


    //implementing the integral control
    //solving for errors
    ek_1old = ek_1;
    ek_1 = Theta1 - theta1motor;
    ek_2old = ek_2;
    ek_2 = Theta2 - theta2motor;
    ek_3old = ek_3;
    ek_3 = Theta3 - theta3motor;

    //solving for intregral tracking error
    IK1_old = IK1;
    IK2_old = IK2;
    IK3_old = IK3;

    Simulink_PlotVar1 = x;
    Simulink_PlotVar2 = y;
    Simulink_PlotVar3 = z;
    Simulink_PlotVar4 = Theta1;

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

