#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
//the offset values have been adjusted in Lab 1, 1.2
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

//setting global variables that we would need to print in lab1 part 2
//the variables for the position of the end effector
float x = 0;
float y = 0;
float z = 0;

//variables for the joint angles.
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;

//thetas that were calculated with inverse kinematics using DH parameters 
float theta1IK_DH = 0;
float theta2IK_DH = 0;
float theta3IK_DH = 0;

//the thetas found using inverse kinematics converted to angles of the motors 
float theta1IK_motor = 0;
float theta2IK_motor = 0;
float theta3IK_motor = 0;



void mains_code(void);

//
// Main
//
void main(void)
{
    mains_code();
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

    //sets theta motors to printable variables. print variables are global variables
    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = 0;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        //currently printing the motor thetas, the xyz potision of the end effector, the motor thetas found using inverse kinematics, and the joint thetas found using DH parmeters and inverse kinematics  
        serial_printf(&SerialA, "Motor Thetas:(%.2f, %.2f, %.2f), FK (xyz): (%.2f, %.2f, %.2f)  \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI, x, y,z);
        serial_printf(&SerialA,"IK Motor Thetas: (%.2f, %.2f, %.2f), DH Thetas:  (%.2f, %.2f, %.2f)  \n\r",theta1IK_motor*180/PI,theta2IK_motor*180/PI,theta3IK_motor*180/PI,theta1IK_DH*180/PI,theta2IK_DH*180/PI,theta3IK_DH*180/PI);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

