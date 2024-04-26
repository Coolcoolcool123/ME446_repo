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

//variables for Lab 3
//viscous and coulmob values
float Viscous_positive1 = 0.2513;
float Viscous_negative1 = 0.2477;
float Coulomb_positive1 = 0.33487;
float Coulomb_negative1 = -0.33523;


float Viscous_positive2 = 0.25;
float Viscous_negative2 = 0.287;
float Coulomb_positive2 = 0.1675;
float Coulomb_negative2 = -0.16565;

float Viscous_positive3 = 0.1922;
float Viscous_negative3 = 0.2132;
float Coulomb_positive3 = 0.17039;
float Coulomb_negative3 = -0.16434;

float ff1 = 1;
float ff2 = 1;
float ff3 = 1;

float minimum_velocity1  = 0.1;
float minimum_velocity2  = 0.05;
float minimum_velocity3  = 0.05;

float slope_between_minimums = 3.6;

float u_fric1 = 0;
float u_fric2 = 0;
float u_fric3 = 0;


//starter code global varables
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;


float thetaz = 0;
float thetax = 0;
float thetay = 0;


float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;


float tstart = 0;
float ttotal = 0;
float t = 0;



float desiredspeed = 0.1;

float xd = 0.254;
float yd = 0.0;
float zd = .254*2;
float xd_dot = 0;
float yd_dot = 0;
float zd_dot = 0;

float KPx = 300.0; //300 for lab 3 part 2
float KPy = 300.0; //300 for lab 3 part 2
float KPz = 300.0; //300 for lab 3 part 2

float KDx = 30.0; //10 for lab 3 part 2
float KDy = 30.0; //10 for lab 3 part 2
float KDz = 30.0; //10 for lab 3 part 2

float x_dot = 0;
float x_dot_old1 = 0;
float x_dot_old2 = 0;
float y_dot = 0;
float y_dot_old1 = 0;
float y_dot_old2 = 0;
float z_dot = 0;
float z_dot_old1 = 0;
float z_dot_old2 = 0;
float x_old = 0.139;
float y_old = 0.000;
float z_old = 0.424;

float Fx = 0;
float Fy = 0;
float Fz = 0;

//lab 3 part 3
float Fzcmd = 0;
float Kt = 6;




typedef struct Waypoint {
    float xDes;
    float yDes;
    float zDes;
} Waypoint;

Waypoint points[] = {{0.139, 0, 0.424}, //initial position 0
                     {0.294,0,0.508}, //move in a straight line (in the x direction) 1
                     {0.23,0.25,0.43}, //starts moving towards peg hole 2
                     {0.034, 0.344, 0.25}, //hover above peg hole 3
                     {0.034, 0.344, 0.15}, //go into peg hole 4
                     {0.034,0.344,0.15}, //stay inside peg hole 5
                     {0.034, 0.344, 0.25}, //get out of peg hole 6


                     {0.233, 0.100, 0.4},//middle waypoint towards zigzag 7
                     {0.375, 0.101, 0.215}, //start of zigzag 8
                     {0.379, 0.095, 0.215}, //pasue at start of zigzag 9
                     {0.398, 0.060, 0.215}, // 1st point corner 10
                     {0.390, 0.042, 0.215}, // 2st point corner 11
                     {0.379, 0.038, 0.215}, // 3st point corner 12
                     {0.324, 0.049, 0.215}, // 1st point corner 13
                     {0.325, 0.040, 0.215}, // 2st point corner 14
                     {0.312, 0.030, 0.215}, // 3st point corner 15
                     {0.370, -0.044, 0.215}, // zigzag exit 16
                     {0.379, -0.070, 0.215}, // exit zigzag more 17
                     {0.354,0,0.400}, // back to initial position 18

                     {0.200, 0.177, 0.305}, //above egg 19
                     {0.200, 0.177, 0.305}, //pause 20
                     {0.200, 0.177, 0.2875}, //press egg 21
                     {0.200, 0.177, 0.2875}, //pause egg 22
                     {0.200, 0.177, 0.305}, //back to above egg 23
                     {0.139, 0, 0.424} //back to initial position 24
                    };

float ts[] = {0, //0
              2, // time of straight line movement 1
              5, //end of line to middle waypoint towards peg hole 2
              8, //middle of peg hole waypoint to above the peg hole 3
              10, //go into peg hole 4
              11, //stay inside peg hole 5
            12, //come out of peg hole 6
        16, //going towards zigzag 7
//              2, //7 2
//              4, //8 3
//              5, //9 4
//              6, //10 5
//              7, // 11 6
//              8, //12 7
//              9, //13 8
//              10, //14 9
//              11, //15 10
//              12, //16 11
//              13 //22 12



           19, //go to the zigzag entrance 8
           19.5, //9
           20, //10
           20.25,//11
           20.5, //12
           21.5, //13
           21.75, //14
           22, //15
           23, //16
           24, //17

           27, //initital position 18
           29, //hovver aboce eg 19
           30, //press egg 20
           31, //pause 21
           32, //above egg 22
           33, //23
           34 //24
};

int index = 0;
int a = sizeof(ts)/sizeof(ts[0]);
void mains_code(void);

//
// Main
//
void main(void)
{
    mains_code();
}


void gains(int index) {
    if (index == 10) {
        thetaz = 36.87 / 180 * PI;
        KPx = 10; //300 for lab 3 part 2
        KDx = 1.0; //10 for lab 3 part 2
//        KPz = 500;
//        KPy = 500;
//        KDz = 20;
//        KDy = 20;
    } else if(index == 11){
        thetaz = -59 / 180 * PI;
        KPx = 10; //300 for lab 3 part 2
        KDx = 1.0; //10 for lab 3 part 2
//        KPz = 500;
//        KPy = 500;
//        KDz = 20;
//        KDy = 20;
    } else if(index == 12) {
        thetaz = -85.6 / 180 * PI;
        KPx = 10; //300 for lab 3 part 2
        KDx = 1.0; //10 for lab 3 part 2
//        KPz = 500;
//        KPy = 500;
//        KDz = 20;
//        KDy = 20;
    } else if(index == 13) {
        thetaz = 65 / 180 * PI;
        KPx = 10; //300 for lab 3 part 2
        KDx = 1.0; //10 for lab 3 part 2
        KPz = 500;
        KPy = 500;
        KDz = 20;
        KDy = 20;
    } else if(index == 14) {
        thetaz = -64.7 / 180 * PI;
        KPx = 10; //300 for lab 3 part 2
        KDx = 1.0; //10 for lab 3 part 2
        KPz = 500;
        KPy = 500;
        KDz = 20;
        KDy = 20;
    } else if(index == 15){
        thetaz = -23.2 / 180 * PI;
        KPx = 10; //300 for lab 3 part 2
        KDx = 1.0; //10 for lab 3 part 2
        KPz = 500;
        KPy = 500;
        KDz = 20;
        KDy = 20;
    } else if(index == 16){
        thetaz = 36.87 / 180 * PI;
        KPx = 10; //300 for lab 3 part 2
        KDx = 1.0; //10 for lab 3 part 2
        KPz = 500;
        KPy = 500;
        KDz = 20;
        KDy = 20;
    } else if(index == 4){
                KPx = 100;
                KDx = 1.0;
                KPy = 100;
                KDy = 1;

    } else {

       thetaz = 0;


       KPx = 300.0; //300 for lab 3 part 2
       KPy = 300.0; //300 for lab 3 part 2
       KPz = 300.0; //300 for lab 3 part 2

       KDx = 30.0; //10 for lab 3 part 2
       KDy = 30.0; //10 for lab 3 part 2
       KDz = 30.0; //10 for lab 3 part 2

    }
}

void part5(float time,float t1, float t2, Waypoint Pos1, Waypoint Pos2) {
    float Time = time/1000;

    tstart = t1;
    xd = (Pos2.xDes-Pos1.xDes) * (Time-t1)/(t2-t1) + Pos1.xDes;
    yd = (Pos2.yDes-Pos1.yDes) * (Time-t1)/(t2-t1) + Pos1.yDes;
    zd = (Pos2.zDes-Pos1.zDes) * (Time-t1)/(t2-t1) + Pos1.zDes;
    if(Time == t2) {
        index++;
    }

}

//lab 3 part 1
void FricComp() {
    if (Omega1 > minimum_velocity1) {
        u_fric1 = Viscous_positive1*Omega1 + Coulomb_positive1;
    } else if (Omega1 < -minimum_velocity1) {
        u_fric1 = Viscous_negative1*Omega1 + Coulomb_negative1;
    } else {
        u_fric1 = slope_between_minimums*Omega1;
    }

    if (Omega2 > minimum_velocity2) {
        u_fric2 = Viscous_positive2*Omega2 + Coulomb_positive2;
    } else if (Omega2 < -minimum_velocity2) {
        u_fric2 = Viscous_negative2*Omega2 + Coulomb_negative2;
    } else {
        u_fric2 = slope_between_minimums*Omega2;
    }

    if (Omega3 > minimum_velocity3) {
        u_fric3 = Viscous_positive3*Omega3 + Coulomb_positive3;
    } else if (Omega3 < -minimum_velocity3) {
        u_fric3 = Viscous_negative3*Omega3 + Coulomb_negative3;
    } else {
        u_fric3 = slope_between_minimums*Omega3;
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

    FricComp();


    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;
    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;

    x = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    y = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    z = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;

   Omega1 = (theta1motor-Theta1_old)/0.001;
   Omega1 = (Omega1+Omega1_old1 + Omega1_old2)/3.0;



   Omega2 = (theta2motor-Theta2_old)/0.001;
   Omega2 = (Omega2+Omega2_old1 + Omega2_old2)/3.0;


   Omega3 = (theta3motor-Theta3_old)/0.001;
   Omega3 = (Omega3+Omega3_old1 + Omega3_old2)/3.0;


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





    t = mycount%34000;
    if (t == 0) {
        index = 0;
    }

    gains(index);
    part5(t, ts[index], ts[index+1], points[index], points[index+1]);


    x_dot = (x-x_old)/0.001;
    x_dot = (x_dot+x_dot_old1 + x_dot_old2)/3.0;

    y_dot = (y-y_old)/0.001;
    y_dot = (y_dot+y_dot_old1 + y_dot_old2)/3.0;

    z_dot = (z-z_old)/0.001;
    z_dot = (z_dot+z_dot_old1 + z_dot_old2)/3.0;

    Fx = KPx*(xd-x)+KDx*(xd_dot-x_dot);
    Fy = KPy*(yd-y)+KDy*(yd_dot-y_dot);
    Fz = KPz*(zd-z)+KDz*(zd_dot-z_dot);

    //tau values for lab 3 part 5
   *tau1 = ff1*u_fric1 - (JT_11*R11 + JT_12*R21 + JT_13*R31)*(KPx*RT11*(x - xd) + KDx*RT11*(x_dot - xd_dot) + KPx*RT12*(y - yd) + KDx*RT12*(y_dot - yd_dot) + KPx*RT13*(z - zd) + KDx*RT13*(z_dot - zd_dot)) - (JT_11*R12 + JT_12*R22 + JT_13*R32)*(KPy*RT21*(x - xd) + KDy*RT21*(x_dot - xd_dot) + KPy*RT22*(y - yd) + KDy*RT22*(y_dot - yd_dot) + KPy*RT23*(z - zd) + KDy*RT23*(z_dot - zd_dot)) - (JT_11*R13 + JT_12*R23 + JT_13*R33)*(KPz*RT31*(x - xd) + KDz*RT31*(x_dot - xd_dot) + KPz*RT32*(y - yd) + KDz*RT32*(y_dot - yd_dot) + KPz*RT33*(z - zd) + KDz*RT33*(z_dot - zd_dot));
   *tau2 = ff2*u_fric2 - (JT_21*R11 + JT_22*R21 + JT_23*R31)*(KPx*RT11*(x - xd) + KDx*RT11*(x_dot - xd_dot) + KPx*RT12*(y - yd) + KDx*RT12*(y_dot - yd_dot) + KPx*RT13*(z - zd) + KDx*RT13*(z_dot - zd_dot)) - (JT_21*R12 + JT_22*R22 + JT_23*R32)*(KPy*RT21*(x - xd) + KDy*RT21*(x_dot - xd_dot) + KPy*RT22*(y - yd) + KDy*RT22*(y_dot - yd_dot) + KPy*RT23*(z - zd) + KDy*RT23*(z_dot - zd_dot)) - (JT_21*R13 + JT_22*R23 + JT_23*R33)*(KPz*RT31*(x - xd) + KDz*RT31*(x_dot - xd_dot) + KPz*RT32*(y - yd) + KDz*RT32*(y_dot - yd_dot) + KPz*RT33*(z - zd) + KDz*RT33*(z_dot - zd_dot));
   *tau3 = ff3*u_fric3 - (JT_31*R11 + JT_32*R21 + JT_33*R31)*(KPx*RT11*(x - xd) + KDx*RT11*(x_dot - xd_dot) + KPx*RT12*(y - yd) + KDx*RT12*(y_dot - yd_dot) + KPx*RT13*(z - zd) + KDx*RT13*(z_dot - zd_dot)) - (JT_31*R12 + JT_32*R22 + JT_33*R32)*(KPy*RT21*(x - xd) + KDy*RT21*(x_dot - xd_dot) + KPy*RT22*(y - yd) + KDy*RT22*(y_dot - yd_dot) + KPy*RT23*(z - zd) + KDy*RT23*(z_dot - zd_dot)) - (JT_31*R13 + JT_32*R23 + JT_33*R33)*(KPz*RT31*(x - xd) + KDz*RT31*(x_dot - xd_dot) + KPz*RT32*(y - yd) + KDz*RT32*(y_dot - yd_dot) + KPz*RT33*(z - zd) + KDz*RT33*(z_dot - zd_dot));




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

    //solving for integral tracking error
    IK1_old = IK1;
    IK2_old = IK2;
    IK3_old = IK3;

    //saving old value for xyz_dot filtering
    x_old = x;
    x_dot_old2 = x_dot_old1;
    x_dot_old1 = x_dot;

    y_old = y;
    y_dot_old2 = y_dot_old1;
    y_dot_old1 = y_dot;

    z_old = z;
    z_dot_old2 = z_dot_old1;
    z_dot_old1 = z_dot;

    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;


    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;


    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = Theta1;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "(xyz): (%.3f, %.3f, %.3f) time:%.3f \n\r", x, y,z,tstart);
        serial_printf(&SerialA, "(xyzd): (%.3f, %.3f, %.3f) \n\r", xd, yd,zd);

    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

