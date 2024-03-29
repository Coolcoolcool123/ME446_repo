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

float xd = 0.254;
float yd = 0.0;
float zd = .254*2;
float xd_dot = 0;
float yd_dot = 0;
float zd_dot = 0;

float KPx = 300.0; //300 for lab 3 part 2
float KPy = 300.0; //300 for lab 3 part 2
float KPz = 300.0; //300 for lab 3 part 2

float KDx = 10.0; //10 for lab 3 part 2
float KDy = 10.0; //10 for lab 3 part 2
float KDz = 10.0; //10 for lab 3 part 2

float x_dot = 0;
float x_dot_old1 = 0;
float x_dot_old2 = 0;
float y_dot = 0;
float y_dot_old1 = 0;
float y_dot_old2 = 0;
float z_dot = 0;
float z_dot_old1 = 0;
float z_dot_old2 = 0;
float x_old = 0;
float y_old = 0;
float z_old = 0;

float Fx = 0;
float Fy = 0;
float Fz = 0;

//lab 3 part 3
float Fzcmd = 0;
float Kt = 6;


void mains_code(void);

//
// Main
//
void main(void)
{
     mains_code();
}


//custom path from lab 2
void custom(float time){
    x = .3+.1*cos(PI*time);
    y = .1*sin(PI*time);
    z = .35;

    //taking inverse kinematic code from lab 1
    theta1IK_DH = atan(y/x);
    theta2IK_DH = -atan( (z-L1)/sqrt(pow(x,2)+pow(y,2)) ) - acos((pow(L2,2)+pow(x,2)+pow(y,2)+pow((z-L1),2)-pow(L3,2)) / (2*L2*sqrt(pow(x,2)+pow(y,2)+pow((z-L1),2))) );
    theta3IK_DH =  PI - acos((pow(L2,2)+pow(L3,2)-(pow(x,2)+pow(y,2)+pow(z-L1,2)))/(2*L2*L3));

    Theta1 = theta1IK_DH;
    Theta2 = theta2IK_DH + PI/2;
    Theta3 = theta3IK_DH + theta2IK_motor - PI/2;

    }


//implementing the cubic polynomial trajectory from lab 2
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

    //Lab 2 part 5
    if ((mycount%2000)<1000) {

        Theta1 = 0;
        Theta2 = 0;
        Theta3 = 0;

    } else {

        Theta1 = PI/6;
        Theta2 = PI/6;
        Theta3 = PI/6;

   }

    //theta((mycount % 2000)/1000.0);
    //custom((mycount % 4000)/1000.0);
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


   FricComp();


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

   x = 0.254*cosq1*(cosq3+sinq2);
   y = 0.254*sinq1*(cosq3+sinq2);
   z = 0.254*(1+cosq3-sinq3);

    x_dot = (x-x_old)/0.001;
    x_dot = (x_dot+x_dot_old1 + x_dot_old2)/3.0;

    y_dot = (y-y_old)/0.001;
    y_dot = (y_dot+y_dot_old1 + y_dot_old2)/3.0;

    z_dot = (z-z_old)/0.001;
    z_dot = (z_dot+z_dot_old1 + z_dot_old2)/3.0;

    Fx = KPx*(xd-x)+KDx*(xd_dot-x_dot);
    Fy = KPy*(yd-y)+KDy*(yd_dot-y_dot);
    Fz = KPz*(zd-z)+KDz*(zd_dot-z_dot);

    //tau values for lab 3 part 2
//   *tau1 = ff1*u_fric1 + Fx*JT_11 + Fy*JT_12 + Fz*JT_13;
//   *tau2 = ff2*u_fric2 + Fx*JT_21 + Fy*JT_22 + Fz*JT_23;
//   *tau3 = ff3*u_fric3 + Fx*JT_31 + Fy*JT_32 + Fz*JT_33;

    //tau values for lab 3 part 3
//   *tau1 = ff1*u_fric1 + Fx*JT_11 + Fy*JT_12 + Fz*JT_13 + Fzcmd*JT_13/Kt;
//   *tau2 = ff2*u_fric2 + Fx*JT_21 + Fy*JT_22 + Fz*JT_23 + Fzcmd*JT_23/Kt;
//   *tau3 = ff3*u_fric3 + Fx*JT_31 + Fy*JT_32 + Fz*JT_33  + Fzcmd*JT_33/Kt;

    //tau values for lab 3 part 4
   *tau1 = ff1*u_fric1 - (JT_11*R11 + JT_12*R21 + JT_13*R31)*(KPx*RT11*(x - xd) + KDx*RT11*(x_dot - xd_dot) + KPx*RT12*(y - yd) + KDx*RT12*(y_dot - yd_dot) + KPx*RT13*(z - zd) + KDx*RT13*(z_dot - zd_dot)) - (JT_11*R12 + JT_12*R22 + JT_13*R32)*(KPy*RT21*(x - xd) + KDy*RT21*(x_dot - xd_dot) + KPy*RT22*(y - yd) + KDy*RT22*(y_dot - yd_dot) + KPy*RT23*(z - zd) + KDy*RT23*(z_dot - zd_dot)) - (JT_11*R13 + JT_12*R23 + JT_13*R33)*(KPz*RT31*(x - xd) + KDz*RT31*(x_dot - xd_dot) + KPz*RT32*(y - yd) + KDz*RT32*(y_dot - yd_dot) + KPz*RT33*(z - zd) + KDz*RT33*(z_dot - zd_dot));
   *tau2 = ff2*u_fric2 - (JT_21*R11 + JT_22*R21 + JT_23*R31)*(KPx*RT11*(x - xd) + KDx*RT11*(x_dot - xd_dot) + KPx*RT12*(y - yd) + KDx*RT12*(y_dot - yd_dot) + KPx*RT13*(z - zd) + KDx*RT13*(z_dot - zd_dot)) - (JT_21*R12 + JT_22*R22 + JT_23*R32)*(KPy*RT21*(x - xd) + KDy*RT21*(x_dot - xd_dot) + KPy*RT22*(y - yd) + KDy*RT22*(y_dot - yd_dot) + KPy*RT23*(z - zd) + KDy*RT23*(z_dot - zd_dot)) - (JT_21*R13 + JT_22*R23 + JT_23*R33)*(KPz*RT31*(x - xd) + KDz*RT31*(x_dot - xd_dot) + KPz*RT32*(y - yd) + KDz*RT32*(y_dot - yd_dot) + KPz*RT33*(z - zd) + KDz*RT33*(z_dot - zd_dot));
   *tau3 = ff3*u_fric3 - (JT_31*R11 + JT_32*R21 + JT_33*R31)*(KPx*RT11*(x - xd) + KDx*RT11*(x_dot - xd_dot) + KPx*RT12*(y - yd) + KDx*RT12*(y_dot - yd_dot) + KPx*RT13*(z - zd) + KDx*RT13*(z_dot - zd_dot)) - (JT_31*R12 + JT_32*R22 + JT_33*R32)*(KPy*RT21*(x - xd) + KDy*RT21*(x_dot - xd_dot) + KPy*RT22*(y - yd) + KDy*RT22*(y_dot - yd_dot) + KPy*RT23*(z - zd) + KDy*RT23*(z_dot - zd_dot)) - (JT_31*R13 + JT_32*R23 + JT_33*R33)*(KPz*RT31*(x - xd) + KDz*RT31*(x_dot - xd_dot) + KPz*RT32*(y - yd) + KDz*RT32*(y_dot - yd_dot) + KPz*RT33*(z - zd) + KDz*RT33*(z_dot - zd_dot));


//    *tau1 = Kp1*(Theta1-theta1motor)-KD1*Omega1 + IK1*KI1;
//    *tau2 = Kp2*(Theta2-theta2motor)-KD2*Omega2 + IK2*KI2;
//    *tau3 = Kp3*(Theta3-theta3motor)-KD3*Omega3 + IK3*KI3;
////
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

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
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

