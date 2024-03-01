function [A, B] = coeff()
% clc;
% clear all;
% close all;

syms a0 a1 a2 a3 b0 b1 b2 b3 t theta_da theta_db thetadot_da thetadot_db
 
eqn1a = theta_da == a0 + a1*t+a2*t^2+a3*t^3;
eqn1b = theta_db == b0 + b1*t+b2*t^2+b3*t^3;

eqn2a = thetadot_da == a1 + 2*a2*t+3*a3*t^2;
eqn2b = thetadot_db == b1 + 2*b2*t+3*b3*t^2;

inputs1 = [0 0 0]; %t, theta_d, thetadot_d
inputs2 = [1 0.5 0]; %t, theta_d, thetadot_d
inputs3 = [2 0 0]; %t, theta_d, thetadot_d

eqn1 = subs(eqn1a, [t, theta_da], [inputs1(1), inputs1(2)]);
eqn2 = subs(eqn2a, [t, thetadot_da], [inputs1(1), inputs1(3)]);

eqn3 = subs(eqn1a, [t, theta_da], [inputs2(1), inputs2(2)]);
eqn4 = subs(eqn2a, [t, thetadot_da], [inputs2(1), inputs2(3)]);

eqn5 = subs(eqn1b, [t, theta_db], [inputs3(1), inputs3(2)]);
eqn6 = subs(eqn2b, [t, thetadot_db], [inputs3(1), inputs3(3)]);

eqn7 = subs(eqn1b, [t, theta_db], [inputs2(1), inputs2(2)]);
eqn8 = subs(eqn2b, [t, thetadot_db], [inputs2(1), inputs2(3)]);

[a0, a1, a2, a3, b0, b1, b2, b3] = solve([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8], [a0 a1 a2 a3 b0 b1 b2 b3]);

A = [a0 a1 a2 a3]
B = [b0 b1 b2 b3]

