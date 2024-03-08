function [Theta, Theta_dot, Theta_ddot] = Plots(time)

x1 = 0.18;
x2 = 0.16;
x3 = 0.25;
y1 = -0.08;
y2 = 0.11;
y3 = -0.10;
z1 = 0.63;
z2 = 0.63;
z3 = 0.20;

syms time

x = x1 + time*((x2-x1)/0.001);
y = y1 + time*((y2-y1)/0.001);
z = z1 + time*((z2-z1)/0.001);


x = x2 + time*((x3-x2)/0.001);
y = y2 + time*((y3-y2)/0.001);
z = z2 + time*((z3-z2)/0.001);

x = x3 + time*((x1-x3)/0.001);
y = y3 + time*((y1-y3)/0.001);
z = z3 + time*((z1-z3)/0.001);

theta1IK_DH = atan(y/x);
theta2IK_DH = -atan( (z-L1)/sqrt(x^2+y^2) ) - acos((L2^2+x^2+y^2+(z-L1)^2-L3^2) / (2*L2*sqrt(x^2+y^2+(z-L1)^2)) );
theta3IK_DH =  PI - acos((L2^2+L3^2-(x^2+y^2+(z-L1)^2))/(2*L2*L3));

Theta1 = zeros(1,100);
Theta2 = zeros(1,100);
Theta3 = zeros(1,100);

t_in = linspace(0, 3, 100);



for ii = 1:100
    if t_in(ii) <= 1
        Theta(ii) = subs(theta_da, t, t_in(ii));
        Theta_dot(ii) = subs(thetadot_da, t, t_in(ii));
        Theta_ddot(ii) = subs(thetaddot_da, t, t_in(ii));

    elseif t_in(ii) > 1 && t_in(ii) <= 2
        Theta(ii) = subs(theta_db, t, t_in(ii));
        Theta_dot(ii) = subs(thetadot_db, t, t_in(ii));
        Theta_ddot(ii) = subs(thetaddot_db, t, t_in(ii));
    elseif t_in(ii) > 
        Theta(ii) = 0;
        Theta_dot(ii) = 0;
        Theta_ddot(ii) = 0;
    end

end


figure(1);
plot(t_in,Theta)

figure(2);
plot(t_in,Theta_dot)

figure(3);
plot(t_in,Theta_ddot)