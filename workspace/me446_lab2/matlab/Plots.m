function [Theta, Theta_dot, Theta_ddot] = Plots(time)


t_in = linspace(0, time, 100);
[a, b] = coeff();

syms t

theta_da = a(1) + a(2)*t+a(3)*t^2+a(4)*t^3;
theta_db = b(1) + b(2)*t+b(3)*t^2+b(4)*t^3;

thetadot_da = a(2) + 2*a(3)*t+3*a(4)*t^2;
thetadot_db = b(2) + 2*b(3)*t+3*b(4)*t^2;

thetaddot_da = 2*a(3)+ 6*a(4)*t;
thetaddot_db = 2*b(3)+ 6*b(4)*t;

Theta = zeros(1,100);
Theta_dot = zeros(1,100);
Theta_ddot = zeros(1,100);

for ii = 1:100
    if t_in(ii) <= 1
        Theta(ii) = subs(theta_da, t, t_in(ii));
        Theta_dot(ii) = subs(thetadot_da, t, t_in(ii));
        Theta_ddot(ii) = subs(thetaddot_da, t, t_in(ii));

    elseif t_in(ii) > 1 && t_in(ii) <= 2
        Theta(ii) = subs(theta_db, t, t_in(ii));
        Theta_dot(ii) = subs(thetadot_db, t, t_in(ii));
        Theta_ddot(ii) = subs(thetaddot_db, t, t_in(ii));
    else
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