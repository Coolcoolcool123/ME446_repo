syms JT_11 JT_12 JT_13 JT_21 JT_22 JT_23 JT_31 JT_32 JT_33
syms x y z x_dot y_dot z_dot KPx KPy KPz KDx KDy KDz
syms uf1 uf2 uf3 xd yd zd xd_dot yd_dot zd_dot  Fzc
syms KPxN KPyN KPzN KDxN KDyN KDzN
syms RT11 RT12 RT13 RT21 RT22 RT23 RT31 RT32 RT33
syms R11 R12 R13 R21 R22 R23 R31 R32 R33
syms Fx Fy Fz
syms xb yb zb xa ya za
syms t ttotal tstart
JT = [JT_11 JT_12 JT_13;
    JT_21 JT_22 JT_23;
    JT_31 JT_32 JT_33];

R = [R11 R12 R13;
    R21 R22 R23;
    R31 R32 R33];

RT = [RT11 RT12 RT13;
    RT21 RT22 RT23;
    RT31 RT32 RT33];

KPN = [KPx 0 0;
        0   KPy    0;
        0   0   KPz];

KDN = [KDx 0 0;
        0   KDy    0;
        0   0   KDz];
deltaX = xb - xa;
deltaY = yb - ya;
deltaZ = zb - za;

% desired = [deltaX;deltaY;deltaX] * ((t-tstart)/ttotal) + [xa; ya; za];
desired = [xd;yd;zd];
dm = desired - [x;y;z];

desired_dot = [xd_dot - x_dot;
        yd_dot - y_dot;
        zd_dot - z_dot];

uf = [uf1;uf2;uf3];

taum = (JT*R*(KPN*RT*dm+KDN*RT*desired_dot))+uf