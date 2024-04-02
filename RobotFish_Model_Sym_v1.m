% Define the symbolic variables
syms alpha1 alpha2 xd yd l1 l2
syms xd_dot yd_dot xd_ddot yd_ddot
syms A_1 A_2 omega t

% Define expressions
alpha1 = A_1*sin(omega*t);
alpha2 = A_2*sin(omega*t + pi/2);

xd = l1*cos(alpha1) + l2*cos(alpha2);
yd = l1*sin(alpha1) + l2*sin(alpha2);

xd_dot = diff(xd, t);
yd_dot = diff(yd, t);

xd_ddot = diff(xd_dot, t);
yd_ddot = diff(yd_dot, t);

% syms V_c m_hat n_hat Vn Vm V mi L

Vn = - xd_dot*sin(alpha2) + yd_dot*cos(alpha2) + V_c*sin(alpha2);
Vm = xd_dot*cos(alpha2) + yd_dot*sin(alpha2) - V_c*cos(alpha2);

% Construct the V_vect and unit vectors m_hat and n_hat
V_vect = [Vn, Vm];
m_hat = [cos(alpha2), sin(alpha2)];
n_hat = [-sin(alpha2), cos(alpha2)];

Vn_dot = diff(V_n, t);
% Calculate the reactive force components
F_xd = (-1/2)*mi*Vn^2*m_hat(1) + mi*Vn*Vm*n_hat(1) - mi*L*Vn_dot*n_hat(1);
F_yd = (-1/2)*mi*Vn^2*m_hat(2) + mi*Vn*Vm*n_hat(2) - mi*L*Vn_dot*n_hat(2);

% Construct the reactive force vector
F_rf = [F_xd; F_yd];


% Display the expressions
disp('xd = '); disp(xd);
disp('yd = '); disp(yd);
disp('xd_dot = '); disp(xd_dot);
disp('yd_dot = '); disp(yd_dot);
disp('xd_ddot = '); disp(xd_ddot);
disp('yd_ddot = '); disp(yd_ddot);
disp('Vn_dot = '); disp(Vn_dot)
disp('F_rf = '); disp(F_rf)
