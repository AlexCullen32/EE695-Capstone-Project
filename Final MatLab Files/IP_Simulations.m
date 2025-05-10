%% PID Simulation
close all;
clc;
clear;

M = 0.254;
m = 0.097;
b = 0.1;
I = 0.005173;
g = 9.81;
l = 0.2;

q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

Kp = 100;
Ki = 15;
Kd = 30;
C = pid(Kp,Ki,Kd);

T_angle_deviation = feedback(P_pend,C);
t_sim = 0:0.01:3;
[y_pend_rad, t_pend] = impulse(T_angle_deviation, t_sim);
y_pend_deg = y_pend_rad * (180/pi);
pend_angle_deg = y_pend_deg + 180;

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
T_cart_position = P_cart / (1 + P_pend*C);
[y_cart_m, t_cart] = impulse(T_cart_position, t_sim);
y_cart_mm = y_cart_m * 1000;

figure;
yyaxis left;
plot(t_pend, pend_angle_deg, 'LineWidth', 2);
ylabel('Pendulum Angle (Degrees)');
ylim([170 190]);
hold on;
yline(180, 'Color', [0 0.65 0 0.2], 'LineWidth', 1, 'Label', '', 'LabelHorizontalAlignment', 'right');
hold off;
yyaxis right;
plot(t_cart, y_cart_mm, 'LineWidth', 2);
ylabel('Cart Position (mm)');
ylim([-100 100]);
xlabel('Time (s)');
title('Simulated Angle PID Controller (Impulse Disturbance), (Kp = 100, Ki = 15, KD = 30)');
legend('Pendulum Angle', 'Target','Cart Position');
grid on;

%% LQR Simulation (Changing each Q Weighting)
clc;
clear;
close all;

defaultFontSize = 24;
lineWidthMultiple = 2;

legendFontSize = defaultFontSize * 0.5;

figure('Position', [100, 100, 1000, 800]);
sgtitle('LQR Effects of Increasing Single Q Elements (R=0.05)', 'FontSize', defaultFontSize);

M = 0.254;
m = 0.097;
b = 0.7;
I = 0.005173;
g = 9.81;
l = 0.2;

p = I*(M+m)+M*m*l^2;
A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q_base = diag([1, 1, 1, 1]);
R = 0.05;
x0 = [0; 0; deg2rad(4); 0];
t = 0:0.01:10;
q_increase_value = 100;

for i = 1:4
    Q_current = Q_base;
    Q_current(i, i) = q_increase_value;
    K = lqr(A, B, Q_current, R);
    Acl = A - B * K;
    sys_cl = ss(Acl, B, C, D);
    [y, t_out, x_out] = lsim(sys_cl, zeros(size(t)), t, x0);

    position = y(:, 1);
    angle_deviation_rad = y(:, 2);
    angle_absolute_deg = rad2deg(pi + angle_deviation_rad);
    subplot(2, 2, i);
    yyaxis left;
    plot(t_out, angle_absolute_deg, 'LineWidth', 1.5*lineWidthMultiple); hold on;

    yline(180, '--g', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1.5*lineWidthMultiple, 'FontSize', defaultFontSize);
    ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
    ylim([170 190]);
    yyaxis right;
    plot(t_out, position * 1000, 'LineWidth', 1.5*lineWidthMultiple);
    ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
    ylim([-500 500]);
    xlabel('Time (s)', 'FontSize', defaultFontSize);
    leg = legend('Pendulum Angle', 'Target Angle', 'Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
    leg.ItemTokenSize = [20*0.5, 20*0.5];
    state_names_diag = {'Q_{11} (Cart Position)', 'Q_{22} (Cart Velocity)', 'Q_{33} (Pendulum Angle)', 'Q_{44} (Pendulum Angle Velocity)'};
    title(['Increasing ', state_names_diag{i}, ' to ', num2str(q_increase_value)], 'FontSize', defaultFontSize);
    grid on;
    xlim([0 10]);

    ax = gca;
    ax.FontSize = defaultFontSize;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';

end

%% Optimal LQR Simulation

clc;
clear;
close all;

defaultFontSize = 24;
lineWidthMultiple = 2;

legendFontSize = defaultFontSize * 0.5;

figure('Position', [100, 100, 800*2, 600*2]);

M = 0.254;
m = 0.097;
b = 0.7;
I = 0.005173;
g = 9.81;
l = 0.2;

p = I*(M+m)+M*m*l^2;

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q = diag([0.001, 0.000001, 27, 0.00001]);
R = 0.005;
K = lqr(A, B, Q, R);
K

Acl = A - B * K;
x0 = [0; 0; deg2rad(4); 0];
t = 0:0.01:10;
sys_cl = ss(Acl, B, C, D);
[y, t_out, x_out] = lsim(sys_cl, zeros(size(t)), t, x0);

position = y(:, 1);
angle_deviation_rad = y(:, 2);

angle_absolute_deg = rad2deg(pi + angle_deviation_rad);

yyaxis left;
plot(t_out, angle_absolute_deg, 'LineWidth', 1.5*lineWidthMultiple); hold on;
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
yline(180, '--g', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1.5*lineWidthMultiple, 'FontSize', defaultFontSize);
yyaxis right;
plot(t_out, position * 1000, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([-500 500]);

xlabel('Time (s)', 'FontSize', defaultFontSize);
leg = legend('Pendulum Angle','Target Angle','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
leg.ItemTokenSize = [20*0.5, 20*0.5];
title('Optimal System Response', 'FontSize', defaultFontSize);
grid on;
xlim([0 5]);

ax = gca;
ax.FontSize = defaultFontSize;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';

det(ctrb(A,B))
rank(A,B)