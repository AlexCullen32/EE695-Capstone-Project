%% Test Plot
clc
clear
close all

data = readmatrix('PID_Adjusted_Position.txt');

x = data(:, 1)/1000;
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([0 360]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x, y2, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle (Kp = 32, Ki = 0.6, Kd = 0.001)','Target','Cart Position (Kp = 0.1, Ki = 0, Kd = 0.06)');
title('Arduino PID');
grid on;
xlim([0 10]);
ylim([180-250 180+250])

%% CHANGING ANGLE Kp

clc
clear

Kp1 = readmatrix('Kp_One.txt');
Kp2 = readmatrix('Kp_Two.txt');
Kp3 = readmatrix('Kp_Three');

x = Kp1(1:1900, 1);
y1 = Kp1(1:1900, 2);
y2 = Kp2(1:1900, 2);
y3 = Kp3(30:1929, 2);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);
y3_filtered = movmean(y3, windowSize);

figure;
plot(x/1000, y1_filtered);
hold on;
plot(x/1000, y2_filtered);
hold on;
plot(x/1000, y3_filtered);
grid on;
legend('Kp = 5', 'Kp = 15', 'Kp = 30');
xlabel('Time (s)');
ylabel('Angle (Degrees)');
xlim([0 3]);
title('Changing Angle Kp Only (from Upright)');

%% CHANGING ANGLE Ki

clc
clear

Ki1 = readmatrix('Ki_One.txt');
Ki2 = readmatrix('Ki_Two.txt');
Ki3 = readmatrix('Ki_Three');

x = Ki1(1:1900, 1);
y1 = Ki1(1:1900, 2);
y2 = Ki2(1:1900, 2);
y3 = Ki3(1+812:1900+812, 2);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);
y3_filtered = movmean(y3, windowSize);

figure;
plot(x/1000, y1_filtered);
hold on;
plot(x/1000, y2_filtered);
hold on;
plot(x/1000, y3_filtered);
grid on;
legend('Ki = 1', 'Ki = 5', 'Ki = 10');
xlabel('Time (s)');
ylabel('Angle (Degrees)');
xlim([0 2]);
title('Changing Angle Ki Only (from Upright)');

%% CHANGING ANGLE Kd

clc
clear

Kd1 = readmatrix('Kd_One.txt');
Kd2 = readmatrix('Kd_Two.txt');
Kd3 = readmatrix('Kd_Three');

x = Kd1(1:1900, 1);
y1 = Kd1(1:1900, 2);
y2 = Kd2(1+350:1900+350, 2);
y3 = Kd3(1+600:1900+600, 2);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);
y3_filtered = movmean(y3, windowSize);

figure;
plot(x/1000, y1_filtered);
hold on;
plot(x/1000, y2_filtered);
hold on;
plot(x/1000, y3_filtered);
grid on;
legend('Kd = 0.1', 'Kd = 0.5', 'Kd = 1');
xlabel('Time (s)');
ylabel('Angle (Degrees)');
xlim([0 2]);
title('Changing Angle Kd Only (from Upright)');

%% ADDING IN POSITION PID

clc
clear
close all

Kp1 = readmatrix('AP_One.txt');
Kp2 = readmatrix('AP_Two.txt');
Kp3 = readmatrix('AP_Three');
Kp4 = readmatrix('AP_Four');

x1 = Kp1(1:5500, 1);

y1 = Kp1(1+200:5500+200, 2);
y2 = Kp2(1:5500, 2);
y3 = Kp3(1:5500, 2);
y4 = Kp4(1+300:5500+300, 2);

yy1 = Kp1(1+200:5500+200, 3);
yy2 = Kp2(1:5500, 3);
yy3 = Kp3(1:5500, 3);
yy4 = Kp4(1+300:5500+300, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);
y3_filtered = movmean(y3, windowSize);
y4_filtered = movmean(y4, windowSize);

yy1_filtered = movmean(yy1, windowSize);
yy2_filtered = movmean(yy2, windowSize);
yy3_filtered = movmean(yy3, windowSize);
yy4_filtered = movmean(yy4, windowSize);

figure;
plot(x1/1000, y1_filtered);
hold on;
plot(x1/1000, yy1_filtered);
hold on;
plot(x1/1000, y2_filtered);
hold on;
plot(x1/1000, yy2_filtered);
hold on;
plot(x1/1000, y3_filtered);
hold on;
plot(x1/1000, yy3_filtered);
hold on;
plot(x1/1000, y4_filtered);
plot(x1/1000, yy4_filtered);
grid on;
xlim([0 4]);

legend('Position Kp = 0, Ki = 0, Kd = 0 (Angle)', ...
       'Position Kp = 0, Ki = 0, Kd = 0 (Position)', ...
       'Position Kp = 0.15, Ki = 0, Kd = 0 (Angle)', ...
       'Position Kp = 0.15, Ki = 0, Kd = 0 (Position)', ...
       'Position Kp = 0.15, Ki = 0.1, Kd = 0 (Angle)', ...
       'Position Kp = 0.15, Ki = 0.1, Kd = 0 (Position)', ...
       'Position Kp = 0.15, Ki = 0.1, Kd = 0.06 (Angle)', ...
       'Position Kp = 0.15, Ki = 0.1, Kd = 0.06 (Position)');

xlabel('Time (s)');
ylabel('Angle (Degrees)');
title('Changing Position PID Values, Angle PID Tuned: Kp = 45, Ki = 0.6, Kd = 0.003 (from Upright)');

%% 1 Cup (Add coin + disturbances every 10 seconds from 5 seconds)

clc
clear
close all

data = readmatrix('One_Cup.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Single Cup Weight');
grid on;
xlim([0 80]);
ylim([180-270 180+270])

%% 2 Cup Offset (Add a coin to the right every 10 sec (10, 20, fail))

clc
clear
close all

data = readmatrix('Two_Cup_Odd_Weighting.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([0 360]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Double Cup Uneven Weight');
grid on;
xlim([0 30]);
ylim([180-270 180+270])

%% 2 Cup (Add coin to side very 5s, swicth side every second coin + Adding disturbances at the end)

clc
clear
close all

data = readmatrix('Two_Cup_Even_Weighting.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Double Cup Even Weight');
grid on;
xlim([0 60]);
ylim([180-270 180+270])

%% Balance Beam (Add marble every 10 seconds from 5s + disturbances un between)

clc
clear
close all

data = readmatrix('Balance_Beam.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2-50, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Balance Beam');
grid on;
xlim([0 43]);
ylim([180-270 180+270])

%% Air Disturbance (Move 50mm closer every 10s, starting at 650mm away from left end at 5s in)

clc
clear
close all

data = readmatrix('Air_Disturbance.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2_filtered-50, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Air Disturbance');
grid on;
xlim([0 91.5]);
ylim([180-270 180+270])

%% Adjustable Weight High (Centered 380mm from fulcrum + Disturbances after 10s)

clc
clear
close all

data = readmatrix('Adj_High.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2-50, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Adjustable Weight (High)');
grid on;
xlim([0 35.8]);
ylim([180-270 180+270])

%% Adjustable Weight Middle (Centered 200mm from fulcrum + Disturbances after 10s)

clc
clear
close all

data = readmatrix('Adj_Middle.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2-50, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Adjustable Weight (Middle)');
grid on;
xlim([0 28.5]);
ylim([180-270 180+270])

%% Adjustable Weight Low (Centered 100mm from fulcrum + Disturbances after 10s)

clc
clear

data = readmatrix('Adj_Low.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 30;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2-50, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Adjustable Weight (Low)');
grid on;
xlim([0 25]);
ylim([180-270 180+270])

%% Long Run (No disturbance)

clc
clear
close all

data = readmatrix('Long_Run.txt');

x = data(1+1200:700000, 1);
y1 = data(1+1200:700000, 2);
y2 = data(1+1200:700000, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2_filtered-50, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Inverted Pendulum Lab Based System (Parallel PID)')
grid on;
ylim([180-270 180+270])
xlim([0 60]);

%% Voltage Divider

clc
clear
close all

data = readmatrix('Voltage_Divider.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered+7, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2-50, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Voltage Divider (No Disturbances)');
grid on;
ylim([180-100 180+100]);
xlim([0 600]);
%% Voltage Divider + Disturbance

clc
clear
close all

data = readmatrix('Voltage_Divider_Disturbance.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered+7.5, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r' , 'Target Angle = 180°', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2+120, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Voltage Divider (With Disturbances)');
grid on;
ylim([200 450]);
xlim([0 14.5]);

%% No Voltage Divider + Disturb

clc
clear
close all

data = readmatrix('Normal_Disturb.txt');

x = data(1200:25800, 1);
y1 = data(1200:25800, 2);
y2 = data(1200:25800, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);

figure;
plot(x/1000, y1_filtered-8.5, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2_filtered+120, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('No Voltage Divider (With Disturbances)');
grid on;
ylim([200 450]);
xlim([0 14.5]);

%% Swing Up Fail

clc
clear
close all

data = readmatrix('Swing_up_V1.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);

figure;
plot(x/1000, y1_filtered, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
yline(180, 'r' , 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);

yyaxis right
plot(x/1000, y2, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Swing Up Fail');
grid on;
ylim([180-200 180+150]);
xlim([0 22]);

%% Swing Up Catch
clc
clear
close all

data = readmatrix('Swing_Up_Catch.txt');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);

figure;
plot(x/1000, y1_filtered-1, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);
ylim([0 360]);

yyaxis right
plot(x/1000, y2_filtered -147, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('Swing Up (Catch)');
grid on;
ylim([180-200 180+150]);
xlim([0 25]);

%% Arduino LQR
clc
clear
close all

data = readmatrix('LQR_ONE');

x = data(:, 1);
y1 = data(:, 2);
y2 = data(:, 3);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);

figure;
plot(x/1000, y1_filtered-1, 'LineWidth',2); hold on;
ylabel('Angle (Degrees)');
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1);
ylim([100 260]);

yyaxis right
plot(x/1000, y2_filtered, 'LineWidth',2);

xlabel('Time (s)');
ylabel('Cart Position (mm)');
legend('Pendulum Angle','Target','Cart Position');
title('LQR Attempt');
grid on;
ylim([0 400]);
xlim([0 25]);