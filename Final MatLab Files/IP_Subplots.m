%% Changing Angle Tunings:

clc
clear
close all

defaultFontSize = 24;

lineWidthMultiple = 2;

Kp1 = readmatrix('Kp_One.txt');
Kp2 = readmatrix('Kp_Two.txt');
Kp3 = readmatrix('Kp_Three.txt');

x = Kp1(1:1900, 1);
y1 = Kp1(1:1900, 2);
y2 = Kp2(1:1900, 2);
y3 = Kp3(30:1929, 2);

windowSize = 15;
y1_filtered = movmean(y1, windowSize);
y2_filtered = movmean(y2, windowSize);
y3_filtered = movmean(y3, windowSize);

Ki1 = readmatrix('Ki_One.txt');
Ki2 = readmatrix('Ki_Two.txt');
Ki3 = readmatrix('Ki_Three.txt');

x_Ki = Ki1(1:1900, 1);
y1_Ki = Ki1(1:1900, 2);
y2_Ki = Ki2(1:1900, 2);
y3_Ki = Ki3(1+812:1900+812, 2);

y1_filtered_Ki = movmean(y1_Ki, windowSize);
y2_filtered_Ki = movmean(y2_Ki, windowSize);
y3_filtered_Ki = movmean(y3_Ki, windowSize);

Kd1 = readmatrix('Kd_One.txt');
Kd2 = readmatrix('Kd_Two.txt');
Kd3 = readmatrix('Kd_Three.txt');

x_Kd = Kd1(1:1900, 1);
y1_Kd = Kd1(1:1900, 2);
y2_Kd = Kd2(1+350:1900+350, 2);
y3_Kd = Kd3(1+600:1900+600, 2);

y1_filtered_Kd = movmean(y1_Kd, windowSize);
y2_filtered_Kd = movmean(y2_Kd, windowSize);
y3_filtered_Kd = movmean(y3_Kd, windowSize);

figure;

subplot(3, 1, 1);
p1 = plot(x/1000, y1_filtered, 'LineWidth', 1.5 * lineWidthMultiple);
hold on;
p2 = plot(x/1000, y2_filtered, 'LineWidth', 1.5 * lineWidthMultiple);
p3 = plot(x/1000, y3_filtered, 'LineWidth', 1.5 * lineWidthMultiple);
hold off;
grid on;
leg1 = legend('Kp = 5', 'Kp = 15', 'Kp = 30');
leg1.ItemTokenSize = [20, 20];
leg1.FontSize = defaultFontSize;
xlabel('Time (s)', 'FontSize', defaultFontSize);
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
xlim([0 3]);
title('Changing Angle Kp Only (from Upright)', 'FontSize', defaultFontSize);
ax = gca;
ax.FontSize = defaultFontSize;

subplot(3, 1, 2);
p4 = plot(x_Ki/1000, y1_filtered_Ki, 'LineWidth', 1.5 * lineWidthMultiple);
hold on;
p5 = plot(x_Ki/1000, y2_filtered_Ki, 'LineWidth', 1.5 * lineWidthMultiple);
p6 = plot(x_Ki/1000, y3_filtered_Ki, 'LineWidth', 1.5 * lineWidthMultiple);
hold off;
grid on;
leg2 = legend('Ki = 1', 'Ki = 5', 'Ki = 10');
leg2.ItemTokenSize = [20, 20];
leg2.FontSize = defaultFontSize;
xlabel('Time (s)', 'FontSize', defaultFontSize);
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
xlim([0 2]);
title('Changing Angle Ki Only (from Upright)', 'FontSize', defaultFontSize);
ax = gca;
ax.FontSize = defaultFontSize;

subplot(3, 1, 3);
p7 = plot(x_Kd/1000, y1_filtered_Kd, 'LineWidth', 1.5 * lineWidthMultiple);
hold on;
p8 = plot(x_Kd/1000, y2_filtered_Kd, 'LineWidth', 1.5 * lineWidthMultiple);
p9 = plot(x_Kd/1000, y3_filtered_Kd, 'LineWidth', 1.5 * lineWidthMultiple);
hold off;
grid on;
leg3 = legend('Kd = 0.1', 'Kd = 0.5', 'Kd = 1');
leg3.ItemTokenSize = [20, 20];
leg3.FontSize = defaultFontSize;
xlabel('Time (s)', 'FontSize', defaultFontSize);
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
xlim([0 2]);
title('Changing Angle Kd Only (from Upright)', 'FontSize', defaultFontSize);
ax = gca;
ax.FontSize = defaultFontSize;

sgtitle('PID Tuning Effects on Inverted Pendulum Angle', 'FontSize', defaultFontSize);

%% Adding Position PID Control

clc
clear
close all

defaultFontSize = 24;

lineWidthMultiple = 2;

color1 = 'b';
color2 = 'r';
color3 = 'g';
color4 = 'k';
color5 = 'm';
color6 = 'c';
color7 = 'y';
color8 = [0.6 0.3 0.1];

Kp1 = readmatrix('AP_One.txt');
Kp2 = readmatrix('AP_Two.txt');
Kp3 = readmatrix('AP_Three.txt');
Kp4 = readmatrix('AP_Four.txt');

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

figure('Position', [100, 100, 800*2, 600*2]);

plot(x1/1000, y1_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color1);
hold on;
plot(x1/1000, yy1_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color2);
plot(x1/1000, y2_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color3);
plot(x1/1000, yy2_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color4);
plot(x1/1000, y3_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color5);
plot(x1/1000, yy3_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color6);
plot(x1/1000, y4_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color7);
plot(x1/1000, yy4_filtered, 'LineWidth', 1.5 * lineWidthMultiple, 'Color', color8);
hold off;

grid on;
xlim([0 4]);

legend( ...
    'Position Kp = 0, Ki = 0, Kd = 0 (Angle)', ...
    'Position Kp = 0, Ki = 0, Kd = 0 (Position)', ...
    'Position Kp = 0.15, Ki = 0, Kd = 0 (Angle)', ...
    'Position Kp = 0.15, Ki = 0, Kd = 0 (Position)', ...
    'Position Kp = 0.15, Ki = 0.1, Kd = 0 (Angle)', ...
    'Position Kp = 0.15, Ki = 0.1, Kd = 0 (Position)', ...
    'Position Kp = 0.15, Ki = 0.1, Kd = 0.06 (Angle)', ...
    'Position Kp = 0.15, Ki = 0.1, Kd = 0.06 (Position)', ...
    'FontSize', defaultFontSize);

xlabel('Time (s)', 'FontSize', defaultFontSize);
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
title('Changing Position PID Values, Angle PID Tuned: Kp = 45, Ki = 0.6, Kd = 0.003 (from Upright)', 'FontSize', defaultFontSize);

ax = gca;
ax.FontSize = defaultFontSize;

%% Cup Weights

clc
clear
close all

defaultFontSize = 24;
lineWidthMultiple = 2;

legendFontSize = defaultFontSize * 0.5;

figure('Position', [100, 100, 800*2, 600*2]);

subplot(3, 1, 1);
data1 = readmatrix('One_Cup.txt');
x1 = data1(:, 1);
y1_1 = data1(:, 2);
y2_1 = data1(:, 3);

windowSize = 15;
y1_filtered1 = movmean(y1_1, windowSize);

plot(x1/1000, y1_filtered1, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
yline(180, '--g', 'LabelHorizontalAlignment', 'right', 'LineWidth', 1.5*lineWidthMultiple, 'FontSize', defaultFontSize);
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);

yyaxis right
plot(x1/1000, y2_1-60, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

xlabel('Time (s)', 'FontSize', defaultFontSize);
leg1 = legend('Angle', 'Target', 'Cart Position', 'FontSize', legendFontSize, 'Location', 'best');
title('Single Cup Weight', 'FontSize', defaultFontSize);
grid on;
xlim([0 85]);

ax1 = gca;
ax1.FontSize = defaultFontSize;
set(ax1, 'YColor', 'k');
yyaxis left
set(ax1, 'YColor', 'k');

leg1.ItemTokenSize = [20*0.5, 20*0.5];

pos1 = get(gca, 'Position');

subplot(3, 1, 2);
data2 = readmatrix('Two_Cup_Odd_Weighting.txt');
x2 = data2(:, 1);
y1_2 = data2(:, 2);
y2_2 = data2(:, 3);

y1_filtered2 = movmean(y1_2, windowSize);

plot(x2/1000, y1_filtered2, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
yline(180, '--g', 'LabelHorizontalAlignment', 'right', 'Color', 'g', 'LineWidth', 1.5*lineWidthMultiple, 'FontSize', defaultFontSize);
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);

yyaxis right
plot(x2/1000, y2_2, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

xlabel('Time (s)', 'FontSize', defaultFontSize);
leg2 = legend('Angle', 'Target', 'Cart Position', 'FontSize', legendFontSize, 'Location', 'best');
title('Double Cup Uneven Weight', 'FontSize', defaultFontSize);
grid on;
xlim([0 25]);

ax2 = gca;
ax2.FontSize = defaultFontSize;
set(ax2, 'YColor', 'k');
yyaxis left
set(ax2, 'YColor', 'k');

leg2.ItemTokenSize = [20*0.5, 20*0.5];

subplot(3, 1, 3);
data3 = readmatrix('Two_Cup_Even_Weighting.txt');
x3 = data3(:, 1);
y1_3 = data3(:, 2);
y2_3 = data3(:, 3);

y1_filtered3 = movmean(y1_3, windowSize);

plot(x3/1000, y1_filtered3, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
yline(180, '--g', 'LabelHorizontalAlignment', 'right', 'Color', 'g', 'LineWidth', 1.5*lineWidthMultiple, 'FontSize', defaultFontSize);
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);

yyaxis right
plot(x3/1000, y2_3, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

xlabel('Time (s)', 'FontSize', defaultFontSize);
leg3 = legend('Angle', 'Target', 'Cart Position', 'FontSize', legendFontSize, 'Location', 'best');
title('Double Cup Even Weight', 'FontSize', defaultFontSize);
grid on;
xlim([0 60]);

ax3 = gca;
ax3.FontSize = defaultFontSize;
set(ax3, 'YColor', 'k');
yyaxis left
set(ax3, 'YColor', 'k');

leg3.ItemTokenSize = [20*0.5, 20*0.5];

sgtitle('Cup Weight Disturbance Effects', 'FontSize', defaultFontSize);

%% Comparing Filters

clc
clear
close all

defaultFontSize = 24;
lineWidthMultiple = 2;

legendFontSize = defaultFontSize * 0.5;

figure('Position', [100, 100, 800*2, 600*2]);

data1 = readmatrix('Long_Run.txt');

x1 = data1(1+1200:700000, 1);
y1 = data1(1+1200:700000, 2);
y2 = data1(1+1200:700000, 3);
windowSize = 15;
y1_filtered1 = movmean(y1, windowSize);
windowSizeBig = 45;
y1_filteredBig = movmean(y1, windowSizeBig);

subplot(3, 1, 1);
plot(x1/1000, y1, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
yline(180, '--g', 'LabelHorizontalAlignment', 'right', 'LineWidth', 1.5*lineWidthMultiple, 'FontSize', defaultFontSize);

yyaxis right
plot(x1/1000, y2-60, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

yyaxis left
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
xlabel('Time (s)', 'FontSize', defaultFontSize);

leg1 = legend('Angle', 'Target', 'Cart Position', 'FontSize', legendFontSize, 'Location', 'best');

title('Unfiltered', 'FontSize', defaultFontSize);
grid on;
xlim([0 10]);

ax1 = gca;
ax1.FontSize = defaultFontSize;
set(ax1, 'YColor', 'k');
yyaxis left;
set(ax1, 'YColor', 'k');

leg1.ItemTokenSize = [20*0.5, 20*0.5];

subplot(3, 1, 2);
plot(x1/1000, y1_filtered1, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
yline(180, '--g', 'LabelHorizontalAlignment', 'right', 'LineWidth', 1.5*lineWidthMultiple, 'FontSize', defaultFontSize);

yyaxis right
plot(x1/1000, y2-60, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

yyaxis left
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
xlabel('Time (s)', 'FontSize', defaultFontSize);

leg2 = legend('Angle', 'Target', 'Cart Position', 'FontSize', legendFontSize, 'Location', 'best');

title('Filtered (Small Window)', 'FontSize', defaultFontSize);
grid on;
xlim([0 10]);

ax2 = gca;
ax2.FontSize = defaultFontSize;
set(ax2, 'YColor', 'k');
yyaxis left;
set(ax2, 'YColor', 'k');

leg2.ItemTokenSize = [20*0.5, 20*0.5];

subplot(3, 1, 3);
plot(x1/1000, y1_filteredBig, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
yline(180, '--g', 'LabelHorizontalAlignment', 'right', 'LineWidth', 1.5*lineWidthMultiple, 'FontSize', defaultFontSize);

yyaxis right
plot(x1/1000, y2-60, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

yyaxis left
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
xlabel('Time (s)', 'FontSize', defaultFontSize);

leg3 = legend('Angle', 'Target', 'Cart Position', 'FontSize', legendFontSize, 'Location', 'best');

title('Filtered (Big Window)', 'FontSize', defaultFontSize);
grid on;
xlim([0 10]);

ax3 = gca;
ax3.FontSize = defaultFontSize;
set(ax3, 'YColor', 'k');
yyaxis left;
set(ax3, 'YColor', 'k');

leg3.ItemTokenSize = [20*0.5, 20*0.5];

sgtitle('Comparing Move-Mean Filter Sizes', 'FontSize', defaultFontSize);

%% Adjustable Weight

clc
clear
close all

defaultFontSize = 24;
lineWidthMultiple = 2;

legendFontSize = defaultFontSize * 0.5;

figure('Position', [100, 100, 800*2, 600*2]);

data_high = readmatrix('Adj_High.txt');
x_high = data_high(:, 1);
y1_high = data_high(:, 2);
y2_high = data_high(:, 3);
windowSize_high = 15;
y1_filtered_high = movmean(y1_high, windowSize_high);

subplot(3, 1, 1);
plot(x_high/1000, y1_filtered_high, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1, 'FontSize', defaultFontSize);

yyaxis right
plot(x_high/1000, y2_high-50, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

yyaxis left
xlabel('Time (s)', 'FontSize', defaultFontSize);
grid on;
xlim([0 35.8]);
title('Adjustable Weight (High)', 'FontSize', defaultFontSize);
legend1 = legend('Pendulum Angle','Target','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
legend1.ItemTokenSize = [20*0.5, 20*0.5];

ax1 = gca;
ax1.FontSize = defaultFontSize;
set(ax1, 'YColor', 'k');
yyaxis left
set(ax1, 'YColor', 'k');

data_middle = readmatrix('Adj_Middle.txt');
x_middle = data_middle(:, 1);
y1_middle = data_middle(:, 2);
y2_middle = data_middle(:, 3);
windowSize_middle = 15;
y1_filtered_middle = movmean(y1_middle, windowSize_middle);

subplot(3, 1, 2);
plot(x_middle/1000, y1_filtered_middle, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
ylabel('Angle (Degrees)',  'FontSize', defaultFontSize);
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1, 'FontSize', defaultFontSize);

yyaxis right
plot(x_middle/1000, y2_middle-50, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

yyaxis left
xlabel('Time (s)', 'FontSize', defaultFontSize);
grid on;
xlim([0 28.5]);
title('Adjustable Weight (Middle)', 'FontSize', defaultFontSize);
legend2 = legend('Pendulum Angle','Target','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
legend2.ItemTokenSize = [20*0.5, 20*0.5];

ax2 = gca;
ax2.FontSize = defaultFontSize;
set(ax2, 'YColor', 'k');
yyaxis left
set(ax2, 'YColor', 'k');

data_low = readmatrix('Adj_Low.txt');
x_low = data_low(:, 1);
y1_low = data_low(:, 2);
y2_low = data_low(:, 3);
windowSize_low = 30;
y1_filtered_low = movmean(y1_low, windowSize_low);

subplot(3, 1, 3);
plot(x_low/1000, y1_filtered_low, 'LineWidth', 1.5*lineWidthMultiple);
hold on;
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1, 'FontSize', defaultFontSize);

yyaxis right
plot(x_low/1000, y2_low-50, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([180-270 180+270]);

yyaxis left
xlabel('Time (s)', 'FontSize', defaultFontSize);
grid on;
xlim([0 25]);
title('Adjustable Weight (Low)', 'FontSize', defaultFontSize);
legend3 = legend('Pendulum Angle','Target','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
legend3.ItemTokenSize = [20*0.5, 20*0.5];

ax3 = gca;
ax3.FontSize = defaultFontSize;
set(ax3, 'YColor', 'k');
yyaxis left
set(ax3, 'YColor', 'k');

sgtitle('Adjustable Weight Experiments', 'FontSize', defaultFontSize);

%% System With/Without Voltage Divider

clc
clear
close all

defaultFontSize = 24;
lineWidthMultiple = 2;

legendFontSize = defaultFontSize * 0.5;

figure('Position', [100, 100, 800*2, 600*2]);

data_longrun = readmatrix('Long_Run.txt');
x_longrun = data_longrun(1+1200:700000, 1);
y1_longrun = data_longrun(1+1200:700000, 2);
y2_longrun = data_longrun(1+1200:700000, 3);
windowSize = 15;
y1_filtered_longrun = movmean(y1_longrun, windowSize);
y2_filtered_longrun = movmean(y2_longrun, windowSize);

data_vd = readmatrix('Voltage_Divider.txt');
x_vd = data_vd(:, 1);
y1_vd = data_vd(:, 2);
y2_vd = data_vd(:, 3);
y1_filtered_vd = movmean(y1_vd, windowSize);

subplot(2, 1, 1);
plot(x_longrun/1000, y1_filtered_longrun, 'LineWidth', 1.5*lineWidthMultiple); hold on;
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1, 'FontSize', defaultFontSize);

yyaxis right
plot(x_longrun/1000, y2_filtered_longrun-50, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);

ylim([180-270 180+270]);

yyaxis left
xlabel('Time (s)', 'FontSize', defaultFontSize);
legend1 = legend('Pendulum Angle','Target','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
legend1.ItemTokenSize = [20*0.5, 20*0.5];
title('No Voltage Divider (No Disturbances)', 'FontSize', defaultFontSize);
grid on;
xlim([0 120]);

ax1 = gca;
ax1.FontSize = defaultFontSize;
set(ax1, 'YColor', 'k');
yyaxis left;
set(ax1, 'YColor', 'k');

subplot(2, 1, 2);
plot(x_vd/1000, y1_filtered_vd+7, 'LineWidth', 1.5*lineWidthMultiple); hold on;
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);

ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1, 'FontSize', defaultFontSize);

yyaxis right
plot(x_vd/1000, y2_vd-50, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);

ylim([180-100 180+100]);

yyaxis left

xlabel('Time (s)', 'FontSize', defaultFontSize);

legend2 = legend('Pendulum Angle','Target','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);

legend2.ItemTokenSize = [20*0.5, 20*0.5];
title('Voltage Divider (No Disturbances)',  'FontSize', defaultFontSize);

grid on;
xlim([0 120]);
ax2 = gca;
ax2.FontSize = defaultFontSize;
set(ax2, 'YColor', 'k');
yyaxis left;
set(ax2, 'YColor', 'k');

sgtitle(['Comparing the use of a Voltage Divider'],  'FontSize', defaultFontSize)

%% System + Distrubances With/Without Voltage Divider

clc
clear
close all

defaultFontSize = 24;
lineWidthMultiple = 2;

legendFontSize = defaultFontSize * 0.5;

figure('Position', [100, 100, 800*2, 600*2]);

data_vd_dist = readmatrix('Voltage_Divider_Disturbance.txt');
x_vd_dist = data_vd_dist(:, 1);
y1_vd_dist = data_vd_dist(:, 2);
y2_vd_dist = data_vd_dist(:, 3);
windowSize = 15;
y1_filtered_vd_dist = movmean(y1_vd_dist, windowSize);

data_normal_dist = readmatrix('Normal_Disturb.txt');
x_normal_dist = data_normal_dist(1200:25800, 1);
y1_normal_dist = data_normal_dist(1200:25800, 2);
y2_normal_dist = data_normal_dist(1200:25800, 3);
y1_filtered_normal_dist = movmean(y1_normal_dist, windowSize);
y2_filtered_normal_dist = movmean(y2_normal_dist, windowSize);

subplot(2, 1, 1);
plot(x_normal_dist/1000, y1_filtered_normal_dist-8.5, 'LineWidth', 1.5*lineWidthMultiple); hold on;
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1, 'FontSize', defaultFontSize);

yyaxis right
plot(x_normal_dist/1000, y2_filtered_normal_dist+120, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([200 550]);

yyaxis left
xlabel('Time (s)', 'FontSize', defaultFontSize);
grid on;
xlim([0 40]);
title('No Voltage Divider (With Disturbances)', 'FontSize', defaultFontSize);
legend1 = legend('Pendulum Angle','Target','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
legend1.ItemTokenSize = [20*0.5, 20*0.5];

ax1 = gca;
ax1.FontSize = defaultFontSize;
set(ax1, 'YColor', 'k');
yyaxis left;
set(ax1, 'YColor', 'k');

subplot(2, 1, 2);
plot(x_vd_dist/1000, y1_filtered_vd_dist+7.5, 'LineWidth', 1.5*lineWidthMultiple); hold on;
ylabel('Angle (Degrees)', 'FontSize', defaultFontSize);
ylim([170 190]);
yline(180, 'r', 'LabelHorizontalAlignment', 'right','Color', [0 0.65 0 0.2],'LineWidth',1, 'FontSize', defaultFontSize);

yyaxis right
plot(x_vd_dist/1000, y2_vd_dist+120, 'LineWidth', 1.5*lineWidthMultiple);
ylabel('Cart Position (mm)', 'FontSize', defaultFontSize);
ylim([200 450]);

yyaxis left
xlabel('Time (s)', 'FontSize', defaultFontSize);
grid on;
xlim([0 14.5]);
title('Voltage Divider (With Disturbances)', 'FontSize', defaultFontSize);
legend2 = legend('Pendulum Angle','Target','Cart Position', 'Location', 'best', 'FontSize', legendFontSize);
legend2.ItemTokenSize = [20*0.5, 20*0.5];

ax2 = gca;
ax2.FontSize = defaultFontSize;
set(ax2, 'YColor', 'k');
yyaxis left;
set(ax2, 'YColor', 'k');

sgtitle('Comparing the use of a Voltage Divider (With Disturbances)',  'FontSize', defaultFontSize);