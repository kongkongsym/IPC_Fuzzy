% Inverted Pendulum Control with PID
% Author: Modified for academic report
% Date: 2025-11-10
% Description: This program implements a PID controller for
%              stabilizing an inverted pendulum system

close all; clear;
addpath ./sub_functions

% Create output directory for figures
current_dir = pwd;
figures_dir = fullfile(current_dir, '..', 'figures');
if ~exist(figures_dir, 'dir')
    mkdir(figures_dir);
    fprintf('Created figures directory: %s\n', figures_dir);
end

global M m bx bq l g J F
%% Pendulum Physical Parameters
M   = 2;        % Cart mass [kg]
m   = 0.8;      % Pendulum mass [kg]
bx  = 0.005;    % Cart damping coefficient [kg/s]
bq  = 0.0005;   % Pendulum damping coefficient [kg m^2/s]
l   = 0.25;     % Pendulum length [m]
g   = 9.81;     % Gravitational acceleration [m/s^2]
J   = 0.0326;   % Moment of inertia [kg m]
F   = 0;        % Applied force [N]
T_final = 10;   % Simulation time [s]
Ts = 0.01;      % Control sampling time [s]

%% Initial Condition
% State vector: X = [cart_position; cart_velocity; pendulum_angle; angular_velocity]
X   = [0; 0; 20*pi/180; 0];  % Initial state
xd      = 0;        % Desired cart position [m]
xd_dot  = 0;        % Desired cart velocity [m/s]
qd      = 0;        % Desired pendulum angle [rad] (upright)
qd_dot  = 0;        % Desired angular velocity [rad/s]

%% PID Control Parameters
% Cart position PID gains
Kpx = 10;       % Proportional gain for cart position
Kdx = 5;        % Derivative gain for cart position
Kix = 2;        % Integral gain for cart position

% Pendulum angle PID gains
Kpq = 70;       % Proportional gain for pendulum angle
Kdq = 5;        % Derivative gain for pendulum angle
Kiq = 5;        % Integral gain for pendulum angle

% Error variables initialization
ex      = 0;        % Cart position error
eq      = 0;        % Pendulum angle error
ex_sum  = 0;        % Accumulated cart position error (integral)
eq_sum  = 0;        % Accumulated pendulum angle error (integral)
ex_dot  = 0;        % Cart position error derivative
eq_dot  = 0;        % Pendulum angle error derivative

%% Data Storage Initialization
X_PID = zeros(T_final/Ts+1, 4);        % State history matrix
time_PID = zeros(T_final/Ts+1, 1);     % Time vector
F_save = zeros(T_final/Ts+1, 1);       % Control force history
PID_components = zeros(T_final/Ts+1, 6); % PID components [Px, Ix, Dx, Pq, Iq, Dq]

X_PID(1,:) = X';
count = 2;

fprintf('Starting PID control simulation...\n');

%% Main Control Loop
for time = Ts:Ts:T_final
    % Cart position error calculation
    ex_prev = ex;
    ex      = xd - X(1);            % Position error [m]
    ex_dot  = (ex - ex_prev)/Ts;    % Error derivative [m/s]
    ex_sum  = ex_sum + ex*Ts;       % Error integral [m·s]

    % Pendulum angle error calculation
    eq_prev = eq;
    eq      = qd - X(3);            % Angle error [rad]
    eq_dot  = (eq - eq_prev)/Ts;    % Error derivative [rad/s]
    eq_sum  = eq_sum + eq*Ts;       % Error integral [rad·s]

    % PID control law: Combined dual-controller
    Px = Kpx*ex;
    Ix = Kix*ex_sum;
    Dx = Kdx*ex_dot;
    Pq = Kpq*eq;
    Iq = Kiq*eq_sum;
    Dq = Kdq*eq_dot;

    F = -(Px + Dx + Ix) + (Pq + Dq + Iq);

    % Store PID components
    PID_components(count,:) = [Px, Ix, Dx, Pq, Iq, Dq];

    % Numerical integration using ODE45
    [T, X_next] = ode45(@diff_pendulum, [0, Ts], X);
    X = X_next(end,:)';

    % Store results
    X_PID(count,:) = X';
    time_PID(count) = time;
    F_save(count) = F;
    count = count + 1;
end

fprintf('Simulation completed.\n');

%% Results Assignment
X_result = X_PID;
Time_result = time_PID;

%% Figure 1: System Snapshots at Key Time Points
fprintf('Generating system snapshots...\n');
fig1 = figure('Position', [100, 100, 1200, 400]);
set(fig1, 'Color', 'w');

% Select key time points
snapshot_times = [0, 2, 5, 10];  % seconds
snapshot_indices = round(snapshot_times/Ts) + 1;

for idx = 1:length(snapshot_times)
    subplot(1, 4, idx);
    i = snapshot_indices(idx);

    cart_x = X_result(i,1);
    pend_x = X_result(i,1) - l*sin(X_result(i,3));
    pend_y = l*cos(X_result(i,3));

    % Draw pendulum
    hold on;
    plot(pend_x, pend_y, 'o', 'MarkerSize', 20, 'MarkerFaceColor', [0.2 0.9 0.2], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 2);

    % Draw cart
    rectangle('Position', [cart_x-0.1, -0.05, 0.2, 0.1], 'FaceColor', [0.8 0.8 0.8], ...
        'EdgeColor', 'k', 'LineWidth', 2);

    % Draw rod
    plot([cart_x, pend_x], [0, pend_y], 'k', 'LineWidth', 3);

    % Draw ground
    plot([-1, 1], [-0.05, -0.05], 'k-', 'LineWidth', 2);

    grid on;
    axis equal;
    axis([-0.3 0.3 -0.1 0.4]);
    xlabel('位置 [m]', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('高度 [m]', 'FontSize', 10, 'FontWeight', 'bold');
    title(sprintf('t = %.1f s\n角度=%.1f°', Time_result(i), X_result(i,3)*180/pi), ...
        'FontSize', 11, 'FontWeight', 'bold');
end

sgtitle('倒立摆系统状态演化 (PID控制 - 关键时刻快照)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figures_dir, 'Fig1_PID_System_Snapshots.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig1_PID_System_Snapshots.png'));

%% Figure 2: State Response Curves
fprintf('Generating state response curves...\n');
fig2 = figure('Position', [100, 100, 1000, 800]);
set(fig2, 'Color', 'w');

% Cart position and velocity
subplot(3,1,1);
yyaxis left
plot(Time_result, X_result(:,1), 'b-', 'LineWidth', 2);
ylabel('小车位置 [m]', 'FontSize', 11, 'FontWeight', 'bold');
yyaxis right
plot(Time_result, X_result(:,2), 'r--', 'LineWidth', 1.5);
ylabel('小车速度 [m/s]', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
xlabel('时间 [s]', 'FontSize', 11, 'FontWeight', 'bold');
title('小车运动状态', 'FontSize', 12, 'FontWeight', 'bold');
legend('位置', '速度', 'Location', 'best');

% Pendulum angle and angular velocity
subplot(3,1,2);
yyaxis left
h1 = plot(Time_result, X_result(:,3)*180/pi, 'b-', 'LineWidth', 2);
ylabel('摆杆角度 [°]', 'FontSize', 11, 'FontWeight', 'bold');
hold on;
h2 = yline(0, 'k--', 'LineWidth', 1.5);
yyaxis right
h3 = plot(Time_result, X_result(:,4)*180/pi, 'r--', 'LineWidth', 1.5);
ylabel('角速度 [°/s]', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
xlabel('时间 [s]', 'FontSize', 11, 'FontWeight', 'bold');
title('摆杆运动状态', 'FontSize', 12, 'FontWeight', 'bold');
legend([h1, h2, h3], '角度', '目标角度', '角速度', 'Location', 'best');

% Control forces
subplot(3,1,3);
plot(Time_result, F_save, 'k-', 'LineWidth', 2);
hold on;
plot(Time_result, -(PID_components(:,1) + PID_components(:,2) + PID_components(:,3)), ...
    'b--', 'LineWidth', 1.5);
plot(Time_result, PID_components(:,4) + PID_components(:,5) + PID_components(:,6), ...
    'r--', 'LineWidth', 1.5);
grid on;
xlabel('时间 [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('控制力 [N]', 'FontSize', 11, 'FontWeight', 'bold');
title('PID控制器输出', 'FontSize', 12, 'FontWeight', 'bold');
legend('总控制力 F', '小车控制力', '摆杆控制力', 'Location', 'best');

sgtitle('倒立摆系统状态响应曲线 (PID控制)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(figures_dir, 'Fig2_PID_State_Response.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig2_PID_State_Response.png'));

%% Figure 3: Error Analysis
fprintf('Generating error analysis...\n');
fig3 = figure('Position', [100, 100, 1000, 600]);
set(fig3, 'Color', 'w');

% Calculate tracking errors
error_x = xd - X_result(:,1);      % Cart position error
error_q = qd - X_result(:,3);      % Pendulum angle error

subplot(2,2,1);
plot(Time_result, error_x, 'b-', 'LineWidth', 2);
grid on;
xlabel('时间 [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('位置误差 [m]', 'FontSize', 11, 'FontWeight', 'bold');
title('小车位置跟踪误差', 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,2,2);
plot(Time_result, error_q*180/pi, 'r-', 'LineWidth', 2);
grid on;
xlabel('时间 [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('角度误差 [°]', 'FontSize', 11, 'FontWeight', 'bold');
title('摆杆角度跟踪误差', 'FontSize', 12, 'FontWeight', 'bold');

% Calculate performance metrics
subplot(2,2,3);
% Settling time: find the last time the error exceeds the threshold
% Then the settling time is the time when it enters and stays within the threshold
threshold_x = 0.05;  % 5cm threshold for cart position
threshold_q = 2;     % 2 degree threshold for pendulum angle

% Find last time error exceeded threshold
last_exceed_x = find(abs(error_x) > threshold_x, 1, 'last');
last_exceed_q = find(abs(error_q)*180/pi > threshold_q, 1, 'last');

% Settling time is when it permanently enters the threshold region
if isempty(last_exceed_x)
    settling_time_x = 0;  % Always within threshold
else
    settling_time_x = last_exceed_x * Ts;
end

if isempty(last_exceed_q)
    settling_time_q = 0;  % Always within threshold
else
    settling_time_q = last_exceed_q * Ts;
end

% If never settles within the simulation time, mark as "not settled"
if last_exceed_x == length(error_x)
    settling_time_x = NaN;  % Not settled
end
if last_exceed_q == length(error_q)
    settling_time_q = NaN;  % Not settled
end

overshoot_q = max(abs(error_q(1:min(500,end))))*180/pi;

bar_data = [settling_time_x, settling_time_q; overshoot_q, abs(error_x(1))];
b = bar(bar_data);
set(gca, 'XTickLabel', {'调节时间 [s]', '最大偏差'});
ylabel('数值', 'FontSize', 11, 'FontWeight', 'bold');
title('性能指标统计', 'FontSize', 12, 'FontWeight', 'bold');
legend('小车', '摆杆', 'Location', 'best');
grid on;

% Energy consumption
subplot(2,2,4);
energy = cumsum(abs(F_save) * Ts);  % Cumulative control effort
plot(Time_result, energy, 'k-', 'LineWidth', 2);
grid on;
xlabel('时间 [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('累积控制能量 [N·s]', 'FontSize', 11, 'FontWeight', 'bold');
title('控制能量消耗', 'FontSize', 12, 'FontWeight', 'bold');

sgtitle('控制性能分析 (PID控制)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig3, fullfile(figures_dir, 'Fig3_PID_Performance_Analysis.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig3_PID_Performance_Analysis.png'));

%% Figure 4: PID Components Analysis
fprintf('Generating PID components analysis...\n');
fig4 = figure('Position', [100, 100, 1200, 800]);
set(fig4, 'Color', 'w');

% Cart PID components
subplot(2,3,1);
plot(Time_result, PID_components(:,1), 'r-', 'LineWidth', 1.5);
hold on;
plot(Time_result, PID_components(:,2), 'g-', 'LineWidth', 1.5);
plot(Time_result, PID_components(:,3), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('时间 [s]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('控制分量 [N]', 'FontSize', 10, 'FontWeight', 'bold');
title('小车PID控制分量', 'FontSize', 11, 'FontWeight', 'bold');
legend('比例 (P)', '积分 (I)', '微分 (D)', 'Location', 'best');

% Pendulum PID components
subplot(2,3,2);
plot(Time_result, PID_components(:,4), 'r-', 'LineWidth', 1.5);
hold on;
plot(Time_result, PID_components(:,5), 'g-', 'LineWidth', 1.5);
plot(Time_result, PID_components(:,6), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('时间 [s]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('控制分量 [N]', 'FontSize', 10, 'FontWeight', 'bold');
title('摆杆PID控制分量', 'FontSize', 11, 'FontWeight', 'bold');
legend('比例 (P)', '积分 (I)', '微分 (D)', 'Location', 'best');

% PID contribution percentages
subplot(2,3,3);
total_cart = sum(abs(PID_components(:,1:3)));
total_pend = sum(abs(PID_components(:,4:6)));
contrib_cart = total_cart / sum(total_cart) * 100;
contrib_pend = total_pend / sum(total_pend) * 100;
bar_data_contrib = [contrib_cart; contrib_pend];
b = bar(bar_data_contrib);
set(gca, 'XTickLabel', {'小车控制', '摆杆控制'});
ylabel('贡献百分比 [%]', 'FontSize', 10, 'FontWeight', 'bold');
title('PID分量贡献比例', 'FontSize', 11, 'FontWeight', 'bold');
legend('P', 'I', 'D', 'Location', 'best');
grid on;

% Cart phase plane
subplot(2,3,4);
plot(X_result(:,1), X_result(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(X_result(1,1), X_result(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(X_result(end,1), X_result(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
xlabel('小车位置 [m]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('小车速度 [m/s]', 'FontSize', 10, 'FontWeight', 'bold');
title('小车相平面轨迹', 'FontSize', 11, 'FontWeight', 'bold');
legend('轨迹', '起点', '终点', 'Location', 'best');

% Pendulum phase plane
subplot(2,3,5);
plot(X_result(:,3)*180/pi, X_result(:,4)*180/pi, 'b-', 'LineWidth', 1.5);
hold on;
plot(X_result(1,3)*180/pi, X_result(1,4)*180/pi, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(X_result(end,3)*180/pi, X_result(end,4)*180/pi, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
xlabel('摆杆角度 [°]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('角速度 [°/s]', 'FontSize', 10, 'FontWeight', 'bold');
title('摆杆相平面轨迹', 'FontSize', 11, 'FontWeight', 'bold');
legend('轨迹', '起点', '终点', 'Location', 'best');

% Control force histogram
subplot(2,3,6);
histogram(F_save, 30, 'FaceColor', [0.3 0.6 0.8]);
grid on;
xlabel('控制力 [N]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('频次', 'FontSize', 10, 'FontWeight', 'bold');
title('控制力分布直方图', 'FontSize', 11, 'FontWeight', 'bold');

sgtitle('PID控制系统详细分析', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig4, fullfile(figures_dir, 'Fig4_PID_Components.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig4_PID_Components.png'));

fprintf('\n=== All PID figures generated successfully ===\n');
fprintf('Please check the "figures" folder for output images.\n');