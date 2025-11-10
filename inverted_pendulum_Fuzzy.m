% Inverted Pendulum Control with Fuzzy Logic
% Author: Modified for academic report
% Date: 2025-11-10
% Description: This program implements a fuzzy logic controller for
%              stabilizing an inverted pendulum system
close all; clear;
addpath ./sub_functions

% Create output directory for figures
current_dir = pwd;
figures_dir = fullfile(current_dir, 'figures');
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
X   = [-0.5; 0; 15*pi/180; 0];  % Initial state
xd      = 0;        % Desired cart position [m]
xd_dot  = 0;        % Desired cart velocity [m/s]
qd      = 0;        % Desired pendulum angle [rad] (upright)
qd_dot  = 0;        % Desired angular velocity [rad/s]

%% Fuzzy Control Parameters
% Normalization factors for mapping physical values to fuzzy domain [-1,1]
x_normal = 12;              % Cart position normalization [m]
dx_normal = 1.5;            % Cart velocity normalization [m/s]
q_normal = 360*pi/180;      % Pendulum angle normalization [rad]
dq_normal = 180*pi/180;     % Angular velocity normalization [rad/s]
u_normal = 1000;            % Control force normalization [N]

% Control gains for dual-controller system
gain_x = 1;     % Cart position controller gain
gain_q = 2;     % Pendulum angle controller gain

%% Data Storage Initialization
X_Fuzzy = zeros(T_final/Ts+1, 4);    % State history matrix
time_Fuzzy = zeros(T_final/Ts+1, 1); % Time vector
F_save = zeros(T_final/Ts+1, 1);     % Control force history
u_save = zeros(T_final/Ts+1, 2);     % Individual controller outputs [u_x, u_q]
fuzzy_data = zeros(T_final/Ts+1, 4); % Fuzzy variables [ex, dex, eq, deq]

X_Fuzzy(1,:) = X';
count = 2;
x_prev = X(1);  % Previous cart position for derivative calculation
q_prev = X(3);  % Previous pendulum angle for derivative calculation

fprintf('Starting fuzzy control simulation...\n');

%% Main Control Loop
for time = Ts:Ts:T_final
    % Calculate errors
    ex  = xd - X(1);         % Cart position error [m]
    dex = x_prev - X(1);     % Cart position error change

    eq  = qd - X(3);         % Pendulum angle error [rad]
    deq = q_prev - X(3);     % Pendulum angle error change

    % Store fuzzy input data
    fuzzy_data(count,:) = [ex, dex, eq, deq];

    % Fuzzy Control Pipeline:
    % 1. Fuzzification: Convert errors to fuzzy membership values
    % 2. Inference: Apply fuzzy rules to determine control action
    % 3. Defuzzification: Convert fuzzy output to crisp control force

    % Cart position controller
    fuzzy_x = fuzzification(ex/x_normal, dex/dx_normal);
    inference_x = fuzzy_inference(fuzzy_x);
    u_x = defuzzification(inference_x) * u_normal * gain_x;

    % Pendulum angle controller
    fuzzy_q = fuzzification(eq/q_normal, deq/dq_normal);
    inference_q = fuzzy_inference(fuzzy_q);
    u_q = defuzzification(inference_q) * u_normal * gain_q;

    u_save(count,:) = [u_x, u_q];

    % Combined control force (cart controller inverted)
    F = -u_x + u_q;

    % Update previous values for next iteration
    x_prev = X(1);
    q_prev = X(3);

    % Numerical integration using ODE45
    [T, X_next] = ode45(@diff_pendulum, [0, Ts], X);
    X = X_next(end,:)';

    % Store results
    X_Fuzzy(count,:) = X';
    time_Fuzzy(count) = time;
    F_save(count) = F;
    count = count + 1;
end

fprintf('Simulation completed.\n');

%% Results Assignment
X_result = X_Fuzzy;
Time_result = time_Fuzzy;

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
    axis([-0.8 0.8 -0.4 0.4]);
    xlabel('位置 [m]', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('高度 [m]', 'FontSize', 10, 'FontWeight', 'bold');
    title(sprintf('t = %.1f s\n角度=%.1f°', Time_result(i), X_result(i,3)*180/pi), ...
        'FontSize', 11, 'FontWeight', 'bold');
end

sgtitle('倒立摆系统状态演化 (关键时刻快照)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figures_dir, 'Fig1_System_Snapshots.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig1_System_Snapshots.png'));

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
plot(Time_result, u_save(:,1), 'b--', 'LineWidth', 1.5);
plot(Time_result, u_save(:,2), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('时间 [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('控制力 [N]', 'FontSize', 11, 'FontWeight', 'bold');
title('模糊控制器输出', 'FontSize', 12, 'FontWeight', 'bold');
legend('总控制力 F', '小车控制力 u_x', '摆杆控制力 u_q', 'Location', 'best');

sgtitle('倒立摆系统状态响应曲线', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(figures_dir, 'Fig2_State_Response.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig2_State_Response.png'));

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
settling_time_x = find(abs(error_x) < 0.05, 1) * Ts;
settling_time_q = find(abs(error_q)*180/pi < 2, 1) * Ts;
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

sgtitle('控制性能分析', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig3, fullfile(figures_dir, 'Fig3_Performance_Analysis.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig3_Performance_Analysis.png'));

%% Figure 4: Fuzzy Logic System Visualization
fprintf('Generating fuzzy logic visualization...\n');
fig4 = figure('Position', [100, 100, 1200, 800]);
set(fig4, 'Color', 'w');

% Membership functions
subplot(2,3,1);
x_range = linspace(-1, 1, 100);
hold on;
for i = 1:5
    mu = zeros(size(x_range));
    for j = 1:length(x_range)
        fuzzy_temp = fuzzification(x_range(j), 0);
        mu(j) = fuzzy_temp(i, 1);
    end
    plot(x_range, mu, 'LineWidth', 2);
end
grid on;
xlabel('归一化误差', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('隶属度', 'FontSize', 10, 'FontWeight', 'bold');
title('输入隶属函数', 'FontSize', 11, 'FontWeight', 'bold');
legend('NB', 'NS', 'ZO', 'PS', 'PB', 'Location', 'best');
ylim([0, 1.1]);

% Fuzzy rules table
subplot(2,3,2);
fuzzy_rule = [1 1 1 2 1;
              1 1 2 5 4;
              1 1 3 5 5;
              4 1 4 5 5;
              5 4 5 5 5];
imagesc(fuzzy_rule);
colormap(jet);
colorbar;
set(gca, 'XTick', 1:5, 'XTickLabel', {'NB', 'NS', 'ZO', 'PS', 'PB'});
set(gca, 'YTick', 1:5, 'YTickLabel', {'NB', 'NS', 'ZO', 'PS', 'PB'});
xlabel('Δe (误差变化率)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('e (误差)', 'FontSize', 10, 'FontWeight', 'bold');
title('模糊规则表', 'FontSize', 11, 'FontWeight', 'bold');

% Output membership functions
subplot(2,3,3);
u_range = linspace(-1, 1, 100);
hold on;
colors = {'b', 'c', 'g', 'y', 'r'};
for rule_idx = 1:5
    mu_out = zeros(size(u_range));
    for j = 1:length(u_range)
        % Simulate output membership for each rule
        if rule_idx == 1  % NB
            if u_range(j) < -2/3
                mu_out(j) = 1;
            elseif u_range(j) >= -2/3 && u_range(j) < -1/3
                mu_out(j) = -3*u_range(j) - 1;
            end
        elseif rule_idx == 2  % NS
            if u_range(j) >= -2/3 && u_range(j) < -1/3
                mu_out(j) = 3*u_range(j) + 2;
            elseif u_range(j) >= -1/3 && u_range(j) < 0
                mu_out(j) = -3*u_range(j);
            end
        elseif rule_idx == 3  % ZO
            if u_range(j) >= -1/3 && u_range(j) < 0
                mu_out(j) = 3*u_range(j) + 1;
            elseif u_range(j) >= 0 && u_range(j) < 1/3
                mu_out(j) = -3*u_range(j) + 1;
            end
        elseif rule_idx == 4  % PS
            if u_range(j) >= 0 && u_range(j) < 1/3
                mu_out(j) = 3*u_range(j);
            elseif u_range(j) >= 1/3 && u_range(j) < 2/3
                mu_out(j) = -3*u_range(j) + 2;
            end
        elseif rule_idx == 5  % PB
            if u_range(j) >= 1/3 && u_range(j) < 2/3
                mu_out(j) = 3*u_range(j) - 1;
            elseif u_range(j) >= 2/3
                mu_out(j) = 1;
            end
        end
    end
    plot(u_range, mu_out, 'Color', colors{rule_idx}, 'LineWidth', 2);
end
grid on;
xlabel('归一化控制输出', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('隶属度', 'FontSize', 10, 'FontWeight', 'bold');
title('输出隶属函数', 'FontSize', 11, 'FontWeight', 'bold');
legend('NB', 'NS', 'ZO', 'PS', 'PB', 'Location', 'best');
ylim([0, 1.1]);

% Control surface for cart
subplot(2,3,4);
e_surf = linspace(-1, 1, 30);
de_surf = linspace(-1, 1, 30);
[E, DE] = meshgrid(e_surf, de_surf);
U = zeros(size(E));
for i = 1:size(E,1)
    for j = 1:size(E,2)
        U(i,j) = defuzzification(fuzzy_inference(fuzzification(E(i,j), DE(i,j))));
    end
end
surf(E, DE, U);
shading interp;
colorbar;
xlabel('误差 e', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('误差变化 Δe', 'FontSize', 10, 'FontWeight', 'bold');
zlabel('控制输出 u', 'FontSize', 10, 'FontWeight', 'bold');
title('模糊控制曲面', 'FontSize', 11, 'FontWeight', 'bold');

% Fuzzy input-output during simulation (cart angle)
subplot(2,3,5);
scatter(fuzzy_data(:,3)*180/pi, fuzzy_data(:,4)*180/pi, 20, Time_result, 'filled');
colorbar;
xlabel('角度误差 [°]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('角速度误差 [°/s]', 'FontSize', 10, 'FontWeight', 'bold');
title('仿真过程中的模糊输入轨迹', 'FontSize', 11, 'FontWeight', 'bold');
grid on;

% Phase plane
subplot(2,3,6);
plot(X_result(:,3)*180/pi, X_result(:,4)*180/pi, 'b-', 'LineWidth', 1.5);
hold on;
plot(X_result(1,3)*180/pi, X_result(1,4)*180/pi, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(X_result(end,3)*180/pi, X_result(end,4)*180/pi, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
xlabel('摆杆角度 [°]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('角速度 [°/s]', 'FontSize', 10, 'FontWeight', 'bold');
title('相平面轨迹', 'FontSize', 11, 'FontWeight', 'bold');
legend('轨迹', '起点', '终点', 'Location', 'best');

sgtitle('模糊控制系统可视化分析', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig4, fullfile(figures_dir, 'Fig4_Fuzzy_System.png'));
fprintf('Saved: %s\n', fullfile(figures_dir, 'Fig4_Fuzzy_System.png'));

fprintf('\n=== All figures generated successfully ===\n');
fprintf('Please check the "figures" folder for output images.\n');
