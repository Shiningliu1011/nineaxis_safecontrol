% clc; clear; close all;

% 目标：生成一条更容易触发短块判定的三维 NURBS 插值曲线。
% 设计思路：
% 1) 主方向保持前进，避免曲线自交或严重回折；
% 2) 在 y/z 方向叠加高频小波动，使局部关键点更密；
% 3) 再加入几个局部“急拐段”，压缩相邻关键点间弧长；
% 4) 保持与原脚本相同的输出接口，直接覆盖 control_points.txt / knot_vector.txt。

%% 1. 构造更曲折的三维数据点
num_points = 34;
t = linspace(0, 1, num_points)';

% 非均匀参数映射：中段和后段略微聚点，制造更短的局部块长
tau = t .^ 0.92;

% 主行进方向：整体仍沿 x 正向推进
Lx = 120;
x = Lx * tau;

% 高频小幅摆动：关键点更密，但不过度压低边界速度
y_wave = 8.0 * sin(12 * pi * tau) + 3.5 * sin(26 * pi * tau + 0.35);
z_wave = 6.5 * cos(11 * pi * tau + 0.2) + 2.8 * sin(24 * pi * tau - 0.45);

% 叠加局部急弯包络：让少数区段更容易形成短块
g1 = exp(-((tau - 0.23) / 0.055) .^ 2);
g2 = exp(-((tau - 0.51) / 0.050) .^ 2);
g3 = exp(-((tau - 0.77) / 0.060) .^ 2);

y_bend = 9.0 * g1 .* sin(36 * pi * tau) ...
       - 8.0 * g2 .* cos(32 * pi * tau) ...
       + 7.0 * g3 .* sin(34 * pi * tau);

z_bend = -7.5 * g1 .* cos(34 * pi * tau) ...
       + 8.5 * g2 .* sin(30 * pi * tau) ...
       - 6.5 * g3 .* cos(36 * pi * tau);

% 小幅确定性扰动，避免过于规则导致关键点过少
rng(7);
x_jitter = 0.45 * sin(18 * pi * tau) + 0.15 * randn(num_points, 1);
y_jitter = 0.60 * randn(num_points, 1);
z_jitter = 0.55 * randn(num_points, 1);

x = x + x_jitter;
y = y_wave + y_bend + y_jitter;
z = 0.22 * Lx * tau + z_wave + z_bend + z_jitter;

data_points = [x, y, z];

% 整体缩放保持在中等尺寸：
% 太大则块长过长，不易出现短块；太小则曲率过大，边界速度会整体掉太低。
scale = 1.0;
data_points = data_points * scale;

%% 2. 构造 NURBS / B-spline 插值曲线
degree = 3;
obj = NURBS(data_points, degree, 'open', 1);
obj = obj.Interpolation();
obj = obj.Cal_CurveData();

%% 3. 可视化
figure;
hold on; axis equal; grid on; view(3);
title('短块测试用 3D NURBS 插值曲线');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

plot3(data_points(:,1), data_points(:,2), data_points(:,3), ...
      'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4, 'LineWidth', 1);

plot3(obj.CurveData(:,1), obj.CurveData(:,2), obj.CurveData(:,3), ...
      'r-', 'LineWidth', 1.6);

plot3(obj.ControlPoints(:,1), obj.ControlPoints(:,2), obj.ControlPoints(:,3), ...
      'c-', 'LineWidth', 1.0);

plot3(obj.ControlPoints(:,1), obj.ControlPoints(:,2), obj.ControlPoints(:,3), ...
      'o', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'MarkerSize', 4, 'LineWidth', 1);

legend({'原始数据点', '插值曲线', '控制多边形', '控制点'}, ...
       'Location', 'bestoutside');

%% 4. 输出关键参数
disp('度 (Degree):'); disp(obj.Degree);
disp('控制点数量:'); disp(obj.Number_of_ControlPoints);
disp('控制点 (ControlPoints, 单位: mm):'); disp(obj.ControlPoints);
disp('节点向量 (UVector):'); disp(obj.UVector');
disp('权重 (Weights, 全为1):'); disp(obj.Weights');

%% 5. 保存为主程序可直接读取的 txt 文件
currentFolder = fileparts(mfilename('fullpath'));
if isempty(currentFolder)
    currentFolder = pwd;
end

controlPointsFilename = fullfile(currentFolder, 'control_points.txt');
fprintf('正在保存控制点到: %s\n', controlPointsFilename);
writematrix(obj.ControlPoints, controlPointsFilename, 'Delimiter', ' ');

knotVectorFilename = fullfile(currentFolder, 'knot_vector.txt');
fprintf('正在保存节点向量到: %s\n', knotVectorFilename);
writematrix(obj.UVector', knotVectorFilename, 'Delimiter', ' ');

fprintf('\n曲线生成完成。\n');
fprintf('建议下一步直接运行 A_di2ci_gogo.m，观察是否出现短块。\n');
fprintf('若短块仍偏少，可继续减小 Lx 或增大局部 bend 振幅。\n');