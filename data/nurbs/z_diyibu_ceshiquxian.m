% clc;clear;close all;

% 生成30个不规则的三维数据点（形成一条空间不规则曲线）
num_points = 30;
t = linspace(0, 10, num_points);  % 参数t从0到10

% 基础路径：一个扭曲的螺旋/波形，作为不规则基础
x_base = t;
y_base = 2 * sin(t) + cos(2*t);
z_base = t/2 + sin(3*t);

% 添加随机扰动，使其明显不规则（扰动幅度适中，确保连贯但无明显规律）
rng(42);  % 固定随机种子，便于复现
x = x_base + randn(num_points,1)' * 0.8;
y = y_base + randn(num_points,1)' * 0.6;
z = z_base + randn(num_points,1)' * 0.5;

data_points = [x' , y' , z'];  % (30x3) 矩阵

% 新增：缩放数据点，使单位为毫米（mm），原尺度~10单位，乘以30使跨度~300 mm，匹配论文尺度
scale = 30;  % 缩放因子（可调整为更大值，如100，使尺度更大，曲率更小）
data_points = data_points * scale;

% 步骤1: 以数据点作为"伪控制点"构建初始NURBS对象（B-spline，度3，open）
degree = 3;
obj = NURBS(data_points, degree, 'open', 1);  % weight=1 表示B-spline

% 步骤2: 调用Interpolation方法进行全局插值（自动计算真正的控制点、clamped节点向量）
obj = obj.Interpolation();

% 步骤3: 计算曲线数据（用于手动绘制平滑曲线）
obj = obj.Cal_CurveData();

% 步骤4: 手动绘制（完全自定义颜色，确保所有元素颜色明显区分）
figure;
hold on; axis equal; grid on; view(3);
title('不规则3D NURBS/B-spline插值曲线（30个数据点，单位: mm）');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

% 1. 原始数据点：绿色实心圆点（最醒目）
plot3(data_points(:,1), data_points(:,2), data_points(:,3), ...
      'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5, 'LineWidth', 1);

% 2. 插值后的平滑曲线：红色粗线（突出曲线本身）
plot3(obj.CurveData(:,1), obj.CurveData(:,2), obj.CurveData(:,3), ...
      'r-', 'LineWidth', 1.5);

% 3. 控制多边形：青色（cyan）线（与红/绿明显区分）
plot3(obj.ControlPoints(:,1), obj.ControlPoints(:,2), obj.ControlPoints(:,3), ...
      'c-', 'LineWidth', 1);

% 4. 控制点：黄色实心圆点 + 黑色边框（与绿/红/青明显区分）
plot3(obj.ControlPoints(:,1), obj.ControlPoints(:,2), obj.ControlPoints(:,3), ...
      'o', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'MarkerSize', 4, 'LineWidth', 1);

legend({'原始数据点 (绿色)', '插值平滑曲线 (红色)', '控制多边形 (青色)', '控制点 (黄色)'}, ...
       'Location', 'bestoutside');

% 步骤5: 输出关键参数到命令窗（保持不变）
disp('度 (Degree):'); disp(obj.Degree);
disp('控制点数量:'); disp(obj.Number_of_ControlPoints);
disp('控制点 (ControlPoints, 30x3, 单位: mm):'); disp(obj.ControlPoints);
disp('节点向量 (UVector):'); disp(obj.UVector');
disp('权重 (Weights, 全为1):'); disp(obj.Weights');

% 步骤6: 将控制点和节点向量保存为txt文件
% =========================================================================
% 获取当前脚本所在目录
currentFolder = fileparts(mfilename('fullpath'));
if isempty(currentFolder)
    currentFolder = pwd;  % 如果从命令窗口运行，使用当前工作目录
end

% 1. 保存控制点到txt文件（单位: mm）
controlPointsFilename = fullfile(currentFolder, 'control_points.txt');
fprintf('正在保存控制点到: %s (单位: mm)\n', controlPointsFilename);

% 使用writematrix保存控制点（每行是一个控制点的x,y,z坐标）
writematrix(obj.ControlPoints, controlPointsFilename, 'Delimiter', ' ');

% 2. 保存节点向量到txt文件
knotVectorFilename = fullfile(currentFolder, 'knot_vector.txt');
fprintf('正在保存节点向量到: %s\n', knotVectorFilename);

% 将节点向量转换为列向量并保存
knot_vector = obj.UVector';
writematrix(knot_vector, knotVectorFilename, 'Delimiter', ' ');

% % 3. (可选) 保存曲线数据到txt文件
% curveDataFilename = fullfile(currentFolder, 'curve_data.txt');
% fprintf('正在保存曲线数据到: %s (单位: mm)\n', curveDataFilename);
% writematrix(obj.CurveData, curveDataFilename, 'Delimiter', ' ');

% % 4. (可选) 保存元数据信息
% metaDataFilename = fullfile(currentFolder, 'nurbs_parameters.txt');
% fid = fopen(metaDataFilename, 'w');
% if fid ~= -1
%     fprintf(fid, 'NURBS曲线参数信息\n');
%     fprintf(fid, '=====================\n');
%     fprintf(fid, '生成时间: %s\n', datestr(now));
%     fprintf(fid, '度 (Degree): %d\n', obj.Degree);
%     fprintf(fid, '控制点数量: %d\n', obj.Number_of_ControlPoints);
%     fprintf(fid, '节点向量长度: %d\n', length(obj.UVector));
%     fprintf(fid, '\n文件说明:\n');
%     fprintf(fid, '1. control_points.txt: 控制点坐标 (每行: x y z, 单位: mm)\n');
%     fprintf(fid, '2. knot_vector.txt: 节点向量\n');
%     fprintf(fid, '3. curve_data.txt: 曲线采样点坐标 (单位: mm)\n');
%     fprintf(fid, '\nMATLAB读取示例:\n');
%     fprintf(fid, '1. control_points = readmatrix(''control_points.txt'');\n');
%     fprintf(fid, 'knot_vector = readmatrix(''knot_vector.txt'');\n');
%     fprintf(fid, 'curve_data = readmatrix(''curve_data.txt'');\n');
%     fclose(fid);
%     fprintf('元数据已保存到: %s\n', metaDataFilename);
% end
% 
% fprintf('\n所有文件已保存到目录: %s (单位: mm)\n', currentFolder);
% fprintf('============================================\n');