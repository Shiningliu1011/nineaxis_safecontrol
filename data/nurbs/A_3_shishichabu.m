% NURBS 实时插补 - 第三阶段 MATLAB 实现
% 说明：
% 1) 本文件直接读取第二阶段离线扫描/调度/终端补偿结果；
% 2) 实时部分仅保留速度命令生成、补偿叠加、PCI 参数更新和曲线求值；
% 3) 代码按功能分段，便于后续逐块定位 bug。

% clear; clc; close all;

%% 1. 全局参数与输入数据读取
offline_file = 'nurbs_blocks.mat';
control_file = 'control_points.txt';
knot_file = 'knot_vector.txt';
result_file = 'realtime_interpolation_results.mat';
ik_result_file = 'ik_input.mat';

if ~exist(offline_file, 'file')
	error('找不到离线结果文件 %s，请先运行第二阶段脚本。', offline_file);
end
if ~exist(control_file, 'file')
	error('找不到控制点文件 %s。', control_file);
end
if ~exist(knot_file, 'file')
	error('找不到节点矢量文件 %s。', knot_file);
end

offline = load(offline_file);
required_fields = { ...
	'delta', 'Ts', 'eps_PCI', 'beta', 'F', 'At', 'Jt', 'An', 'Jn', 'min_du', 'max_corr_iter', ...
	'block_u', 'block_S', ...
	'sched_vs', 'sched_ve', 'sched_vfs', 'sched_na', 'sched_nc', 'sched_nd', ...
	'comp_N_tail', 'comp_dV_com', 'comp_dV_seq'};
validate_required_fields(offline, required_fields);

% 与离线阶段同名的参数直接从 nurbs_blocks.mat 读取，避免修改参数时维护两份配置。
delta = offline.delta;        % 弦误差阈值，单位 mm
Ts = offline.Ts;              % 采样周期，单位 s
eps_PCI = offline.eps_PCI;    % PCI 相对速度误差阈值
beta = offline.beta;          % PCI 校正系数
F = offline.F;                % 最大允许进给率，单位 mm/s
At = offline.At;              % 切向加速度约束，单位 mm/s^2
Jt = offline.Jt;              % 切向加加速度约束，单位 mm/s^3
An = offline.An;              % 向心加速度约束，单位 mm/s^2
Jn = offline.Jn;              % 向心加加速度约束，单位 mm/s^3
min_du = offline.min_du;      % 参数最小推进量
max_pci_iter = offline.max_corr_iter;   % PCI 最大校正次数，复用离线补偿阶段设置
u_tol = 1e-10;                % 实时阶段参数边界容差

ctrl_pts = readmatrix(control_file);
knots = importdata(knot_file);
w = ones(size(ctrl_pts, 1), 1);
p = length(knots) - size(ctrl_pts, 1) - 1;

num_blocks = length(offline.block_u) - 1;
use_comp = offline.comp_N_tail > 0;

fprintf('========== 实时插补输入检查 ==========%s', newline);
fprintf('块数量: %d%s', num_blocks, newline);
fprintf('控制点数量: %d, 阶数 p = %d%s', size(ctrl_pts, 1), p, newline);
fprintf('采样周期 Ts = %.6f s, beta = %.3f, eps_PCI = %.6f%s', Ts, beta, eps_PCI, newline);
fprintf('切向约束 At = %.3f mm/s^2, Jt = %.3f mm/s^3%s', At, Jt, newline);
fprintf('向心约束 An = %.3f mm/s^2, Jn = %.3f mm/s^3%s', An, Jn, newline);
fprintf('=====================================%s%s', newline, newline);

%% 2. 预分配结果结构与调试变量
u_series = [];
position_series = [];
time_series = [];
point_index = [];
block_index = [];
block_step_index = [];
feedrate_cmd_series = [];
tangent_acc_cmd_series = [];
tangent_jerk_cmd_series = [];
actual_feedrate_series = [];
pci_iter_series = [];
pci_pred_series = [];
pci_corr_series = [];

block_debug = repmat(struct( ...
	'block_id', 0, ...
	'u_start', 0, ...
	'u_end', 0, ...
	'planned_steps', 0, ...
	'actual_new_points', 0, ...
	'extra_tail_steps', 0, ...
	'exit_reason', '', ...
	'time_start', 0, ...
	'time_end', 0, ...
	'u_last', 0), num_blocks, 1);

%% 3. 第一个块的二阶 Taylor 初始化
blk_idx = 1;
first_block = read_block_data(offline, blk_idx);
u_start_1 = first_block.u_start;
u_end_1 = first_block.u_end;

[u_hist, P_hist, init_feed, init_acc, init_jerk] = init_first_block_history( ...
	first_block, u_start_1, u_end_1, Ts, ctrl_pts, w, knots, p, min_du, u_tol);

u_series = u_hist(:);
position_series = P_hist;
time_series = (0:2)' * Ts;
point_index = (1:3)';
block_index = ones(3, 1);
block_step_index = (0:2)';
feedrate_cmd_series = init_feed(:);
tangent_acc_cmd_series = init_acc(:);
tangent_jerk_cmd_series = init_jerk(:);
v01 = norm(P_hist(2, :) - P_hist(1, :)) / Ts;
v12 = norm(P_hist(3, :) - P_hist(2, :)) / Ts;
actual_feedrate_series = [v01; v01; v12];
pci_iter_series = zeros(3, 1);
pci_pred_series = u_hist(:);
pci_corr_series = u_hist(:);

fprintf('========== 首块 Taylor 初始化完成 ==========%s', newline);
fprintf('u0 = %.10f, u1 = %.10f, u2 = %.10f%s', u_hist(1), u_hist(2), u_hist(3), newline);
fprintf('==========================================%s%s', newline, newline);

%% 4. 实时逐块插补主循环
global_sample_idx = 3;

for blk_idx = 1:max(num_blocks - 1, 0)
	blk = read_block_data(offline, blk_idx);
	N_blk = blk.Na + blk.Nc + blk.Nd;

	if blk_idx == 1
		k_blk = 2;
		block_time_start = 0;
	else
		k_blk = 0;
		block_time_start = time_series(end);
	end

	block_debug(blk_idx).block_id = blk_idx;
	block_debug(blk_idx).u_start = blk.u_start;
	block_debug(blk_idx).u_end = blk.u_end;
	block_debug(blk_idx).planned_steps = N_blk;
	block_debug(blk_idx).time_start = block_time_start;

	fprintf('---- 开始实时插补：块 %d / %d ----%s', blk_idx, num_blocks, newline);
	fprintf('u 区间 [%.10f, %.10f], 计划步数 N_blk = %d%s', blk.u_start, blk.u_end, N_blk, newline);
	fprintf('Vs = %.4f, Vfs = %.4f, Ve = %.4f, Na = %d, Nc = %d, Nd = %d%s', ...
		blk.Vs, blk.Vfs, blk.Ve, blk.Na, blk.Nc, blk.Nd, newline);

	new_points_in_block = 0;
	extra_tail_steps = 0;
	exit_reason = 'planned_steps_reached';

	while true
		current_time = time_series(end);
		V_tan = compute_Vtan_step(k_blk, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve);
		A_tan = compute_Atan_step(k_blk, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
		J_tan = compute_Jtan_step(k_blk, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
		[V_cmd, comp_idx] = apply_terminal_compensation(V_tan, k_blk, N_blk, blk.use_comp, blk.N_tail, blk.dV_seq);

		[u_next, P_next, iter_count, u_pred, u_corr] = pci_step_realtime( ...
			u_hist, P_hist, V_cmd, Ts, beta, eps_PCI, ctrl_pts, w, knots, p, ...
			blk.u_end, min_du, u_tol, max_pci_iter);

		reached_end_by_u = u_next >= blk.u_end - u_tol;
		reached_end_by_k = (k_blk + 1) >= N_blk;

		actual_step_feed = norm(P_next.' - P_hist(3, :)) / Ts;

		global_sample_idx = global_sample_idx + 1;
		u_series(end+1, 1) = u_next;
		position_series(end+1, :) = P_next.';
		time_series(end+1, 1) = current_time + Ts;
		point_index(end+1, 1) = global_sample_idx;
		block_index(end+1, 1) = blk_idx;
		block_step_index(end+1, 1) = k_blk + 1;
		feedrate_cmd_series(end+1, 1) = V_cmd;
		tangent_acc_cmd_series(end+1, 1) = A_tan;
		tangent_jerk_cmd_series(end+1, 1) = J_tan;
		actual_feedrate_series(end+1, 1) = actual_step_feed;
		pci_iter_series(end+1, 1) = iter_count;
		pci_pred_series(end+1, 1) = u_pred;
		pci_corr_series(end+1, 1) = u_corr;

		u_hist = [u_hist(2:3), u_next];
		P_hist = [P_hist(2:3, :); P_next.'];
		k_blk = k_blk + 1;
		new_points_in_block = new_points_in_block + 1;

		if reached_end_by_u && reached_end_by_k
			exit_reason = 'planned_steps_and_cross_boundary';
			break;
		end
		if reached_end_by_u
			exit_reason = 'natural_cross_boundary';
			break;
		end
		if reached_end_by_k
			exit_reason = 'planned_steps_switch_block';
			break;
		end

		if mod(new_points_in_block, 200) == 0
			fprintf('块 %d 已生成 %d 个新点，当前 u = %.10f, Vcmd = %.4f, comp_idx = %d%s', ...
				blk_idx, new_points_in_block, u_next, V_cmd, comp_idx, newline);
		end
	end

	block_debug(blk_idx).actual_new_points = new_points_in_block;
	block_debug(blk_idx).extra_tail_steps = extra_tail_steps;
	block_debug(blk_idx).exit_reason = exit_reason;
	block_debug(blk_idx).time_end = time_series(end);
	block_debug(blk_idx).u_last = u_series(end);

	fprintf('块 %d 结束：新增点数 = %d, 退出原因 = %s, 最终 u = %.10f%s%s', ...
		blk_idx, new_points_in_block, exit_reason, u_series(end), newline, newline);
end

if num_blocks >= 1
	blk_idx = num_blocks;
	blk = read_block_data(offline, blk_idx);
	[P_blk_end, ~, ~] = nurbs_eval(blk.u_end, ctrl_pts, w, knots, p);
	replan_u_start = min(max(u_hist(3), knots(1)), blk.u_end);
	replan_S = approximate_nurbs_arc_length(replan_u_start, blk.u_end, ctrl_pts, w, knots, p);
	replan_Vs = min(max(actual_feedrate_series(end), 0), F);
	replan_Ve = min(max(blk.Ve, 0), F);
	[blk.Vfs, blk.Na, blk.Nd, blk.Nc, blk.Vs, blk.Ve, replan_step_used] = ...
		schedule_single_block(replan_Vs, replan_Ve, replan_S, F, At, Jt, Ts);
	blk.use_comp = false;
	blk.N_tail = 0;
	blk.dV_seq = [];
	N_blk = blk.Na + blk.Nc + blk.Nd;

	if blk_idx == 1
		k_blk = 2;
		block_time_start = 0;
	else
		k_blk = 0;
		block_time_start = time_series(end);
	end

	block_debug(blk_idx).block_id = blk_idx;
	block_debug(blk_idx).u_start = replan_u_start;
	block_debug(blk_idx).u_end = blk.u_end;
	block_debug(blk_idx).planned_steps = N_blk;
	block_debug(blk_idx).time_start = block_time_start;

	fprintf('---- 开始实时插补：块 %d / %d ----%s', blk_idx, num_blocks, newline);
	fprintf('实时重规划 u 区间 [%.10f, %.10f], 重算剩余弧长 S = %.6f mm%s', ...
		replan_u_start, blk.u_end, replan_S, newline);
	fprintf('重规划速度：Vs = %.4f, Vfs = %.4f, Ve = %.4f, Na = %d, Nc = %d, Nd = %d, Step = %d%s', ...
		blk.Vs, blk.Vfs, blk.Ve, blk.Na, blk.Nc, blk.Nd, replan_step_used, newline);

	new_points_in_block = 0;
	extra_tail_steps = 0;
	exit_reason = 'planned_steps_reached';

	while true
		current_time = time_series(end);
		if k_blk >= N_blk
			V_tan = 0;
			A_tan = 0;
			J_tan = 0;
			V_cmd = 0;
			comp_idx = 0;
			u_next = blk.u_end;
			P_next = P_blk_end;
			iter_count = 0;
			u_pred = u_hist(3);
			u_corr = blk.u_end;
		else
			V_tan = compute_Vtan_step(k_blk, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve);
			A_tan = compute_Atan_step(k_blk, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
			J_tan = compute_Jtan_step(k_blk, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
			[V_cmd, comp_idx] = apply_terminal_compensation(V_tan, k_blk, N_blk, blk.use_comp, blk.N_tail, blk.dV_seq);

			[u_next, P_next, iter_count, u_pred, u_corr] = pci_step_realtime( ...
				u_hist, P_hist, V_cmd, Ts, beta, eps_PCI, ctrl_pts, w, knots, p, ...
				blk.u_end, min_du, u_tol, max_pci_iter);
		end

		reached_end_by_u = u_next >= blk.u_end - u_tol;
		reached_end_by_k = (k_blk + 1) >= N_blk;
		if reached_end_by_u
			u_next = blk.u_end;
			[P_next, ~, ~] = nurbs_eval(u_next, ctrl_pts, w, knots, p);
		end
		remaining_chord_after = norm(P_blk_end - P_next);

		actual_step_feed = norm(P_next.' - P_hist(3, :)) / Ts;

		global_sample_idx = global_sample_idx + 1;
		u_series(end+1, 1) = u_next;
		position_series(end+1, :) = P_next.';
		time_series(end+1, 1) = current_time + Ts;
		point_index(end+1, 1) = global_sample_idx;
		block_index(end+1, 1) = blk_idx;
		block_step_index(end+1, 1) = k_blk + 1;
		feedrate_cmd_series(end+1, 1) = V_cmd;
		tangent_acc_cmd_series(end+1, 1) = A_tan;
		tangent_jerk_cmd_series(end+1, 1) = J_tan;
		actual_feedrate_series(end+1, 1) = actual_step_feed;
		pci_iter_series(end+1, 1) = iter_count;
		pci_pred_series(end+1, 1) = u_pred;
		pci_corr_series(end+1, 1) = u_corr;

		u_hist = [u_hist(2:3), u_next];
		P_hist = [P_hist(2:3, :); P_next.'];
		k_blk = k_blk + 1;
		new_points_in_block = new_points_in_block + 1;

		if reached_end_by_u
			exit_reason = 'parameter_reached_end';
			break;
		end
		if u_next > blk.u_end + 1e-8
			exit_reason = 'parameter_exceeded_end';
			break;
		end
		if reached_end_by_k
			if remaining_chord_after <= max(1e-4, F * Ts)
				exit_reason = 'planned_steps_end_within_tolerance';
			else
				extra_tail_steps = 1;
				exit_reason = 'planned_steps_without_reaching_end';
				warning('块 %d 经末块重规划后仍未在计划步数内到达终点，剩余弦长 %.6e mm。', ...
					blk_idx, remaining_chord_after);
			end
			break;
		end

		if mod(new_points_in_block, 200) == 0
			fprintf('块 %d 已生成 %d 个新点，当前 u = %.10f, Vcmd = %.4f, comp_idx = %d%s', ...
				blk_idx, new_points_in_block, u_next, V_cmd, comp_idx, newline);
		end
	end

	block_debug(blk_idx).actual_new_points = new_points_in_block;
	block_debug(blk_idx).extra_tail_steps = extra_tail_steps;
	block_debug(blk_idx).exit_reason = exit_reason;
	block_debug(blk_idx).time_end = time_series(end);
	block_debug(blk_idx).u_last = u_series(end);

	fprintf('块 %d 结束：新增点数 = %d, 退出原因 = %s, 最终 u = %.10f%s%s', ...
		blk_idx, new_points_in_block, exit_reason, u_series(end), newline, newline);

	if abs(u_series(end) - blk.u_end) <= u_tol
		global_sample_idx = global_sample_idx + 1;
		u_series(end+1, 1) = blk.u_end;
		position_series(end+1, :) = P_blk_end.';
		time_series(end+1, 1) = time_series(end) + Ts;
		point_index(end+1, 1) = global_sample_idx;
		block_index(end+1, 1) = blk_idx;
		block_step_index(end+1, 1) = block_step_index(end) + 1;
		feedrate_cmd_series(end+1, 1) = 0;
		tangent_acc_cmd_series(end+1, 1) = 0;
		tangent_jerk_cmd_series(end+1, 1) = 0;
		actual_feedrate_series(end+1, 1) = 0;
		pci_iter_series(end+1, 1) = 0;
		pci_pred_series(end+1, 1) = blk.u_end;
		pci_corr_series(end+1, 1) = blk.u_end;
		block_debug(blk_idx).time_end = time_series(end);
		block_debug(blk_idx).u_last = u_series(end);
	end
end

%% 5. 结果后处理：速度/加速度/加加速度/弦误差
diff_position_series = position_series;
has_terminal_hold_point = size(position_series, 1) >= 2 && ...
	norm(position_series(end, :) - position_series(end - 1, :)) <= 1e-9;
if has_terminal_hold_point
	diff_position_series = position_series(1:end-1, :);
end

velocity_series = finite_difference_series(diff_position_series, Ts);
acceleration_series = finite_difference_series(velocity_series, Ts);
jerk_series = finite_difference_series(acceleration_series, Ts);
if has_terminal_hold_point
	velocity_series(end+1, :) = 0;
	acceleration_series(end+1, :) = 0;
	jerk_series(end+1, :) = 0;
end

speed_series = align_step_series_to_current_sample(actual_feedrate_series);
feedrate_plot_series = align_step_series_to_current_sample(feedrate_cmd_series);
[speed_series, feedrate_plot_series] = enforce_terminal_static_display( ...
	u_series, speed_series, feedrate_plot_series, u_tol);
acceleration_norm_series = vecnorm(acceleration_series, 2, 2);
jerk_norm_series = vecnorm(jerk_series, 2, 2);
tangent_unit_series = compute_tangent_unit_series(u_series, ctrl_pts, w, knots, p);
tangent_acc_projection_series = tangent_acc_cmd_series(:) .* tangent_unit_series;
tangent_jerk_projection_series = tangent_jerk_cmd_series(:) .* tangent_unit_series;

[chord_error_time, chord_error_series] = compute_chord_error_series( ...
	time_series, u_series, position_series, ctrl_pts, w, knots, p);

boundary_point_index = compute_block_switch_indices(block_index, length(offline.block_u));
boundary_step_index = boundary_point_index;   % 当前实现中采样步序列与插补点序列一一对应
boundary_points = zeros(length(offline.block_u), 3);
for idx = 1:length(offline.block_u)
	[C_block, ~, ~] = nurbs_eval(offline.block_u(idx), ctrl_pts, w, knots, p);
	boundary_points(idx, :) = C_block.';
end

%% 6. 输出保存结果
num_points = size(position_series, 1);   % 插补点总数

ik_input = struct();
ik_input.Ts = Ts;   % 实时插补采样周期，单位 s
ik_input.num_points = num_points;   % 插补点总数
ik_input.num_blocks = num_blocks;   % NURBS 分块总数
ik_input.point_index = point_index;   % 插补点序号，长度为 num_points x 1
ik_input.time_series = time_series;   % 每个插补点对应的时间戳，单位 s
ik_input.u_series = u_series;   % 每个插补点对应的 NURBS 参数值
ik_input.position_series = position_series;   % 末端位置序列，每行为 [x, y, z]，单位 mm
ik_input.block_index = block_index;   % 每个插补点所属的 NURBS 分块编号
ik_input.velocity_series = velocity_series;   % 末端线速度序列，每行为 [vx, vy, vz]，单位 mm/s
ik_input.speed_series = speed_series;   % 末端速度模长序列，单位 mm/s
ik_input.acceleration_series = acceleration_series;   % 末端线加速度序列，单位 mm/s^2
ik_input.acceleration_norm_series = acceleration_norm_series;   % 末端加速度模长序列，单位 mm/s^2
ik_input.jerk_series = jerk_series;   % 末端线加加速度序列，单位 mm/s^3
ik_input.jerk_norm_series = jerk_norm_series;   % 末端加加速度模长序列，单位 mm/s^3
ik_input.tangent_acc_cmd_series = tangent_acc_cmd_series;   % 切向加速度命令序列，单位 mm/s^2
ik_input.tangent_acc_projection_series = tangent_acc_projection_series;   % 切向加速度在 X/Y/Z 轴上的投影，单位 mm/s^2
ik_input.tangent_jerk_cmd_series = tangent_jerk_cmd_series;   % 切向加加速度命令序列，单位 mm/s^3
ik_input.tangent_jerk_projection_series = tangent_jerk_projection_series;   % 切向加加速度在 X/Y/Z 轴上的投影，单位 mm/s^3
ik_input.feedrate_cmd_series = feedrate_cmd_series;   % 每个采样步的命令进给速度，单位 mm/s
ik_input.boundary_point_index = boundary_point_index;   % 分块边界在插补点序列中的位置索引
ik_input.boundary_step_index = boundary_step_index;   % 分块边界在采样步序列中的位置索引
ik_input.chord_error_series = chord_error_series;   % 相邻插补点对应的弦误差序列，单位 mm
ik_input.remark = '当前版本仅包含位置型反解输入，姿态信息后续补充';   % 备注说明

ik_input.field_description = struct();
ik_input.field_description.Ts = '实时插补采样周期，单位 s';
ik_input.field_description.num_points = '插补点总数';
ik_input.field_description.num_blocks = 'NURBS 分块总数';
ik_input.field_description.point_index = '插补点序号，长度为 num_points x 1';
ik_input.field_description.time_series = '每个插补点对应的时间戳，长度为 num_points x 1，单位 s';
ik_input.field_description.u_series = '每个插补点对应的 NURBS 曲线参数值，长度为 num_points x 1';
ik_input.field_description.position_series = '末端位置序列，大小为 num_points x 3，每行为 [x, y, z]，单位 mm';
ik_input.field_description.block_index = '每个插补点所属的 NURBS 分块编号，长度为 num_points x 1';
ik_input.field_description.velocity_series = '末端线速度序列，大小为 num_points x 3，每行为 [vx, vy, vz]，单位 mm/s';
ik_input.field_description.speed_series = '末端速度模长序列，长度为 num_points x 1，单位 mm/s';
ik_input.field_description.acceleration_series = '末端线加速度序列，大小为 num_points x 3，每行为 [ax, ay, az]，单位 mm/s^2';
ik_input.field_description.acceleration_norm_series = '末端加速度模长序列，长度为 num_points x 1，单位 mm/s^2';
ik_input.field_description.jerk_series = '末端线加加速度序列，大小为 num_points x 3，每行为 [jx, jy, jz]，单位 mm/s^3';
ik_input.field_description.jerk_norm_series = '末端加加速度模长序列，长度为 num_points x 1，单位 mm/s^3';
ik_input.field_description.tangent_acc_cmd_series = '切向加速度命令序列，长度为 num_points x 1，单位 mm/s^2';
ik_input.field_description.tangent_acc_projection_series = '切向加速度在 X/Y/Z 轴上的投影，大小为 num_points x 3，单位 mm/s^2';
ik_input.field_description.tangent_jerk_cmd_series = '切向加加速度命令序列，长度为 num_points x 1，单位 mm/s^3';
ik_input.field_description.tangent_jerk_projection_series = '切向加加速度在 X/Y/Z 轴上的投影，大小为 num_points x 3，单位 mm/s^3';
ik_input.field_description.feedrate_cmd_series = '每个采样步的命令进给速度，长度为 num_points x 1，单位 mm/s';
ik_input.field_description.boundary_point_index = '各 NURBS 分块边界在插补点序列中的位置索引，长度为 num_blocks + 1';
ik_input.field_description.boundary_step_index = '各 NURBS 分块边界在采样步序列中的位置索引，长度为 num_blocks + 1';
ik_input.field_description.chord_error_series = '相邻插补点对应的弦误差序列，长度为 num_points - 1，单位 mm';
ik_input.field_description.remark = '当前版本数据范围与用途说明';
ik_input.field_description.field_description = '各字段含义说明结构体';

save(result_file, ...
	'Ts', 'num_points', 'num_blocks', ...
	'point_index', 'time_series', 'u_series', 'position_series', 'block_index', ...
	'velocity_series', 'speed_series', ...
	'acceleration_series', 'acceleration_norm_series', ...
	'jerk_series', 'jerk_norm_series', ...
	'tangent_acc_cmd_series', 'tangent_acc_projection_series', ...
	'tangent_jerk_cmd_series', 'tangent_jerk_projection_series', ...
	'feedrate_cmd_series', ...
	'boundary_point_index', 'boundary_step_index', ...
	'chord_error_series');
save(ik_result_file, 'ik_input');
fprintf('实时插补结果已保存至 %s%s', result_file, newline);
fprintf('反解输入结果已保存至 %s%s', ik_result_file, newline);

%% 7. 绘图输出
plot_results(time_series, chord_error_time, chord_error_series, ...
	position_series, u_series, ctrl_pts, w, knots, p, offline.block_u(:), ...
	boundary_points, boundary_point_index, speed_series, feedrate_plot_series, ...
	acceleration_norm_series, jerk_norm_series, velocity_series, ...
	acceleration_series, jerk_series, tangent_acc_cmd_series, tangent_acc_projection_series, ...
	tangent_jerk_cmd_series, tangent_jerk_projection_series);

fprintf('%s========== 实时插补完成 ==========%s', newline, newline);
fprintf('总插补点数: %d%s', size(position_series, 1), newline);
fprintf('总时长: %.6f s%s', time_series(end), newline);
fprintf('===================================%s', newline);

%% ================================================================
%  以下为局部函数定义
%  ================================================================

function validate_required_fields(data_struct, field_names)
	for i = 1:length(field_names)
		if ~isfield(data_struct, field_names{i})
			error('离线结果中缺少字段 %s。', field_names{i});
		end
	end
end

function blk = read_block_data(offline, blk_idx)
	blk = struct();
	blk.u_start = offline.block_u(blk_idx);
	blk.u_end = offline.block_u(blk_idx + 1);
	blk.Vs = offline.sched_vs(blk_idx);
	blk.Vfs = offline.sched_vfs(blk_idx);
	blk.Ve = offline.sched_ve(blk_idx);
	blk.Na = offline.sched_na(blk_idx);
	blk.Nc = offline.sched_nc(blk_idx);
	blk.Nd = offline.sched_nd(blk_idx);
	blk.use_comp = offline.comp_N_tail(blk_idx) > 0;
	blk.N_tail = offline.comp_N_tail(blk_idx);
	if blk.use_comp && ~isempty(offline.comp_dV_seq{blk_idx})
		blk.dV_seq = offline.comp_dV_seq{blk_idx}(:);
	else
		blk.dV_seq = [];
	end
end

function [u_hist, P_hist, init_feed, init_acc, init_jerk] = init_first_block_history( ...
		blk, u0, u_end, Ts, ctrl_pts, w, knots, p, min_du, u_tol)
	[P0, C0d, C0dd] = nurbs_eval(u0, ctrl_pts, w, knots, p);

	V0 = compute_Vtan_step(0, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve);
	A0 = compute_Atan_step(0, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
	J0 = compute_Jtan_step(0, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
	u1 = taylor_step_with_guard(u0, V0, A0, Ts, C0d, C0dd, u_end, min_du, u_tol);

	[P1, C1d, C1dd] = nurbs_eval(u1, ctrl_pts, w, knots, p);
	V1 = compute_Vtan_step(1, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve);
	A1 = compute_Atan_step(1, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
	J1 = compute_Jtan_step(1, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
	u2 = taylor_step_with_guard(u1, V1, A1, Ts, C1d, C1dd, u_end, min_du, u_tol);

	[P2, ~, ~] = nurbs_eval(u2, ctrl_pts, w, knots, p);
	V2 = compute_Vtan_step(2, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve);
	A2 = compute_Atan_step(2, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);
	J2 = compute_Jtan_step(2, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve, Ts);

	u_hist = [u0, u1, u2];
	P_hist = [P0.'; P1.'; P2.'];
	init_feed = [V0; V1; V2];
	init_acc = [A0; A1; A2];
	init_jerk = [J0; J1; J2];
end

function u_next = taylor_step_with_guard(u_curr, V_curr, A_curr, Ts, Cd, Cdd, u_end, min_du, u_tol)
	speed_norm = norm(Cd);
	if speed_norm < 1e-12
		du = min_du;
	else
		curvature_term = dot(Cd, Cdd) / max(speed_norm^3, 1e-12);
		du = V_curr * Ts / speed_norm + (A_curr - curvature_term * V_curr^2) * Ts^2 / (2 * speed_norm);
		du = max(du, min_du);
	end

	u_next = u_curr + du;
	upper_guard = max(u_curr + min_du, min(u_end, 1 - u_tol));
	u_next = min(u_next, upper_guard);

	if u_end <= u_curr + min_du
		u_next = u_end;
	end
end

function [V_cmd, comp_idx] = apply_terminal_compensation(V_tan, k_blk, N_blk, use_comp, N_tail, dV_seq)
	V_cmd = V_tan;
	comp_idx = 0;

	if ~use_comp || N_tail <= 0 || isempty(dV_seq)
		return;
	end

	comp_start = N_blk - N_tail;
	if k_blk >= comp_start
		comp_idx = k_blk - comp_start + 1;
		comp_idx = min(max(comp_idx, 1), length(dV_seq));
		V_cmd = V_tan + dV_seq(comp_idx);
	end
end

function [comp_seq, comp_info] = build_last_block_error_compensation( ...
		blk, u_hist_init, P_hist_init, Ts, beta, eps_PCI, ctrl_pts, w, knots, p, ...
		min_du, u_tol, max_pci_iter)
	N_blk = blk.Na + blk.Nc + blk.Nd;
	base_cmd_seq = zeros(N_blk, 1);
	for k = 0:(N_blk - 1)
		V_tan = compute_Vtan_step(k, blk.Na, blk.Nc, blk.Nd, blk.Vs, blk.Vfs, blk.Ve);
		base_cmd_seq(k + 1) = apply_terminal_compensation(V_tan, k, N_blk, blk.use_comp, blk.N_tail, blk.dV_seq);
	end

	comp_seq = zeros(N_blk, 1);
	shape = build_zero_end_comp_shape(N_blk);
	sim_info = simulate_last_block_profile(blk, u_hist_init, P_hist_init, base_cmd_seq, ...
		Ts, beta, eps_PCI, ctrl_pts, w, knots, p, min_du, u_tol, max_pci_iter);
	initial_error = compute_last_block_signed_error(sim_info, base_cmd_seq, Ts);
	current_error = initial_error;

	for iter = 1:3
		if abs(current_error) <= 1e-4 || sum(shape) <= 1e-12
			break;
		end

		delta_seq = current_error / (Ts * sum(shape)) * shape;
		comp_seq = clamp_last_block_compensation(base_cmd_seq, comp_seq + delta_seq);

		sim_info = simulate_last_block_profile(blk, u_hist_init, P_hist_init, base_cmd_seq + comp_seq, ...
			Ts, beta, eps_PCI, ctrl_pts, w, knots, p, min_du, u_tol, max_pci_iter);
		new_error = compute_last_block_signed_error(sim_info, base_cmd_seq + comp_seq, Ts);
		if abs(new_error) >= abs(current_error)
			break;
		end
		current_error = new_error;
	end

	comp_info = struct();
	if initial_error > 1e-4
		comp_info.mode = 'undershoot';
	elseif initial_error < -1e-4
		comp_info.mode = 'overshoot';
	else
		comp_info.mode = 'matched';
	end
	comp_info.initial_error = initial_error;
	comp_info.final_error = current_error;
	comp_info.max_comp_delta = max(abs(comp_seq));
	comp_info.hit_step = sim_info.hit_step;
end

function shape = build_zero_end_comp_shape(N_blk)
	if N_blk <= 2
		shape = zeros(N_blk, 1);
		return;
	end

	k_vec = (0:(N_blk - 1)).';
	shape = sin(pi * k_vec / (N_blk - 1));
	shape(1) = 0;
	shape(end) = 0;
end

function comp_seq = clamp_last_block_compensation(base_cmd_seq, comp_seq)
	min_cmd = 1e-9;
	comp_seq = max(comp_seq, min_cmd - base_cmd_seq);
	comp_seq(end) = 0;
	comp_seq(1) = 0;
end

function sim_info = simulate_last_block_profile(blk, u_hist_init, P_hist_init, cmd_seq, ...
		Ts, beta, eps_PCI, ctrl_pts, w, knots, p, min_du, u_tol, max_pci_iter)
	u_hist_sim = u_hist_init;
	P_hist_sim = P_hist_init;
	[P_end, ~, ~] = nurbs_eval(blk.u_end, ctrl_pts, w, knots, p);

	sim_info = struct();
	sim_info.hit_step = 0;
	sim_info.reached_end = false;
	sim_info.final_u = u_hist_sim(3);
	sim_info.remaining_chord = norm(P_end - P_hist_sim(3, :).');

	for step = 1:length(cmd_seq)
		V_cmd = max(cmd_seq(step), 1e-9);
		[u_next, P_next, ~, ~, ~] = pci_step_realtime( ...
			u_hist_sim, P_hist_sim, V_cmd, Ts, beta, eps_PCI, ctrl_pts, w, knots, p, ...
			blk.u_end, min_du, u_tol, max_pci_iter);

		u_hist_sim = [u_hist_sim(2:3), u_next];
		P_hist_sim = [P_hist_sim(2:3, :); P_next.'];

		if u_next >= blk.u_end - u_tol
			sim_info.hit_step = step;
			sim_info.reached_end = true;
			sim_info.final_u = blk.u_end;
			sim_info.remaining_chord = 0;
			return;
		end
	end

	sim_info.final_u = u_hist_sim(3);
	sim_info.remaining_chord = norm(P_end - P_hist_sim(3, :).');
end

function signed_error = compute_last_block_signed_error(sim_info, cmd_seq, Ts)
	if sim_info.reached_end
		if sim_info.hit_step >= length(cmd_seq)
			signed_error = 0;
		else
			signed_error = -sum(cmd_seq((sim_info.hit_step + 1):end)) * Ts;
		end
	else
		signed_error = sim_info.remaining_chord;
	end
end

function [u_next, P_next, iter_count, u_pred, u_corr] = pci_step_realtime( ...
		u_hist, P_hist, V_cmd, Ts, beta, eps_PCI, ctrl_pts, w, knots, p, ...
		u_end, min_du, u_tol, max_pci_iter)
	u_n2 = u_hist(1);
	u_n1 = u_hist(2);
	u_n = u_hist(3);
	P_n = P_hist(3, :).';

	u_pred = 3 * u_n - 3 * u_n1 + u_n2;
	u_pred = min(max(u_pred, u_n + min_du), min(u_end, 1 - u_tol));

	if u_pred <= u_n + min_du
		u_pred = min(u_n + max(10 * min_du, min_du), min(u_end, 1 - u_tol));
	end

	u_corr = u_pred;
	iter_count = 0;

	for iter = 1:max_pci_iter
		iter_count = iter;
		[P_corr, ~, ~] = nurbs_eval(u_corr, ctrl_pts, w, knots, p);
		V_actual = norm(P_corr - P_n) / Ts;

		if V_cmd <= 1e-12
			break;
		end

		rel_err = abs(V_cmd - V_actual) / max(V_cmd, 1e-12);
		denom = V_cmd - beta * (V_cmd - V_actual);
		if abs(denom) < 1e-12
			alpha = 1;
		else
			alpha = V_cmd / denom;
		end
		alpha = max(min(alpha, 2.0), 0.1);

		u_new = u_n + alpha * (u_corr - u_n);
		u_corr = min(max(u_new, u_n + min_du), min(u_end, 1 - u_tol));

		if rel_err <= eps_PCI
			break;
		end
	end

	if u_corr <= u_n + min_du
		[~, Cd_fb, ~] = nurbs_eval(u_n, ctrl_pts, w, knots, p);
		speed_fb = norm(Cd_fb);
		if speed_fb < 1e-12
			du_fb = min_du;
		else
			du_fb = V_cmd * Ts / speed_fb;
		end
		u_corr = min(max(u_n + max(du_fb, min_du), u_n + min_du), min(u_end, 1 - u_tol));
	end

	u_next = u_corr;
	[P_next, ~, ~] = nurbs_eval(u_next, ctrl_pts, w, knots, p);
end

function S = approximate_nurbs_arc_length(u_start, u_end, ctrl_pts, w, knots, p)
	if u_end <= u_start
		S = 0;
		return;
	end

	% 末块实时重规划只执行一次，使用较密参数采样估算剩余弧长。
	n_segments = max(200, ceil(abs(u_end - u_start) * 5000));
	u_vec = linspace(u_start, u_end, n_segments + 1);
	pos = zeros(n_segments + 1, 3);
	for i = 1:(n_segments + 1)
		[P, ~, ~] = nurbs_eval(u_vec(i), ctrl_pts, w, knots, p);
		pos(i, :) = P.';
	end

	diffs = diff(pos, 1, 1);
	S = sum(vecnorm(diffs, 2, 2));
end

function [vfs, na, nd, nc, vs_out, ve_out, step_used] = ...
		schedule_single_block(Vs, Ve, Si, Vmax, At, Jt, Ts)
	Vs = min(max(Vs, 0), Vmax);
	Ve = min(max(Ve, 0), Vmax);
	Si = max(Si, 0);
	vs_out = Vs;
	ve_out = Ve;

	vfs_I = Vmax;
	[na_I, nd_I] = compute_NaNd(vfs_I, Vs, Ve, At, Jt, Ts);
	Sa_I = (Vs + vfs_I) * na_I * Ts / 2;
	Sd_I = (Ve + vfs_I) * nd_I * Ts / 2;
	nc_I = 0;
	if vfs_I * Ts > 0
		nc_I = (Si - Sa_I - Sd_I) / (vfs_I * Ts);
	end
	nc_I_floor = floor(nc_I);

	if nc_I_floor >= 1
		vfs = vfs_I;
		na = na_I;
		nd = nd_I;
		nc = nc_I_floor;
		step_used = 1;
		return;
	end

	Vfs_lo = max(max(Vs, Ve), 1e-6);
	Vfs_hi = Vmax;
	[na_lo, nd_lo] = compute_NaNd(Vfs_lo, Vs, Ve, At, Jt, Ts);
	S_lo = (Vs + Vfs_lo) * na_lo * Ts / 2 + (Ve + Vfs_lo) * nd_lo * Ts / 2;

	if S_lo <= Si
		best_vfs = Vfs_lo;
		best_na = na_lo;
		best_nd = nd_lo;
		best_S = S_lo;
		for iter = 1:50
			Vfs_mid = (Vfs_lo + Vfs_hi) / 2;
			[na_m, nd_m] = compute_NaNd(Vfs_mid, Vs, Ve, At, Jt, Ts);
			S_m = (Vs + Vfs_mid) * na_m * Ts / 2 + (Ve + Vfs_mid) * nd_m * Ts / 2;
			if S_m <= Si
				Vfs_lo = Vfs_mid;
				best_vfs = Vfs_mid;
				best_na = na_m;
				best_nd = nd_m;
				best_S = S_m;
			else
				Vfs_hi = Vfs_mid;
			end
			if Vfs_hi - Vfs_lo < 0.01
				break;
			end
		end

		vfs = best_vfs;
		na = best_na;
		nd = best_nd;
		remaining = Si - best_S;
		if remaining > 0 && vfs * Ts > 0
			nc = max(0, floor(remaining / (vfs * Ts)));
		else
			nc = 0;
		end
		step_used = 2;
		return;
	end

	if abs(Vs - Ve) > 1e-6
		Vfs_lo3 = max(min(Vs, Ve), 1e-6);
		Vfs_hi3 = max(Vs, Ve);
		best_vfs = Vfs_lo3;
		best_na = 0;
		best_nd = 0;

		for iter = 1:50
			Vfs_mid = (Vfs_lo3 + Vfs_hi3) / 2;
			[na_m, nd_m] = compute_NaNd(Vfs_mid, Vs, Ve, At, Jt, Ts);
			S_m = (Vs + Vfs_mid) * na_m * Ts / 2 + (Ve + Vfs_mid) * nd_m * Ts / 2;
			if S_m <= Si
				Vfs_lo3 = Vfs_mid;
				best_vfs = Vfs_mid;
				best_na = na_m;
				best_nd = nd_m;
			else
				Vfs_hi3 = Vfs_mid;
			end
			if Vfs_hi3 - Vfs_lo3 < 0.01
				break;
			end
		end

		vfs = best_vfs;
		na = best_na;
		nd = best_nd;
		nc = 0;
		if Vs > Ve
			vs_out = vfs;
		else
			ve_out = vfs;
		end
		step_used = 3;
		return;
	end

	V_min = max(min(Vs, Ve), 1e-6);
	nc = max(1, ceil(Si / (V_min * Ts)));
	vfs = Si / (nc * Ts);
	vs_out = vfs;
	ve_out = vfs;
	na = 0;
	nd = 0;
	step_used = 4;
end

function [Na, Nd] = compute_NaNd(Vfs, Vs, Ve, At, Jt, Ts)
	delta_v_a = (Vfs - Vs) / 2;
	if delta_v_a <= 0
		Na = 0;
	else
		Na_by_acc = delta_v_a * pi / (At * Ts);
		Na_by_jerk = sqrt(delta_v_a / Jt * (pi / Ts)^2);
		Na = ceil(max(Na_by_acc, Na_by_jerk));
	end

	delta_v_d = (Vfs - Ve) / 2;
	if delta_v_d <= 0
		Nd = 0;
	else
		Nd_by_acc = delta_v_d * pi / (At * Ts);
		Nd_by_jerk = sqrt(delta_v_d / Jt * (pi / Ts)^2);
		Nd = ceil(max(Nd_by_acc, Nd_by_jerk));
	end
end

function V = compute_Vtan_step(k, Na, Nc, Nd, Vs, Vfs, Ve)
	if Na > 0 && k < Na
		V = (Vfs - Vs) / 2 * (sin(pi * (k / Na - 0.5)) + 1) + Vs;
	elseif k < Na + Nc
		V = Vfs;
	elseif Nd > 0
		j_dec = k - Na - Nc;
		V = (Vfs - Ve) / 2 * (sin(pi * (j_dec / Nd - 1.5)) + 1) + Ve;
	else
		V = Vfs;
	end
	V = max(V, 1e-9);
end

function A = compute_Atan_step(k, Na, Nc, Nd, Vs, Vfs, Ve, Ts)
	if Na > 0 && k < Na
		Ta = Na * Ts;
		t = k * Ts;
		A = (Vfs - Vs) / 2 * (pi / Ta) * cos(pi * (t / Ta - 0.5));
	elseif k < Na + Nc
		A = 0;
	elseif Nd > 0
		Td = Nd * Ts;
		t = (k - Na - Nc) * Ts;
		A = (Vfs - Ve) / 2 * (pi / Td) * cos(pi * (t / Td - 1.5));
	else
		A = 0;
	end
end

function J = compute_Jtan_step(k, Na, Nc, Nd, Vs, Vfs, Ve, Ts)
	if Na > 0 && k < Na
		Ta = Na * Ts;
		t = k * Ts;
		J = -(Vfs - Vs) / 2 * (pi / Ta)^2 * sin(pi * (t / Ta - 0.5));
	elseif k < Na + Nc
		J = 0;
	elseif Nd > 0
		Td = Nd * Ts;
		t = (k - Na - Nc) * Ts;
		J = -(Vfs - Ve) / 2 * (pi / Td)^2 * sin(pi * (t / Td - 1.5));
	else
		J = 0;
	end
end

function series_d = finite_difference_series(series, Ts)
	[n, dim] = size(series);
	series_d = zeros(n, dim);

	if n == 1
		return;
	end

	if n == 2
		diff_val = (series(2, :) - series(1, :)) / Ts;
		series_d(1, :) = diff_val;
		series_d(2, :) = diff_val;
		return;
	end

	series_d(1, :) = (-3 * series(1, :) + 4 * series(2, :) - series(3, :)) / (2 * Ts);
	for i = 2:n-1
		series_d(i, :) = (series(i + 1, :) - series(i - 1, :)) / (2 * Ts);
	end
	series_d(n, :) = (3 * series(n, :) - 4 * series(n - 1, :) + series(n - 2, :)) / (2 * Ts);
end

function [chord_error_time, chord_error_series] = compute_chord_error_series( ...
		time_series, u_series, position_series, ctrl_pts, w, knots, p)
	num_segments = length(u_series) - 1;
	chord_error_time = zeros(num_segments, 1);
	chord_error_series = zeros(num_segments, 1);

	for i = 1:num_segments
		u_mid = 0.5 * (u_series(i) + u_series(i + 1));
		[C_mid, ~, ~] = nurbs_eval(u_mid, ctrl_pts, w, knots, p);
		chord_mid = 0.5 * (position_series(i, :) + position_series(i + 1, :));
		chord_error_series(i) = norm(C_mid.' - chord_mid);
		chord_error_time(i) = 0.5 * (time_series(i) + time_series(i + 1));
	end
end

function tangent_unit_series = compute_tangent_unit_series(u_series, ctrl_pts, w, knots, p)
	tangent_unit_series = zeros(length(u_series), 3);
	last_valid_tangent = [1, 0, 0];

	for i = 1:length(u_series)
		[~, Cd, ~] = nurbs_eval(u_series(i), ctrl_pts, w, knots, p);
		tangent_norm = norm(Cd);
		if tangent_norm > 1e-12
			last_valid_tangent = (Cd / tangent_norm).';
		end
		tangent_unit_series(i, :) = last_valid_tangent;
	end
end

function boundary_indices = compute_block_switch_indices(block_index, num_boundaries)
	boundary_indices = zeros(num_boundaries, 1);
	boundary_indices(1) = 1;

	for i = 2:(num_boundaries - 1)
		idx = find(block_index == i, 1, 'first');
		if isempty(idx)
			idx = boundary_indices(i - 1);
		end
		boundary_indices(i) = idx;
	end

	boundary_indices(end) = length(block_index);
end

function aligned_series = align_step_series_to_current_sample(step_series)
	aligned_series = step_series(:);
	if isempty(aligned_series)
		return;
	end
	if length(aligned_series) == 1
		return;
	end
	aligned_series(1:end-1) = aligned_series(2:end);
end

function [speed_plot_series, cmd_plot_series] = enforce_terminal_static_display( ...
		u_series, speed_plot_series, cmd_plot_series, u_tol)
	terminal_mask = abs(u_series - u_series(end)) <= max(u_tol, 1e-12);
	terminal_indices = find(terminal_mask);
	if length(terminal_indices) <= 1
		return;
	end

	terminal_indices = terminal_indices(1:end-1);
	speed_plot_series(terminal_indices) = NaN;
	cmd_plot_series(terminal_indices) = NaN;
end

function plot_results(time_series, chord_error_time, chord_error_series, ...
		position_series, u_series, ctrl_pts, w, knots, p, block_u, boundary_points, ...
		boundary_indices, speed_series, feedrate_cmd_series, acceleration_norm_series, ...
		jerk_norm_series, velocity_series, acceleration_series, jerk_series, ...
		tangent_acc_cmd_series, tangent_acc_projection_series, ...
		tangent_jerk_cmd_series, tangent_jerk_projection_series)
	label_offset = max(range(boundary_points, 1), 1) * 0.01;

	figure('Name', 'Chord Error', 'Color', 'w');
	plot(chord_error_time, chord_error_series, 'b-', 'LineWidth', 1.4);
	xlabel('Time (s)'); ylabel('Chord error (mm)'); title('Chord Error Curve'); grid on;

	figure('Name', 'Acceleration Curve', 'Color', 'w');
	plot(time_series, tangent_acc_cmd_series, 'r-', 'LineWidth', 1.4);
	xlabel('Time (s)'); ylabel('Tangential acceleration (mm/s^2)'); title('Acceleration Curve'); grid on;

	figure('Name', 'Jerk Curve', 'Color', 'w');
	plot(time_series, tangent_jerk_cmd_series, 'm-', 'LineWidth', 1.4);
	xlabel('Time (s)'); ylabel('Tangential jerk (mm/s^3)'); title('Jerk Curve'); grid on;

	figure('Name', 'Experimental Reference Acceleration Norm', 'Color', 'w');
	plot(time_series, acceleration_norm_series, 'r-', 'LineWidth', 1.4);
	xlabel('Time (s)'); ylabel('|a| (mm/s^2)'); title('Experimental Reference Acceleration Norm Curve'); grid on;

	figure('Name', 'Experimental Reference Jerk Norm', 'Color', 'w');
	plot(time_series, jerk_norm_series, 'm-', 'LineWidth', 1.4);
	xlabel('Time (s)'); ylabel('|j| (mm/s^3)'); title('Experimental Reference Jerk Norm Curve'); grid on;

	figure('Name', 'NURBS Curve With Blocks', 'Color', 'w');
	plot3(position_series(:,1), position_series(:,2), position_series(:,3), 'k-', 'LineWidth', 1.0); hold on;
	scatter3(boundary_points(:,1), boundary_points(:,2), boundary_points(:,3), 48, 'r', 'filled');
	for i = 1:size(boundary_points, 1)
		text(boundary_points(i,1) + label_offset(1), boundary_points(i,2) + label_offset(2), ...
			boundary_points(i,3) + label_offset(3), sprintf('%d', i), ...
			'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold');
	end
	xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
	title('Full NURBS Curve With Ordered Block Labels'); grid on; axis equal; view(3);
	legend({'Realtime interpolation', 'Block boundaries'}, 'Location', 'best');

	figure('Name', 'Speed Curve', 'Color', 'w');
	plot(time_series, speed_series, 'b-', 'LineWidth', 1.4); hold on;
	plot(time_series, feedrate_cmd_series, '--', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2);
	scatter(time_series(boundary_indices), speed_series(boundary_indices), 35, 'r', 'filled');
	for i = 1:length(boundary_indices)
		text(time_series(boundary_indices(i)), speed_series(boundary_indices(i)), sprintf('  %d', i), ...
			'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold');
	end
	xlabel('Time (s)'); ylabel('Speed (mm/s)'); title('Speed Curve With Ordered Block Labels');
	legend({'Actual speed', 'Command feedrate', 'Block boundaries'}, 'Location', 'best');
	grid on;

	figure('Name', 'XYZ Velocity', 'Color', 'w');
	tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
	nexttile; plot(time_series, velocity_series(:,1), 'r-', 'LineWidth', 1.2); grid on; ylabel('Vx'); title('X/Y/Z Velocity');
	nexttile; plot(time_series, velocity_series(:,2), 'g-', 'LineWidth', 1.2); grid on; ylabel('Vy');
	nexttile; plot(time_series, velocity_series(:,3), 'b-', 'LineWidth', 1.2); grid on; ylabel('Vz'); xlabel('Time (s)');

	figure('Name', 'X/Y/Z Acceleration', 'Color', 'w');
	tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
	nexttile; plot(time_series, tangent_acc_projection_series(:,1), 'r-', 'LineWidth', 1.2); grid on; ylabel('Ax'); title('X/Y/Z Acceleration');
	nexttile; plot(time_series, tangent_acc_projection_series(:,2), 'g-', 'LineWidth', 1.2); grid on; ylabel('Ay');
	nexttile; plot(time_series, tangent_acc_projection_series(:,3), 'b-', 'LineWidth', 1.2); grid on; ylabel('Az'); xlabel('Time (s)');

	figure('Name', 'X/Y/Z Jerk', 'Color', 'w');
	tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
	nexttile; plot(time_series, tangent_jerk_projection_series(:,1), 'r-', 'LineWidth', 1.2); grid on; ylabel('Jx'); title('X/Y/Z Jerk');
	nexttile; plot(time_series, tangent_jerk_projection_series(:,2), 'g-', 'LineWidth', 1.2); grid on; ylabel('Jy');
	nexttile; plot(time_series, tangent_jerk_projection_series(:,3), 'b-', 'LineWidth', 1.2); grid on; ylabel('Jz'); xlabel('Time (s)');

	figure('Name', 'Experimental Reference XYZ Acceleration', 'Color', 'w');
	tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
	nexttile; plot(time_series, acceleration_series(:,1), 'r-', 'LineWidth', 1.2); grid on; ylabel('Ax'); title('Experimental Reference X/Y/Z Acceleration');
	nexttile; plot(time_series, acceleration_series(:,2), 'g-', 'LineWidth', 1.2); grid on; ylabel('Ay');
	nexttile; plot(time_series, acceleration_series(:,3), 'b-', 'LineWidth', 1.2); grid on; ylabel('Az'); xlabel('Time (s)');

	figure('Name', 'Experimental Reference XYZ Jerk', 'Color', 'w');
	tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
	nexttile; plot(time_series, jerk_series(:,1), 'r-', 'LineWidth', 1.2); grid on; ylabel('Jx'); title('Experimental Reference X/Y/Z Jerk');
	nexttile; plot(time_series, jerk_series(:,2), 'g-', 'LineWidth', 1.2); grid on; ylabel('Jy');
	nexttile; plot(time_series, jerk_series(:,3), 'b-', 'LineWidth', 1.2); grid on; ylabel('Jz'); xlabel('Time (s)');
end

function N = basis_func(i, p, u, knots)
	if p == 0
		tol = 1e-10;
		if abs(u - knots(1)) < tol
			target_i = 0;
			for jj = 1:length(knots)-1
				if knots(jj) + tol < knots(jj + 1)
					target_i = jj;
					break;
				end
			end
			N = double(i == target_i);
		elseif abs(u - knots(end)) < tol
			target_i = 0;
			for jj = length(knots)-1:-1:1
				if knots(jj) + tol < knots(jj + 1)
					target_i = jj;
					break;
				end
			end
			N = double(i == target_i);
		else
			N = double(u >= knots(i) - tol && u < knots(i + 1) + tol);
		end
		return;
	end

	denom1 = knots(i + p) - knots(i);
	denom2 = knots(i + p + 1) - knots(i + 1);
	c1 = 0;
	c2 = 0;

	if abs(denom1) > 1e-10
		c1 = (u - knots(i)) / denom1 * basis_func(i, p - 1, u, knots);
	end
	if abs(denom2) > 1e-10
		c2 = (knots(i + p + 1) - u) / denom2 * basis_func(i + 1, p - 1, u, knots);
	end

	N = c1 + c2;
end

function dN = basis_func_deriv(i, p, u, knots)
	if p == 0
		dN = 0;
		return;
	end

	denom1 = knots(i + p) - knots(i);
	denom2 = knots(i + p + 1) - knots(i + 1);
	term1 = 0;
	term2 = 0;

	if denom1 > 1e-10
		term1 = p / denom1 * basis_func(i, p - 1, u, knots);
	end
	if denom2 > 1e-10
		term2 = p / denom2 * basis_func(i + 1, p - 1, u, knots);
	end

	dN = term1 - term2;
end

function ddN = basis_func_second_deriv(i, p, u, knots)
	if p <= 1
		ddN = 0;
		return;
	end

	denom1 = knots(i + p) - knots(i);
	denom2 = knots(i + p + 1) - knots(i + 1);
	term1 = 0;
	term2 = 0;

	if denom1 > 1e-10
		term1 = p / denom1 * basis_func_deriv(i, p - 1, u, knots);
	end
	if denom2 > 1e-10
		term2 = p / denom2 * basis_func_deriv(i + 1, p - 1, u, knots);
	end

	ddN = term1 - term2;
end

function [C, Cd, Cdd] = nurbs_eval(u, ctrl_pts, w, knots, p)
	n = size(ctrl_pts, 1) - 1;
	sum_NwP = zeros(3, 1);
	sum_Nw = 0;
	sum_dNwP = zeros(3, 1);
	sum_dNw = 0;
	sum_ddNwP = zeros(3, 1);
	sum_ddNw = 0;

	for i = 0:n
		ii = i + 1;
		N = basis_func(ii, p, u, knots);
		dN = basis_func_deriv(ii, p, u, knots);
		ddN = basis_func_second_deriv(ii, p, u, knots);

		wi = w(ii);
		Pi = ctrl_pts(ii, :).';

		sum_NwP = sum_NwP + N * wi * Pi;
		sum_Nw = sum_Nw + N * wi;
		sum_dNwP = sum_dNwP + dN * wi * Pi;
		sum_dNw = sum_dNw + dN * wi;
		sum_ddNwP = sum_ddNwP + ddN * wi * Pi;
		sum_ddNw = sum_ddNw + ddN * wi;
	end

	if sum_Nw < 1e-12
		error('nurbs_eval 分母过小，可能存在无效参数值 u = %.12f。', u);
	end

	C = sum_NwP / sum_Nw;
	Cd = (sum_dNwP * sum_Nw - sum_NwP * sum_dNw) / (sum_Nw ^ 2);
	temp = sum_dNwP * sum_Nw - sum_NwP * sum_dNw;
	Cdd = (sum_ddNwP * sum_Nw - sum_NwP * sum_ddNw) / (sum_Nw ^ 2) ...
		- 2 * temp * sum_dNw / (sum_Nw ^ 3);
end












