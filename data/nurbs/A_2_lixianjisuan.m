% NURBS 曲线自适应进给率规划 - MATLAB 实现
% 根据提供的逻辑文档、控制点和节点向量生成关键点、边界速度和块长度

clear; clc; close all;

%% 1. 参数初始化（从文档中直接读取）
delta = 1e-3;           % chord tolerance δ = 1 µm = 0.001 mm
F = 80;                % desired/maximum feedrate V_max 或 F (mm/s)
At = 100;               % Tangential acceleration limit (mm/s²)
Jt = 16400;             % Tangential jerk limit (mm/s³)
An = 400;               % Centripetal acceleration limit (mm/s²)
Jn = 16400;             % Centripetal jerk limit (mm/s³)
Ts = 0.002;             % Sampling time Ts = 0.002 s
eps = 0.1e-3;           % Length estimation error ε = 0.1 µm = 0.0001 mm
eps_PCI = 0.001;        % Feedrate error of PCI εPCI = 0.1% = 0.001
beta = 0.9;             % The correctional coefficient of PCI β = 0.9
Fmin = 10;              % Minimum feedrate Fmin = 20 mm/s
kcbc = 1.01;            % Curvature constant kcbc = 1.01 / mm

%% 2. 读取 NURBS 数据
% 直接读取数据（无需减一）
ctrl_pts = readmatrix('control_points.txt');  % 30x3
knots = importdata('knot_vector.txt');        % 1x34
p = 3;                                        % 已知阶数
w = ones(size(ctrl_pts,1), 1);                % 权重全1，长度自动匹配 30

fprintf('NURBS 曲线信息：阶数 p = %d, 控制点数 = %d, 节点数 = %d\n', p, size(ctrl_pts, 1), length(knots));



%% 4. 初始化起点参数 u0 = 0
u0 = 0;
[C0, C0d, C0dd] = nurbs_eval(u0, ctrl_pts, w, knots, p);

% fprintf('u0=%.6f\n', u0);
% fprintf('C0d=[%.6f, %.6f, %.6f]\n', C0d);
% fprintf('norm(C0d)=%.6f\n', norm(C0d));
% fprintf('C0dd=[%.6f, %.6f, %.6f]\n', C0dd);
% fprintf('cross_prod norm=%.6f\n', norm(cross(C0d, C0dd)));


kappa0 = curvature(u0, ctrl_pts, w, knots, p);

rho0 = 1 / max(kappa0, 1e-6);   % 曲率半径

% V_af (chord error 约束)
V_af0 = min(F, 2/Ts * sqrt(rho0^2 - (rho0 - delta)^2));

% V_cbf (curvature based feedrate)
V_cbf0 = kcbc / (kcbc + kappa0) * F;

% 期望进给率 V*
V_star0 = max(min([V_af0, V_cbf0, F]), Fmin);


% fprintf('      V_star0=%.2f\n  '       ,        V_star0   );
% fprintf('    \n')
% fprintf(' \n')
% fprintf('      V_af0=%.2f\n  '       ,       V_af0  );
% fprintf('      V_cbf0=%.2f\n  '       ,       V_cbf0 );
% fprintf('      rho0=%.2f\n  '       ,       rho0 );
% fprintf('      kappa0=%.2f\n  '       ,       kappa0 );



%% 5. 使用 Taylor 展开初始化前两个点 u1, u2
% 假设起点加速度 A(u0) ≈ 0 (简化，实际可从 C'' 计算)
A0 = 0;  % 或更精确计算

du1 = V_star0 * Ts / norm(C0d) + ...
      1/norm(C0d) * (A0 - (dot(C0d, C0dd)/norm(C0d)^3) * V_star0^2) * Ts^2 / 2;
u1 = u0 + du1;

% 重新计算 u1 处的量
[C1, C1d, C1dd] = nurbs_eval(u1, ctrl_pts, w, knots, p);
kappa1 = curvature(u1, ctrl_pts, w, knots, p);
rho1 = 1 / max(kappa1, 1e-6);
V_af1 = min(F, 2/Ts * sqrt(rho1^2 - (rho1 - delta)^2));
V_cbf1 = kcbc / (kcbc + kappa1) * F;
V_star1 = max(min([V_af1, V_cbf1, F]), Fmin);

du2 = V_star1 * Ts / norm(C1d) + ...
      1/norm(C1d) * (A0 - (dot(C1d, C1dd)/norm(C1d)^3) * V_star1^2) * Ts^2 / 2;
u2 = u1 + du2;

% 重新计算 u2 处的量（这是新增的关键步骤）
[C2, C2d, C2dd] = nurbs_eval(u2, ctrl_pts, w, knots, p);
kappa2 = curvature(u2, ctrl_pts, w, knots, p);
rho2 = 1 / max(kappa2, 1e-6);
V_af2 = min(F, 2/Ts * sqrt(rho2^2 - (rho2 - delta)^2));
V_cbf2 = kcbc / (kcbc + kappa2) * F;
V_star2 = max(min([V_af2, V_cbf2, F]), Fmin);  % 新增：计算V_star2

% %% 调试输出：u1和u2的相关参数
% fprintf('\n========== Taylor展开初始化参数调试输出 ==========\n');
% fprintf('【参数u1的详细计算信息】\n');
% fprintf('  u0 = %.6f, du1 = %.6e, u1 = %.6f\n', u0, du1, u1);
% fprintf('  V_af1 = %.6f, V_cbf1 = %.6f, V_star1 = %.6f\n', V_af1, V_cbf1, V_star1);
% fprintf('  C1 位置: [%.6f, %.6f, %.6f]\n', C1(1), C1(2), C1(3));
% 
% fprintf('\n【参数u2的详细计算信息】\n');
% fprintf('  u1 = %.6f, du2 = %.6e, u2 = %.6f\n', u1, du2, u2);
% fprintf('  V_af2 = %.6f, V_cbf2 = %.6f, V_star2 = %.6f\n', V_af2, V_cbf2, V_star2);
% fprintf('  C2 位置: [%.6f, %.6f, %.6f]\n', C2(1), C2(2), C2(3));


%% 初始化起点和前两个点的位置（在 Taylor 初始化后添加）
pos_list = [C0'; C1'; C2'];  % 初始化 u0, u1, u2 的 C


%% 6. PCI 离线扫描生成采样点序列（完善版：修复边界越界问题 + 进度显示）
u_list = [u0, u1, u2];
V_star_list = [V_star0, V_star1, V_star2];  % 包含 V_star2
kappa_list = [kappa0, kappa1, kappa2];      % 包含 kappa2

max_iter = 20000;  % 适当增大上限，避免过早停止（可根据需要调整）
tol_PCI = eps_PCI;
min_du = 1e-10;    % 最小参数推进步长，防止完全卡死
u_max_tol = 1 - 1e-10;  % 新增：u 上界容差，防止浮点超1

% 进度显示相关变量
progress_interval = 100;
fprintf('PCI 采样点生成开始...\n');
fprintf('初始点数: %d (u = %.6f)\n', length(u_list), u_list(end));

% 连续重复u计数器（用于检测卡死）
stuck_count = 0;
max_stuck = 50;  % 连续重复超过此值视为严重卡死，强制小步推进

while u_list(end) < 0.9999 && length(u_list) < max_iter
    prev_u = u_list(end);
    
    % 预测
    u_hat = 3*u_list(end) - 3*u_list(end-1) + u_list(end-2);
    
    % 新增：预测后立即夹紧 u_hat，防止 overshoot >1
    u_hat = min(max(u_hat, u_list(end) + min_du), u_max_tol);
    
    % 若预测步长过小或倒退，提前使用备用步长
    if u_hat <= u_list(end) + min_du
        u_hat = u_list(end) + min_du * 10;  % 强制稍大预测
        u_hat = min(u_hat, u_max_tol);  % 再次夹紧
    end
    
    % 校正迭代
    u_next = u_hat;
    j = 0;
    while true
        j = j + 1;
        [C_next, ~, ~] = nurbs_eval(u_next, ctrl_pts, w, knots, p);
        V_j = norm(C_next - pos_list(end,:)') / Ts;
        
        denom = V_star_list(end) - beta * (V_star_list(end) - V_j);
        if abs(denom) < 1e-10
            alpha = 1;
        else
            alpha = V_star_list(end) / denom;
        end
        
        % 限制 alpha 范围，防止极端负值导致大幅回退
        alpha = max(min(alpha, 2.0), 0.1);  % 限制在 [0.1, 2.0]，避免负值或过大
        
        u_new = u_list(end) + alpha * (u_next - u_list(end));  % 改写公式，更直观
        
        % 新增：计算 u_new 后立即夹紧，防止下次迭代 u_next >1 或 <u(end)
        u_next = min(max(u_new, u_list(end) + min_du), u_max_tol);
        
        if abs(V_star_list(end) - V_j)/V_star_list(end) <= tol_PCI || j > 20
            break;
        end
        % 无需再设置 u_next = u_new，因为已夹紧
    end
    
    % 关键修复：如果校正后 u_next 没有有效推进，使用备用一阶 Taylor 步长
    if u_next <= u_list(end) + min_du || stuck_count > max_stuck
        [~, Cd, ~] = nurbs_eval(u_list(end), ctrl_pts, w, knots, p);
        speed = norm(Cd);
        if speed < 1e-8  % 驻点或导数接近0
            du_fallback = min_du * 10;
        else
            du_fallback = V_star_list(end) * Ts / speed * 0.5;  % 保守半步
        end
        u_next = u_list(end) + max(du_fallback, min_du);
        u_next = min(u_next, u_max_tol);  % 新增：夹紧备用 u_next
        [C_next, ~, ~] = nurbs_eval(u_next, ctrl_pts, w, knots, p);  % 重新计算位置
        stuck_count = stuck_count + 1;
        fprintf('警告: 检测到步长过小，使用备用步长推进 (u=%.6f -> %.6f, stuck_count=%d)\n', ...
                u_list(end), u_next, stuck_count);
    else
        stuck_count = 0;  % 重置计数器
    end
    
    % 最终夹紧（冗余，但保留）
    u_next = min(max(u_next, u_list(end) + min_du), u_max_tol);
    
    % 添加新点
    u_list(end+1) = u_next;
    pos_list = [pos_list; C_next'];
    
    % 计算新点的曲率和 V*
    kappa_new = curvature(u_next, ctrl_pts, w, knots, p);
    kappa_list(end+1) = kappa_new;
    
    rho_new = 1 / max(kappa_new, 1e-6);
    V_af = min(F, 2/Ts * sqrt(max(rho_new^2 - (rho_new - delta)^2, 0)));  % 防止负数开方
    V_cbf = kcbc / (kcbc + kappa_new) * F;
    V_star_new = max(min([V_af, V_cbf, F]), Fmin);
    V_star_list(end+1) = V_star_new;
    
    % 进度显示
    if mod(length(u_list), progress_interval) == 0 || length(u_list) <= 3
        progress_percent = u_list(end) * 100;
        fprintf('进度: 已生成 %d 个点, 当前 u = %.6f (约 %.2f%%), 当前 V* = %.2f mm/s\n', ...
                length(u_list), u_list(end), progress_percent, V_star_new);
    end
end

% 强制添加终点 u=1（新增 tol 检查，避免重复添加）
if u_list(end) < 1 - min_du
    u_next = 1;
    [C_next, ~, ~] = nurbs_eval(u_next, ctrl_pts, w, knots, p);
    u_list(end+1) = u_next;
    pos_list = [pos_list; C_next'];
    kappa_new = curvature(u_next, ctrl_pts, w, knots, p);
    kappa_list(end+1) = kappa_new;
    rho_new = 1 / max(kappa_new, 1e-6);
    V_af = min(F, 2/Ts * sqrt(max(rho_new^2 - (rho_new - delta)^2, 0)));
    V_cbf = kcbc / (kcbc + kappa_new) * F;
    V_star_new = max(min([V_af, V_cbf, F]), Fmin);
    V_star_list(end+1) = V_star_new;
    
    fprintf('强制添加终点 u=1\n');
end

fprintf('PCI 采样点生成完成！\n');
fprintf('最终采样点数：%d (u = %.6f)\n', length(u_list), u_list(end));


%% 7. 计算临界曲率 κ_cr
term1 = 8*delta / ((F*Ts)^2 + 4*delta^2);
term2 = An / F^2;
term3 = (Jn / F^3)^(1/2);
kappa_cr = min([term1, term2, term3]);

% 新增：输出临界曲率及其各分量（便于调试）
fprintf('\n========== 临界曲率 κ_cr 计算结果 ==========\n');
fprintf('term1 (弦误差约束) = %.10f /mm\n', term1);
fprintf('term2 (向心加速度约束 An)   = %.10f /mm\n', term2);
fprintf('term3 (向心加加速度约束 Jn)  = %.10f /mm\n', term3);
fprintf('临界曲率 κ_cr = min(term1, term2, term3) = %.10f /mm\n', kappa_cr);
fprintf('==============================================\n\n');

%% 8. 识别关键点（局部最大曲率点）
% 筛选候选点：曲率 > kappa_cr（使用原始序列）
candidates = find(kappa_list > kappa_cr);

% 使用 findpeaks 检测局部最大点（更鲁棒，处理平顶和噪声）
if ~isempty(candidates)
    % 限制 findpeaks 只在候选区域搜索（通过子序列）
    [~, locs] = findpeaks(kappa_list(candidates), ...
                          'MinPeakHeight', kappa_cr, ...
                          'MinPeakProminence', 1e-6);  % 小突出度，避免噪声
    critical_idx = candidates(locs);  % 映射回原索引
else
    critical_idx = [];
end

% 检查起点和终点是否是局部最大（如果 > kappa_cr 且满足条件）
if kappa_list(1) > kappa_cr && kappa_list(1) >= kappa_list(2)
    critical_idx = [1; critical_idx];
end
if kappa_list(end) > kappa_cr && kappa_list(end) >= kappa_list(end-1)
    critical_idx = [critical_idx; length(kappa_list)];
end

% 去重并排序（以防）
critical_idx = unique(sort(critical_idx));

% 加入起点和终点到分块（但不计入 critical_idx 的内部关键点计数）
block_u = unique(sort([0; u_list(critical_idx)'; 1]));  % 确保唯一
block_idx = [];
for iu = 1:length(block_u)
    % 找到最接近的索引（处理浮点误差）
    [~, idx] = min(abs(u_list - block_u(iu)));
    block_idx(iu) = idx;
end
block_idx = block_idx';  % 列向量

fprintf('识别到 %d 个关键点（不含起点终点）\n', length(critical_idx) - (ismember(1, critical_idx) + ismember(length(u_list), critical_idx)));

% 可选调试：可视化曲率和关键点
figure; 
hold on;  % 先 hold on，确保后续叠加
% 绘制原始曲率序列（蓝实线）
plot(u_list, kappa_list, 'b-', 'LineWidth', 1.5);  % 原始曲率序列（加粗以突出）
% 在原始曲率序列上标记关键点（红圈）
plot(u_list(critical_idx), kappa_list(critical_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% 绘制临界曲率红虚线（从 y 轴伸出）
yline(kappa_cr, 'r--', 'kappa_cr', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'right'); 
xlabel('u'); ylabel('kappa'); title('曲率序列与关键点');
grid on;  % 添加网格，便于观察
legend('原始曲率', '关键点', 'kappa_cr', 'Location', 'best');  % 添加图例


%% 9. 计算每个边界点的限制进给率 V_i
block_V = zeros(length(block_u), 1);
block_kappa = zeros(length(block_u), 1);  % 新增：存储每个边界点的曲率

block_V(1) = 0;         % 起点速度为 0（固定不变）
block_V(end) = 0;       % 终点速度为 0（固定不变）

% 计算起点和终点的曲率
block_kappa(1) = curvature(0, ctrl_pts, w, knots, p);
block_kappa(end) = curvature(1, ctrl_pts, w, knots, p);

for k = 2:length(block_u)-1
    u_i = block_u(k);
    kappa_i = curvature(u_i, ctrl_pts, w, knots, p);
    block_kappa(k) = kappa_i;  % 存储曲率
    
    rho_i = 1 / max(kappa_i, 1e-6);
    
    % 完善：弦误差约束 V1，使用更精确的公式形式，并防止负数开方
    inside_sqrt = max(rho_i^2 - (rho_i - delta)^2, 0);
    V1 = min(F, 2 / Ts * sqrt(inside_sqrt));  % 避免 NaN
    
    % 完善：向心加速度约束 V2，添加 kappa 小值保护
    if kappa_i < 1e-6
        V2 = F;  % 如果曲率极小，V2 不受限
    else
        V2 = sqrt(An / kappa_i);
    end
    
    % 完善：向心加加速度约束 V3，添加 kappa 小值保护
    if kappa_i < 1e-6
        V3 = F;  % 如果曲率极小，V3 不受限
    else
        V3 = (Jn / kappa_i^2)^(1/3);
    end
    
    block_V(k) = max(min([V1, V2, V3, F]), Fmin);  % 完善：取 min，并夹在 Fmin 以上（避免过低）
end

% % 新增：输出调整后的速度、对应曲率和边界点（使用表格显示，便于阅读）
% fprintf('\n========== 边界点限制进给率与曲率计算结果 ==========\n');
% disp('边界点 u 值、曲率 kappa (/mm) 及其对应限制速度 V_i (mm/s):');
% output_table = table(block_u, block_kappa, block_V, ...
%                      'VariableNames', {'u (参数值)', 'kappa (曲率 /mm)', 'V_i (限制速度 mm/s)'});
% disp(output_table);
% fprintf('==============================================\n\n');


%% 10. 计算每个块的弧长 S_i（使用 pos_list 差分近似，避开积分问题）
% 假设 pos_list 是行矩阵 (点数 x 3)，从前 PCI 生成

block_S = zeros(length(block_u)-1, 1);

for i = 1:length(block_S)
    % 找到当前块对应的 pos_list 索引范围
    idx_start = block_idx(i);
    idx_end = block_idx(i+1);
    
    if idx_end <= idx_start
        block_S(i) = 0;
        continue;
    end
    
    % 子段坐标
    segment_pos = pos_list(idx_start:idx_end, :);
    
    % 差分计算弦长累加 ≈ 弧长（高密度，精度好）
    diffs = diff(segment_pos, 1, 1);  % (点数-1) x 3
    segment_lengths = vecnorm(diffs, 2, 2);  % 每段弦长
    block_S(i) = sum(segment_lengths);
end

% 计算总弧长（全 pos_list）
total_diffs = diff(pos_list, 1, 1);
total_lengths = vecnorm(total_diffs, 2, 2);
total_S = sum(total_lengths);

% 输出语句 - 显示各段弧长、累加和、与总弧长比较
fprintf('\n========== 每个块的弧长 S_i 计算结果 (pos_list 近似) ==========\n');
disp('块序号 | u_start | u_end | S_i (mm)');
for i = 1:length(block_S)
    fprintf('%d | %.6f | %.6f | %.6f\n', i, block_u(i), block_u(i+1), block_S(i));
end

cum_S = sum(block_S);
fprintf('\n各段弧长累加: %.6f mm\n', cum_S);
fprintf('总弧长 (全曲线近似): %.6f mm\n', total_S);
if abs(cum_S - total_S) < 1e-4  % 近似允许小误差
    fprintf('验证: 累加等于总弧长 (误差 < 1e-4)\n');
else
    fprintf('注意: 累加与总弧长略差 (误差 = %.6f)，因端点处理\n', abs(cum_S - total_S));
end
fprintf('==============================================\n\n');



%% 11. 输出结果（可保存为文件或用于后续速度规划）
disp('边界参数 u (block_u):');
disp(block_u');

disp('边界速度 V (mm/s):');
disp(block_V');

disp('块弧长 S (mm):');
disp(block_S');

% 可选：保存为 mat 文件
save('nurbs_blocks.mat', 'delta', 'F', 'At', 'Jt', 'An', 'Jn', 'Ts', 'eps', ...
     'eps_PCI', 'beta', 'Fmin', 'kcbc', 'min_du', ...
     'block_u', 'block_V', 'block_S');


%% ================================================================
%  以下为新编写代码：进给率调度（基于 implementation_guide.md）
%  ================================================================

%% 12. 进给率调度 - 准备调度参数
num_blocks = length(block_S);       % 块总数
Vmax = F;                           % 最大允许进给率

% 复制边界速度用于调度（调度过程中可能被修改）
sched_vs = block_V(1:end-1);       % 每块起始速度（列向量）
sched_ve = block_V(2:end);         % 每块结束速度（列向量）

% 初始化调度结果数组
sched_vfs  = zeros(num_blocks, 1);  % 块内峰值/恒速段速度
sched_na   = zeros(num_blocks, 1);  % 加速段采样数
sched_nd   = zeros(num_blocks, 1);  % 减速段采样数
sched_nc   = zeros(num_blocks, 1);  % 恒速段采样数
sched_step = zeros(num_blocks, 1);  % 记录使用了哪个 Step


%% 13. 长短块判定
is_short = false(num_blocks, 1);
Sstd_all  = zeros(num_blocks, 1);

for i = 1:num_blocks
    [is_short(i), Sstd_all(i)] = judge_block_short( ...
        sched_vs(i), sched_ve(i), block_S(i), At, Jt, Ts);
end

fprintf('\n========== 长短块判定结果 ==========\n');
for i = 1:num_blocks
    if is_short(i)
        fprintf('块 %d: 短块 (Si=%.4f < Sstd=%.4f)\n', i, block_S(i), Sstd_all(i));
    else
        fprintf('块 %d: 长块 (Si=%.4f >= Sstd=%.4f)\n', i, block_S(i), Sstd_all(i));
    end
end
fprintf('====================================\n\n');


%% 14. 全局调度循环（含连续短块反向处理）
i = 1;
while i <= num_blocks
    if i < num_blocks && is_short(i + 1)
        % —— 发现下一块为短块，搜索连续短块范围 [i+1, ..., i+k] ——
        k = 1;
        while (i + k) < num_blocks && is_short(i + k + 1)
            k = k + 1;
        end
        fprintf('块 %d ~ %d: 检测到 %d 个连续短块，执行反向调度\n', i+1, i+k, k);

        % 反向调度：只处理短块 [i+1 .. i+k]
        for idx = (i + k) : -1 : (i + 1)
            vs_old = sched_vs(idx);
            ve_old = sched_ve(idx);
            [sched_vfs(idx), sched_na(idx), sched_nd(idx), sched_nc(idx), ...
             vs_new, ve_new, sched_step(idx)] = ...
                schedule_single_block( ...
                    sched_vs(idx), sched_ve(idx), block_S(idx), Vmax, At, Jt, Ts);

            sched_vs(idx) = vs_new;
            sched_ve(idx) = ve_new;

            fprintf('短块 %d 调度后：', idx);
            change_count = 0;
            if abs(vs_new - vs_old) > 1e-6
                fprintf(' Vs %.3f -> %.3f', vs_old, vs_new);
                change_count = change_count + 1;
            end
            if abs(ve_new - ve_old) > 1e-6
                if change_count > 0
                    fprintf(',');
                end
                fprintf(' Ve %.3f -> %.3f', ve_old, ve_new);
                change_count = change_count + 1;
            end
            if change_count == 0
                fprintf(' 边界速度未修改');
            end
            fprintf('；Step=%d, Vfs=%.3f\n', sched_step(idx), sched_vfs(idx));

            % 反向传播：当前块修正后的 Vs 传给前一块的 Ve
            if idx > 1
                prev_ve_old = sched_ve(idx - 1);
                sched_ve(idx - 1) = vs_new;
                if abs(sched_ve(idx - 1) - prev_ve_old) > 1e-6
                    fprintf('  传播更新：块 %d 的 Ve %.3f -> %.3f\n', ...
                        idx - 1, prev_ve_old, sched_ve(idx - 1));
                end
            end
        end

        % 反向调度完后，sched_ve(i) 已由第一个短块回传更新
        % 对块 i（长块）执行正向调度
        [sched_vfs(i), sched_na(i), sched_nd(i), sched_nc(i), ...
         vs_new, ve_new, sched_step(i)] = ...
            schedule_single_block( ...
                sched_vs(i), sched_ve(i), block_S(i), Vmax, At, Jt, Ts);
        sched_vs(i) = vs_new;
        sched_ve(i) = ve_new;

        % 块 i 的 Vs 修正需回写给前一块
        if i > 1
            sched_ve(i - 1) = vs_new;
        end

        % 正向衔接：短块段最后一块的 Ve 传给下一块的 Vs
        if (i + k) < num_blocks
            sched_vs(i + k + 1) = sched_ve(i + k);
        end

        i = i + k + 1;
    else
        % —— 正向调度当前块 ——
        [sched_vfs(i), sched_na(i), sched_nd(i), sched_nc(i), ...
         vs_new, ve_new, sched_step(i)] = ...
            schedule_single_block( ...
                sched_vs(i), sched_ve(i), block_S(i), Vmax, At, Jt, Ts);

        sched_vs(i) = vs_new;
        sched_ve(i) = ve_new;

        % 正向传播：Ve → 下一块 Vs
        if i < num_blocks
            sched_vs(i + 1) = ve_new;
        end

        % 【修复】反向传播：若 Vs 被修正，回写前一块的 Ve
        if i > 1
            sched_ve(i - 1) = vs_new;
        end

        i = i + 1;
    end
end


%% 15. 输出调度结果
fprintf('\n========== 进给率调度结果 ==========\n');
fprintf('块  | Step |    Vs    |    Ve    |   Vfs    |  Na  |  Nd  |  Nc\n');
fprintf('----+------+----------+----------+----------+------+------+------\n');
for i = 1:num_blocks
    fprintf('%3d |  %d   | %8.3f | %8.3f | %8.3f | %4d | %4d | %4d\n', ...
        i, sched_step(i), sched_vs(i), sched_ve(i), sched_vfs(i), ...
        sched_na(i), sched_nd(i), sched_nc(i));
end
fprintf('====================================\n');

% 保存调度结果
save('nurbs_blocks.mat', 'delta', 'F', 'At', 'Jt', 'An', 'Jn', 'Ts', 'eps', ...
     'eps_PCI', 'beta', 'Fmin', 'kcbc', 'min_du', ...
     'block_u', 'block_V', 'block_S', ...
     'sched_vs', 'sched_ve', 'sched_vfs', 'sched_na', 'sched_nd', 'sched_nc');
fprintf('调度结果已保存至 nurbs_blocks.mat\n');


%% ================================================================
%  以下为新编写代码：终端误差补偿（基于 terminal_error_compensation_revision.md）
%  ================================================================

%% 16. 终端误差补偿 - 参数初始化
Ncom = 50;                       % 补偿窗口采样数（论文设定）
eps_L = 1e-4;                    % 长度误差阈值 (mm)
tol_PCI_comp = eps_PCI;           % PCI 收敛阈值（复用已有参数）
min_du_comp = 1e-10;              % 最小参数步长
u_max_comp = 1 - 1e-10;           % u 上界容差
max_corr_iter = 20;               % PCI 校正最大迭代次数

% 初始化每块补偿结果数组
comp_S_tr     = zeros(num_blocks, 1);   % 每块真实移动长度 S_i^tr
comp_delta_S  = zeros(num_blocks, 1);   % 每块长度估计误差 ΔS_i
comp_N_itr    = zeros(num_blocks, 1);   % 每块实际插补步数 N_itr
comp_N_tail   = zeros(num_blocks, 1);   % 每块有效补偿采样数 N_tail
comp_dV_com   = zeros(num_blocks, 1);   % 每块补偿幅值 ΔV_com
comp_dV_seq   = cell(num_blocks, 1);    % 每块补偿速度序列 ΔV(j)

fprintf('\n========== 开始终端误差补偿计算 ==========\n');


%% 17. 逐块模拟插补与终端误差补偿
for blk = 1:num_blocks

    %% --- 17.1 读取当前块调度参数 ---
    Vs_blk  = sched_vs(blk);
    Ve_blk  = sched_ve(blk);
    Vfs_blk = sched_vfs(blk);
    Na_blk  = sched_na(blk);
    Nc_blk  = sched_nc(blk);
    Nd_blk  = sched_nd(blk);
    Si_blk  = block_S(blk);
    u_end_blk = block_u(blk + 1);       % 当前块末端参数

    Ntotal = Na_blk + Nc_blk + Nd_blk;  % 当前块总调度步数

    %% --- 17.2 初始化模拟插补变量 ---
    if blk == 1
        % 第一个块：复用关键点扫描阶段的前三个参数点（§5 Taylor 初始化结果）
        sim_u = [u0, u1, u2];            % 已有的三个参数值
        sim_C = [C0'; C1'; C2'];         % 对应的曲线点 (3×3)，每行一个点

        % 前两段离散长度计入 S_tr
        S_tr = norm(C1 - C0) + norm(C2 - C1);
        N_itr = 2;       % 已完成 2 个插补步
        k_start = 2;     % PCI 从第 3 步开始（步索引从 0 计）
    else
        % 其余块：沿用上一个块末尾的三个参数值和曲线点
        sim_u = prev_u_hist;             % 从上一个块结束时保留的 3 个 u 值
        sim_C = prev_C_hist;             % 对应的 3 个曲线点 (3×3)
        S_tr  = 0;
        N_itr = 0;
        k_start = 0;
    end

    %% --- 17.3 按 sine-curve velocity profile 逐步 PCI 模拟 ---
    for k = k_start : (Ntotal - 1)

        % ---- (a) 计算当前步期望进给速度 V*(k) ----
        V_star_k = compute_Vtan_step(k, Na_blk, Nc_blk, Nd_blk, ...
                                     Vs_blk, Vfs_blk, Ve_blk);

        % ---- (b) PCI 预测阶段 ----
        n_sim = length(sim_u);
        u_pred = 3*sim_u(n_sim) - 3*sim_u(n_sim-1) + sim_u(n_sim-2);

        % 夹紧预测值
        u_pred = min(max(u_pred, sim_u(n_sim) + min_du_comp), u_max_comp);

        % 若预测步长过小，使用备用步长
        if u_pred <= sim_u(n_sim) + min_du_comp
            u_pred = sim_u(n_sim) + min_du_comp * 10;
            u_pred = min(u_pred, u_max_comp);
        end

        % ---- (c) PCI 校正阶段 ----
        u_corr = u_pred;
        for m = 1:max_corr_iter
            [C_corr, ~, ~] = nurbs_eval(u_corr, ctrl_pts, w, knots, p);
            V_m = norm(C_corr - sim_C(end,:)') / Ts;

            % 计算校正系数 α
            denom_corr = V_star_k - beta * (V_star_k - V_m);
            if abs(denom_corr) < 1e-10
                alpha_corr = 1;
            else
                alpha_corr = V_star_k / denom_corr;
            end
            alpha_corr = max(min(alpha_corr, 2.0), 0.1);

            % 更新参数
            u_new = sim_u(n_sim) + alpha_corr * (u_corr - sim_u(n_sim));
            u_corr = min(max(u_new, sim_u(n_sim) + min_du_comp), u_max_comp);

            % 检查收敛
            if abs(V_star_k - V_m) / V_star_k <= tol_PCI_comp
                break;
            end
        end

        % ---- (d) 备用步长保护 ----
        if u_corr <= sim_u(n_sim) + min_du_comp
            [~, Cd_fb, ~] = nurbs_eval(sim_u(n_sim), ctrl_pts, w, knots, p);
            speed_fb = norm(Cd_fb);
            if speed_fb < 1e-8
                du_fb = min_du_comp * 10;
            else
                du_fb = V_star_k * Ts / speed_fb * 0.5;
            end
            u_corr = sim_u(n_sim) + max(du_fb, min_du_comp);
            u_corr = min(u_corr, u_max_comp);
            [C_corr, ~, ~] = nurbs_eval(u_corr, ctrl_pts, w, knots, p);
        end

        % ---- (e) 记录新点并累加长度 ----
        dist_step = norm(C_corr - sim_C(end,:)');
        sim_u(end+1) = u_corr;
        sim_C = [sim_C; C_corr'];
        S_tr  = S_tr + dist_step;
        N_itr = N_itr + 1;

        % ---- (f) 块末端参数检查 ----
        if u_corr >= u_end_blk - 1e-10
            break;
        end
    end

    %% --- 17.4 保留当前块末尾 3 个点供下一块使用 ---
    n_sim_end = length(sim_u);
    prev_u_hist = sim_u(n_sim_end-2 : n_sim_end);
    prev_C_hist = sim_C(n_sim_end-2 : n_sim_end, :);

    %% --- 17.5 计算当前块长度估计误差 ΔS_i ---
    delta_S = S_tr - Si_blk;

    comp_S_tr(blk)    = S_tr;
    comp_delta_S(blk) = delta_S;
    comp_N_itr(blk)   = N_itr;

    %% --- 17.6 求解终端误差补偿曲线 ---
    if abs(delta_S) <= eps_L
        % 误差在阈值内，不需要补偿
        comp_N_tail(blk)  = 0;
        comp_dV_com(blk)  = 0;
        comp_dV_seq{blk}  = [];

        fprintf('块 %2d: S_tr=%.6f, Si=%.6f, ΔS=%+.2e → 无需补偿\n', ...
                blk, S_tr, Si_blk, delta_S);
    else
        % 确定有效补偿采样数 N_tail = min(Ncom, N_itr)
        N_tail = min(Ncom, N_itr);

        % 计算补偿幅值 ΔV_com = 2 * ΔS_i / (N_tail * Ts)
        dV_com = 2 * delta_S / (N_tail * Ts);

        % 生成补偿速度序列 ΔV(j), j = 0,...,N_tail-1
        j_vec  = 0 : (N_tail - 1);
        dV_seq = (dV_com / 2) * (sin(pi * (2*j_vec/N_tail - 0.5)) + 1);

        comp_N_tail(blk)  = N_tail;
        comp_dV_com(blk)  = dV_com;
        comp_dV_seq{blk}  = dV_seq;

        fprintf('块 %2d: S_tr=%.6f, Si=%.6f, ΔS=%+.2e, Ntail=%d, ΔVcom=%+.4f\n', ...
                blk, S_tr, Si_blk, delta_S, N_tail, dV_com);
    end
end


%% 18. 输出终端误差补偿结果
fprintf('\n========== 终端误差补偿结果 ==========\n');
fprintf('块  |    S_tr    |    Si     |    ΔS      | N_itr | N_tail |  ΔVcom\n');
fprintf('----+------------+-----------+------------+-------+--------+---------\n');
for blk = 1:num_blocks
    fprintf('%3d | %10.6f | %9.6f | %+10.6f | %5d | %6d | %+8.4f\n', ...
        blk, comp_S_tr(blk), block_S(blk), comp_delta_S(blk), ...
        comp_N_itr(blk), comp_N_tail(blk), comp_dV_com(blk));
end
fprintf('==========================================\n');

% 保存补偿结果（追加到已有 mat 文件）
save('nurbs_blocks.mat', 'delta', 'F', 'At', 'Jt', 'An', 'Jn', 'Ts', 'eps', ...
     'eps_PCI', 'beta', 'Fmin', 'kcbc', 'min_du', ...
     'Ncom', 'eps_L', 'tol_PCI_comp', 'min_du_comp', 'u_max_comp', 'max_corr_iter', ...
     'block_u', 'block_V', 'block_S', ...
     'sched_vs', 'sched_ve', 'sched_vfs', 'sched_na', 'sched_nd', 'sched_nc',  ...
       'comp_S_tr', 'comp_delta_S', 'comp_N_itr', 'comp_N_tail', 'comp_dV_com', 'comp_dV_seq');
fprintf('补偿结果已保存至 nurbs_blocks.mat\n');


%% ================================================================
%  以下为已有函数定义（旧代码，不修改）
%  ================================================================

% 完善的 Cox-de Boor 递归计算基函数 N_{i,p}(u)
% 输入:
%   i     - 基函数索引 (从1开始，对应标准数学中的i=0)
%   p     - 阶数 (degree)
%   u     - 参数值 (当前支持标量；若需向量，可外部循环调用)
%   knots - 节点向量 (非递减序列，长度至少 i+p+1)
% 输出:
%   N     - 基函数值
function N = basis_func(i, p, u, knots)
    % 输入验证
    if nargin < 4
        error('basis_func:MissingInputs', '需要提供 i, p, u, knots.');
    end
    if ~isscalar(i) || ~isscalar(p) || i < 1 || p < 0 || floor(i) ~= i || floor(p) ~= p
        error('basis_func:InvalidInputs', 'i 和 p 必须是非负整数标量，i >= 1.');
    end
    if ~isvector(knots) || length(knots) < i + p + 1
        error('basis_func:InvalidKnots', 'knots 必须是向量，长度至少为 %d.', i + p + 1);
    end
    if any(diff(knots) < 0)
        error('basis_func:NonIncreasingKnots', 'knots 必须是非递减序列.');
    end
    if ~isscalar(u)
        error('basis_func:NonScalarU', '当前 u 必须是标量；若需向量支持，请外部循环调用.');
    end
    
    % 基本情况：p == 0
    if p == 0
        tol = 1e-10;  % 浮点容差
        if abs(u - knots(1)) < tol  % 左端点
            target_i = 0;
            for jj = 1 : length(knots)-1
                if knots(jj) + tol < knots(jj+1)
                    target_i = jj;
                    break;
                end
            end
            if i == target_i
                N = 1;
            else
                N = 0;
            end
        elseif abs(u - knots(end)) < tol  % 右端点
            target_i = 0;
            for jj = length(knots)-1 : -1 : 1
                if knots(jj) + tol < knots(jj+1)
                    target_i = jj;
                    break;
                end
            end
            if i == target_i
                N = 1;
            else
                N = 0;
            end
        else  % 内部点
            if u >= knots(i) - tol && u < knots(i+1) + tol
                N = 1;
            else
                N = 0;
            end
        end
        return;
    end
    
    % 递归情况
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


% 基函数一阶导数 N'_{i,p}(u)
function dN = basis_func_deriv(i, p, u, knots)
    if nargin < 4
        error('basis_func_deriv:MissingInputs', '需要提供 i, p, u, knots.');
    end
    if ~isscalar(i) || ~isscalar(p) || i < 1 || p < 0 ||  floor(i) ~= i || floor(p) ~= p
        error('basis_func_deriv:InvalidInputs', 'i 和 p 必须是非负整数标量，i >= 1.');
    end
    if ~isvector(knots) || length(knots) < i + p + 1
        error('basis_func_deriv:InvalidKnots', 'knots 必须是向量，长度至少为 %d.', i + p + 1);
    end
    if any(diff(knots) < 0)
        error('basis_func_deriv:NonIncreasingKnots', 'knots 必须是非递减序列.');
    end
    if ~isscalar(u)
        error('basis_func_deriv:NonScalarU', '当前 u 必须是标量。');
    end
    
    if p == 0
        dN = 0;
        return;
    end
    
    denom1 = knots(i + p) - knots(i);
    denom2 = knots(i + p + 1) - knots(i + 1);
    
    term1 = 0;
    term2 = 0;
    
    if denom1 > 1e-10
        term1 = p / denom1 * basis_func(i, p-1, u, knots);
    end
    
    if denom2 > 1e-10
        term2 = p / denom2 * basis_func(i+1, p-1, u, knots);
    end
    
    dN = term1 - term2;
end

% 基函数二阶导数 N''_{i,p}(u)
function ddN = basis_func_second_deriv(i, p, u, knots)
    if nargin < 4
        error('basis_func_second_deriv:MissingInputs', '需要提供 i, p, u, knots.');
    end
    if ~isscalar(i) || ~isscalar(p) || i < 1 || p < 0 ||  floor(i) ~= i || floor(p) ~= p
        error('basis_func_second_deriv:InvalidInputs', 'i 和 p 必须是非负整数标量，i >= 1.');
    end
    if ~isvector(knots) || length(knots) < i + p + 1
        error('basis_func_second_deriv:InvalidKnots', 'knots 必须是向量，长度至少为 %d.', i + p + 1);
    end
    if any(diff(knots) < 0)
        error('basis_func_second_deriv:NonIncreasingKnots', 'knots 必须是非递减序列.');
    end
    if ~isscalar(u)
        error('basis_func_second_deriv:NonScalarU', '当前 u 必须是标量。');
    end
    
    if p <= 1
        ddN = 0;
        return;
    end
    
    denom1 = knots(i + p) - knots(i);
    denom2 = knots(i + p + 1) - knots(i + 1);
    
    term1 = 0;
    term2 = 0;
    
    if denom1 > 1e-10
        term1 = p / denom1 * basis_func_deriv(i, p-1, u, knots);
    end
    
    if denom2 > 1e-10
        term2 = p / denom2 * basis_func_deriv(i+1, p-1, u, knots);
    end
    
    ddN = term1 - term2;
end

% 计算 NURBS 曲线点 C(u) 及其一阶、二阶导数（解析方法）
function [C, Cd, Cdd] = nurbs_eval(u, ctrl_pts, w, knots, p)
    n = size(ctrl_pts, 1) - 1;
    
    % 输入验证（同上）
    if length(w) ~= n+1
        error('权重向量 w 长度必须为 %d', n+1);
    end
    if length(knots) < n + p + 2
        error('结向量 knots 长度至少为 %d', n + p + 2);
    end
    if size(ctrl_pts, 2) ~= 3
        error('控制点 ctrl_pts 必须为 3D (列数=3)');
    end
    
    sum_NwP   = zeros(3,1);   % A
    sum_Nw    = 0;            % B
    sum_dNwP  = zeros(3,1);   % A'
    sum_dNw   = 0;            % B'
    sum_ddNwP = zeros(3,1);   % A''
    sum_ddNw  = 0;            % B''
    
    for i = 0:n
        ii = i + 1;
        N   = basis_func(ii, p, u, knots);
        dN  = basis_func_deriv(ii, p, u, knots);
        ddN = basis_func_second_deriv(ii, p, u, knots);
        
        wi = w(ii);
        Pi = ctrl_pts(ii,:)';
        
        sum_NwP   = sum_NwP   + N   * wi * Pi;
        sum_Nw    = sum_Nw    + N   * wi;
        sum_dNwP  = sum_dNwP  + dN  * wi * Pi;
        sum_dNw   = sum_dNw   + dN  * wi;
        sum_ddNwP = sum_ddNwP + ddN * wi * Pi;
        sum_ddNw  = sum_ddNw  + ddN * wi;
    end
    
    if sum_Nw < 1e-12
        error('分母 sum_Nw 接近零，可能 u 超出范围或权重异常');
    end
    
    C = sum_NwP / sum_Nw;
    
    % 一阶导数
    Cd = (sum_dNwP * sum_Nw - sum_NwP * sum_dNw) / (sum_Nw ^ 2);
    
    % 二阶导数
    temp = sum_dNwP * sum_Nw - sum_NwP * sum_dNw;
    Cdd = (sum_ddNwP * sum_Nw - sum_NwP * sum_ddNw) / (sum_Nw ^ 2) ...
          - 2 * temp * sum_dNw / (sum_Nw ^ 3);
end

% 计算 NURBS 曲线曲率 κ(u) - 完善版
% 输入:
%   u         - 参数值 (标量)
%   ctrl_pts  - 控制点矩阵 ((n+1)×d, d=2 或 3 为维度)
%   w         - 权重向量 ((n+1)×1)
%   knots     - 结向量 (1×(n+p+2))
%   p         - 阶数 (degree)
% 输出:
%   kappa     - 曲率值 (标量，非负)
function kappa = curvature(u, ctrl_pts, w, knots, p)
    % 输入验证
    if nargin < 5
        error('curvature:MissingInputs', '需要提供 u, ctrl_pts, w, knots, p.');
    end
    if ~isscalar(u)
        error('curvature:NonScalarU', 'u 必须是标量.');
    end
    n = size(ctrl_pts, 1) - 1;
    d = size(ctrl_pts, 2);  % 维度 (2 或 3)
    if d ~= 2 && d ~= 3
        error('curvature:InvalidDimension', '控制点 ctrl_pts 必须为 2D 或 3D (列数=2 或 3).');
    end
    if length(w) ~= n+1
        error('curvature:InvalidWeights', '权重向量 w 长度必须为 %d.', n+1);
    end
    if length(knots) < n + p + 2
        error('curvature:InvalidKnots', '结向量 knots 长度至少为 %d.', n + p + 2);
    end
    if u < knots(1) || u > knots(end)
        error('curvature:OutOfRange', 'u (%f) 超出 knots 范围 [%f, %f].', u, knots(1), knots(end));
    end
    
    % 调用 nurbs_eval 获取导数 (假设返回 [C, Cd, Cdd]，Cd 和 Cdd 为 d×1 列向量)
    [~, Cd, Cdd] = nurbs_eval(u, ctrl_pts, w, knots, p);
    
    % 计算曲率
    v = norm(Cd);  % ||C'|| (速度模长)
    if v < 1e-8  % 阈值处理退化情况 (接近驻点，曲率为 0)
        kappa = 0;
        return;
    end
    
    if d == 3  % 3D 曲线
        cross_prod = cross(Cd, Cdd);
        kappa = norm(cross_prod) / (v ^ 3);
    else  % d == 2, 2D 曲线
        % 2D 叉积标量: C'_x * C''_y - C'_y * C''_x
        cross_prod = Cd(1) * Cdd(2) - Cd(2) * Cdd(1);
        kappa = abs(cross_prod) / (v ^ 3);
    end
end


%% ================================================================
%  以下为新增函数定义（进给率调度相关）
%  ================================================================

% 计算加速段和减速段采样时间数（取整为 ceil）
% 输入: Vfs - 块内目标最高速度
%       Vs  - 起始速度
%       Ve  - 结束速度
%       At  - 切向加速度约束
%       Jt  - 切向 jerk 约束
%       Ts  - 采样时间
% 输出: Na  - 加速段采样数 (整数)
%       Nd  - 减速段采样数 (整数)
function [Na, Nd] = compute_NaNd(Vfs, Vs, Ve, At, Jt, Ts)
    % --- 加速段 ---
    delta_v_a = (Vfs - Vs) / 2;
    if delta_v_a <= 0
        Na = 0;
    else
        Na_by_acc  = delta_v_a * pi / (At * Ts);
        Na_by_jerk = sqrt(delta_v_a / Jt * (pi / Ts)^2);
        Na = ceil(max(Na_by_acc, Na_by_jerk));
    end

    % --- 减速段 ---
    delta_v_d = (Vfs - Ve) / 2;
    if delta_v_d <= 0
        Nd = 0;
    else
        Nd_by_acc  = delta_v_d * pi / (At * Ts);
        Nd_by_jerk = sqrt(delta_v_d / Jt * (pi / Ts)^2);
        Nd = ceil(max(Nd_by_acc, Nd_by_jerk));
    end
end


% 计算正弦求和项 a（加速段位移系数）
function a = compute_sine_sum_a(Na)
    a = 0;
    for j = 0 : Na - 1
        a = a + sin(pi * (j / Na - 0.5)) + 1;
    end
end


% 计算正弦求和项 b（减速段位移系数）
function b = compute_sine_sum_b(Nd)
    b = 0;
    for j = 0 : Nd - 1
        b = b + sin(pi * (j / Nd - 1.5)) + 1;
    end
end


% 长短块判定函数
% 输入: Vs, Ve, Si, At, Jt, Ts
% 输出: is_short - 是否为短块（逻辑值）
%       Sstd    - 临界长度
function [is_short, Sstd] = judge_block_short(Vs, Ve, Si, At, Jt, Ts)
    % §2.2: Vfs_judge = max(Vs, Ve)
    Vfs_judge = max(Vs, Ve);
    [Na_judge, Nd_judge] = compute_NaNd(Vfs_judge, Vs, Ve, At, Jt, Ts);

    % §2.3: 计算临界长度 Sstd
    if Vs < Ve
        Sstd = 0.5 * (Vs + Ve) * Na_judge * Ts + Ve * Nd_judge * Ts;
    elseif Vs > Ve
        Sstd = 0.5 * (Vs + Ve) * Nd_judge * Ts + Vs * Na_judge * Ts;
    else  % Vs == Ve
        Sstd = (Na_judge + Nd_judge) * Vs * Ts;
    end

    % §2.4: 判定
    is_short = (Si < Sstd);
end


% 单块进给率调度（Step I ~ Step IV）
% 每一步使用独立变量名（含步骤后缀），避免 vfs 等变量多次赋值造成混淆。
%
% 输入: Vs, Ve   - 块起始/结束速度
%       Si       - 块弧长
%       Vmax     - 最大允许进给率
%       At, Jt   - 切向加速度/jerk 约束
%       Ts       - 采样时间
% 输出: vfs, na, nd, nc - 调度结果
%       vs_out, ve_out  - 可能修正后的边界速度
%       step_used       - 使用的步骤编号 (1/2/3/4)
function [vfs, na, nd, nc, vs_out, ve_out, step_used] = ...
        schedule_single_block(Vs, Ve, Si, Vmax, At, Jt, Ts)
    %
    % 改进版：Step II/III 使用二分搜索，每轮重算 Na/Nd，确保自洽
    %

    % -------- 输入保护：钳位到 [0, Vmax] --------
    Vs = min(max(Vs, 0), Vmax);
    Ve = min(max(Ve, 0), Vmax);
    vs_out = Vs;
    ve_out = Ve;

    % ===================== Step I: ACC + CF + DEC =====================
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
        na  = na_I;
        nd  = nd_I;
        nc  = nc_I_floor;
        step_used = 1;
        return;
    end

    % ===================== Step II: 二分搜索 Vfs ∈ [max(Vs,Ve), Vmax] =====================
    Vfs_lo = max(max(Vs, Ve), 1e-6);
    Vfs_hi = Vmax;

    % 检查下界处位移（= Sstd）
    [na_lo, nd_lo] = compute_NaNd(Vfs_lo, Vs, Ve, At, Jt, Ts);
    S_lo = (Vs + Vfs_lo) * na_lo * Ts / 2 + (Ve + Vfs_lo) * nd_lo * Ts / 2;

    if S_lo <= Si
        % 下界可行 → 长块，二分搜索最大可行 Vfs
        best_vfs = Vfs_lo;
        best_na  = na_lo;
        best_nd  = nd_lo;
        best_S   = S_lo;

        for iter = 1:50
            Vfs_mid = (Vfs_lo + Vfs_hi) / 2;
            [na_m, nd_m] = compute_NaNd(Vfs_mid, Vs, Ve, At, Jt, Ts);
            S_m = (Vs + Vfs_mid) * na_m * Ts / 2 + (Ve + Vfs_mid) * nd_m * Ts / 2;

            if S_m <= Si
                Vfs_lo   = Vfs_mid;
                best_vfs = Vfs_mid;
                best_na  = na_m;
                best_nd  = nd_m;
                best_S   = S_m;
            else
                Vfs_hi = Vfs_mid;
            end

            if Vfs_hi - Vfs_lo < 0.01
                break;
            end
        end

        vfs = best_vfs;
        na  = best_na;
        nd  = best_nd;

        % 剩余弧长分配给恒速段
        remaining = Si - best_S;
        if remaining > 0 && vfs * Ts > 0
            nc = max(0, floor(remaining / (vfs * Ts)));
        else
            nc = 0;
        end
        step_used = 2;
        return;
    end

    % ===================== Step III: 二分搜索 Vfs ∈ [min(Vs,Ve), max(Vs,Ve)] =====================
    % 到达此处 → 短块（S_lo > Si），即使 Vfs=max(Vs,Ve) 的加减速位移仍超弧长
    % 只能做单边变速（纯加速或纯减速），Nc = 0，并修正一侧边界速度

    if abs(Vs - Ve) > 1e-6
        Vfs_lo3 = max(min(Vs, Ve), 1e-6);
        Vfs_hi3 = max(Vs, Ve);

        best_vfs = Vfs_lo3;
        best_na  = 0;
        best_nd  = 0;

        for iter = 1:50
            Vfs_mid = (Vfs_lo3 + Vfs_hi3) / 2;
            [na_m, nd_m] = compute_NaNd(Vfs_mid, Vs, Ve, At, Jt, Ts);
            S_m = (Vs + Vfs_mid) * na_m * Ts / 2 + (Ve + Vfs_mid) * nd_m * Ts / 2;

            if S_m <= Si
                Vfs_lo3  = Vfs_mid;
                best_vfs = Vfs_mid;
                best_na  = na_m;
                best_nd  = nd_m;
            else
                Vfs_hi3 = Vfs_mid;
            end

            if Vfs_hi3 - Vfs_lo3 < 0.01
                break;
            end
        end

        vfs = best_vfs;
        na  = best_na;
        nd  = best_nd;
        nc  = 0;    % 短块，无恒速段

        % 修正边界速度：降低较大的一侧
        if Vs > Ve
            vs_out = vfs;       % 纯减速，降低起始速度
        else
            ve_out = vfs;       % 纯加速，降低结束速度
        end
        step_used = 3;
        return;
    end

    % ===================== Step IV: 纯恒速（Vs ≈ Ve） =====================
    V_min = max(min(Vs, Ve), 1e-6);

    nc_IV = ceil(Si / (V_min * Ts));
    if nc_IV < 1
        nc_IV = 1;
    end

    vfs_IV = Si / (nc_IV * Ts);

    vs_out = vfs_IV;
    ve_out = vfs_IV;
    vfs = vfs_IV;
    na  = 0;
    nd  = 0;
    nc  = nc_IV;
    step_used = 4;
end


% 计算 sine-curve velocity profile 在第 k 步的期望进给速度
% 输入:
%   k    - 当前步索引（从 0 开始）
%   Na   - 加速段采样数
%   Nc   - 恒速段采样数
%   Nd   - 减速段采样数
%   Vs   - 起始速度
%   Vfs  - 峰值/恒速速度
%   Ve   - 结束速度
% 输出:
%   V    - 当前步的期望进给速度
function V = compute_Vtan_step(k, Na, Nc, Nd, Vs, Vfs, Ve)
    if Na > 0 && k < Na
        % 加速段: V = (Vfs-Vs)/2 * [sin(π(k/Na - 1/2)) + 1] + Vs
        V = (Vfs - Vs)/2 * (sin(pi * (k/Na - 0.5)) + 1) + Vs;
    elseif k < Na + Nc
        % 恒速段: V = Vfs
        V = Vfs;
    elseif Nd > 0
        % 减速段: V = (Vfs-Ve)/2 * [sin(π(j/Nd - 3/2)) + 1] + Ve
        j_dec = k - Na - Nc;
        V = (Vfs - Ve)/2 * (sin(pi * (j_dec/Nd - 1.5)) + 1) + Ve;
    else
        % 退化情况（Na=0, Nd=0）: 纯恒速
        V = Vfs;
    end
    V = max(V, 1e-6);  % 速度下限保护
end
