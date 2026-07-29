% Generate a 3D open butterfly NURBS curve based on the paper contour.
% The 2D reference points approximate Fig. 12 in the paper.  The curve is
% scaled by 3, starts at the butterfly head, and ends 1 mm away from the
% start point so the result is an open curve.

%% User parameters
scale_factor = 3.0;        % Paper contour scale factor.
gap_mm = 1.0;              % Distance between start and end points.
degree = 3;                % Cubic NURBS/B-spline curve.
max_wing_lift_mm = 60.0;   % Maximum z lift of the two wings.
head_lift_mm = 53.0;       % Lift the head/start/end part in y direction.

%% 1. Approximate paper butterfly points in paper-size millimeters
% Coordinates are centered at the butterfly head/body center.  They are
% ordered as: head -> right wing -> lower body -> left wing -> near head.
paper_xy = [
     0.0000    4.0000
     7.3094   15.3348
    23.3692   32.3311
    39.1989   37.0918
    53.8351   35.3017
    60.4606   25.7013
    52.3226   17.8929
    42.2489   16.7271
    51.6818    8.1989
    42.0240    1.4774
    32.2221    9.8915
    24.0877    0.9567
    36.4272   -1.7183
    46.2033   -6.0836
    57.7594  -15.9473
    46.7591  -26.1273
    34.2042  -23.5436
    26.4241  -14.6984
    16.0993  -25.6629
    11.1785  -14.6499
     8.1866   -8.7375
     4.0750   -0.9335
     3.2000   -7.8667
     5.2000  -16.5333
     0.0000  -20.1333
    -5.2000  -16.5333
    -3.2000   -7.8667
    -4.0750   -0.9335
    -8.1866   -8.7375
   -11.1785  -14.6499
   -16.0993  -25.6629
   -26.4241  -14.6984
   -34.2042  -23.5436
   -46.7591  -26.1273
   -57.7594  -15.9473
   -46.2033   -6.0836
   -36.4272   -1.7183
   -24.0877    0.9567
   -32.2221    9.8915
   -42.0240    1.4774
   -51.6818    8.1989
   -42.2489   16.7271
   -52.3226   17.8929
   -60.4606   25.7013
   -53.8351   35.3017
   -39.1989   37.0918
   -23.3692   32.3311
    -7.3094   15.3348
     0.0000    4.0000
];

%% 2. Scale to the required machining size and make the curve open
xy = paper_xy * scale_factor;
xy([1, end], 2) = xy([1, end], 2) + head_lift_mm;
xy(end, :) = xy(1, :) + [0, -gap_mm];

% Lift both wings in z while keeping the head/body nearly on z = 0.  The
% smooth gate avoids a sharp z jump near the body centerline.
x_abs = abs(xy(:, 1));
x_span = max(x_abs);
y_min = min(xy(:, 2));
y_span = max(xy(:, 2)) - y_min;
x_norm = x_abs / x_span;
y_norm = (xy(:, 2) - y_min) / y_span;

wing_gate = (x_norm - 0.12) / (1.0 - 0.12);
wing_gate = min(max(wing_gate, 0), 1);
wing_gate = wing_gate .^ 2 .* (3 - 2 * wing_gate);
z = max_wing_lift_mm * wing_gate .* (0.55 + 0.45 * y_norm);
z([1, end]) = 0;

data_points = [xy, z];

%% 3. Build NURBS/B-spline curve
has_nurbs_class = exist('NURBS', 'class') == 8 || exist('NURBS', 'file') == 2;

if has_nurbs_class
    obj = NURBS(data_points, degree, 'open', 1);
    obj = obj.Interpolation();
    obj = obj.Cal_CurveData();

    control_points = obj.ControlPoints;
    knot_vector = obj.UVector(:)';
    weights = obj.Weights(:);
    curve_data = obj.CurveData;
else
    warning('NURBS class was not found. Saving the data points as cubic B-spline control points.');
    control_points = data_points;
    knot_vector = make_open_clamped_knot_vector(size(control_points, 1), degree);
    weights = ones(size(control_points, 1), 1);
    curve_data = sample_nurbs_curve(control_points, weights, knot_vector, degree, 1000);
end

%% 4. Plot for checking
figure;
hold on; axis equal; grid on; view(3);
title('3D open butterfly NURBS curve');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

plot3(data_points(:, 1), data_points(:, 2), data_points(:, 3), ...
      'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4, 'LineWidth', 1);
plot3(curve_data(:, 1), curve_data(:, 2), curve_data(:, 3), ...
      'r-', 'LineWidth', 1.6);
plot3(control_points(:, 1), control_points(:, 2), control_points(:, 3), ...
      'c-', 'LineWidth', 1.0);
plot3(control_points(:, 1), control_points(:, 2), control_points(:, 3), ...
      'o', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'MarkerSize', 4, 'LineWidth', 1);
legend({'Interpolation points', 'NURBS curve', 'Control polygon', 'Control points'}, ...
       'Location', 'bestoutside');

%% 5. Print key checks
start_end_gap = norm(data_points(end, :) - data_points(1, :));
fprintf('Degree: %d\n', degree);
fprintf('Interpolation points: %d\n', size(data_points, 1));
fprintf('Control points: %d\n', size(control_points, 1));
fprintf('Start-end gap: %.6f mm\n', start_end_gap);
fprintf('X span: %.6f mm\n', max(data_points(:, 1)) - min(data_points(:, 1)));
fprintf('Y span: %.6f mm\n', max(data_points(:, 2)) - min(data_points(:, 2)));
fprintf('Z range: %.6f to %.6f mm\n', min(data_points(:, 3)), max(data_points(:, 3)));

%% 6. Save files
currentFolder = fileparts(mfilename('fullpath'));
if isempty(currentFolder)
    currentFolder = pwd;
end

controlPointsFile = fullfile(currentFolder, 'control_points.txt');
knotVectorFile = fullfile(currentFolder, 'knot_vector.txt');

writematrix(control_points, controlPointsFile, 'Delimiter', ' ');
writematrix(knot_vector(:), knotVectorFile, 'Delimiter', ' ');

fprintf('Updated control points: %s\n', controlPointsFile);
fprintf('Updated knot vector: %s\n', knotVectorFile);

%% Local fallback helpers
function knots = make_open_clamped_knot_vector(num_ctrl, degree)
    internal_count = num_ctrl - degree - 1;
    if internal_count > 0
        internal = (1:internal_count) / (internal_count + 1);
    else
        internal = [];
    end
    knots = [zeros(1, degree + 1), internal, ones(1, degree + 1)];
end

function curve = sample_nurbs_curve(control_points, weights, knots, degree, sample_count)
    u_values = linspace(knots(degree + 1), knots(end - degree), sample_count);
    curve = zeros(sample_count, size(control_points, 2));

    for k = 1:sample_count
        u = u_values(k);
        numerator = zeros(1, size(control_points, 2));
        denominator = 0;

        for i = 1:size(control_points, 1)
            basis_value = bspline_basis(i, degree, u, knots);
            weighted_basis = basis_value * weights(i);
            numerator = numerator + weighted_basis * control_points(i, :);
            denominator = denominator + weighted_basis;
        end

        if denominator < 1e-12
            error('NURBS denominator is too small at u = %.12f.', u);
        end

        curve(k, :) = numerator / denominator;
    end
end

function value = bspline_basis(i, degree, u, knots)
    tol = 1e-12;

    if degree == 0
        if (u >= knots(i) - tol && u < knots(i + 1) - tol) || ...
           (abs(u - knots(end)) <= tol && i == length(knots) - 1)
            value = 1;
        else
            value = 0;
        end
        return;
    end

    left_den = knots(i + degree) - knots(i);
    right_den = knots(i + degree + 1) - knots(i + 1);
    left_value = 0;
    right_value = 0;

    if abs(left_den) > tol
        left_value = (u - knots(i)) / left_den * bspline_basis(i, degree - 1, u, knots);
    end

    if abs(right_den) > tol
        right_value = (knots(i + degree + 1) - u) / right_den * bspline_basis(i + 1, degree - 1, u, knots);
    end

    value = left_value + right_value;
end