% MATLAB script for computing the ellipsoid invariant and maximal positive
% invariant set with input constraints in 3D.
close all; clear; clc;

currentFolder = pwd;
figuresFolder = fullfile(currentFolder, 'figures');

if ~exist(figuresFolder, 'dir')
    mkdir(figuresFolder);
end

steadystate_value_selector = 'positive';

switch steadystate_value_selector
    case 'positive'
        x_e = [-1.375000000000000; 2.625000000000000; 325];
        u_e = [0.433333333333333; 0.466666666666667];
        
    case 'negative'
        x_e = [-1.375000000000000; -5.375000000000000; -325];
        u_e = [0.033333333333333; 0.466666666666667];
        
    case 'zero'
        x_e = [1.875000000000000; 1.875000000000000; 0];
        u_e = [0.466666666666667; 0.466666666666667];
        
    otherwise
        error('Unknown case');
end

if x_e(3) >= 0
    A = [0.990429637098200,0.009570362901800,0.001869303522832;0.009570362901800,0.990429637098200,-0.001869303522832;-9.346517614158701,9.346517614158701,0.865825211252908];
    B = [-1.490319453525038,-1.495159726762519;1.490319453525038,-0.004840273237481;14.355544352700347,7.177772176350174];
else
    A = [0.990429637098200,0.009570362901800,0.001869303522832;0.009570362901800,0.990429637098200,-0.001869303522832;-9.346517614158701,9.346517614158701,0.865825211252908];
    B = [-1.490319453525038,-0.004840273237481;1.490319453525038,-1.495159726762519;14.355544352700347,-7.177772176350174];
end

nx = 3; % Dimension of state space
nu = 2; % Number of control inputs

Hx = [eye(nx); -eye(nx)];
hx = [10; 10; 400; 10; 10; 400] - Hx * x_e;
Hu = [1, 1; -eye(nu)];
hu = [1; 0; 0] - Hu * u_e;

% Improved initial guess to ensure positive definite S
S_init = eye(nx);
F_init = zeros(nu, nx);
x0 = [S_init(:); F_init(:)]; % Convert to vector form

options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'iter');

problem.objective = @(x) objective_function(x, nx);
problem.x0 = x0;
problem.Aineq = [];
problem.bineq = [];
problem.Aeq = [];
problem.beq = [];
problem.lb = [];
problem.ub = [];
problem.nonlcon = @(x) nonlinear_constraints(x, nx, nu, A, B, Hx, hx, Hu, hu);
problem.solver = 'fmincon';
problem.options = options;

x_opt = fmincon(problem);

optimal_S = reshape(x_opt(1:nx^2), nx, nx);
optimal_P = inv(optimal_S);
optimal_F = reshape(x_opt(nx^2+1:end), nu, nx);
optimal_K = optimal_F * optimal_P;

disp('Optimal S:'), disp(optimal_S)
disp('Optimal P (Ellipsoid Shape Matrix):'), disp(optimal_P)
disp('Optimal F:'), disp(optimal_F)
disp('Optimal K:'), disp(optimal_K)

H_combined = [Hx; Hu * optimal_K];
h_combined = [hx; hu];

% Calculate maximal invariant set
[O_inf_logS, X_k_logS] = max_positive_invariant_set(A, B, optimal_K, Hx, hx, Hu, hu);

V = inequalities_to_vertices(Hx, hx);
V_k_logS = inequalities_to_vertices(X_k_logS.H, X_k_logS.h);
V_O_logS = inequalities_to_vertices(O_inf_logS.H, O_inf_logS.h);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen
set(gcf, 'Renderer', 'opengl'); % use OpenGL
set(gca, 'SortMethod', 'childorder'); % improves draw order of patches
rotate3d on;  % ← This re-enables mouse rotation

% Plot Polyhedrons
K = convhull(V(:,1), V(:,2), V(:,3));
trisurf(K, V(:,1), V(:,2), V(:,3), 'FaceColor', 'blue', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'EdgeAlpha', '0.2');
K = convhull(V_k_logS(:,1), V_k_logS(:,2), V_k_logS(:,3));
trisurf(K, V_k_logS(:,1), V_k_logS(:,2), V_k_logS(:,3), 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'black', 'EdgeAlpha', '0.2');
K = convhull(V_O_logS(:,1), V_O_logS(:,2), V_O_logS(:,3));
trisurf(K, V_O_logS(:,1), V_O_logS(:,2), V_O_logS(:,3), 'FaceColor', 'yellow', 'FaceAlpha', 0.4, 'EdgeColor', 'black', 'EdgeAlpha', '0.2');

% Plot Ellipsoid max(log(S))
f = @(x, y, z) [x, y, z] * optimal_P * [x; y; z] - 1;
fimplicit3(f, 'MeshDensity', 100, 'FaceColor', 'red', 'FaceAlpha', 0.6, 'EdgeColor', 'none');

xlabel('$x_1 - x_1^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
ylabel('$x_2 - x_2^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
zlabel('$x_3 - x_3^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
view([45, 30]);
axis tight;
title('Ellipsoid Invariant and MPI Terminal Set with control policy that maximizes the volume with $\alpha = 1$', 'Interpreter', 'latex', 'FontSize', 32);
leg = legend('$\mathcal{X}$', '$\mathcal{X}_\kappa$', '$\mathcal{O}_\infty$', '$\mathcal{E}$');
set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
hold off;

exportgraphics(gcf, 'figures/ellipsoid_maxlogS.pdf', 'ContentType', 'vector');

% LQR solution
R = diag([1e1, 1e1]);
Q = diag([1e0, 1e0, 3e-4]);
P = idare(A, B, Q, R, [], []);
K = -dlqr(A, B, Q, R);
H = [Hx; Hu * K];
h = [hx; hu];
alpha = min(h.^2 ./ diag(H * (P \ H')));

% Calculate maximal invariant set
[O_inf_lqr, X_k_lqr] = max_positive_invariant_set(A, B, K, Hx, hx, Hu, hu);

V_k_lqr = inequalities_to_vertices(X_k_lqr.H, X_k_lqr.h);
V_O_lqr = inequalities_to_vertices(O_inf_lqr.H, O_inf_lqr.h);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen
set(gcf, 'Renderer', 'opengl'); % use OpenGL
set(gca, 'SortMethod', 'childorder'); % improves draw order of patches
rotate3d on;  % ← This re-enables mouse rotation

% Plot Polyhedrons
K = convhull(V(:,1), V(:,2), V(:,3));
trisurf(K, V(:,1), V(:,2), V(:,3), 'FaceColor', 'blue', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'EdgeAlpha', '0.2');
K = convhull(V_k_lqr(:,1), V_k_lqr(:,2), V_k_lqr(:,3));
trisurf(K, V_k_lqr(:,1), V_k_lqr(:,2), V_k_lqr(:,3), 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'black', 'EdgeAlpha', '0.2');
K = convhull(V_O_lqr(:,1), V_O_lqr(:,2), V_O_lqr(:,3));
trisurf(K, V_O_lqr(:,1), V_O_lqr(:,2), V_O_lqr(:,3), 'FaceColor', 'yellow', 'FaceAlpha', 0.4, 'EdgeColor', 'black', 'EdgeAlpha', '0.2');

% Plot Ellipsoid LQR
f = @(x, y, z) [x, y, z] * P * [x; y; z] - alpha;
fimplicit3(f, 'MeshDensity', 100, 'FaceColor', 'red', 'FaceAlpha', 0.6, 'EdgeColor', 'none');

xlabel('$x_1 - x_1^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
ylabel('$x_2 - x_2^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
zlabel('$x_3 - x_3^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
view([45, 30]);
axis tight;
title('Ellipsoid Invariant and MPI Terminal Set with optimal control policy $\kappa(x) = - K_{LQR} x$', 'Interpreter', 'latex', 'FontSize', 32);
leg = legend('$\mathcal{X}$', '$\mathcal{X}_\kappa$', '$\mathcal{O}_\infty$', '$\mathcal{E}$');
set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
hold off;

exportgraphics(gcf, 'figures/ellipsoid_LQR.pdf', 'ContentType', 'vector');

function obj = objective_function(x, nx)
    S = reshape(x(1:nx^2), nx, nx);
    S = (S + S') / 2; % Ensure symmetry
    obj = -log(det(S)); % Ensure det(S) > 0
end

function [c, ceq] = nonlinear_constraints(x, nx, nu, A, B, Hx, hx, Hu, hu)
    S = reshape(x(1:nx^2), nx, nx);
    F = reshape(x(nx^2+1:end), nu, nx);
    S = (S + S') / 2; % Ensure symmetry

    % Positive semidefiniteness constraint M >= 0
    M = [S, (A * S + B * F)'; (A * S + B * F), S];
    M = (M + M') / 2; % Enforce symmetry
    min_eigenvalue_M = real(min(eig(M))); % Ensure real values

    % State constraints
    c_hx = zeros(length(hx), 1);
    for i = 1:length(hx)
        c_hx(i) = Hx(i, :) * S * Hx(i, :)' - hx(i)^2;
    end

    % Input constraints
    c_hu = zeros(length(hu), 1);
    for i = 1:length(hu)
        M2 = [hu(i)^2, (F' * Hu(i, :)')'; (F' * Hu(i, :)'), S];
        M2 = (M2 + M2') / 2; % Enforce symmetry
        min_eigenvalue_M2 = real(min(eig(M2))); % Ensure real values
        c_hu(i) = -min_eigenvalue_M2;
    end

    % Inequality constraints (must be <= 0)
    c = [-min_eigenvalue_M; c_hx; c_hu];

    % No equality constraints
    ceq = [];
end

function [O_inf, X_k] = max_positive_invariant_set(A, B, K, Hx, hx, Hu, hu)
    % Compute closed-loop system matrix
    A_cl = A + B * K;

    % Calculate X_k
    [H_k, h_k] = intersect_constraints(Hx, hx, Hu * K, hu);
    X_k.H = H_k;
    X_k.h = h_k;

    % Initialize O_0
    H = H_k;
    h = h_k;
    while true
        [H_pre, h_pre] = pre(A_cl, H, h);
        [H_next, h_next] = intersect_constraints(H_pre, h_pre, H_k, h_k);
        
        if isequal(H_next, H) && isequal(h_next, h)
            O_inf.H = H;
            O_inf.h = h;
            return;
        end
        
        % Update O_k
        H = H_next;
        h = h_next;
    end
end

function [H_pre, h_pre] = pre(A_cl, H, h)
    H_pre = H * A_cl;
    h_pre = h;
end

function [H_int, h_int] = intersect_constraints(H1, h1, H2, h2)
    % Computes the intersection of two sets defined by inequalities
    H_int = [H1; H2];
    h_int = [h1; h2];
    
    % Remove redundant constraints using linear programming
    [H_int, h_int] = remove_redundant_constraints(H_int, h_int);
end

function [H_out, h_out] = remove_redundant_constraints(H, h)
    % Removes redundant constraints from a set of inequalities
    H_out = H;
    h_out = h;
    
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    
    i = 1;
    while i <= size(H_out, 1)
        % Remove the current constraint from the list
        H_test = H_out;
        h_test = h_out;
        H_test(i, :) = [];
        h_test(i) = [];

        % Define the optimization problem to check feasibility without the current constraint
        problem.objective = @(x) -H_out(i, :) * x;
        problem.x0 = zeros(size(H_out, 2), 1);  % Initial guess
        problem.Aineq = H_test;
        problem.bineq = h_test;
        problem.solver = 'fmincon';
        problem.options = options;

        % Solve the optimization problem
        [~, fval, exitflag] = fmincon(problem);
        
        if exitflag == 1 && -fval <= h_out(i)  % Constraint is redundant
            H_out(i, :) = [];
            h_out(i) = [];
        else
            i = i + 1;
        end
    end
end

function V = inequalities_to_vertices(A, b)
    % Converts linear inequalities A*x <= b into a set of ordered vertices
    % Uses linear programming to find vertices of the feasible polytope.

    n = size(A, 2); % Number of variables (state dimension)
    V = [];

    % Try all combinations of n constraints as equalities (vertex candidates)
    combs = nchoosek(1:size(A, 1), n);
    
    for i = 1:size(combs, 1)
        A_eq = A(combs(i, :), :);
        b_eq = b(combs(i, :));
        
        % Solve for x
        x_vertex = pinv(A_eq) * b_eq; % Solve A_eq * x = b_eq
        
        % Check if x satisfies all inequalities
        if all(A * x_vertex <= b + 1e-12)
            V = [V; x_vertex']; % Store as row vector
        end
    end

    % Remove duplicates
    V = unique(V, 'rows');
end
