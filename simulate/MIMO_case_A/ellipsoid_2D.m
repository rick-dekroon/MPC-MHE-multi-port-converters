% MATLAB script for computing the ellipsoid invariant and maximal positive
% invariant set with input constraints in 2D.
close all; clear; clc;

A = [1, 0.3; 0, 1];  % Example system dynamics matrix
B = [0; 0.3];        % Control input matrix

p_max =  1;   % Max position
p_min = -120; % Min position
v_max =  25;  % Max velocity
v_min = -50;  % Min velocity
u_max =  10;  % Max control input
u_min = -20;  % Min control input

Hx = [eye(2); -eye(2)];
hx = [p_max; v_max; -p_min; -v_min];
Hu = [eye(1); -eye(1)];
hu = [u_max; -u_min];

nx = 2; % Dimension of state space
nu = 1; % Number of control inputs

% Improved initial guess to ensure positive definite S
S_init = eye(nx);
F_init = zeros(nu, nx);
x0 = [S_init(:); F_init(:)]; % Convert to vector form

options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'iter', 'MaxIterations', 1000, 'MaxFunctionEvaluations', 1e5);

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

R = 1e-2;
Q = diag([1e1, 1e0]);
P = idare(A, B, Q, R, [], []);
K = -dlqr(A, B, Q, R);
H = [Hx; Hu * K];
h = [hx; hu];
alpha = min(h.^2 ./ sum(H .* (P \ H')', 2));

% Calculate maximal invariant sets
[O_inf_logS, X_k_logS] = max_positive_invariant_set(A, B, optimal_K, Hx, hx, Hu, hu);
[O_inf_lqr, X_k_lqr] = max_positive_invariant_set(A, B, K, Hx, hx, Hu, hu);

V = inequalities_to_vertices(Hx, hx);
V_k_logS = inequalities_to_vertices(X_k_logS.H, X_k_logS.h);
V_k_lqr = inequalities_to_vertices(X_k_lqr.H, X_k_lqr.h);
V_O_logS = inequalities_to_vertices(O_inf_logS.H, O_inf_logS.h);
V_O_lqr = inequalities_to_vertices(O_inf_lqr.H, O_inf_lqr.h);

X_poly = polyshape(V(:,1), V(:,2));
X_k_logS_poly = polyshape(V_k_logS(:,1), V_k_logS(:,2));
X_k_lqr_poly = polyshape(V_k_lqr(:,1), V_k_lqr(:,2));
O_logS_poly = polyshape(V_O_logS(:,1), V_O_logS(:,2));
O_lqr_poly = polyshape(V_O_lqr(:,1), V_O_lqr(:,2));

% Plot results
figure; hold on; grid on;

% Plot Polyhedrons
plot(X_poly, 'FaceColor', 'blue', 'FaceAlpha', 0.1);
plot(X_k_logS_poly, 'FaceColor', 'green', 'FaceAlpha', 0.2);
plot(X_k_lqr_poly, 'FaceColor', 'cyan', 'FaceAlpha', 0.2);
plot(O_logS_poly, 'FaceColor', 'yellow', 'FaceAlpha', 0.4);
plot(O_lqr_poly, 'FaceColor', 'red', 'FaceAlpha', 0.4);

% Plot Ellipsoid max(log(S))
fimplicit(@(x, y) [x, y] * optimal_P * [x; y] - 1, [-10, 10, -5, 5], 'r', 'LineWidth', 2);

% Plot Ellipsoid LQR
fimplicit(@(x, y) [x, y] * P * [x; y] - alpha, [-10, 10, -5, 5], 'k', 'LineWidth', 2);

xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('Ellipsoid Invariant and MPI Terminal Set', 'Interpreter', 'latex');
leg = legend('$\mathcal{X}$', '$\mathcal{X}_{\kappa, \max(log(S))}$', '$\mathcal{X}_{\kappa, LQR}$', '$\mathcal{O}_{\infty, \max(log(S))}$', '$\mathcal{O}_{\infty, LQR}$', '$\mathcal{E}_{\max(log(S))}$', '$\mathcal{E}_{LQR}$');
set(gca, 'FontSize', 14);
set(gca, 'TickLabelInterpreter', 'latex');
set(leg, 'Interpreter', 'latex');
hold off;

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

function [O_inf, X_k] = max_positive_invariant_set(A, B, K, H_x, h_x, H_u, h_u)
    % Compute closed-loop system matrix
    A_cl = A + B * K;

    % Calculate X_k
    [H_k, h_k] = intersect_constraints(H_x, h_x, H_u * K, h_u);
    X_k.H = H_k;
    X_k.h = h_k;

    % Initialize O_0
    H = H_k;
    h = h_k;
    while true
        % Compute preimage under the closed-loop system
        [H_pre, h_pre] = preimage(A_cl, H, h);
        
        % Compute the next set in the sequence
        [H_next, h_next] = intersect_constraints(H_pre, h_pre, H_k, h_k);
        
        % Check for convergence
        if isequal(H_next, H) && isequal(h_next, h)
            O_inf.H = H;
            O_inf.h = h;
            return;
        end
        
        % Update for the next iteration
        H = H_next;
        h = h_next;
    end
end

function [H_pre, h_pre] = preimage(A_cl, H, h)
    % Computes the preimage of set Hx <= h under A_cl with input constraints
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

    % Order vertices counterclockwise
    if size(V, 1) > 2
        K = convhull(V(:,1), V(:,2)); % Compute convex hull indices
        V = V(K(1:end-1), :); % Extract ordered vertices (exclude duplicate endpoint)
    end
end
