% MATLAB script for computing the maximal positive invariant set with input
% constraints in 3D.
close all; clear; clc

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

R = diag([1e1, 1e1]);
Q = diag([1e0, 1e0, 3e-4]);
K = -dlqr(A, B, Q, R);

nx = 3; % Dimension of state space
nu = 2; % Number of control inputs

Hx = [eye(nx); -eye(nx)];
hx = [10; 10; 400; 10; 10; 400] - Hx * x_e;
Hu = [1, 1; -eye(nu)];
hu = [1; 0; 0] - Hu * u_e;

% Compute the maximal positive invariant set
[O_inf, X_k] = max_positive_invariant_set(A, B, K, Hx, hx, Hu, hu);

% Display results
disp('Maximal Positive Invariant Set Constraints:');
disp('H matrix:');
disp(O_inf.H);
disp('h vector:');
disp(O_inf.h);

V = inequalities_to_vertices(Hx, hx);
V_k = inequalities_to_vertices(X_k.H, X_k.h);
V_O = inequalities_to_vertices(O_inf.H, O_inf.h);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen
set(gcf, 'Renderer', 'opengl'); % use OpenGL
set(gca, 'SortMethod', 'childorder'); % improves draw order of patches
rotate3d on;  % ← This re-enables mouse rotation

% Plot Polyhedrons
K = convhull(V(:,1), V(:,2), V(:,3));
trisurf(K, V(:,1), V(:,2), V(:,3), 'FaceColor', 'blue', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'EdgeAlpha', '0.2');
K = convhull(V_k(:,1), V_k(:,2), V_k(:,3));
trisurf(K, V_k(:,1), V_k(:,2), V_k(:,3), 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'black', 'EdgeAlpha', '0.2');
K = convhull(V_O(:,1), V_O(:,2), V_O(:,3));
trisurf(K, V_O(:,1), V_O(:,2), V_O(:,3), 'FaceColor', 'yellow', 'FaceAlpha', 0.4, 'EdgeColor', 'black', 'EdgeAlpha', '0.2');

xlabel('$x_1 - x_1^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
ylabel('$x_2 - x_2^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
zlabel('$x_3 - x_3^{ss}$', 'Interpreter', 'latex', 'FontSize', 32);
view([45, 30]);
axis tight;
title('MPI Terminal Set', 'Interpreter', 'latex', 'FontSize', 32);
leg = legend('$\mathcal{X}$', '$\mathcal{X}_\kappa$', '$\mathcal{O}_\infty$');
set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
hold off;

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
    iter = 0;
    while iter < 100
        iter = iter + 1;
        disp(iter)
        
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
    O_inf.H = H;
    O_inf.h = h;
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
    
    options = optimoptions('linprog', 'Display', 'none', 'Algorithm', 'dual-simplex');
    
    i = 1;
    while i <= size(H_out, 1)
        H_test = H_out;
        h_test = h_out;
        H_test(i, :) = [];
        h_test(i) = [];
        
        % Solve LP to check if the constraint is necessary
        [~, fval, exitflag] = linprog(-H_out(i, :), H_test, h_test, [], [], [], [], options);
        
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
