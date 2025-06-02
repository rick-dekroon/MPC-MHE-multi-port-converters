% MATLAB script for computing the maximal positive invariant set with input
% constraints in 2D.
close all; clear; clc;

% Define system matrices
A = [1 0.3; 0 1];
B = [0; 0.3];

% LQR design
R = 1e-2;
Q = diag([1e1, 1e0]);
K = -dlqr(A, B, Q, R);

% Define state constraint set Hx * x <= hx
Hx = [1  0;
      0  1;
     -1  0;
      0 -1];
hx = [1;
      25;
      120;
      50];

% Define input constraint set Hu * u <= hu
Hu = [1;
     -1];
hu = [10;
      20];

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

X_poly = polyshape(V(:,1), V(:,2));
X_k_poly = polyshape(V_k(:,1), V_k(:,2));
O_poly = polyshape(V_O(:,1), V_O(:,2));

% Plot results
figure; hold on; grid on;

% Plot Polyhedrons
plot(X_poly, 'FaceColor', 'blue', 'FaceAlpha', 0.2);
plot(X_k_poly, 'FaceColor', 'green', 'FaceAlpha', 0.4);
plot(O_poly, 'FaceColor', 'red', 'FaceAlpha', 0.6);

xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('MPI Terminal Set', 'Interpreter', 'latex');
leg = legend('$\mathcal{X}$', '$\mathcal{X}_\kappa$', '$\mathcal{O}_\infty$');
set(gca, 'FontSize', 14);
set(gca, 'TickLabelInterpreter', 'latex');
set(leg, 'Interpreter', 'latex');
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

    % Order vertices counterclockwise
    if size(V, 1) > 2
        K = convhull(V(:,1), V(:,2)); % Compute convex hull indices
        V = V(K(1:end-1), :); % Extract ordered vertices (exclude duplicate endpoint)
    end
end
