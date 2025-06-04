% MATLAB script for computing the maximal controlled positive invariant set
% with input constraints in 3D.
close all; clear; clc;

currentFolder = pwd;
figuresFolder = fullfile(currentFolder, 'figures');

if ~exist(figuresFolder, 'dir')
    mkdir(figuresFolder);
end

switch_mode = 'SM1';

switch switch_mode
    case 'SM1'
        A = [0.990429637098200,0.009570362901800,0.001869303522832;0.009570362901800,0.990429637098200,-0.001869303522832;-9.346517614158701,9.346517614158701,0.865825211252908];
        B = [-1.490319453525038,-1.495159726762519;1.490319453525038,-0.004840273237481;14.355544352700347,7.177772176350174];

    case 'SM2'
        A = [0.990429637098200,0.009570362901800,0.001869303522832;0.009570362901800,0.990429637098200,-0.001869303522832;-9.346517614158701,9.346517614158701,0.865825211252908];
        B = [-1.490319453525038,-0.004840273237481;1.490319453525038,-1.495159726762519;14.355544352700347,-7.177772176350174];

    otherwise
        error('Unknown case');
end

nx = 3; % Dimension of state space
nu = 2; % Number of control inputs

Hx = [eye(nx); -eye(nx)];
hx = [10; 10; 400; 10; 10; 400];
Hu = [1, 1; -eye(nu)];
hu = [1; 0; 0];

% Compute the maximal positive invariant set
C_inf = max_controlled_positive_invariant_set(A, B, Hx, hx, Hu, hu);

% Display results
disp('Maximal Controlled Positive Invariant Set Constraints:');
disp('H matrix:');
disp(C_inf.H);
disp('h vector:');
disp(C_inf.h);

V = inequalities_to_vertices(Hx, hx);
V_C = inequalities_to_vertices(C_inf.H, C_inf.h);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen
set(gcf, 'Renderer', 'opengl'); % use OpenGL
set(gca, 'SortMethod', 'childorder'); % improves draw order of patches
rotate3d on;  % ← This re-enables mouse rotation

% Plot Polyhedrons
K = convhull(V(:,1), V(:,2), V(:,3));
trisurf(K, V(:,1), V(:,2), V(:,3), 'FaceColor', 'blue', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'EdgeAlpha', '0.2');
K = convhull(V_C(:,1), V_C(:,2), V_C(:,3));
trisurf(K, V_C(:,1), V_C(:,2), V_C(:,3), 'FaceColor', 'red', 'FaceAlpha', 0.6, 'EdgeColor', 'black', 'EdgeAlpha', '0.2');

xlabel('$x_1$', 'Interpreter', 'latex', 'FontSize', 32);
ylabel('$x_2$', 'Interpreter', 'latex', 'FontSize', 32);
zlabel('$x_3$', 'Interpreter', 'latex', 'FontSize', 32);
view([135, 30]);
axis tight;
title('MCPI Terminal Set', 'Interpreter', 'latex', 'FontSize', 32);
leg = legend('$\mathcal{X}$', '$\mathcal{C}_\infty$');
set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);
set(gca, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
hold off;

exportgraphics(gcf, 'figures/MCPI_set.pdf', 'ContentType', 'vector');

function C_inf = max_controlled_positive_invariant_set(A, B, Hx, hx, Hu, hu)
    % Initialize C_0
    H = Hx;
    h = hx;
    iter = 0;
    while iter < 5
        iter = iter + 1;
        disp(iter)

        [H_pre, h_pre] = pre(A, B, Hu, hu, H, h);
        [H_next, h_next] = intersect_constraints(H_pre, h_pre, Hx, hx);
        
        if isequal(H_next, H) && isequal(h_next, h)
            C_inf.H = H;
            C_inf.h = h;
            return;
        end
        
        % Update C_k
        H = H_next;
        h = h_next;
    end
    C_inf.H = H;
    C_inf.h = h;
end

function [H_pre, h_pre] = pre(A, B, Hu, hu, H, h)
    H_aug = [H * [A, B];
             zeros(size(Hu, 1), size(A, 1)), Hu];
    h_aug = [h;
             hu];
    [H_pre, h_pre] = projection(H_aug, h_aug, size(A, 1));
end

function [H_proj, h_proj] = projection(H, h, nx)
    % Projects the polyhedron H * z <= h onto the first nx variables (x)
    % using Fourier-Motzkin elimination (no vertices, no toolboxes)

    z_dim = size(H, 2);        % total dimension of z = [x; u]

    % Start elimination from the last variable (assumed to be u_nu down to u_1)
    for elim_var = z_dim:-1:(nx+1)
        keep_rows = abs(H(:, elim_var)) > 1e-10;
        zero_rows = ~keep_rows;

        % Rows with positive and negative coefficients of variable to eliminate
        pos_rows = find(H(:, elim_var) > 1e-10);
        neg_rows = find(H(:, elim_var) < -1e-10);

        new_H = [];
        new_h = [];

        % For each pair (i,j), eliminate variable
        for i = 1:length(pos_rows)
            for j = 1:length(neg_rows)
                pi = pos_rows(i);
                nj = neg_rows(j);

                % Coefficients
                alpha = H(pi, elim_var);
                beta  = H(nj, elim_var);

                % New inequality: combine row pi and nj to eliminate variable
                row = (H(nj, :) / abs(beta)) + (H(pi, :) / abs(alpha));
                rhs = (h(nj) / abs(beta)) + (h(pi) / abs(alpha));

                row(elim_var) = [];  % Remove eliminated variable
                new_H = [new_H; row];
                new_h = [new_h; rhs];
            end
        end

        % Also keep inequalities that don't involve the variable
        H = [H(zero_rows, [1:elim_var-1, elim_var+1:end]); new_H];
        h = [h(zero_rows); new_h];
    end

    % Final projected inequalities (only over x)
    H_proj = H;
    h_proj = h;
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
