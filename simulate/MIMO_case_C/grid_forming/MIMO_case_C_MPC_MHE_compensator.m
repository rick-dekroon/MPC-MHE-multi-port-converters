close all; clear; clc;

% Converter parameters
p = MIMO_case_C_params();
nx = 3;
nu = 2;
ny = 2;

% Prediction horizon
N = 4;
N_MPC = N;

% Estimation horizon
N_MHE = 4;

% Initial state
x_0 = [0;
       0;
       0];

% Reference
iin_ref = 1;

r = [p.Vac;
     iin_ref];

% Converter model
f = @(x, u) dynamics_f(x, u, p);
g = @(x, u) dynamics_g(x, u, p);

function xdot = dynamics_f(x, u, p)
    [xdot, ~] = MIMO_case_C_dynamics(x, u, p);
end

function y = dynamics_g(x, u, p)
    [~, y] = MIMO_case_C_dynamics(x, u, p);
end

k1 = @(x, u) f(x, u);
k2 = @(x, u) f(x + p.Ts/2*k1(x,u), u);
k3 = @(x, u) f(x + p.Ts/2*k2(x,u), u);
k4 = @(x, u) f(x + p.Ts*k3(x,u), u);
fd = @(x, u) x + p.Ts/6*(k1(x,u) + 2*k2(x,u) + 2*k3(x,u) + k4(x,u));
gd = @(x, u) g(x, u);

fun = @(xu) [fd(xu(1:nx), xu(nx+1:nx+nu)); gd(xu(1:nx), xu(nx+1:nx+nu))] - [xu(1:nx); r];

options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt');

xu_0 = zeros(nx+nu,1);
f_sol = fsolve(fun, xu_0, options);
disp("Equilibrium: ")
disp(f_sol)

syms iL iLm vac

x = [iL;
     iLm;
     vac];
assume(x, 'real')

syms d1 d2

u = [d1;
     d2];
assume(u, 'real')
assume([eye(nu); -eye(nu)] * u <= [1; 1; 0; 0])

xnext = fd(x, u);
y = gd(x, u);

sol = solve([xnext; y] == [x; r], [x; u], 'ReturnConditions', true);
x_e = double([sol.iL;
              sol.iLm;
              sol.vac]);
u_e = double([sol.d1;
              sol.d2]);
y_e = double(subs(subs(y, x, x_e), u, u_e));

A = jacobian(xnext, x);
matlabFunction(A, 'Vars', {x, u}, 'File', 'Aekf');

B = jacobian(xnext, u);

C = jacobian(y, x);
matlabFunction(C, 'Vars', {x, u}, 'File', 'Cekf');

D = jacobian(y, u);

% MPC controller
mpc = MPC_controller(N_MPC);

% MHE estimator
mhe = MHE_estimator(N_MHE);
