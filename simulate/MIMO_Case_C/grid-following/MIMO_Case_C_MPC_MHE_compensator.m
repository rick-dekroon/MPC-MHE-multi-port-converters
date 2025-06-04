close all; clear; clc;

% Converter parameters
p = MIMO_Case_C_params();
nx = 2;
nu = 2;
ny = 2;

% Prediction horizon
N = 4;
N_MPC = N;

% Estimation horizon
N_MHE = 4;

% Initial state
x_0 = [0;
       0];
phase_0 = 0;

% Reference
iin_ref = 1;

r = [p.Vac/p.Rload_ac;
     iin_ref];

% Converter model
f = @(x, u, phi) dynamics_f(x, u, phi, p);
g = @(x, u, phi) dynamics_g(x, u, phi, p);

function xdot = dynamics_f(x, u, phi, p)
    [xdot, ~] = MIMO_Case_C_dynamics(x, u, phi, p);
end

function y = dynamics_g(x, u, phi, p)
    [~, y] = MIMO_Case_C_dynamics(x, u, phi, p);
end

k1 = @(x, u, phi) f(x, u, phi);
k2 = @(x, u, phi) f(x + p.Ts/2*k1(x,u,phi), u, phi);
k3 = @(x, u, phi) f(x + p.Ts/2*k2(x,u,phi), u, phi);
k4 = @(x, u, phi) f(x + p.Ts*k3(x,u,phi), u, phi);
fd = @(x, u, phi) x + p.Ts/6*(k1(x,u,phi) + 2*k2(x,u,phi) + 2*k3(x,u,phi) + k4(x,u,phi));
gd = @(x, u, phi) g(x, u, phi);

fun = @(xu) [fd(xu(1:nx), xu(nx+1:nx+nu), pi/2); gd(xu(1:nx), xu(nx+1:nx+nu), pi/2)] - [xu(1:nx); r];

options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt');

xu_0 = zeros(nx+nu,1);
f_sol = fsolve(fun, xu_0, options);
disp("Equilibrium: ")
disp(f_sol)

syms iL1 iL2

x = [iL1;
     iL2];
assume(x, 'real')

syms d1 d2

u = [d1;
     d2];
assume(u, 'real')
assume([eye(nu); -eye(nu)] * u <= [1; 1; 0; 0])

syms phi
assume(phi, 'real')
assume([1; -1] * phi <= [2*pi; 0])

xnext = fd(x, u, phi);
y = gd(x, u, phi);

sol = solve([xnext; y] == [x; r], [x; u], 'ReturnConditions', true);

A = jacobian(xnext, x);
matlabFunction(A, 'Vars', {x, u, phi}, 'File', 'Aekf');

B = jacobian(xnext, u);

C = jacobian(y, x);
matlabFunction(C, 'Vars', {x, u, phi}, 'File', 'Cekf');

D = jacobian(y, u);

% MPC controller
mpc = MPC_controller(N_MPC);

% MHE estimator
mhe = MHE_estimator(N_MHE);
