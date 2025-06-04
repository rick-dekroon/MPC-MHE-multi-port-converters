close all; clear; clc;

% Converter parameters
p = MIMO_Case_A_params();
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

r1 = [p.Vac/p.Rload_ac;
      iin_ref];
r2 = [-p.Vac/p.Rload_ac;
       iin_ref];

% Converter model
f1 = @(x, u, phi) dynamics_f(x, u, phi, p);
g1 = @(x, u, phi) dynamics_g(x, u, phi, p);
f2 = @(x, u, phi) dynamics_f(x, u, phi, p);
g2 = @(x, u, phi) dynamics_g(x, u, phi, p);

function xdot = dynamics_f(x, u, phi, p)
    if phi <= pi
        [xdot, ~] = MIMO_Case_A_dynamics_SM1(x, u, phi, p);
    else
        [xdot, ~] = MIMO_Case_A_dynamics_SM2(x, u, phi, p);
    end
end

function y = dynamics_g(x, u, phi, p)
    if phi <= pi
        [~, y] = MIMO_Case_A_dynamics_SM1(x, u, phi, p);
    else
        [~, y] = MIMO_Case_A_dynamics_SM2(x, u, phi, p);
    end
end

k1 = @(x, u, phi) f1(x, u, phi);
k2 = @(x, u, phi) f1(x + p.Ts/2*k1(x,u,phi), u, phi);
k3 = @(x, u, phi) f1(x + p.Ts/2*k2(x,u,phi), u, phi);
k4 = @(x, u, phi) f1(x + p.Ts*k3(x,u,phi), u, phi);
fd1 = @(x, u, phi) x + p.Ts/6*(k1(x,u,phi) + 2*k2(x,u,phi) + 2*k3(x,u,phi) + k4(x,u,phi));
gd1 = @(x, u, phi) g1(x, u, phi);

k1 = @(x, u, phi) f2(x, u, phi);
k2 = @(x, u, phi) f2(x + p.Ts/2*k1(x,u,phi), u, phi);
k3 = @(x, u, phi) f2(x + p.Ts/2*k2(x,u,phi), u, phi);
k4 = @(x, u, phi) f2(x + p.Ts*k3(x,u,phi), u, phi);
fd2 = @(x, u, phi) x + p.Ts/6*(k1(x,u,phi) + 2*k2(x,u,phi) + 2*k3(x,u,phi) + k4(x,u,phi));
gd2 = @(x, u, phi) g2(x, u, phi);

fun1 = @(xu) [fd1(xu(1:nx), xu(nx+1:nx+nu), pi/2); gd1(xu(1:nx), xu(nx+1:nx+nu), pi/2)] - [xu(1:nx); r1];
fun2 = @(xu) [fd2(xu(1:nx), xu(nx+1:nx+nu), 3*pi/2); gd2(xu(1:nx), xu(nx+1:nx+nu), 3*pi/2)] - [xu(1:nx); r2];

options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt');

xu_0 = zeros(nx+nu,1);
f_sol1 = fsolve(fun1, xu_0, options);
f_sol2 = fsolve(fun2, xu_0, options);
disp("SM1 equilibrium: ")
disp(f_sol1)
disp("SM2 equilibrium: ")
disp(f_sol2)

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

f1 = @(x, u, phi) dynamics_f1(x, u, phi, p);
g1 = @(x, u, phi) dynamics_g1(x, u, phi, p);
f2 = @(x, u, phi) dynamics_f2(x, u, phi, p);
g2 = @(x, u, phi) dynamics_g2(x, u, phi, p);

function xdot = dynamics_f1(x, u, phi, p)
    [xdot, ~] = MIMO_Case_A_dynamics_SM1(x, u, phi, p);
end

function y = dynamics_g1(x, u, phi, p)
    [~, y] = MIMO_Case_A_dynamics_SM1(x, u, phi, p);
end

function xdot = dynamics_f2(x, u, phi, p)
    [xdot, ~] = MIMO_Case_A_dynamics_SM2(x, u, phi, p);
end

function y = dynamics_g2(x, u, phi, p)
    [~, y] = MIMO_Case_A_dynamics_SM2(x, u, phi, p);
end

k1 = @(x, u, phi) f1(x, u, phi);
k2 = @(x, u, phi) f1(x + p.Ts/2*k1(x,u,phi), u, phi);
k3 = @(x, u, phi) f1(x + p.Ts/2*k2(x,u,phi), u, phi);
k4 = @(x, u, phi) f1(x + p.Ts*k3(x,u,phi), u, phi);
fd1 = @(x, u, phi) x + p.Ts/6*(k1(x,u,phi) + 2*k2(x,u,phi) + 2*k3(x,u,phi) + k4(x,u,phi));
gd1 = @(x, u, phi) g1(x, u, phi);

k1 = @(x, u, phi) f2(x, u, phi);
k2 = @(x, u, phi) f2(x + p.Ts/2*k1(x,u,phi), u, phi);
k3 = @(x, u, phi) f2(x + p.Ts/2*k2(x,u,phi), u, phi);
k4 = @(x, u, phi) f2(x + p.Ts*k3(x,u,phi), u, phi);
fd2 = @(x, u, phi) x + p.Ts/6*(k1(x,u,phi) + 2*k2(x,u,phi) + 2*k3(x,u,phi) + k4(x,u,phi));
gd2 = @(x, u, phi) g2(x, u, phi);

xnext1 = fd1(x, u, phi);
y1 = gd1(x, u, phi);
xnext2 = fd2(x, u, phi);
y2 = gd2(x, u, phi);

sol1 = solve([xnext1; y1] == [x; r1], [x; u], 'ReturnConditions', true);
sol2 = solve([xnext2; y2] == [x; r2], [x; u], 'ReturnConditions', true);

A1 = jacobian(xnext1, x);
matlabFunction(A1, 'Vars', {x, u, phi}, 'File', 'A1');
A1 = double(A1);
A2 = jacobian(xnext2, x);
matlabFunction(A2, 'Vars', {x, u, phi}, 'File', 'A2');
A2 = double(A2);

B1 = jacobian(xnext1, u);
B1 = double(B1);
B2 = jacobian(xnext2, u);
B2 = double(B2);

C1 = jacobian(y1, x);
matlabFunction(C1, 'Vars', {x, u, phi}, 'File', 'C1');
C2 = jacobian(y2, x);
matlabFunction(C2, 'Vars', {x, u, phi}, 'File', 'C2');

D1 = jacobian(y1, u);
D2 = jacobian(y2, u);

% MPC controller
mpc = MPC_controller(A1, B1, A2, B2, N_MPC);

% MHE estimator
mhe = MHE_estimator(N_MHE);
