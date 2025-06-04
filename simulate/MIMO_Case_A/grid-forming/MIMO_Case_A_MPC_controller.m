close all; clear; clc;

% Converter parameters
p = MIMO_Case_A_params();
nx = 3;
nu = 2;
ny = 2;

% Prediction horizon
N = 4;

% Initial state
x_0 = [0;
       0;
       0];

% Reference
iin_ref = 1;

r1 = [p.Vac;
      iin_ref];
r2 = [-p.Vac;
      iin_ref];

% Converter model
f1 = @(x, u) dynamics_f(x, u, r1, p);
g1 = @(x, u) dynamics_g(x, u, r1, p);
f2 = @(x, u) dynamics_f(x, u, r2, p);
g2 = @(x, u) dynamics_g(x, u, r2, p);

function xdot = dynamics_f(x, u, r, p)
    if r(1) >= 0
        [xdot, ~] = MIMO_Case_A_dynamics_SM1(x, u, p);
    else
        [xdot, ~] = MIMO_Case_A_dynamics_SM2(x, u, p);
    end
end

function y = dynamics_g(x, u, r, p)
    if r(1) >= 0
        [~, y] = MIMO_Case_A_dynamics_SM1(x, u, p);
    else
        [~, y] = MIMO_Case_A_dynamics_SM2(x, u, p);
    end
end

k1 = @(x, u) f1(x, u);
k2 = @(x, u) f1(x + p.Ts/2*k1(x,u), u);
k3 = @(x, u) f1(x + p.Ts/2*k2(x,u), u);
k4 = @(x, u) f1(x + p.Ts*k3(x,u), u);
fd1 = @(x, u) x + p.Ts/6*(k1(x,u) + 2*k2(x,u) + 2*k3(x,u) + k4(x,u));
gd1 = @(x, u) g1(x, u);

k1 = @(x, u) f2(x, u);
k2 = @(x, u) f2(x + p.Ts/2*k1(x,u), u);
k3 = @(x, u) f2(x + p.Ts/2*k2(x,u), u);
k4 = @(x, u) f2(x + p.Ts*k3(x,u), u);
fd2 = @(x, u) x + p.Ts/6*(k1(x,u) + 2*k2(x,u) + 2*k3(x,u) + k4(x,u));
gd2 = @(x, u) g2(x, u);

fun1 = @(xu) [fd1(xu(1:nx), xu(nx+1:nx+nu)); gd1(xu(1:nx), xu(nx+1:nx+nu))] - [xu(1:nx); r1];
fun2 = @(xu) [fd2(xu(1:nx), xu(nx+1:nx+nu)); gd2(xu(1:nx), xu(nx+1:nx+nu))] - [xu(1:nx); r2];

xu_0 = zeros(nx+nu,1);
f_sol1 = fsolve(fun1, xu_0);
f_sol2 = fsolve(fun2, xu_0);
disp("SM1 equilibrium: ")
disp(f_sol1)
disp("SM2 equilibrium: ")
disp(f_sol2)

syms iL1 iL2 vac

x = [iL1;
     iL2;
     vac];
assume(x, 'real')

syms d1 d2

u = [d1;
     d2];
assume(u, 'real')
assume([eye(nu); -eye(nu)] * u <= [1; 1; 0; 0])

xnext1 = fd1(x, u);
y1 = gd1(x, u);
xnext2 = fd2(x, u);
y2 = gd2(x, u);

sol1 = solve([xnext1; y1] == [x; r1], [x; u], 'ReturnConditions', true);
sol2 = solve([xnext2; y2] == [x; r2], [x; u], 'ReturnConditions', true);

A1 = jacobian(xnext1, x);
A1 = double(A1);
A2 = jacobian(xnext2, x);
A2 = double(A2);

B1 = jacobian(xnext1, u);
B1 = double(B1);
B2 = jacobian(xnext2, u);
B2 = double(B2);

C1 = jacobian(y1, x);
C2 = jacobian(y2, x);

D1 = jacobian(y1, u);
D2 = jacobian(y2, u);

% MPC controller
mpc = MPC_controller(A1, B1, A2, B2, N);
