function mpc = MPC_controller(A1, B1, A2, B2, N)
% Converter parameters
p = MIMO_Case_A_params();
mpc.p = p;
nx = 2;
mpc.nx = nx;
nu = 2;
mpc.nu = nu;

% Prediction horizon
mpc.N = N;

% Cost matrices
R = diag([1e-1, 1e1]);
%R = diag([1e0, 1e2]);
Q = diag([1e0, 1e0]);

% Constraints
mpc.Hx = [1  0;
          0  1;
         -1  0;
          0 -1];
mpc.hx = [10;
          10;
          10;
          10];

mpc.Hu = [1  1;
         -1  0;
          0 -1];
mpc.hu = [1;
          0;
          0];

% Terminal cost
mpc.Pinf1 = idare(A1, B1, Q, R, [], []);
mpc.Pinf2 = idare(A2, B2, Q, R, [], []);

% Initialization
X0 = zeros((N+1)*nx, 1);
U0 = zeros(N*nu, 1);
mpc.z_0 = [X0;
           U0];

% constructing QX,RU
mpc.QX = kron(eye(N), Q);
mpc.RU = kron(eye(N), R);

end
