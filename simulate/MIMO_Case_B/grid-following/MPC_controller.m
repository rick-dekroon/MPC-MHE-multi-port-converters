function mpc = MPC_controller(N)
% Converter parameters
p = MIMO_Case_B_params();
mpc.p = p;
nx = 2;
mpc.nx = nx;
nu = 2;
mpc.nu = nu;

% Prediction horizon
mpc.N = N;

% Cost matrices
R = diag([1e2, 1e2]);
mpc.R = R;
Q = diag([1e0, 1e0]);
mpc.Q = Q;

% Constraints
mpc.Hx = [1  0;
          0  1;
         -1  0;
          0 -1];
mpc.hx = [100;
          100;
          100;
          100];

mpc.Hu = [1  1;
         -1  0;
          0 -1];
mpc.hu = [1;
          0;
          0];

% Initialization
X0 = zeros((N+1)*nx, 1);
U0 = zeros(N*nu, 1);
mpc.z_0 = [X0;
           U0];

% constructing QX,RU
mpc.QX = kron(eye(N), Q);
mpc.RU = kron(eye(N), R);

end
