function mhe = MHE_estimator(N)
% Converter parameters
p = MIMO_case_C_params();
mhe.p = p;
nx = 2;
mhe.nx = nx;
nu = 2;
mhe.nu = nu;
ny = 2;
mhe.ny = ny;

% Estimation horizon
mhe.N = N;

% Cost matrices
R = diag([1e0, 1e0]);
mhe.R = inv(R);
Q = diag([1e0, 1e0]);
mhe.Q = inv(Q);

% Initialization
X0 = zeros((N+1)*nx, 1);
W0 = zeros(N*nx, 1);
mhe.z_0 = [X0;
           W0];

% constructing QW,RV
mhe.QW = kron(eye(N), Q);
mhe.RV = kron(eye(N), R);

end
