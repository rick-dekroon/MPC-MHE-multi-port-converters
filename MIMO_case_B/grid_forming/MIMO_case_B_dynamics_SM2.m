function [xdot, y] = MIMO_case_B_dynamics_SM2(x, u, p)
%MIMO_CASE_B_DYNAMICS_SM2 describes the converter dynamics using the state vector
% x = [iL1
%      iL2
%      vac] (column vector)
%
%Syntax
% [xdot, y] = MIMO_case_B_dynamics_SM2(x, u, p)
%
%Input arguments:
% x     system state
% u     control actions
% p     converter parameters as returned by MIMO_case_B_params
%
%Output arguments:
% xdot  derivative of system state
% y     output vector
%
%See also
%MIMO_case_B_params
%

idx_iL1 = 1; idx_iL2 = 2; idx_vac = 3;
iL1 = x(idx_iL1); iL2 = x(idx_iL2); vac = x(idx_vac);

idx_d1 = 1; idx_d2 = 2;
d1 = u(idx_d1); d2 = u(idx_d2);

xdot = zeros(3,1,'like',x);
xdot(idx_iL1) = 1/p.L1*(p.Vin*d1 + (p.Vin - p.Vbi)*d2 + p.Vin*(1-d1-d2));
xdot(idx_iL2) = 1/p.L2*((p.Vin - p.Vbi)*d1 + (p.Vin - p.Vbi)*d2 + (p.Vin - vac)*(1-d1-d2));
xdot(idx_vac) = 1/p.Cac*(- vac/p.Rload_ac + iL2*(1-d1-d2));

y = zeros(2,1,'like',x);
y(1) = vac;
y(2) = iL1 + iL2; % iin


end
