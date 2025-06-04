function [xdot, y] = MIMO_Case_A_dynamics_SM1(x, u, p)
%MIMO_CASE_A_DYNAMICS_SM1 describes the converter dynamics using the state vector
% x = [iL1
%      iL2
%      vac] (column vector)
%
%Where vo represents the output voltage
%
%Syntax
% [xdot, y] = MIMO_Case_A_dynamics_SM1(x, u, p)
%
%Input arguments:
% x     system state
% u     control actions
% p     converter parameters as returned by MIMO_Case_A_params
%
%Output arguments:
% xdot  derivative of system state
% y     output vector
%
%See also
%MIMO_Case_A_params
%

idx_iL1 = 1; idx_iL2 = 2; idx_vac = 3;
iL1 = x(idx_iL1); iL2 = x(idx_iL2); vac = x(idx_vac);

idx_d1 = 1; idx_d2 = 2;
d1 = u(idx_d1); d2 = u(idx_d2);

xdot = zeros(3,1,'like',x);
xdot(idx_iL1) = 1/p.L1*((vac - p.Vbi)*d1 + (vac - p.Vbi)*d2 + (vac + p.Vin)*(1-d1-d2));
xdot(idx_iL2) = 1/p.L2*((p.Vin + p.Vbi - vac)*d1 - vac*d2 - vac*(1-d1-d2));
xdot(idx_vac) = 1/p.Cac*(iL2 - iL1 - vac/p.Rload_ac);

y = zeros(2,1,'like',x);
y(1) = vac;
y(2) = (iL2)*d1 + (iL1)*(1-d1-d2); % iin


end
