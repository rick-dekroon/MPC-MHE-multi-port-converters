function [xdot, y] = MIMO_case_C_dynamics(x, u, p)
%MIMO_CASE_C_DYNAMICS describes the converter dynamics using the state vector
% x = [iL
%      iLm
%      vac] (column vector)
%
%Syntax
% [xdot, y] = MIMO_case_C_dynamics(x, u, p)
%
%Input arguments:
% x     system state
% u     control actions
% p     converter parameters as returned by MIMO_case_C_params
%
%Output arguments:
% xdot  derivative of system state
% y     output vector
%
%See also
%MIMO_case_C_params
%

idx_iL = 1; idx_iLm = 2; idx_vac = 3;
iL = x(idx_iL); iLm = x(idx_iLm); vac = x(idx_vac);

idx_d1 = 1; idx_d2 = 2;
d1 = u(idx_d1); d2 = u(idx_d2);

xdot = zeros(3,1,'like',x);
xdot(idx_iL) = 1/p.L*((p.Vin + p.Vbi)*d1 - (vac/p.n + p.Vbi)*(1-d1-d2));
xdot(idx_iLm) = 1/p.Lm*(-p.Vbi*d1 + p.Vin*d2 + vac/p.n*(1-d1-d2));
xdot(idx_vac) = 1/p.Cac*(1/p.n*(iL - iLm)*(1-d1-d2) - vac/p.Rload_ac);

y = zeros(2,1,'like',x);
y(1) = vac;
y(2) = iL*d1 + iLm*d2; % iin


end
