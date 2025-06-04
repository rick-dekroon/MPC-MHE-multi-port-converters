function p = MIMO_Case_A_params(varargin)
%MIMO_CASE_A_PARAMS returns the nominal parameters of the
%buck-buck converter in a structure.
%
%Syntax:
% p = MIMO_Case_A_params();
% p = MIMO_Case_A_params(ops);
%
%Input arguments:
% ops       A structure where one may provide parameter values to be
%           overriden. 
%

p.nominal = true;

% ---- Fixed Quantities ----

% General parameters
p.fsw = 50e3;
p.fs = 10e3;
p.Ts = 1/p.fs;
p.fgrid = 50;
p.Tmax = 100e-3;

% Voltages
p.Vin = 350;
p.Vbi = 400;
p.Vac = 325;

% Currents
p.Iac = 4;
p.Ibi = 1;

% Parasetic Resistances of Sources
p.Rp_Vin = 0.1;
p.Rp_Vbi = 0.1;
p.Rp_Vac = 0.1;

% Inductances
p.L1 = 0.05;
p.L2 = 0.05;

% Parasetic Resistances of Inductors
p.Rp_L1 = 0.1;
p.Rp_L2 = 0.1;

% Capacitances
p.Cin = 100e-6;
p.Cbi = 100e-6;
p.Cac = 10e-6;

% Parasetic Resistances of Capacitors
p.Rp_Cin = 0.01;
p.Rp_Cbi = 0.01;
p.Rp_Cac = 0.01;

% On Resistance MOSFET
p.Rdson = 0.1;

% Averages
p.Iac_avg = p.Iac/sqrt(2);
p.Vac_avg = p.Vac/sqrt(2);

% Powers
p.Pac = p.Vac_avg*p.Iac_avg;
p.Pbi = p.Vbi*p.Ibi;
p.Pin = p.Pac-p.Pbi;

% Input Current
p.Iin = p.Pin/p.Vin;
if p.Iin<0 
    disp("ERROR: Input current is smaller than 0")
end

p.Vin_set = p.Vin-p.Iin*p.Rp_Vin;
p.Vbi_set = p.Vbi-p.Ibi*p.Rp_Vbi;
p.Rload_ac = p.Vac/p.Iac;

% ---- Override constants using options ----

if nargin > 0
    o = varargin{1};
    if isstruct(o)
        pn = fieldnames(p);
        for i=1:length(pn)
            fname = pn(i);
            fname = fname{1};
            if isfield(o, fname)
                p.(fname) = o.(fname);
                p.nominal = false;
            end
        end
    end
end

end
