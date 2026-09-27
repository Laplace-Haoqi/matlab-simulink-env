function [xcdot_nom,ycdot_nom,xr,yr,Dhatx,Dhaty,zetax_dot,zetay_dot,ebar] = ...
    top_nominal_estimator_safe(t,x,y,v,chi,zetax,zetay)
%TOP_NOMINAL_ESTIMATOR_SAFE Nominal upper-layer control + disturbance estimate.

% design parameters
cx = 10;
cy = 10;

% reference signal
xr = 0.5*t;
yr = 5;
xrdot = 0.5;
yrdot = 0;

xtilde = x - xr;
ytilde = y - yr;

xcdot_nom = -cx*xtilde + xrdot;
ycdot_nom = -cy*ytilde + yrdot;

lambda_x = 5;
lambda_y = 5;

Dhatx = lambda_x*x - zetax;
Dhaty = lambda_y*y - zetay;

eta_x = v*cos(chi);
eta_y = v*sin(chi);

zetax_dot = lambda_x*(eta_x + Dhatx);
zetay_dot = lambda_y*(eta_y + Dhaty);

% robust estimation-error bound
% Can be increased to improve conservativeness when estimator transient is large.
ebar = 50;

% finite-value guards
if ~isfinite(xcdot_nom), xcdot_nom = 0; end
if ~isfinite(ycdot_nom), ycdot_nom = 0; end
if ~isfinite(Dhatx), Dhatx = 0; end
if ~isfinite(Dhaty), Dhaty = 0; end
if ~isfinite(zetax_dot), zetax_dot = 0; end
if ~isfinite(zetay_dot), zetay_dot = 0; end

end
