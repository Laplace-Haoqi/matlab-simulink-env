function [dvbar,dchibar,uv,u1,zetav_dot,zetachi_dot] = ...
    low_level_tracking_controller_safe(t,xcdot_d,ydot_d,vbar,chibar,v,chi,d,zetav,zetachi,uv_prev,u1_prev)
%LOW_LEVEL_TRACKING_CONTROLLER_SAFE Dynamic-surface backstepping with protections.

m = 20;

c2v = 100;
c2chi = 100;

lambda_v = 5;
lambda_chi = 5;

% singularity protections
v_eps = 0.5;
v_safe = sign(v) * max(abs(v), v_eps);
if abs(v_safe) < v_eps
    v_safe = v_eps;
end

v2d = sqrt(xcdot_d^2 + ydot_d^2);
chi2d = atan2(ydot_d, xcdot_d);

ds2 = 0.05;
dvbar = (v2d - vbar)/ds2;

% heading error via wrapped angle avoids large transient jumps
chi_err = atan2(sin(chi - chibar), cos(chi - chibar));
dchibar = (chi2d - chibar)/ds2;

v2tilde = v - vbar;
chi2tilde = chi_err;

uv_nom = m*(-c2v*v2tilde + dvbar) + d;
u1_nom = m*v_safe*(-c2chi*chi2tilde + dchibar);

Dhatv = lambda_v*v - zetav;
Dhatchi = lambda_chi*chi - zetachi;

% Use previous-step control signals inside estimator to avoid algebraic loop.
zetav_dot = lambda_v*((uv_prev - d)/m + Dhatv);
zetachi_dot = lambda_chi*(u1_prev/(m*v_safe) + Dhatchi);

uv = uv_nom - m*Dhatv;
u1 = u1_nom - m*v_safe*Dhatchi;

if ~isfinite(dvbar), dvbar = 0; end
if ~isfinite(dchibar), dchibar = 0; end
if ~isfinite(uv), uv = 0; end
if ~isfinite(u1), u1 = 0; end
if ~isfinite(zetav_dot), zetav_dot = 0; end
if ~isfinite(zetachi_dot), zetachi_dot = 0; end

% suppress unused warning
if t < -1
    uv = uv_prev;
    u1 = u1_prev;
end

end
