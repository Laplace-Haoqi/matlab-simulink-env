function [xdot,ydot,vdot,chidot,d] = system_model_safe(t,v,chi,uv,u1)
%SYSTEM_MODEL_SAFE Robust UAV kinematics/dynamics with singularity protection.

m = 20;
g = 9.81;
z = 80;

% --- singularity protections ---
v_eps = 0.5;                % minimum speed used in denominators
rel_v_eps = 0.5;            % minimum |v-vm| used in drag denominator

v_safe = sign(v) * max(abs(v), v_eps);
if abs(v_safe) < v_eps
    v_safe = v_eps;
end

uchi = sqrt(u1^2 + (m*g)^2);
vm = 0.215*4*log10(abs(z)) + 0.285*4;

rel_v = v - vm;
rel_v_sq = max(rel_v^2, rel_v_eps^2);

d = (0.5*1.225*rel_v_sq*1.37*0.02) + ...
    (2*0.01*0.1^2*uchi^2/g^2)/(1.225*rel_v_sq*1.37);

% system model
xdot = v*cos(chi) + (0.5*v*cos(chi) + 0.15*sin(0.3*t));
ydot = v*sin(chi) + (0.3*v*sin(chi) + 0.12*cos(0.25*t));

vdot = (uv - d)/m + (0.04*v^2 + 0.25*cos(0.4*t));
chidot = u1/(m*v_safe) + (0.7*cos(chi) + 0.2*sin(0.3*t));

% hard finite-value guard for Simulink integrators
if ~isfinite(xdot), xdot = 0; end
if ~isfinite(ydot), ydot = 0; end
if ~isfinite(vdot), vdot = 0; end
if ~isfinite(chidot), chidot = 0; end
if ~isfinite(d), d = 0; end

end
