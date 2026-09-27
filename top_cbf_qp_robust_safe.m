function [xcdot_d, ycdot_d] = top_cbf_qp_robust_safe(x, y, xcdot_nom, ycdot_nom, Dhatx, Dhaty, ebar)
%TOP_CBF_QP_ROBUST_SAFE Robust SP-CBF-QP with codegen-safe closed-form solver.
% Problem form:
%   min 0.5*||eta - eta_nom||^2
%   s.t. A*eta <= b
% where eta = [xcdot; ycdot].
%
% For this single linear inequality and H=I, the solution is the Euclidean
% projection of eta_nom onto the half-space A*eta <= b.

x0 = 7;
y0 = 5;
r0 = 2;
a = 50;
alpha = 0.2;
gamma = 2.5;

hx = x - x0;
hy = y - y0;
h = hx^2 + hy^2 - r0^2;

if h >= gamma
    grad_B = zeros(2,1);
else
    s = (gamma - h)/gamma;
    grad_h = [2*hx; 2*hy];
    dBdh = -2*a*(gamma-h)/(gamma^2)*exp(s^2);
    grad_B = dBdh*grad_h;
end

eta_nom = [xcdot_nom; ycdot_nom];
A = grad_B';
b = alpha*h - norm(grad_B)*ebar;

% Codegen-safe QP solution (no try/catch, no quadprog)
A_norm_sq = A(1)*A(1) + A(2)*A(2);
if A_norm_sq < 1e-12
    etabar = eta_nom;
else
    vio = A*eta_nom - b;
    if vio <= 0
        etabar = eta_nom;
    else
        % Project eta_nom onto the boundary A*eta=b
        etabar = eta_nom - (vio/A_norm_sq)*A';
    end
end

xcdot_d = etabar(1) - Dhatx;
ycdot_d = etabar(2) - Dhaty;

if ~isfinite(xcdot_d), xcdot_d = 0; end
if ~isfinite(ycdot_d), ycdot_d = 0; end

end
