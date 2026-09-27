# UAV Simulink 数值稳定性排查与修复建议

你报错的位置是积分器状态导数非有限值（NaN/Inf），最常见来源是分母接近 0。

## 你当前模型的高风险点

1. `chidot = u1/(m*v)`：当 `v -> 0` 时发散。
2. 扰动项中的 `1/(v-vm)^2`：当 `v ≈ vm` 时发散。
3. 底层估计器中的 `u1/(m*v)`：同样会在 `v -> 0` 时发散。
4. QP层若使用代码生成不支持的语法（如 `try/catch`）会直接报错终止。

## 本次新增的安全版本文件

- `system_model_safe.m`
- `top_nominal_estimator_safe.m`
- `top_cbf_qp_robust_safe.m`
- `low_level_tracking_controller_safe.m`

## 在 Simulink 中替换建议

- 将四个 MATLAB Function Block 的函数体分别替换为对应 `*_safe.m` 的代码。
- 确保初始速度 `v(0)` 不小于 `0.5`（或与你的 `v_eps` 一致）。
- 积分器可先限制上下界，帮助定位问题。
- 求解器建议先试：`ode45`（variable-step），并收紧相对误差到 `1e-4` 或 `1e-5`。

## 进一步调参建议

- 若仍抖振：将 `c2v,c2chi` 从 `100` 降到 `20~60` 再逐步增大。
- 若安全约束过激导致跳变：减小 `a`，或增大 `gamma`；必要时对 `ebar` 做平滑。
- 若估计器太快引发噪声放大：把 `lambda_*` 从 `5` 降到 `1~3`。

