# CAC/DPJMC 左脚鞋垫离线分析报告

## 结论边界

本报告使用单只左脚 18-zone pressure-only data，输出 CoP（center of pressure）、步态状态机（gait finite-state machine）和跖屈为正（plantarflexion-positive）的踝矩代理量（ankle moment proxy）。
该 `tau_proxy_pf_nm` 不是完整逆动力学（inverse dynamics）踝关节力矩，因为没有 GRF vector、COM acceleration、ankle kinematics 和 body mass。

## 数据分离

- 原始行数（raw rows）：58647
- clean 行数（clean rows）：15710
- 跨文件重叠删除（removed overlap rows）：42937
- 去重策略：跨文件按 `(timestamp, L1-L18 pressure vector)` 做 multiset subtraction；文件内不按秒级 timestamp 压缩。
- 时间轴：CSV timestamp 为秒级，`time_s` 按 20 Hz sampling rate 从 row order 重建。

| source_file | activity_hint | raw_rows | clean_rows | cycles | removed_overlap_rows |
|---|---:|---:|---:|---:|---:|
| 2026-5-14_levelground 1_left.csv | level_walking | 1173 | 1173 | 19 | 0 |
| 2026-5-14_levelground 2_left.csv | level_walking | 2651 | 1478 | 17 | 1173 |
| 2026-5-14_sit to stand_left.csv | sittostand | 5695 | 3044 | 45 | 2651 |
| 2026-5-14_ramp_left.csv | ramp | 8834 | 3139 | 1 | 5695 |
| 2026-5-14_stair1_left.csv | stair1 | 11507 | 2673 | 23 | 8834 |
| 2026-5-14_stair2_left.csv | stair2 | 13077 | 1570 | 18 | 11507 |
| 2026-5-14_noexo.csv | noexo_control | 15710 | 2633 | 30 | 13077 |

## 方法

- 坐标系（coordinate system）：AP 0=heel、1=toe；ML 0=centerline、左脚 +ML=lateral。
- 单位（unit）：pressure 为 g，正值表示 compressive normal load；`Fz_N = total_g * 0.00980665`。
- FSM 阈值：`TOTAL_STANCE_THRESHOLD_G=2000.0`、`COP_MIN_TOTAL_G=200.0`、`MIN_STATE_FRAMES=3`、timeout=150 ms。
- 踝矩代理量：`tau_proxy_pf_nm=max(0,(cop_ap_norm-0.18)*0.21089*Fz_N*1.0)`，另输出 clipped column，`tau_max_nm=1.5`。
- Failure behavior：low pressure、timeout、invalid CoP 时 `tau_proxy_pf_nm=0`，并写入 `quality_flags`。

## CAC 分类限制

当前 `activity_pred` 是 rule-based baseline，不训练机器学习模型（machine learning model）。单只左脚 pressure-only 特征通常不足以可靠区分上坡/下坡与上台阶/下台阶，因此 ramp/stair trial 输出 `ambiguous_ramp` 或 `ambiguous_stair`。

## Exo vs No-Exo 探索性统计

统计以 stance cycle 为单位，过滤 saturation/timeout cycle。由于当前数据未证明严格 paired trial，只能报告 exploratory effect，不能直接宣称 causal significance。

| metric | n_exo | n_noexo | median_exo | median_noexo | median_diff | 95% CI | Cliff's delta | p_value | interpretation |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| stance_duration_s | 123 | 30 | 0.7 | 0.6 | 0.1 | [0.05, 0.15] | 0.382114 | 0.00119651 | exploratory_significant |
| swing_duration_s | 123 | 30 | 0.55 | 0.55 | 0 | [-0.05, 0.05] | -0.0168022 | 0.88672 | not_significant |
| peak_total_g | 123 | 30 | 26097 | 29317.5 | -3220.5 | [-4824, -966.3] | -0.376152 | 0.00142686 | exploratory_significant |
| loading_rate_g_s | 123 | 30 | 95840 | 110530 | -14690 | [-5.193e+04, 1.381e+04] | -0.157182 | 0.182648 | not_significant |
| cop_ap_excursion_norm | 123 | 30 | 0.363269 | 0.496839 | -0.13357 | [-0.179, -0.0746] | -0.299729 | 0.0110466 | exploratory_significant |
| cop_ml_excursion_norm | 123 | 30 | 0.368544 | 0.823917 | -0.455373 | [-0.5396, -0.08698] | -0.287263 | 0.0148701 | exploratory_significant |
| peak_tau_proxy_pf_nm | 123 | 30 | 11.8263 | 22.7203 | -10.894 | [-14.38, -6.254] | -0.489702 | 3.29749e-05 | exploratory_significant |
| tau_proxy_impulse_nm_s | 123 | 30 | 3.47125 | 5.53862 | -2.06737 | [-3.469, 0.04703] | -0.279404 | 0.0178413 | not_significant |

## Verification

- CoP AP 已 clamp 到 `[0,1]`，ML 已 clamp 到 `[-1,1]`。
- `quality_flags` 记录 low_pressure、cop_invalid、saturation、timeout。
- 与 firmware 常量保持一致：20 Hz sampling、150 ms pressure timeout、g pressure unit、plantarflexion-positive tau proxy、OpenExo ankle command dorsiflexion-positive。
