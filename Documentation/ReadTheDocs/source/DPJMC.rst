Dynamic Proportional Joint Moment Controller
============================================

The Dynamic Proportional Joint Moment Controller (DPJMC) is an ankle
controller that uses plantar-pressure and COP high-level input when those
signals are valid.

The v1 controller is fail-safe by default. If the pressure/COP interface is
not populated, stale, invalid, or faulted, the controller returns zero torque.

Sign Convention
---------------

- Internal ``tau_des_pf_nm > 0`` means plantarflexion assistance.
- OpenExo ankle motor commands use positive torque for dorsiflexion.
- DPJMC therefore returns ``-tau_des_pf_nm`` to the OpenExo motor layer.

Required High-Level Input
-------------------------

The controller reads ``SideData::dpjmc_hl``. This interface is reserved for
future pressure features, COP estimation, gait state estimation, and motion
intent recognition.

Until that interface is valid, DPJMC remains in zero-assist.

Parameter Index Order
---------------------

Parameter index order can be found in ``ControllerData.h``.

- ``alpha_base`` - baseline dynamic proportional gain.
- ``alpha_min`` - minimum dynamic gain.
- ``alpha_max`` - maximum dynamic gain.
- ``tau_max_nm`` - plantarflexion assistance torque saturation in Nm.
- ``tau_slew_rate_nm_s`` - torque slew-rate limit in Nm/s.
- ``pressure_timeout_ms`` - pressure/COP timeout in ms.
- ``p_total_min`` - minimum valid total pressure.
- ``ankle_x_ap_norm`` - normalized AP ankle location.
- ``tau_proxy_scale_nm`` - pressure moment proxy scale.
- ``cop_ap_midfoot_on_norm`` - reserved midfoot threshold.
- ``cop_ap_forefoot_on_norm`` - forefoot threshold.
- ``cop_ml_dev_threshold_norm`` - ML COP deviation threshold.
- ``cop_ml_dot_threshold_norm_s`` - ML COP velocity threshold.
- ``ml_pressure_diff_threshold`` - medial-lateral pressure difference threshold.
- ``k_ap_pos`` - AP COP position modulation gain.
- ``k_ap_vel`` - AP COP velocity modulation gain.
- ``t_off_ms`` - fast unloading time constant.
- ``t_on_ms`` - slow recovery time constant.
- ``torque_filter_alpha`` - torque command EWMA alpha.
- ``use_pid`` - torque-sensor PID enable flag.
- ``p_gain`` - PID proportional gain.
- ``i_gain`` - PID integral gain.
- ``d_gain`` - PID derivative gain.
