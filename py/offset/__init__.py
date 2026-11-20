"""Upright-offset estimate for the balance controller: simulation prototype, PC only. docs/offset_estimator.md.

identity  the estimator: the drift identity fitted online over quiet balance, pooled and carried between catches
observer  a fixed-gain observer for angle, rate and offset: too noisy as an offset estimate on this rig (kept for
          the open rate-estimate question)
replay    log loading, ground truth (stage1_report quiet_offsets) and replays for py/offset_check.py
"""
