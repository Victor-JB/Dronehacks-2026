# Rohan Tummala — Agent Context

**Role:** Dead Reckoning / Wall Vault

_Update this file every time your agent context changes. See `shared-context/README.md` for the rules._

---

## 2026-11-07 — Wall Vault Routine (`policy/vault`, mode 7 `wall_vault`)
- Implemented the 2-stage wall-vault routine in `refnav/core.py` (`_vault`, `_vault_wall_distance`, `_start_vault`) and added mode `7` (`wall_vault`) to `modes.json`.
- Handles walls with an empty gap underneath when the drone is on the ground or hovering low:
  1. `vault_find`: Climbs vertically out of the bottom gap while holding `(n, e)` until the wall is confirmed in the forward horizontal band (`<= vault_detect_max_m` across `confirm_frames`).
  2. `vault_approach`: Advances at that altitude to `vault_standoff_m` in front of the wall.
  3. `vault_climb`: Climbs vertically until all points at the wall distance drop below `vault_climb_margin_m` (top edge cleared) for `confirm_frames`, capped at `vault_max_alt_m` (2.8 m, inside Skybeta's 3.0 m envelope).
  4. `vault_cross` -> `vault_descend` -> `vault_settle` -> `land`: Flies forward `wall_dist + vault_overstep_m`, descends to `vault_descend_alt_m` on the far side, settles, and lands (`complete`).
- Added unit tests in `tests/test_refnav_core.py` (`test_vault_only_wall_with_gap_beneath_climbs_identifies_vaults_and_lands`, `test_vault_on_obstacle_climbs_over_wall_mid_route`).
