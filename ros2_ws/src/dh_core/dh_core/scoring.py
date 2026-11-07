"""Official DroneHacks 2026 run-score formulas (Challenge page, "Scoring").

CP values 4, 7, 10, 7, 7. Unfinished: sum(reached) + 7 * T*_last / T_last.
Finished below walls: 35 + 20 * T*_finish / T_finish + 5 (window).
Return completed: run points x 1.25. Over-wall checkpoints earn 15 %.
"""
from __future__ import annotations

CP_POINTS = (4, 7, 10, 7, 7)


def finished_run_points(t_finish: float, t_fastest: float, window: bool, returned: bool) -> float:
    pts = sum(CP_POINTS) + 20.0 * min(1.0, t_fastest / t_finish) + (5.0 if window else 0.0)
    return pts * (1.25 if returned else 1.0)


def unfinished_run_points(n_checkpoints: int, t_last: float, t_fastest_last: float) -> float:
    if n_checkpoints <= 0:
        return 0.0
    return float(sum(CP_POINTS[:n_checkpoints])) + 7.0 * min(1.0, t_fastest_last / t_last)


def expected_run_points(p_finish: float, t_finish: float, t_fastest: float, window: bool,
                        p_return: float = 0.0, partial_points_if_fail: float = 14.0) -> float:
    """Expected points of ONE attempt; the best of several attempts counts (see best_of)."""
    fin = finished_run_points(t_finish, t_fastest, window, False)
    fin_ret = finished_run_points(t_finish, t_fastest, window, True)
    return p_finish * ((1 - p_return) * fin + p_return * fin_ret) + (1 - p_finish) * partial_points_if_fail


def best_of(n_attempts: int, p_success: float) -> float:
    """Probability that at least one of n independent attempts succeeds."""
    return 1.0 - (1.0 - p_success) ** n_attempts
