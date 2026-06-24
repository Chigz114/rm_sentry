"""
MINCO 2D trajectory solver for holonomic robot.

Implements minimum-jerk (5th order polynomial) trajectory generation
with L-BFGS optimization over intermediate waypoint positions and durations.

For the initial validation (step 4), only smoothness + time cost is used.
Obstacle cost will be added in step 5.

Key components:
  - compute_coefficients: closed-form 5th order polynomial per segment
  - jerk_cost: ∫ |jerk|² dt in closed form
  - optimize: L-BFGS over intermediate positions (durations fixed initially)
  - sample_trajectory: dense sampling for visualization/tracking
"""
import math
import numpy as np
from scipy.optimize import minimize


def _poly_eval(t: float, coeffs: np.ndarray, deriv: int = 0) -> float:
    """Evaluate polynomial p(t) or its derivative."""
    if deriv == 0:
        return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5
    elif deriv == 1:
        return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 4*coeffs[4]*t**3 + 5*coeffs[5]*t**4
    elif deriv == 2:
        return 2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 20*coeffs[5]*t**3
    elif deriv == 3:
        return 6*coeffs[3] + 24*coeffs[4]*t + 60*coeffs[5]*t**2
    return 0.0


def _segment_coeffs(x0, v0, a0, x1, v1, a1, T):
    """Closed-form 5th order polynomial coefficients for one segment.

    Boundary: p(0)=x0, p'(0)=v0, p''(0)=a0, p(T)=x1, p'(T)=v1, p''(T)=a1
    Returns: [c0, c1, c2, c3, c4, c5]
    """
    c0 = x0
    c1 = v0
    c2 = a0 / 2.0
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5

    M = np.array([
        [T3,   T4,    T5],
        [3*T2, 4*T3,  5*T4],
        [6*T,  12*T2, 20*T3],
    ])
    b = np.array([
        x1 - c0 - c1*T - c2*T2,
        v1 - c1 - 2*c2*T,
        a1 - 2*c2,
    ])
    c345 = np.linalg.solve(M, b)
    return np.array([c0, c1, c2, c345[0], c345[1], c345[2]])


def _segment_jerk_cost(coeffs: np.ndarray, T: float) -> float:
    """Closed-form ∫_0^T |p'''(t)|² dt for 5th order polynomial."""
    c3, c4, c5 = coeffs[3], coeffs[4], coeffs[5]
    # p'''(t) = 6*c3 + 24*c4*t + 60*c5*t^2
    a = 6*c3
    b = 24*c4
    c = 60*c5
    # ∫(a + b*t + c*t^2)^2 dt = a^2*T + a*b*T^2 + (b^2+2ac)*T^3/3 + b*c*T^4/2 + c^2*T^5/5
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    return (a*a*T + a*b*T2 + (b*b + 2*a*c)*T3/3.0 + b*c*T4/2.0 + c*c*T5/5.0)


def _heuristic_intermediate_va(waypoints, durations):
    """Estimate intermediate velocities and accelerations.

    Velocity at waypoint i: direction from prev to next (chord direction),
    magnitude scaled by the shorter adjacent segment's average speed.
    This naturally reduces speed at sharp turns (where one segment is short)
    while maintaining continuous velocity through gentle turns.

    Acceleration: set to 0 (simple heuristic).
    """
    N = len(waypoints) - 1  # number of segments
    vels = []
    accs = []

    for i in range(len(waypoints)):
        if i == 0 or i == len(waypoints) - 1:
            vels.append((0.0, 0.0))
            accs.append((0.0, 0.0))
        else:
            # Chord direction: from waypoint i-1 to i+1
            dx = waypoints[i + 1][0] - waypoints[i - 1][0]
            dy = waypoints[i + 1][1] - waypoints[i - 1][1]
            chord_len = math.hypot(dx, dy)

            # Adjacent segment lengths
            d_prev = math.hypot(
                waypoints[i][0] - waypoints[i - 1][0],
                waypoints[i][1] - waypoints[i - 1][1],
            )
            d_next = math.hypot(
                waypoints[i + 1][0] - waypoints[i][0],
                waypoints[i + 1][1] - waypoints[i][1],
            )
            d_min = min(d_prev, d_next)

            # Velocity magnitude: based on shorter segment's average speed,
            # scaled by 0.5 to stay conservative
            T_prev = durations[i - 1] if i > 0 else 1.0
            T_next = durations[i] if i < N else 1.0
            v_mag = 0.8 * d_min / min(T_prev, T_next) if min(T_prev, T_next) > 1e-6 else 0.0

            if chord_len > 1e-6 and v_mag > 1e-6:
                vx = dx / chord_len * v_mag
                vy = dy / chord_len * v_mag
            else:
                vx, vy = 0.0, 0.0

            # Clamp to v_max
            vmag = math.hypot(vx, vy)
            if vmag > 4.0:
                vx = vx / vmag * 4.0
                vy = vy / vmag * 4.0

            vels.append((vx, vy))
            accs.append((0.0, 0.0))

    return vels, accs


class MincoSolver2D:
    """MINCO 2D trajectory solver for holonomic robot."""

    def __init__(self, v_max=4.0, a_max=4.0, w_smooth=1.0, w_time=10.0,
                 w_obs=0.0, w_dyn=0.0, w_collision=0.0,
                 d_soft=0.36, d_hard=0.28,
                 w_ref=0.0, waypoint_bound_m=0.0):
        self.v_max = v_max
        self.a_max = a_max
        self.w_smooth = w_smooth
        self.w_time = w_time
        self.w_obs = w_obs
        self.w_dyn = w_dyn
        self.w_collision = w_collision
        self.d_soft = d_soft
        self.d_hard = d_hard
        self.w_ref = w_ref
        self.waypoint_bound_m = waypoint_bound_m
        self.esdf_map = None

    def set_esdf(self, esdf_map):
        """Inject ESDF map for clearance cost computation."""
        self.esdf_map = esdf_map

    def compute_coefficients(self, waypoints, durations, boundary=None):
        """Compute polynomial coefficients for all segments.

        Args:
            waypoints: [(x, y), ...] including start and end (N+1 points for N segments)
            durations: [T1, T2, ...] per segment (N values)
            boundary: dict with 'start_vel', 'start_acc', 'end_vel', 'end_acc'
                      each (vx, vy) or (ax, ay). Default: all zeros.

        Returns:
            coeffs_x: (N, 6) array
            coeffs_y: (N, 6) array
        """
        if boundary is None:
            boundary = {
                'start_vel': (0.0, 0.0), 'start_acc': (0.0, 0.0),
                'end_vel': (0.0, 0.0), 'end_acc': (0.0, 0.0),
            }

        N = len(waypoints) - 1
        vels, accs = _heuristic_intermediate_va(waypoints, durations)

        # Override boundary velocities/accelerations
        vels[0] = boundary['start_vel']
        accs[0] = boundary['start_acc']
        vels[-1] = boundary['end_vel']
        accs[-1] = boundary['end_acc']

        coeffs_x = np.zeros((N, 6))
        coeffs_y = np.zeros((N, 6))

        for i in range(N):
            T = durations[i]
            coeffs_x[i] = _segment_coeffs(
                waypoints[i][0], vels[i][0], accs[i][0],
                waypoints[i+1][0], vels[i+1][0], accs[i+1][0], T
            )
            coeffs_y[i] = _segment_coeffs(
                waypoints[i][1], vels[i][1], accs[i][1],
                waypoints[i+1][1], vels[i+1][1], accs[i+1][1], T
            )

        return coeffs_x, coeffs_y

    def evaluate(self, coeffs_x, coeffs_y, durations, t):
        """Evaluate trajectory at time t.

        Returns: (x, y, vx, vy, ax, ay, jx, jy)
        """
        # Find which segment t falls in
        cum_T = np.cumsum(durations)
        seg_idx = np.searchsorted(cum_T, t, side='right')
        if seg_idx >= len(durations):
            seg_idx = len(durations) - 1
            local_t = durations[seg_idx]
        else:
            local_t = t - (cum_T[seg_idx] - durations[seg_idx]) if seg_idx > 0 else t

        cx = coeffs_x[seg_idx]
        cy = coeffs_y[seg_idx]

        x = _poly_eval(local_t, cx, 0)
        y = _poly_eval(local_t, cy, 0)
        vx = _poly_eval(local_t, cx, 1)
        vy = _poly_eval(local_t, cy, 1)
        ax = _poly_eval(local_t, cx, 2)
        ay = _poly_eval(local_t, cy, 2)
        jx = _poly_eval(local_t, cx, 3)
        jy = _poly_eval(local_t, cy, 3)

        return x, y, vx, vy, ax, ay, jx, jy

    def _sample_xy(self, coeffs_x, coeffs_y, durations, dt=0.03, with_va=False):
        """Vectorized trajectory sampling — returns (xs, ys) or (xs,ys,vmag,amag).

        Much faster than sample_trajectory for cost evaluation (no Python loop
        over samples, no tuple creation). Set with_va=True to also get velocity
        and acceleration magnitudes (for the dynamics penalty).
        """
        total_T = sum(durations)
        cum_T = np.cumsum(durations)
        n_samples = int(np.ceil(total_T / dt)) + 1
        ts = np.linspace(0.0, total_T, n_samples)

        xs = np.empty(n_samples)
        ys = np.empty(n_samples)
        if with_va:
            vmag = np.empty(n_samples)
            amag = np.empty(n_samples)

        for i in range(len(durations)):
            t_start = cum_T[i] - durations[i] if i > 0 else 0.0
            mask = (ts >= t_start - 1e-9) & (ts <= cum_T[i] + 1e-9)
            t = ts[mask] - t_start
            cx = coeffs_x[i]
            cy = coeffs_y[i]
            t2, t3, t4, t5 = t*t, t*t*t, t*t*t*t, t*t*t*t*t
            xs[mask] = cx[0] + cx[1]*t + cx[2]*t2 + cx[3]*t3 + cx[4]*t4 + cx[5]*t5
            ys[mask] = cy[0] + cy[1]*t + cy[2]*t2 + cy[3]*t3 + cy[4]*t4 + cy[5]*t5
            if with_va:
                vx = cx[1] + 2*cx[2]*t + 3*cx[3]*t2 + 4*cx[4]*t3 + 5*cx[5]*t4
                vy = cy[1] + 2*cy[2]*t + 3*cy[3]*t2 + 4*cy[4]*t3 + 5*cy[5]*t4
                ax = 2*cx[2] + 6*cx[3]*t + 12*cx[4]*t2 + 20*cx[5]*t3
                ay = 2*cy[2] + 6*cy[3]*t + 12*cy[4]*t2 + 20*cy[5]*t3
                vmag[mask] = np.sqrt(vx*vx + vy*vy)
                amag[mask] = np.sqrt(ax*ax + ay*ay)

        if with_va:
            return xs, ys, vmag, amag
        return xs, ys

    def _clearance_cost(self, coeffs_x, coeffs_y, durations):
        """Dual-barrier ESDF clearance cost via dense temporal sampling.

        Samples trajectory at fine dt (0.02s → ~0.08m at v_max=4) and evaluates:
          - soft barrier: w_obs * max(0, d_soft - d)^2  (centerline preference)
          - hard barrier: w_collision * max(0, d_hard - d)^2  (safety floor)

        d_signed > 0 in free space, < 0 inside obstacle.
        """
        if self.esdf_map is None or not self.esdf_map.valid:
            return 0.0

        xs, ys = self._sample_xy(coeffs_x, coeffs_y, durations, dt=0.05)
        ds = self.esdf_map.get_distances_batch(xs, ys)

        soft_cost = self.w_obs * np.sum(np.maximum(0.0, self.d_soft - ds) ** 2)
        hard_cost = self.w_collision * np.sum(np.maximum(0.0, self.d_hard - ds) ** 2)
        return soft_cost + hard_cost

    def total_cost(self, coeffs_x, coeffs_y, durations):
        """Compute total cost: smoothness + time + clearance + dynamics penalty."""
        N = len(durations)
        cost = 0.0

        # Smoothness cost (jerk integral)
        for i in range(N):
            cost += self.w_smooth * (
                _segment_jerk_cost(coeffs_x[i], durations[i]) +
                _segment_jerk_cost(coeffs_y[i], durations[i])
            )

        # Time cost
        cost += self.w_time * sum(durations)

        # Clearance cost (ESDF dual barrier)
        cost += self._clearance_cost(coeffs_x, coeffs_y, durations)

        # Dynamics penalty (squared hinge loss on velocity and acceleration)
        if self.w_dyn > 0:
            _, _, vmag, amag = self._sample_xy(
                coeffs_x, coeffs_y, durations, dt=0.08, with_va=True)
            v_excess = np.maximum(0.0, vmag - self.v_max)
            a_excess = np.maximum(0.0, amag - self.a_max)
            cost += self.w_dyn * (np.sum(v_excess ** 2) + np.sum(a_excess ** 2))

        return cost

    def optimize(self, waypoints_init, durations_init, boundary=None,
                 max_iter=100):
        """Optimize intermediate waypoint positions (durations fixed).

        When w_obs=0, skips optimization entirely — the L-BFGS optimizer with
        only jerk + time cost will collapse intermediate waypoints onto the
        straight line between start and end (minimum distance = minimum jerk).
        Optimization is only useful when obstacle cost pushes waypoints away
        from obstacles.

        Args:
            waypoints_init: initial waypoints [(x, y), ...]
            durations_init: initial durations [T1, ...]
            boundary: boundary conditions dict

        Returns:
            (opt_waypoints, durations, coeffs_x, coeffs_y, cost)
        """
        N = len(waypoints_init) - 1

        # Without clearance cost, optimization only harms path shape
        if (self.w_obs <= 0.0 and self.w_collision <= 0.0) or N < 2:
            cx, cy = self.compute_coefficients(waypoints_init, durations_init, boundary)
            cost = self.total_cost(cx, cy, durations_init)
            return waypoints_init, durations_init, cx, cy, cost

        # Decision variables: intermediate waypoint positions (flattened)
        # Start and end are fixed
        n_var = 2 * (N - 1)  # (N-1) intermediate points, each 2D

        def unpack(x):
            waypoints = [waypoints_init[0]]  # fixed start
            for i in range(N - 1):
                waypoints.append((x[2*i], x[2*i + 1]))
            waypoints.append(waypoints_init[-1])  # fixed end
            return waypoints

        def cost_fn(x):
            wps = unpack(x)
            cx, cy = self.compute_coefficients(wps, durations_init, boundary)
            cost = self.total_cost(cx, cy, durations_init)
            if self.w_ref > 0.0:
                dx = x - x0
                cost += self.w_ref * float(np.dot(dx, dx))
            return cost

        x0 = np.array([coord for wp in waypoints_init[1:-1] for coord in wp])
        bounds = None
        if self.waypoint_bound_m > 0.0:
            b = self.waypoint_bound_m
            bounds = [(v - b, v + b) for v in x0]

        # ftol=1e-2 stops at the diminishing-returns knee (~10-15 iters) instead
        # of grinding to nit=75 for <8% extra cost reduction. Safety is enforced
        # separately by the post-optimization check_clearance() hard floor.
        result = minimize(cost_fn, x0, method='L-BFGS-B',
                          bounds=bounds,
                          options={'maxiter': max_iter, 'ftol': 1e-2,
                                   'gtol': 1e-3, 'disp': False})

        opt_waypoints = unpack(result.x)
        cx, cy = self.compute_coefficients(opt_waypoints, durations_init, boundary)
        cost = self.total_cost(cx, cy, durations_init)

        return opt_waypoints, durations_init, cx, cy, cost

    def check_clearance(self, coeffs_x, coeffs_y, durations):
        """Check minimum clearance distance along trajectory.

        Returns: (min_d, min_d_x, min_d_y) — the minimum signed distance
        and the position where it occurs.
        """
        if self.esdf_map is None or not self.esdf_map.valid:
            return float('inf'), 0.0, 0.0

        xs, ys = self._sample_xy(coeffs_x, coeffs_y, durations, dt=0.03)
        ds = self.esdf_map.get_distances_batch(xs, ys)

        idx = np.argmin(ds)
        return float(ds[idx]), float(xs[idx]), float(ys[idx])

    def sample_trajectory(self, coeffs_x, coeffs_y, durations, dt=0.05):
        """Sample trajectory at fixed time intervals.

        Returns: list of (t, x, y, vx, vy, ax, ay, yaw)
        """
        total_T = sum(durations)
        samples = []
        t = 0.0
        while t <= total_T + 1e-6:
            x, y, vx, vy, ax, ay, jx, jy = self.evaluate(coeffs_x, coeffs_y, durations, t)
            yaw = math.atan2(vy, vx) if (abs(vx) > 1e-6 or abs(vy) > 1e-6) else 0.0
            samples.append((t, x, y, vx, vy, ax, ay, yaw))
            t += dt
        return samples

    def check_dynamics(self, coeffs_x, coeffs_y, durations):
        """Check if velocity/acceleration exceed limits.

        Returns: (max_v, max_a, v_violations, a_violations)
        """
        samples = self.sample_trajectory(coeffs_x, coeffs_y, durations, dt=0.02)
        max_v = 0.0
        max_a = 0.0
        v_violations = 0
        a_violations = 0
        for _, _, _, vx, vy, ax, ay, _ in samples:
            v = math.hypot(vx, vy)
            a = math.hypot(ax, ay)
            max_v = max(max_v, v)
            max_a = max(max_a, a)
            if v > self.v_max:
                v_violations += 1
            if a > self.a_max:
                a_violations += 1
        return max_v, max_a, v_violations, a_violations
