# so101-kinematics: theory

The theory behind M1: what each method does and why it works (or where it doesn't). Read it with the code.
[DESIGN.md](DESIGN.md) explains the design choices, [README](../README.md) has the numbers, and
[DECISIONS.md](../DECISIONS.md) records when and why each decision was made.

Notation: $q \in \mathbb{R}^5$ are the arm joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll; the gripper is not part of the kinematics). $x$ is the task: what we ask the tool to do.
Lengths are in metres, angles in radians, unless stated otherwise.

Contents:

1. [Forward kinematics](#1-forward-kinematics)
2. [The Jacobian](#2-the-jacobian)
3. [Choosing the task space](#3-choosing-the-task-space)
4. [IK as nonlinear least squares](#4-ik-as-nonlinear-least-squares)
5. [Damping: singularities and step size](#5-damping-singularities-and-step-size)
6. [Weighting position against angle](#6-weighting-position-against-angle)
7. [Joint limits: bounded least squares](#7-joint-limits-bounded-least-squares)
8. [Unreachable targets, local minima and seeding](#8-unreachable-targets-local-minima-and-seeding)
9. [The reference solvers](#9-the-reference-solvers)
10. [From a shape to a joint trajectory](#10-from-a-shape-to-a-joint-trajectory)
11. [The safety filter as a control-theory object](#11-the-safety-filter-as-a-control-theory-object)
12. [How a position servo follows a moving command](#12-how-a-position-servo-follows-a-moving-command)
13. [What the measurements can and cannot see](#13-what-the-measurements-can-and-cannot-see)
14. [Benchmark method](#14-benchmark-method)
15. [References](#15-references)

---

## 1. Forward kinematics

**Problem.** Given the joint angles $q$, where is the tool?

A serial arm is a chain of rigid links. Each joint adds one transform: a fixed offset from the URDF (where the
joint sits on the previous link) times a rotation about the joint axis by $q_i$. Each transform is a
homogeneous matrix in $SE(3)$, a $4\times4$ matrix holding a rotation $R$ and a translation $p$:

$$
T_{\text{tool}}(q) = T_{0}\, \mathrm{Rot}(a_1, q_1)\, T_{1}\, \mathrm{Rot}(a_2, q_2) \cdots T_{5}\, \mathrm{Rot}(a_5, q_5)\, T_{\text{tool}}
=\begin{bmatrix} R(q) & p(q) \\ 0 & 1 \end{bmatrix}
$$

This is the URDF way of writing the chain: each joint has an origin and an axis. Denavit-Hartenberg parameters
describe the same chain with four numbers per joint; Lynch and Park's product of exponentials describes it
with twists. All three give the same $T(q)$.

- **In the code:** `PinArm.fk` (Pinocchio, C++) and `LegacyArm` (arm_control 2023: sympy-generated
  expressions evaluated in Python).
- **S1 check:** both implementations walk the same URDF, so their results should agree to floating-point
  precision. They do: 3.6e-16 m over 1000 random configurations. A difference of 1e-6 m would have meant a
  bug, not rounding.
- **What FK does not tell you:** whether the URDF matches the physical arm. FK is exact for the model; the
  model's link lengths and joint zeros are what M2 calibrates (section 13).

## 2. The Jacobian

**Problem.** If the joints move a little, how does the tool move?

The position Jacobian $J_p(q) = \partial p / \partial q \in \mathbb{R}^{3\times5}$ is the first-order answer:

$$
\delta p \approx J_p(q)\, \delta q
$$

For a revolute joint $i$ with unit axis $a_i$ (in the world frame) through the point $o_i$, column $i$ is

$$
J_{p,i} = a_i \times (p - o_i)
$$

that is, the velocity of the tool point when only joint $i$ turns at 1 rad/s. Pinocchio computes this
analytically in the same pass as FK; `PinArm.pos_jac` returns both from one pass, which matters because IK
calls it every iteration.

- **In the code:** `tests/test_models.py` checks the Pinocchio Jacobian against finite differences
  $\big(p(q + h e_i) - p(q - h e_i)\big) / 2h$. That catches frame mix-ups (world vs local), the most common
  Jacobian bug.
- **Singularity:** where $J$ loses rank, some tool direction cannot be produced by any small joint motion.
  For the SO-101 position part, this happens when the arm is stretched straight (elbow at 0) or when the
  wrist is right above the base axis. Section 5 explains why this matters for IK.

## 3. Choosing the task space

**Problem.** The SO-101 has five arm joints. A full tool pose (3 position + 3 orientation) is six numbers.
What should IK solve for?

### 3.1 Counting constraints

Five joints can satisfy at most five independent equations at a time. With six (a full pose), almost every
target has no exact solution; the solver would always be compromising. With fewer than five, there is a
**null space**: a family of joint motions that leave the task unchanged, which a solver can use for
secondary goals (such as staying away from limits).

We chose exactly five: position $x, y, z$ plus two angles of the tool's approach axis (the tool frame's $z$
axis):

- **pitch**: its elevation in the arm's vertical plane (0 = horizontal pointing outwards, $-\pi/2$ =
  straight down);
- **roll**: rotation of the tool about that axis.

The sixth number, yaw, is not free: shoulder_pan decides which way the arm's vertical plane faces, and the
tool always lies in that plane. That is why a shape drawn off the centre line is not seen head-on (about 9
degrees at the corners of the square).

Consequence: **5 equations, 5 unknowns, no null space.** Every joint is fully used by the task. This one fact
drives the joint-limit design in section 7: there is no spare freedom to push joints away from limits, so the
limits have to enter the step itself.

### 3.2 Why pitch and roll are linear in the joints

shoulder_lift, elbow_flex and wrist_flex all turn about parallel horizontal axes. Viewed in the arm's
vertical plane, they form a **planar chain**, and in a planar chain the orientation of the last link is the
sum of the joint angles. So

$$
\text{pitch}(q) = -(q_2 + q_3 + q_4), \qquad \text{roll}(q) = q_5 + c
$$

where the sign comes from the URDF's axis directions and $c = -2.789°$ comes from a small tilt of the
wrist_roll axis in the URDF. Both were checked against the URDF's actual tool frame to 1e-11 rad
(`tests/test_ik.py`).

Two consequences:

1. **Their Jacobian rows are constant:** $[0, -1, -1, -1, 0]$ and $[0, 0, 0, 0, 1]$. No computation per
   iteration, no approximation.
2. **No orientation singularity.** A general way to express "tool pointing down" is with Euler angles. They
   have a singularity (gimbal lock) at exactly the configuration we care about: straight down, where two of
   the three angles turn about the same axis and one degree of freedom disappears from the representation.
   The joint-sum definition has no such point; straight down is just pitch $= -\pi/2$.

Angle errors are wrapped to $(-\pi, \pi]$ with $\operatorname{atan2}(\sin e, \cos e)$, so an error of
$359°$ is treated as $-1°$.

## 4. IK as nonlinear least squares

**Problem.** Find $q$ with $f(q) = x^*$, where $f$ is the (nonlinear) map from joints to task and $x^*$ the
target.

There is no closed form for general arms (there is one for this arm with some effort; see section 8). The
standard approach: write the residual $e(q) = x^* - f(q)$ and minimise its squared norm,

$$
\min_q \; \tfrac12 \| e(q) \|^2 .
$$

Linearise around the current $q$: $f(q + \delta q) \approx f(q) + J\,\delta q$. The best step for the
linearised problem is

$$
\min_{\delta q} \| J\,\delta q - e \|^2 .
$$

Solving that and repeating is the **Gauss-Newton** method. When $J$ is square and invertible, the step is
$\delta q = J^{-1} e$ (Newton-Raphson on $f(q) = x^*$); in general, $\delta q = J^{+} e$ with the
pseudo-inverse $J^+$. Near the solution it converges quadratically: the number of correct digits roughly
doubles per iteration. That is why a warm start (a solution from the previous point on the path) takes 2-3
iterations.

Other classic update rules, used in arm_control (2023) and compared in S1:

| Method | Step | Behaviour |
|---|---|---|
| Jacobian transpose | $\delta q = \alpha J^T e$ | Gradient descent on $\frac12\|e\|^2$. Cheap, no inversion, but slow (linear convergence, step size to tune): 16/200 in S1. |
| Pseudo-inverse | $\delta q = J^+ e$ | Gauss-Newton. Fast near a solution, unstable near singularities. |
| Damped least squares | $\delta q = (J^TJ + \lambda^2 I)^{-1} J^T e$ | Gauss-Newton with damping (section 5). What Pinocchio's example and our solver use. |

## 5. Damping: singularities and step size

**Problem.** Near a singularity, $J^+$ blows up: a tiny error in a nearly unreachable direction asks for a
huge joint motion.

The singular value decomposition $J = U \Sigma V^T$ shows why. The pseudo-inverse step along the $i$-th
singular direction is $\frac{1}{\sigma_i} (u_i^T e)$. As $\sigma_i \to 0$, that goes to infinity.

**Damped least squares** (Nakamura and Hanafusa 1986; Wampler 1986), the same idea as Levenberg-Marquardt in
optimisation, adds a penalty on the step size:

$$
\min_{\delta q} \| J\,\delta q - e \|^2 + \lambda^2 \|\delta q\|^2
\quad\Rightarrow\quad
\delta q = (J^TJ + \lambda^2 I)^{-1} J^T e = J^T (J J^T + \lambda^2 I)^{-1} e .
$$

(The two forms are equal; the first inverts a joint-sized matrix, the second a task-sized one. Here both are
$5\times5$.) Along singular direction $i$ the step becomes

$$
\frac{\sigma_i}{\sigma_i^2 + \lambda^2}\,(u_i^T e) ,
$$

so directions with $\sigma_i \gg \lambda$ are almost unaffected, and those with $\sigma_i \ll \lambda$ are
cut off smoothly instead of exploding. The factor $\sigma_i^2 / (\sigma_i^2 + \lambda^2)$ is how much of the
undamped step survives.

**Why our damping had to drop from 0.01 to 0.003.** Damping is in the same units as the weighted Jacobian. With
the angle weight 0.01 (section 6), the weighted $J$ has singular values around 0.47, 0.35, 0.08 (mostly
position) and **0.010, 0.004** (mostly the two angles), checked at two typical poses. With $\lambda = 0.01$,
the surviving fractions for the angle directions are $0.01^2/(0.01^2+0.01^2) = 0.5$ and about 0.13: the angle
steps were cut to half or less, and cold-start success fell from 173 to 145 of 200. With $\lambda = 0.003$
they are 0.92 and 0.63. Rule of thumb: **$\lambda$ must stay below the smallest singular value you want to
move along at full speed**, so it must scale with the weights.

**Step cap.** In addition, each joint moves at most 0.5 rad per iteration (`max_step`). This is a simple trust
region: the linearisation is only good near the current $q$, so long steps are not trusted.

## 6. Weighting position against angle

**Problem.** The residual mixes metres and radians. $\|e\|^2$ only means something once you decide how much a
radian is worth in metres.

Weighted least squares scales each row: $e_w = W e$, $J_w = W J$, with $W = \mathrm{diag}(1, 1, 1, w, w)$
(position in metres, angles in radians). The solver then minimises

$$
\|p^* - p(q)\|^2 + w^2 \big( (\text{pitch}^* - \text{pitch})^2 + (\text{roll}^* - \text{roll})^2 \big) .
$$

When the target is reachable, all errors go to zero and $w$ barely matters. **It matters when the target
cannot be reached**: then the solver trades one error against the other, and $w$ sets the exchange rate. With
$w = 0.01$ m/rad, 1 degree of angle error costs the same as $0.01 \times 0.01745 \approx 0.17$ mm of position
error. In other words, the solver gives up about 6 degrees of pitch before 1 mm of position. That is the
"position first" compromise we decided on (DECISIONS.md): for drawing, where the pen is matters more than
exactly how it is tilted.

Measured effect (S2): for an unreachable pitch, $w = 0.05$ gave up about 17 mm of position; $w = 0.01$
gives about 0.8 mm. Smaller weights did not help further and reached fewer poses.

The alternative to weights is **strict priority** (lexicographic): solve position exactly, then use the
remaining freedom for the angle. That needs a null space (section 3.1), which a 5-for-5 task does not have.

## 7. Joint limits: bounded least squares

**Problem.** Every joint has a range $q_{\min} \le q \le q_{\max}$. The IK must not return, or pass through,
configurations outside it.

### 7.1 Limits as bounds on the step

At the current $q$, the step $\delta q$ keeps the next iterate inside the limits if and only if

$$
\ell = \max(q_{\min} - q, -s) \;\le\; \delta q \;\le\; u = \min(q_{\max} - q, s)
$$

(with $s$ the step cap). So each iteration becomes a **box-constrained least squares** problem:

$$
\min_{\delta q} \; \| J\,\delta q - e \|^2 + \lambda^2 \|\delta q\|^2
\quad \text{s.t.} \quad \ell \le \delta q \le u .
$$

This is a convex quadratic program (QP): the cost is a convex quadratic, $H = J^TJ + \lambda^2 I$ is positive
definite (thanks to $\lambda > 0$), and the constraints are a box. It has exactly one solution. This is the
same formulation used by QP-based differential IK libraries (Pink, mink, placo): they also turn joint limits
into bounds on the joint velocity or step at each iteration.

### 7.2 Optimality: the KKT conditions

For a box-constrained convex QP, $\delta q^\star$ is optimal if and only if, with gradient
$g = H\,\delta q^\star - J^T e$ (half the cost gradient), for each joint $i$:

| Where $\delta q_i^\star$ is | Condition on $g_i$ | Meaning |
|---|---|---|
| strictly inside $(\ell_i, u_i)$ | $g_i = 0$ | no gain from moving this joint either way |
| on the lower bound $\ell_i$ | $g_i \ge 0$ | the cost would only drop by going further down, which is not allowed |
| on the upper bound $u_i$ | $g_i \le 0$ | likewise, upwards |

These are the Karush-Kuhn-Tucker (KKT) conditions specialised to a box.

### 7.3 The active-set method (`ik.bounded_dls_step`)

An **active set** is a guess of which joints sit on a bound. Given the guess, the problem is an unconstrained
least squares in the remaining (free) joints, solved with one linear system. The method updates the guess
until the KKT table above holds:

1. **Fast path.** Solve with no joint fixed. If the step is inside the box, it satisfies KKT (all $g_i=0$):
   done. Away from the limits this is every iteration, so the cost equals plain DLS.
2. **Add.** Joints whose step left the box are fixed on the bound they crossed. Re-solve for the free ones,
   with the fixed joints' contribution moved to the right-hand side:
   $H_{FF}\,\delta q_F = J_F^T e - H_{FB}\,\delta q_B$. The free joints now **compensate** for the fixed ones.
3. **Release.** If a fixed joint's gradient has the wrong sign (the cost would drop by moving it back inside),
   free it and re-solve.

This is the primal active-set method for convex QPs (Nocedal and Wright, ch. 16), close to Lawson and Hanson's
NNLS and to BVLS (Stark and Parker 1995). With five variables it ends in a few passes; `tests/test_ik.py`
checks on 300 random bounded problems that it reaches a cost no worse than scipy's BVLS.

### 7.4 Why plain clamping is different

The naive method computes the unconstrained step and then clips: $q \leftarrow \operatorname{clip}(q +
\delta q, q_{\min}, q_{\max})$. Geometrically, it **projects** the full step onto the box. The other joints
were computed assuming the clipped joint would move, and they do not compensate. The clipped step can even
point uphill.

**What the experiments showed (honestly):** in practice this did not make clamping worse at finding solutions
on this arm. Over many iterations, clamping often slides along the bound to a solution anyway, and from a cold
start it found slightly more (163 vs 159 of 200; 273 vs 244 of 500 near the limits). The bounded step's
advantage is a property of each step (it is the best admissible step, and when it fails it stops at a point
that satisfies KKT on the bounds, which can be diagnosed), not a higher success rate.

### 7.5 What industry does, and why most of it does not apply here

| Approach | Idea | Fits a 5-for-5 task? |
|---|---|---|
| Clip after the step | Project onto the box | Yes; our baseline |
| Null-space gradient projection (Liégeois 1977) | Add $(I - J^+J)\nabla h(q)$ that pushes joints away from limits without moving the tool | No: needs a null space |
| Saturation in the null space, SNS (Flacco, De Luca, Khatib) | Saturate a joint at its limit and re-solve the task with the others, using redundancy | Needs redundancy to keep the task exact |
| QP-based differential IK (Pink, mink, placo, TSID) | Each step a QP with joint bounds as constraints | Yes; our method is the same idea with a hand-written solver |
| Full nonlinear optimisation (SLSQP, IPOPT, TRAC-IK's SQP half) | Joint limits as constraints of the whole problem | Yes; our SLSQP reference |

## 8. Unreachable targets, local minima and seeding

**Unreachable targets.** If no $q$ within the limits reaches $x^*$, there is no zero of $e$. Least squares
still converges, to the point that minimises the weighted error: a **compromise**. The solver returns its
residuals and an `at_limit` flag, and the caller decides whether the compromise is acceptable. A compromise
is acceptable for drawing near the edge of the workspace; for a grasp it may not be.

**Local minima.** $\|e(q)\|^2$ is not convex in $q$: the arm geometry is made of sines and cosines, and the
box creates corners. Gauss-Newton type methods only find a local minimum. A typical failure: a joint runs
into its limit early, the remaining joints cannot finish the job from there, and the solver stops at a point
that is optimal **given** that the joint is on its bound (KKT holds) but is not a solution. S2 found that most
cold-start failures are of this kind.

The remedy is not a better step but a **better starting point**:

- **Warm start (continuation).** Along a path, the previous point's solution is close to the next one, so
  Gauss-Newton starts in the quadratic-convergence region: 200/200 warm vs 159/200 cold. This is what
  `drawing.track` does.
- **Several seeds.** Try a few fixed starting configurations and keep the best. Used only as a fallback, and
  then the plan is refused, because another seed can land on a different **branch** (elbow up vs elbow down);
  switching branch between two neighbouring path points would make the arm swing through a large motion.
- **Structured or analytical seed (not done).** Roll fixes $q_5$ exactly, the pitch target fixes
  $q_2 + q_3 + q_4$, and shoulder_pan follows from $\operatorname{atan2}(y, x)$. What remains is a
  two-link planar problem for shoulder_lift and elbow, which has a closed form (law of cosines, two
  branches). That would give an exact or near-exact start.

## 9. The reference solvers

Every claim about our solver is made against references on the same targets.

- **SLSQP** (Sequential Least SQuares Programming, Kraft 1988; `scipy.optimize.minimize`). A sequential
  quadratic programming method: at each iteration it builds a quadratic model of the cost with a
  quasi-Newton (BFGS) estimate of the Hessian, linearises the constraints, solves that QP, and does a line
  search. It treats the joint limits as constraints of the whole problem, not of each local step, and its
  line search lets it escape some shallow local minima. That is why it finds the most solutions from a cold
  start (174/200), at about 3 ms per solve in our setup.
- **Pink** (Caron et al.). Differential IK on Pinocchio: each step is a QP over joint velocities, with tasks
  as weighted costs and configuration limits as inequality constraints, solved by a generic QP solver
  (quadprog here). Same model, same Jacobian, same idea as our step; the difference is the generic solver and
  its own damping/limit handling. It is the closest external comparison: 159/200 cold, like ours.
- **Clamp** and **BVLS**: the same solver loop with the step replaced by "clip after" and by scipy's bounded
  least squares, isolating the effect of the step from everything else.

## 10. From a shape to a joint trajectory

**Problem.** Turn "a 7 cm circle on the plane $x = 20$ cm, at 5 cm/s" into one joint command every 1/30 s.

1. **Sample the shape** every 2 mm in task space, and solve IK at each point with a warm start and tight
   tolerances (0.2 mm, 0.1 deg). Tight, because the tolerance is the error you accept before the arm moves.
2. **Check the plan.** Every point must converge, from the warm start (no fallback seed: no branch switch),
   and no joint may move more than 5 degrees between neighbouring points. Large joint motion for a small tool
   motion is the signature of a singularity or a branch switch.
3. **Resample in time by arc length.** For constant tool speed $v$, the $k$-th command must be at arc length
   $s_k = v\,k/f$ (with $f = 30$ Hz). Arc length is accumulated along the tool path (and closed back to the
   start for a closed shape), and the joint vectors are interpolated linearly between path points at those
   arc lengths. Between two points 2 mm apart, linear interpolation in joint space is very close to linear
   in task space (the error is second order in the step).
4. **Ramp** to the start pose with a slow joint-space motion (20 deg/s).

**Workspace margin.** The reachable region of the plane at pitch 0 is limited at the top by shoulder_lift.
The shape is sized so every joint stays at least 10 degrees from its limit, twice the safety layer's margin.
A plan that touches the safety margin would be modified by the filter (section 11), so the arm would not draw
what was planned.

## 11. The safety filter as a control-theory object

so101-core's `SafetyLayer.filter` applies two operations to every command:

1. **Saturation** (position limit): clip to $[q_{\min} + m, q_{\max} - m]$ with margin $m = 5°$. This is a
   projection onto a box.
2. **Rate limiting** (step cap): $|q_{\text{cmd},k} - q_{\text{cmd},k-1}| \le 2°$ per tick, which at 30 Hz is a
   velocity limit of 60 deg/s per joint.

Both are **nonlinear** operations: if either is active, the arm no longer follows the plan, and the shape
comes out distorted. That is why the plan is checked offline first (preflight: 0 clipped and 0 capped
commands), and why the shapes are sized with margin. The filter is a safety net, not part of the plan.

**Stall detection** is a following-error monitor, the same idea as the "following error" alarm in industrial
servo drives and CNC controllers: if a joint stays more than a threshold away from its command for longer
than a time window (5 deg for 0.3 s while drawing), something is blocking it. The arm then holds its present
position (command = measured), so it stops pushing. The threshold must sit above normal following error and
below what could hurt. Both the size and the time window count: in S3 the elbow's following error peaked at
5.2 deg at 5 cm/s (section 12: lag grows with speed), above the 5 deg threshold, but only for moments, far
shorter than 0.3 s. The steady part (sag) was at most 1.9 deg. Faster drawing would need a larger threshold
or a compensated command.

## 12. How a position servo follows a moving command

The SO-101 servos run their own position loop internally: they receive a goal position and drive the motor
from the error between goal and encoder reading, essentially a proportional controller. Three effects follow
from that, and S3 measured each of them.

**1. Steady-state offset under load (gravity sag).** A proportional controller produces torque only from an
error: $\tau = K_p (q_{\text{cmd}} - q)$. To hold a link against gravity torque $\tau_g$, the joint must sit
at $q = q_{\text{cmd}} - \tau_g / K_p$. The offset is proportional to the load and inversely to the gain.
Measured: the elbow has the largest offset, 0.9 deg in the fit over the drawing runs and up to 1.9 deg when
holding the start pose; the other joints stay within 0.5 deg in the fit.
An integral term would remove it; these servos do not rely on one. Feed-forward (command
$q^* + \tau_g(q^*)/K_p$) or a higher $K_p$ would.

**2. Lag proportional to speed (delay).** A first-order system with time constant $T$ following a ramp
$q_{\text{cmd}}(t) = v t$ settles at a constant tracking error $v\,T$: it runs behind by a fixed time. More
generally, any loop with a delay of $T$ seconds follows a constant-speed command at a distance $v\,T$ behind.
Measured: about 100 ms for all four joints, which includes the communication time and about one tick (33 ms)
from reading the position right after each command. At 5 cm/s of tool speed, 100 ms is 5 mm of lag along
the path, which is the main part of the S3 error.

**3. Deadband (speed-independent lag).** Friction and backlash in the gearbox, plus any error band inside
which the servo does not react, produce a lag that does not grow with speed. It changes sign with the
direction of motion: Coulomb friction acts against the motion. Measured: 0.1-0.4 deg.

**The joint model** combines the three and is linear in its unknowns, so ordinary least squares fits it:

$$
q_{\text{meas}} - q_{\text{cmd}} = \text{offset} - \operatorname{sign}(v)\,\big(\text{deadband} + T\,|v|\big)
$$

with $v$ the commanded joint velocity (only moving ticks are used, where $\operatorname{sign}(v)$ is defined).
The columns of the regression are $[1, -\operatorname{sign}(v), -v]$ (since $\operatorname{sign}(v)|v| = v$).
The fit explains 87-95% of the variance ($R^2$), so these three effects are most of what happens.

How this connects to the task error: through the Jacobian, $\delta p \approx J_p\,\delta q$. A joint lag
$\delta q = -\dot q\,T$ becomes a tool lag $-J_p \dot q\, T = -\dot p\, T$: the tool runs behind its command
by the same $T$, along the path. That is why the delay can also be estimated directly in task space (section
13).

## 13. What the measurements can and cannot see

**Analysis of a run** (`benchmarks/s3_analyze.py`). Both the commanded and the measured joints are pushed
through FK, giving $p_{\text{cmd}}(t)$ and $p_{\text{meas}}(t)$, and the error $e(t) = p_{\text{meas}} -
p_{\text{cmd}}$ is decomposed:

- **Offset:** the mean of $e$, the constant part (mostly sag).
- **Shape error:** what remains, split into the component **along** the direction of motion (lag) and the
  component **across** it (the shape itself deforms).
- **Delay:** the shift $k$ (in ticks) that minimises the mean distance between $p_{\text{meas}}(t + k)$ and
  $p_{\text{cmd}}(t)$ after removing the offset. This is a brute-force time-delay estimate, the same idea as
  finding the peak of a cross-correlation.

**What this cannot see: model errors.** Commanded and measured positions both pass through the same FK. If
the URDF's link lengths or the encoders' zero positions are wrong, both are wrong the same way, and the
error cancels out. So the S3 numbers say how well the **servos follow the commands**, not where the **tool
really is**. Only an external measurement can see the model's error. The ruler check was one: 10 cm on the
table measured 8.66 cm by FK. That is a **systematic kinematic error**, in the model or the calibration
(most likely a joint zero offset, which changes the effective geometry). Finding which parameter is wrong is
**kinematic calibration**: measure the tool externally (camera, ruler, touch points) at many poses, then fit
the model parameters by least squares on the difference. It needs more than two points, since each point
gives three equations and there are several unknowns. That is M2.

## 14. Benchmark method

- **Fixed random seeds** for the target sets and initial guesses: every number can be regenerated and
  compared between commits.
- **Reachable targets by construction:** sample random $q$ within the limits, take its FK as the target
  ($x^* = f(q_{\text{true}})$). A solution exists, so a failure is the solver's fault, not the target's. The
  near-limit set puts $q_{\text{true}}$ 0.5-5 degrees from a limit to stress the limit handling.
- **Paired comparison:** all solvers get the same targets and the same starts, so per-target outcomes can be
  compared (which targets one solver gets and another misses), not only totals.
- **One definition of success** for all: position under 2 mm and both angles under 0.5 deg, and inside the
  limits.
- **Starts that mean something:** cold (zero pose, a stand-in for "no idea"), far (0.3 rad away) and warm
  (0.05 rad, the distance between two points of a path).
- **Simulation before hardware:** the full planned run is replayed in the MuJoCo twin with every tick through
  the same safety filter, before the real arm moves.

## 15. References

Textbooks:

- B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer,
  2009. Kinematics, the Jacobian, singularities, inverse kinematics algorithms.
- K. M. Lynch, F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press,
  2017. Product of exponentials, numerical IK.
- J. Nocedal, S. J. Wright, *Numerical Optimization*, 2nd ed., Springer, 2006. Gauss-Newton,
  Levenberg-Marquardt, KKT conditions, active-set methods for QPs (ch. 16), SQP (ch. 18).
- C. L. Lawson, R. J. Hanson, *Solving Least Squares Problems*, Prentice-Hall, 1974. NNLS, the ancestor of
  bounded active-set least squares.

Papers and notes:

- S. R. Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least
  Squares methods", technical note, UC San Diego, 2004. A short, clear overview of section 4-5.
- Y. Nakamura, H. Hanafusa, "Inverse kinematic solutions with singularity robustness for robot manipulator
  control", *J. Dynamic Systems, Measurement, and Control*, 1986.
- C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped
  least-squares methods", *IEEE Trans. Systems, Man, and Cybernetics*, 1986.
- A. Liégeois, "Automatic supervisory control of the configuration and behavior of multibody mechanisms",
  *IEEE Trans. Systems, Man, and Cybernetics*, 1977. Null-space gradient projection.
- F. Flacco, A. De Luca, O. Khatib, "Control of redundant robots under hard joint constraints: Saturation in
  the null space", *IEEE Trans. Robotics*, 2015.
- P. B. Stark, R. L. Parker, "Bounded-variable least-squares: an algorithm and applications",
  *Computational Statistics*, 1995. The BVLS algorithm behind `method="bvls"`.
- D. Kraft, "A software package for sequential quadratic programming", DFVLR, 1988. SLSQP.

Software:

- Pinocchio (Carpentier et al.): rigid-body kinematics and dynamics, https://github.com/stack-of-tasks/pinocchio
- Pink (S. Caron et al.): QP-based differential IK on Pinocchio, https://github.com/stephane-caron/pink
- MuJoCo: the simulator behind so101-core's digital twin, https://mujoco.org
