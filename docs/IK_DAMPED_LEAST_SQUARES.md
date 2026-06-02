# Why Damped Least-Squares Resolves IK Singularities

Background note for the IK solver (`app/src/kinematics/inverse_kinematics_poe.c`, `jacobian_damped_pinv` in `kinematics_math.c`). Explains why `JJ^T + λ²I` is invertible even when `JJ^T` is singular, and the accuracy/robustness trade-off behind choosing `lambda = 0.01`.

## 1. What we actually solve each iteration

Newton-Raphson IK linearizes the problem. At the current joint angles $\theta$, the body Jacobian $J$ relates a small joint change $\Delta\theta$ to the resulting end-effector twist:

$$
J\,\Delta\theta = V_b
$$

where $V_b$ is the twist error (how far the current pose is from the target). We want the $\Delta\theta$ that achieves it. For a square/wide $J$ the **minimum-norm least-squares** solution is the right pseudo-inverse:

$$
\Delta\theta = J^{+}V_b = J^{T}\,(JJ^{T})^{-1}\,V_b
$$

That $(JJ^{T})^{-1}$ is exactly what `jacobian_damped_pinv` computes (with $\lambda = 0$ it is this plain form).

## 2. Why `JJ^T` dies at a singularity

The clean way to see everything is the **SVD**: $J = U\Sigma V^{T}$, where $\Sigma = \mathrm{diag}(\sigma_1,\dots,\sigma_6)$ are the **singular values** — they measure how much end-effector motion you get per unit joint motion in 6 independent directions.

A **kinematic singularity** is precisely *rank loss*: the arm can no longer move the end-effector in some direction (e.g. a fully-straight elbow can't move further along its own axis). Mathematically one or more $\sigma_i \to 0$.

Now look at $JJ^{T}$:

$$
JJ^{T} = U\Sigma\Sigma^{T}U^{T} = U\,\mathrm{diag}(\sigma_i^2)\,U^{T}
$$

Its eigenvalues are $\sigma_i^2$. So when $\sigma_i \to 0$:

- $\det(JJ^{T}) = \prod_i \sigma_i^2 \to 0$ → **not invertible**. Your `invert_6x6` fails → the solve aborts. (That was the literal crash.)
- Even just *near* a singularity, $(JJ^{T})^{-1}$ has eigenvalue $1/\sigma_i^2 \to \infty$, so the step explodes:

$$
\Delta\theta = J^{+}V_b = V\,\mathrm{diag}\!\Big(\tfrac{1}{\sigma_i}\Big)U^{T}V_b
$$

A tiny twist component in the near-singular direction gets multiplied by $1/\sigma_i$ (huge) → a giant joint jump → the iterate leaps to a wild config → next iteration is garbage → divergence. **That is why far targets "failed easily."**

## 3. The fix: shift every eigenvalue up

Damped least-squares inverts $JJ^{T} + \lambda^2 I$ instead. Why is *that* always invertible?

$JJ^{T}$ is **symmetric positive semi-definite** — all its eigenvalues are $\sigma_i^2 \ge 0$ (never negative, but possibly zero). Adding $\lambda^2 I$ adds $\lambda^2$ to **every** eigenvalue:

$$
JJ^{T} + \lambda^2 I = U\,\mathrm{diag}(\sigma_i^2 + \lambda^2)\,U^{T}
$$

Now every eigenvalue is $\sigma_i^2 + \lambda^2 \ge \lambda^2 > 0$ — **strictly positive**. So:

$$
\det(JJ^{T}+\lambda^2 I) = \prod_i (\sigma_i^2 + \lambda^2) > 0
$$

The matrix is **positive definite**, hence invertible — *even when some $\sigma_i = 0$ exactly*. That is the whole answer: the $\lambda^2 I$ term floors the smallest eigenvalue at $\lambda^2$, so the matrix can never be singular.

## 4. What it does to the step (the bounded gain)

Plug the damped inverse back through the SVD:

$$
J^{T}(JJ^{T}+\lambda^2 I)^{-1} = V\,\mathrm{diag}\!\Big(\underbrace{\tfrac{\sigma_i}{\sigma_i^2 + \lambda^2}}_{\text{damped gain}}\Big)U^{T}
$$

Compare the per-direction gain to the undamped $1/\sigma_i$:

| direction | $\sigma_i$ | undamped $1/\sigma_i$ | damped $\sigma_i/(\sigma_i^2+\lambda^2)$ |
|---|---|---|---|
| well-conditioned | 0.5 | 2.0 | 0.4998 ≈ **2.0** (unchanged) |
| near singular | 0.001 | **1000** 💥 | **≈ 9.9** ✅ |

(using $\lambda = 0.01$.)

Two things happen automatically:

- **Large $\sigma$** (healthy directions): $\lambda^2 \ll \sigma^2$, so the gain $\approx 1/\sigma$ — basically untouched, full accuracy.
- **Small $\sigma$** (singular directions): the gain $\sigma/(\sigma^2+\lambda^2)$ is *suppressed toward 0* instead of exploding. Its maximum over all $\sigma$ is at $\sigma = \lambda$, giving exactly $\tfrac{1}{2\lambda}$ — a hard ceiling on the step gain. With $\lambda = 0.01$ the worst-case gain is 50 instead of $\infty$.

So the singular direction, which used to produce an infinite kick, now produces a small, bounded nudge.

## 5. What you are really minimizing (the honest trade-off)

Undamped least-squares solves $\min \lVert J\Delta\theta - V_b\rVert^2$ — "hit the target exactly, whatever joint speeds it takes." DLS instead solves the **regularized** problem (a.k.a. Tikhonov regularization / ridge regression):

$$
\min_{\Delta\theta}\;\;\lVert J\,\Delta\theta - V_b\rVert^2 \;+\; \lambda^2\,\lVert\Delta\theta\rVert^2
$$

The new $\lambda^2\lVert\Delta\theta\rVert^2$ term penalizes large joint motions. So DLS makes a deliberate bargain: **accept a little tracking error in exchange for a bounded, stable step.** Near a singularity the arm *physically can't* follow the commanded direction anyway, so giving up exact tracking there costs almost nothing and buys stability.

## 6. Why λ matters and why we picked 0.01

That same trade-off is why $\lambda = 0.05$ was *too much*. The penalty $\lambda^2\lVert\Delta\theta\rVert^2$ introduces a small steady-state tracking error of order $\lambda^2$ even at good configs — and our convergence tolerance is very tight ($e_v = 10^{-4}\,\text{m}$). At $\lambda = 0.05$ the residual on `ik_ground_truth` vector 4 could not shrink below the threshold → `NO_CONVERGENCE`. At $\lambda = 0.01$ the damping is strong enough to keep $JJ^{T}+\lambda^2 I$ well-conditioned (floor $10^{-4}$, gain ceiling 50) yet small enough that healthy directions still converge to $10^{-4}$. It is the classic **robustness ↔ accuracy** knob, and 0.01 sits in the sweet spot.

The fully principled version is *adaptive* damping — large $\lambda$ when the error is big/far, shrinking $\lambda \to 0$ as you home in — so you get robustness far away *and* exact convergence near the solution. We did not need it once the multi-seed retry handled the remaining basin issues, but it is the textbook next step if you ever want both.

## One-line takeaway

$JJ^{T}$ loses invertibility because a singular value hits zero; $+\lambda^2 I$ lifts every eigenvalue to at least $\lambda^2$, guaranteeing invertibility, and in SVD terms it swaps the explosive $1/\sigma$ gain for the bounded $\sigma/(\sigma^2+\lambda^2)$ — trading a touch of accuracy for a stable, finite step.

## Related: seed restarts (the complementary fix)

Damping cures the *per-iteration* singularity, but IK is still a **local** solver — it only finds a solution whose basin of attraction contains the seed. A single seed (the current servo config) can therefore miss a perfectly reachable target. The sharpest example is the near-singular all-zero "home" config: seeded from a far pose, the undamped *and* damped solve can both wander off and return `NO_CONVERGENCE`, even though `theta = 0` is the exact answer.

`cartesian_move` handles this by retrying IK across a small seed list — current servo config first, then zero/home, then two elbow-bent fallbacks — stopping at the first seed that returns `OK` or `OUT_OF_LIMITS`. On hardware the home target now solves from the zero seed in 0 iterations once the current-config seed misses it.

The two fixes are complementary and address different failure modes:

| Fix | Failure mode it removes | Mechanism |
|---|---|---|
| Damping (`lambda`) | $JJ^{T}$ singular → solve aborts mid-iteration | $+\lambda^2 I$ keeps every step invertible and bounded |
| Seed restarts | reachable target outside the seed's basin of attraction | give the local solver several basins to try |

Damping keeps each individual solve numerically alive through singularities; seed restarts give the local solver more than one starting basin so a reachable-but-missed pose still gets found.
