/*
 * OctroBot - IK Ground-Truth Cross-Validation Tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Each vector holds a seed, a target pose T_target produced by FK on a known
 * config, and whether modern_robotics.IKinBody converged from that seed.
 * For every vector where the reference converged, the C solver must converge
 * too and FK(out) must match T_target (poses, since IK solutions are not
 * unique).
 *
 * Note: theta_ref in each vector is the modern_robotics IKinBody reference
 * solution, retained in the header for human inspection and the Python
 * cross-validator.  It is intentionally NOT compared here because IK
 * solutions are non-unique; the pose round-trip FK(IK(T)) == T is the
 * correctness criterion.
 *
 * Regenerate vectors:
 *   cd validation && python gen_test_vectors.py
 * Then: make test-native
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "robot_geometry.h"
#include "forward_kinematics_poe.h"
#include "inverse_kinematics_poe.h"
#include "ik_test_vectors.h"
#include <math.h>

/* Solver converges to eomg=1e-3, ev=1e-4; float32 FK round-trip adds rounding. 5e-3 leaves margin. */
#define IK_GT_POSE_TOL 5e-3f

/* ======================================================================== */
/* Fixture                                                                  */
/* ======================================================================== */

struct ik_ground_truth_fixture {
	poe_robot_model_t model;
};

static void *ik_gt_setup(void)
{
	static struct ik_ground_truth_fixture fixture;
	fixture.model = robot_geometry_factory_defaults();
	return &fixture;
}

/* ======================================================================== */
/* Tests                                                                    */
/* ======================================================================== */

ZTEST_F(ik_ground_truth, test_all_vectors)
{
	for (int v = 0; v < NUM_IK_TEST_VECTORS; v++) {
		const ik_test_vector_t *vec = &IK_TEST_VECTORS[v];

		if (!vec->converged) {
			continue; /* reference did not converge; skip */
		}

		mat4x4_t T_target;
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				T_target.m[r][c] = vec->T_target[r][c];
			}
		}

		float out[NUM_JOINTS] = {0};
		ik_status_t st = inverse_kinematics_compute(
			&fixture->model, &T_target, vec->seed, out, NULL, NULL);

		zassert_true(st == IK_SUCCESS || st == IK_OUT_OF_LIMITS,
			     "vector %d (%s): expected convergence, got %d",
			     v, vec->description, st);

		mat4x4_t T_check;
		zassert_true(forward_kinematics_compute(&fixture->model, out,
							&T_check),
			     "vector %d (%s): FK recheck failed",
			     v, vec->description);
		zassert_true(mat4x4_is_equal(&T_check, &T_target, IK_GT_POSE_TOL),
			     "vector %d (%s): FK(IK(T)) != T within %.0e",
			     v, vec->description, (double)IK_GT_POSE_TOL);
	}
}

/* ======================================================================== */
/* Suite registration                                                       */
/* ======================================================================== */

ZTEST_SUITE(ik_ground_truth, NULL, ik_gt_setup, NULL, NULL, NULL);
