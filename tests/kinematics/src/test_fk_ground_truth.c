/*
 * OctroBot - FK Ground-Truth Cross-Validation Tests
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compares forward_kinematics_compute() against reference values produced by
 * the modern_robotics Python library (float64).  Test vectors are embedded via
 * the auto-generated header fk_test_vectors.h.
 *
 * Regenerate vectors:
 *   cd validation && python gen_test_vectors.py
 * Then rebuild native tests:
 *   make test-native
 *
 * Tolerance: 1e-4 — accounts for float32 vs float64 accumulated error over
 * 6 chained matrix exponentials.
 */

#include <zephyr/ztest.h>
#include "kinematics_math.h"
#include "robot_geometry.h"
#include "forward_kinematics_poe.h"
#include "fk_test_vectors.h"
#include <math.h>

#define FK_GT_TOL 1e-4f

/* ======================================================================== */
/* Fixture                                                                  */
/* ======================================================================== */

struct fk_ground_truth_fixture {
	poe_robot_model_t model;
};

static void *fk_gt_setup(void)
{
	static struct fk_ground_truth_fixture fixture;
	fixture.model = robot_geometry_factory_defaults();
	return &fixture;
}

/* ======================================================================== */
/* Tests                                                                    */
/* ======================================================================== */

ZTEST_F(fk_ground_truth, test_all_vectors)
{
	for (int v = 0; v < NUM_FK_TEST_VECTORS; v++) {
		const fk_test_vector_t *vec = &FK_TEST_VECTORS[v];
		mat4x4_t T;

		bool ok = forward_kinematics_compute(&fixture->model, vec->theta, &T);
		zassert_true(ok, "vector %d (%s): compute returned false",
			     v, vec->description);

		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				float got      = T.m[r][c];
				float expected = vec->T_expected[r][c];

				zassert_true(
					is_near_equal(got, expected, FK_GT_TOL),
					"vector %d (%s) T[%d][%d]: "
					"got %.6f expected %.6f delta %.2e",
					v, vec->description, r, c,
					(double)got, (double)expected,
					(double)fabsf(got - expected));
			}
		}
	}
}

ZTEST_F(fk_ground_truth, test_zero_config_equals_M)
{
	/* Explicit check: FK(zero) == M, matching vector index 0 */
	const fk_test_vector_t *vec = &FK_TEST_VECTORS[0];
	float theta[NUM_JOINTS] = {0};
	mat4x4_t T;

	zassert_true(forward_kinematics_compute(&fixture->model, theta, &T));
	zassert_true(mat4x4_is_equal(&T, &fixture->model.M, FK_GT_TOL),
		     "FK at zero config should equal home config M");

	/* Also verify our embedded M matches the C factory default */
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			zassert_true(
				is_near_equal(fixture->model.M.m[r][c],
					      vec->T_expected[r][c], FK_GT_TOL),
				"C factory M[%d][%d]=%.6f != YAML M[%d][%d]=%.6f — "
				"robot_geometry.c and robot_config.yaml are out of sync",
				r, c, (double)fixture->model.M.m[r][c],
				r, c, (double)vec->T_expected[r][c]);
		}
	}
}

/* ======================================================================== */
/* Suite registration                                                       */
/* ======================================================================== */

ZTEST_SUITE(fk_ground_truth, NULL, fk_gt_setup, NULL, NULL, NULL);
