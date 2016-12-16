#ifndef __TEST_UTILS_H__
#define __TEST_UTILS_H__

#include <math.h>
#include <chrono>
#include <random>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace test {

	// Initializes the random generator to be based on the current time. Does not
	// have to be called before calling RandDouble, but it works best if it is.
	void InitRandomGenerator();

	// Get a random double between lower and upper (inclusive).
	double RandDouble(double lower, double upper);

	// Get a random int between lower and upper (inclusive).
	int RandInt(int lower, int upper);

	// Gaussian Distribution with the corresponding mean and std dev.
	double RandGaussian(double mean, double std_dev);

	// Assert that values of the two matrices are nearly the same.
	template <typename Derived>
	void ExpectMatricesNear(const Eigen::MatrixBase<Derived>& a,
		const Eigen::MatrixBase<Derived>& b,
		double tolerance);

	void ExpectArraysNear(int n,
		const double* a,
		const double* b,
		double tolerance);

	// Expects that for all i = 1,.., n - 1
	//
	//   |p[i] / max_norm_p - q[i] / max_norm_q| < tolerance
	//
	// where max_norm_p and max_norm_q are the max norms of the arrays p
	// and q respectively.
	bool ArraysEqualUpToScale(int n, const double* p, const double* q, double tolerance);

	// Adds noise to the 3D point passed in.
	void AddNoiseToPoint(const double noise_factor, Eigen::Vector3d* point);

	// Adds noise to the ray i.e. the projection of the point.
	void AddNoiseToProjection(const double noise_factor, Eigen::Vector2d* ray);

} // namesapce test

#endif // __TEST_UTILS_H__
