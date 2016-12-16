#ifndef __TEST_UTILS_H__
#define __TEST_UTILS_H__

#include <math.h>
#include <chrono>
#include <random>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace test {

	namespace {
		std::default_random_engine util_generator;
	}  // namespace

	// Initializes the random generator to be based on the current time. Does not
	// have to be called before calling RandDouble, but it works best if it is.
	void InitRandomGenerator() 
	{
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		util_generator.seed(seed);
	}

	// Get a random double between lower and upper (inclusive).
	double RandDouble(double lower, double upper) 
	{
		std::uniform_real_distribution<double> distribution(lower, upper);
		return distribution(util_generator);
	}

	// Get a random int between lower and upper (inclusive).
	int RandInt(int lower, int upper) 
	{
		std::uniform_int_distribution<int> distribution(lower, upper);
		return distribution(util_generator);
	}

	// Gaussian Distribution with the corresponding mean and std dev.
	double RandGaussian(double mean, double std_dev) 
	{
		std::normal_distribution<double> distribution(mean, std_dev);
		return distribution(util_generator);
	}

	// Assert that values of the two matrices are nearly the same.
	template <typename Derived>
	void ExpectMatricesNear(const Eigen::MatrixBase<Derived>& a,
		const Eigen::MatrixBase<Derived>& b,
		double tolerance) 
	{
		ASSERT_EQ(a.rows(), b.rows());
		ASSERT_EQ(a.cols(), b.cols());
		for (int i = 0; i < a.rows(); i++)
		for (int j = 0; j < a.cols(); j++)
		{

			CHECK_LE(std::abs(a(i, j) - b(i, j)), std::abs(tolerance))
				<< "Entry (" << i << ", " << j << ") did not meet the tolerance!";
		}
			
	}

	void ExpectArraysNear(int n,
		const double* a,
		const double* b,
		double tolerance) 
	{
		CHECK_GE(n, 0);
		CHECK(a);
		CHECK(b);
		for (int i = 0; i < n; i++) 
		{
			CHECK_LE(std::abs(a[i] - b[i]), std::abs(tolerance)) << "i = " << i;
		}
	}

	// Expects that for all i = 1,.., n - 1
	//
	//   |p[i] / max_norm_p - q[i] / max_norm_q| < tolerance
	//
	// where max_norm_p and max_norm_q are the max norms of the arrays p
	// and q respectively.
	bool ArraysEqualUpToScale(int n, const double* p, const double* q,
		double tolerance) {
		Eigen::Map<const Eigen::VectorXd> p_vec(p, n);
		Eigen::Map<const Eigen::VectorXd> q_vec(q, n);

		// Use the cos term in the dot product to determine equality normalized for
		// scale.
		const double cos_diff = p_vec.dot(q_vec) / (p_vec.norm() * q_vec.norm());
		return std::abs(cos_diff) >= 1.0 - tolerance;
	}

	// Adds noise to the 3D point passed in.
	void AddNoiseToPoint(const double noise_factor, Eigen::Vector3d* point) 
	{
		*point += Eigen::Vector3d((-0.5 + RandDouble(0.0, 1.0)) * 2.0 * noise_factor,
			(-0.5 + RandDouble(0.0, 1.0)) * 2.0 * noise_factor,
			(-0.5 + RandDouble(0.0, 1.0)) * 2.0 * noise_factor);
	}

	// Adds noise to the ray i.e. the projection of the point.
	void AddNoiseToProjection(const double noise_factor, Eigen::Vector2d* ray) 
	{
		*ray += noise_factor * Eigen::Vector2d::Random();
	}
}

#endif // __TEST_UTILS_H__
