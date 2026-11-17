// RocketSim: scalar math that gives the same results as Rocket League.
// Ported from the native-math backend of RocketSim v3 (game_rotation/native.rs, ucrt_math/trig.rs),
// which reproduces the game's operation order and its C runtime's sinf/cosf.
// Keep every operation in order: the library is built with -ffp-contract=off, so only the explicit
// std::fma calls fuse.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include "btMatrix3x3.h"

namespace btGameMath {

inline uint32_t AbsBits(float x) {
	uint32_t bits;
	std::memcpy(&bits, &x, sizeof(bits));
	return bits & 0x7fffffffu;
}

inline double FromBits(uint64_t bits) {
	double d;
	std::memcpy(&d, &bits, sizeof(d));
	return d;
}

namespace Trig {
	const double SIN_C1 = FromBits(0x3ec71de3a556c734ull);
	const double SIN_C2 = FromBits(0xbf2a01a01a01a01aull);
	const double SIN_C3 = FromBits(0x3f81111111111111ull);
	const double SIN_C4 = FromBits(0xbfc5555555555555ull);
	const double COS_C1 = FromBits(0xbe927e4fb7789f5cull);
	const double COS_C2 = FromBits(0x3efa01a01a01a019ull);
	const double COS_C3 = FromBits(0xbf56c16c16c16c16ull);
	const double COS_C4 = FromBits(0x3fa5555555555555ull);
	const double INV_PIO2 = FromBits(0x3fe45f306dc9c883ull);
	const double PIO2_HI = FromBits(0x3ff921fb54400000ull);
	const double PIO2_LO = FromBits(0x3dd0b4611a626331ull);
	const double ONE_SIXTH = FromBits(0x3fc5555555555555ull);

	const uint32_t SINF_LIMIT = 0x4b800456u;
	const uint32_t COSF_LIMIT = 0x4f490fdbu;

	inline double SinPoly(double r) {
		double r2 = r * r;
		double p = std::fma(r2, SIN_C1, SIN_C2);
		p = std::fma(p, r2, SIN_C3);
		p = std::fma(p, r2, SIN_C4);
		return std::fma(p, r * r2, r);
	}

	inline double CosPoly(double r) {
		double r2 = r * r;
		double base = 1.0 - r2 * 0.5;
		double p = std::fma(r2, COS_C1, COS_C2);
		p = std::fma(p, r2, COS_C3);
		p = std::fma(p, r2, COS_C4);
		return std::fma(p, r2 * r2, base);
	}

	// Reduces a by multiples of pi/2
	inline double Reduce(double a, int& quadrant) {
		int n = (int)std::fma(a, INV_PIO2, 0.5);
		double nd = (double)n;
		double r1 = std::fma(-nd, PIO2_HI, a);
		quadrant = n & 3;
		return r1 - nd * PIO2_LO;
	}
}

inline float Sin(float x) {
	using namespace Trig;
	uint32_t ax = AbsBits(x);
	if (ax >= SINF_LIMIT)
		return std::sin(x);
	if (ax < 0x39000000u)
		return x;
	double d = (double)x;
	if (ax <= 0x3f490fdbu) {
		if (ax < 0x3c000000u)
			return (float)std::fma(-((d * d) * d), ONE_SIXTH, d);
		return (float)SinPoly(d);
	}
	int q = 0;
	double r = Reduce(std::fabs(d), q);
	double y = (q & 1) ? CosPoly(r) : SinPoly(r);
	return ((q >= 2) != (x < 0)) ? (float)(-y) : (float)y;
}

inline float Cos(float x) {
	using namespace Trig;
	uint32_t ax = AbsBits(x);
	if (ax >= COSF_LIMIT)
		return std::cos(x);
	if (ax < 0x39000000u)
		return 1.f;
	double d = (double)x;
	if (ax <= 0x3f490fdbu) {
		if (ax < 0x3c000000u)
			return (float)std::fma(-(d * 0.5), d, 1.0);
		return (float)CosPoly(d);
	}
	int q = 0;
	double r = Reduce(std::fabs(d), q);
	double y = (q & 1) ? SinPoly(r) : CosPoly(r);
	return (q == 1 || q == 2) ? (float)(-y) : (float)y;
}

struct Quat {
	float x, y, z, w;
};

// Quaternion of a basis whose columns are forward, right and up
inline Quat QuatFromBasis(const btMatrix3x3& m) {
	float m00 = m[0][0], m01 = m[0][1], m02 = m[0][2];
	float m10 = m[1][0], m11 = m[1][1], m12 = m[1][2];
	float m20 = m[2][0], m21 = m[2][1], m22 = m[2][2];
	float trace = (m00 + m11) + m22;
	if (trace > 0) {
		float s = sqrtf(trace + 1);
		float w = s * 0.5f;
		s = 0.5f / s;
		return { (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, w };
	}
	if (m00 < m11 ? m11 < m22 : m00 < m22) {
		float s = sqrtf(((m22 - m00) - m11) + 1);
		float z = s * 0.5f;
		s = 0.5f / s;
		return { (m02 + m20) * s, (m12 + m21) * s, z, (m10 - m01) * s };
	}
	if (m00 < m11) {
		float s = sqrtf(((m11 - m22) - m00) + 1);
		float y = s * 0.5f;
		s = 0.5f / s;
		return { (m01 + m10) * s, y, (m21 + m12) * s, (m02 - m20) * s };
	}
	float s = sqrtf(((m00 - m11) - m22) + 1);
	float x = s * 0.5f;
	s = 0.5f / s;
	return { x, (m10 + m01) * s, (m20 + m02) * s, (m21 - m12) * s };
}

inline btMatrix3x3 BasisFromQuat(const Quat& q) {
	float d = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
	float s = 2 / d;
	float xs = q.x * s, ys = q.y * s, zs = q.z * s;
	float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
	float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
	float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
	return btMatrix3x3(
		1 - (yy + zz), xy - wz, xz + wy,
		xy + wz, 1 - (xx + zz), yz - wx,
		xz - wy, yz + wx, 1 - (xx + yy)
	);
}

// Returns the identity when the squared length is NaN or not above minLenSq
inline Quat QuatNormalize(const Quat& q, float minLenSq) {
	float lenSq = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
	if (!(lenSq > minLenSq))
		return { 0, 0, 0, 1 };
	float s = 1 / sqrtf(lenSq);
	return { q.x * s, q.y * s, q.z * s, q.w * s };
}

// Rotates a basis by a world-space angular velocity over dt
inline btMatrix3x3 IntegrateOrientation(const btMatrix3x3& basis, const btVector3& angVel, float dt) {
	const float ANGULAR_MOTION_THRESHOLD = 0.785398163f; // pi/4
	float angleSq = (angVel.x() * angVel.x() + angVel.y() * angVel.y()) + angVel.z() * angVel.z();
	float angle = sqrtf(angleSq);
	if (angle * dt > ANGULAR_MOTION_THRESHOLD)
		angle = ANGULAR_MOTION_THRESHOLD / dt;

	float f;
	if (angle < 0.001f) {
		f = dt * 0.5f - (((dt * dt) * dt) * 0.020833334f * angle) * angle;
	} else {
		float halfAngle = (angle * 0.5f) * dt;
		f = Sin(halfAngle) / angle;
	}

	float ax = angVel.x() * f, ay = angVel.y() * f, az = angVel.z() * f;
	float c = Cos((angle * dt) * 0.5f);

	Quat o = QuatFromBasis(basis);
	Quat p = {
		((ax * o.w + c * o.x) + ay * o.z) - az * o.y,
		((c * o.y + ay * o.w) + az * o.x) - ax * o.z,
		((c * o.z + az * o.w) + ax * o.y) - ay * o.x,
		((c * o.w - ax * o.x) - ay * o.y) - az * o.z,
	};
	return BasisFromQuat(QuatNormalize(p, -1));
}

}  // namespace btGameMath
