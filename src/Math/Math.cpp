#include <algorithm>
#include "Math.h"

#include <thread>

RS_NS_START

LinearPieceCurve::LinearPieceCurve(std::initializer_list<std::pair<float, float>> points) : valueMappings(points) {
	// Sorted, keeping the first point of equal inputs (as std::map's initialization did)
	std::stable_sort(valueMappings.begin(), valueMappings.end(),
		[](const std::pair<float, float>& a, const std::pair<float, float>& b) { return a.first < b.first; });
	valueMappings.erase(std::unique(valueMappings.begin(), valueMappings.end(),
		[](const std::pair<float, float>& a, const std::pair<float, float>& b) { return a.first == b.first; }), valueMappings.end());
}

float LinearPieceCurve::GetOutput(float input, float defaultOutput) const {
	size_t count = valueMappings.size();
	if (count == 0)
		return defaultOutput;

	// Make sure it isnt before/at the first value mapping
	const auto* points = valueMappings.data();
	if (input <= points[0].first)
		return points[0].second;

	for (size_t i = 1; i < count; i++) {
		if (points[i].first > input) {
			// Found the point bigger than it! Get surrounding
			const auto& afterPair = points[i];
			const auto& beforePair = points[i - 1];

			float rangeBetween = afterPair.first - beforePair.first;
			float valDiffBetween = afterPair.second - beforePair.second;
			float linearInterpFactor = (input - beforePair.first) / rangeBetween;
			return beforePair.second + valDiffBetween * linearInterpFactor;
		}
	}

	// Must be beyond the largest input mapping, return that
	return points[count - 1].second;
}

btVector3 Math::RoundVec(btVector3 vec, float precision) {
	vec.x() = roundf(vec.x() / precision) * precision;
	vec.y() = roundf(vec.y() / precision) * precision;
	vec.z() = roundf(vec.z() / precision) * precision;
	return vec;
}

int Math::RandInt(int min, int max, int seed) {
	if (seed != -1) {
		std::default_random_engine tempEngine = std::default_random_engine(seed);
		return min + (tempEngine() % (max - min));
	} else {
		std::default_random_engine& randEngine = GetRandEngine();
		return min + (randEngine() % (max - min));
	}
}

float Math::RandFloat(float min, float max) {
	std::default_random_engine& randEngine = GetRandEngine();
	return min + ((randEngine() / (float)randEngine.max()) * (max - min));
}

std::default_random_engine& Math::GetRandEngine() {
	static thread_local auto hashThreadID = std::hash<std::thread::id>();
	static thread_local uint64_t seed = RS_CUR_MS() + hashThreadID(std::this_thread::get_id());
	static thread_local std::default_random_engine randEngine = std::default_random_engine(seed);
	return randEngine;
}

float Math::WrapNormalizeFloat(float val, float minmax) {
	float result = fmod(val, minmax * 2);
	if (result > minmax)
		result -= minmax * 2;
	else if (result < -minmax)
		result += minmax * 2;
	return result;
}

Angle Math::RoundAngleUE3(Angle ang) {
	// See: https://unrealarchive.org/wikis/unreal-wiki/Rotator.html
	// You can determine the rounding from measuring the resulting vector directions from conversions
	// This was very, very annoying to figure out :/

	constexpr float TO_INTS = (float)(1 << 15) / M_PI;
	constexpr float BACK_TO_RADIANS = (1.f / TO_INTS) * 4.f;
	constexpr int ROUNDING_MASK = 0x4000 - 1;

	int rYaw = ((int)(ang.yaw * TO_INTS) >> 2) & ROUNDING_MASK;
	int rPitch = ((int)(ang.pitch * TO_INTS) >> 2) & ROUNDING_MASK;
	ang.yaw = rYaw * BACK_TO_RADIANS;
	ang.pitch = rPitch * BACK_TO_RADIANS;
	assert(ang.roll == 0);
	
	return ang;
}

RS_NS_END
