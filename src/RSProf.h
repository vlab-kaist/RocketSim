// Temporary phase timer for profiling (enabled with -DRS_PROFILE)
#pragma once
#include <cstdint>
#ifdef RS_PROFILE
#include <x86intrin.h>
namespace RSProf {
	inline uint64_t acc[32] = {};
	inline const char* names[32] = {};
	struct Scope {
		int i; uint64_t t0;
		Scope(int i, const char* n) : i(i), t0(__rdtsc()) { names[i] = n; }
		~Scope() { acc[i] += __rdtsc() - t0; }
	};
}
#define RS_PROF(i, n) RSProf::Scope _rsprof##i(i, n)
#else
#define RS_PROF(i, n)
#endif
