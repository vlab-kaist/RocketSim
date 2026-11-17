// Many arenas stepped on many threads (OpenMP), the way a training environment runs them.
//
//   bench_par <meshes> <threads> <arenas> [cars_per_team=1] [seconds=60]
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <omp.h>
#include <atomic>
#include <csignal>
#include <cstring>
#include <ctime>
#include <ucontext.h>
#include <sys/syscall.h>
#include <unistd.h>

// Optional per-thread CPU sampler (SAMPLE=file): records the interrupted PC every 0.25 ms of thread CPU time
static unsigned long gPcs[8000000];
static std::atomic<long> gNumPcs{0};
static void SampleHandler(int, siginfo_t*, void* uc) {
	long i = gNumPcs.fetch_add(1);
	if (i < 8000000) gPcs[i] = ((ucontext_t*)uc)->uc_mcontext.gregs[REG_RIP];
}
static void StartThreadSampler() {
	sigevent sev = {};
	sev.sigev_notify = SIGEV_THREAD_ID;
	sev.sigev_signo = SIGPROF;
	sev._sigev_un._tid = (pid_t)syscall(SYS_gettid);
	timer_t t;
	timer_create(CLOCK_THREAD_CPUTIME_ID, &sev, &t);
	itimerspec its = {{0, 250000}, {0, 250000}};
	timer_settime(t, 0, &its, nullptr);
}

#include "RocketSim.h"

using namespace RocketSim;

int main(int argc, char** argv) {
	int threads = std::atoi(argv[2]), numArenas = std::atoi(argv[3]);
	int perTeam = argc > 4 ? std::atoi(argv[4]) : 1;
	int seconds = argc > 5 ? std::atoi(argv[5]) : 60;
	RocketSim::Init(argv[1], true);

	std::vector<Arena*> arenas(numArenas);
	auto makeArena = [&](Arena*& a) {
		ArenaConfig cfg = {};
		if (getenv("LIGHT")) cfg.memWeightMode = ArenaMemWeightMode::LIGHT;
		a = Arena::Create(GameMode::SOCCAR, cfg);
		for (int t = 0; t < 2; t++)
			for (int i = 0; i < perTeam; i++)
				a->AddCar(t == 0 ? Team::BLUE : Team::ORANGE);
	};
	if (getenv("LOCALALLOC")) {
		// Each thread creates the arenas it will step (no heap interleaving between threads)
#pragma omp parallel for schedule(static) num_threads(threads)
		for (int i = 0; i < numArenas; i++)
			makeArena(arenas[i]);
	} else {
		for (auto& a : arenas)
			makeArena(a);
	}

	const char* sampleOut = getenv("SAMPLE");
	if (sampleOut) {
		struct sigaction sa = {};
		sa.sa_sigaction = SampleHandler;
		sa.sa_flags = SA_SIGINFO | SA_RESTART;
		sigaction(SIGPROF, &sa, nullptr);
#pragma omp parallel num_threads(threads)
		StartThreadSampler();
	}
	const int ticks = seconds * 120, repeat = getenv("REPEAT") ? atoi(getenv("REPEAT")) : 8;
	auto t0 = std::chrono::steady_clock::now();
	const bool noBarrier = getenv("NOBARRIER") != nullptr;
	const bool dynamicSched = getenv("DYNAMIC") != nullptr;
	auto stepArena = [&](int i, int block) {
			Arena* a = arenas[i];
			uint32_t h = (uint32_t)(i * 2654435761u + block * 40503u);
			for (Car* c : a->GetCars()) {
				h ^= h << 13; h ^= h >> 17; h ^= h << 5;
				CarControls cc;
				cc.throttle = (float)(h % 3) - 1; cc.steer = (float)((h >> 2) % 3) - 1;
				cc.pitch = (float)((h >> 4) % 3) - 1; cc.yaw = (float)((h >> 6) % 3) - 1; cc.roll = (float)((h >> 8) % 3) - 1;
				cc.jump = (h >> 10) % 6 == 0; cc.boost = (h >> 13) % 3 == 0; cc.handbrake = (h >> 15) % 8 == 0;
				c->controls = cc;
			}
			if (block % 150 == 0)
				a->ResetToRandomKickoff(block + i);
			a->Step(repeat);
	};
	if (noBarrier) {
		// Each thread runs its own arenas for the whole time (no lockstep)
#pragma omp parallel for schedule(static) num_threads(threads)
		for (int i = 0; i < numArenas; i++)
			for (int block = 0; block < ticks / repeat; block++)
				stepArena(i, block);
	} else {
		for (int block = 0; block < ticks / repeat; block++) {
			if (dynamicSched) {
#pragma omp parallel for schedule(dynamic, 1) num_threads(threads)
				for (int i = 0; i < numArenas; i++)
					stepArena(i, block);
			} else {
#pragma omp parallel for schedule(static) num_threads(threads)
				for (int i = 0; i < numArenas; i++)
					stepArena(i, block);
			}
		}
	}
	double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	double total = (double)ticks * numArenas;
	std::printf("threads=%d arenas=%d cars=%d  %.0f arena-ticks/s  (%.0f per thread)\n", threads, numArenas, perTeam * 2, total / dt, total / dt / threads);
	if (sampleOut) {
		FILE* f = std::fopen(sampleOut, "w");
		FILE* m = std::fopen("/proc/self/maps", "r");
		char line[512];
		std::fprintf(f, "MAPS\n");
		while (std::fgets(line, sizeof line, m)) std::fputs(line, f);
		std::fclose(m);
		std::fprintf(f, "PCS\n");
		long k = std::min<long>(gNumPcs.load(), 8000000);
		for (long i = 0; i < k; i++) std::fprintf(f, "%lx\n", gPcs[i]);
		std::fclose(f);
	}
	return 0;
}
