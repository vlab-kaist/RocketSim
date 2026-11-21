// Single-thread throughput and determinism check for the simulator core.
//
//   bench <meshes> [cars_per_team=1] [seconds=600] [mode=soccar|ball|wall]
//
// wall: every 2 s the cars are put on the side walls (driving along them, full throttle, no jumps), so that
// their wheels are on the arena meshes rather than the floor plane.
// Every 8 ticks each car gets new random controls from a fixed-seed generator,
// and the arena goes back to a kickoff every 10 s of game time. The printed
// checksum hashes the full car and ball state every 120 ticks, so any change
// in simulation results changes it.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "RocketSim.h"
#include "RSProf.h"

using namespace RocketSim;

static uint64_t rng_state = 0x9E3779B97F4A7C15ull;
static uint32_t next_u32() {
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 7;
	rng_state ^= rng_state << 17;
	return (uint32_t)(rng_state >> 32);
}
static float next_axis() { return (float)(next_u32() % 3) - 1.f; }
static bool next_bit(uint32_t oneIn) { return next_u32() % oneIn == 0; }

static uint64_t hash_bytes(uint64_t h, const void* p, size_t n) {
	const uint8_t* b = (const uint8_t*)p;
	for (size_t i = 0; i < n; i++) h = (h ^ b[i]) * 0x100000001B3ull;
	return h;
}
static uint64_t hash_phys(uint64_t h, const PhysState& s) {
	float v[15] = {s.pos.x, s.pos.y, s.pos.z, s.vel.x, s.vel.y, s.vel.z, s.angVel.x, s.angVel.y, s.angVel.z,
	               s.rotMat.forward.x, s.rotMat.forward.y, s.rotMat.forward.z, s.rotMat.up.x, s.rotMat.up.y, s.rotMat.up.z};
	return hash_bytes(h, v, sizeof(v));
}

int main(int argc, char** argv) {
	if (argc < 2) {
		std::fprintf(stderr, "usage: bench <meshes> [cars_per_team] [seconds] [soccar|ball]\n");
		return 2;
	}
	int perTeam = argc > 2 ? std::atoi(argv[2]) : 1;
	int seconds = argc > 3 ? std::atoi(argv[3]) : 600;
	bool ballOnly = argc > 4 && std::string(argv[4]) == "ball";
	bool onWalls = argc > 4 && std::string(argv[4]) == "wall";

	RocketSim::Init(argv[1], true);
	ArenaConfig cfg = {};
	cfg.quantizePhysics = getenv("NOQ") == nullptr;
	Arena* arena = Arena::Create(GameMode::SOCCAR, cfg);
	std::vector<Car*> cars;
	if (!ballOnly)
		for (int t = 0; t < 2; t++)
			for (int i = 0; i < perTeam; i++) cars.push_back(arena->AddCar(t == 0 ? Team::BLUE : Team::ORANGE));

	const int ticks = seconds * 120;
	uint64_t checksum = 0xCBF29CE484222325ull;
	auto t0 = std::chrono::steady_clock::now();
	for (int tick = 0; tick < ticks; tick++) {
		if (tick % 1200 == 0) {
			arena->ResetToRandomKickoff(tick / 1200);
			if (ballOnly) {
				BallState bs = arena->ball->GetState();
				bs.pos = Vec(0, 0, 800);
				bs.vel = Vec((float)(next_u32() % 4000) - 2000.f, (float)(next_u32() % 4000) - 2000.f, (float)(next_u32() % 2000));
				arena->ball->SetState(bs);
			}
		}
		if (onWalls && tick % 240 == 0)
			for (Car* c : cars) {
				// On a side wall: up points into the field, forward along the wall, at a random place and heading
				float side = next_bit(2) ? 1.f : -1.f;
				float heading = (float)(next_u32() % 628) / 100.f;
				CarState cs = CarState();
				cs.pos = Vec(side * (4096.f - 17.f), (float)(next_u32() % 6000) - 3000.f, 300.f + (float)(next_u32() % 1200));
				Vec up(-side, 0, 0);
				Vec forward(0, cosf(heading), sinf(heading));
				cs.rotMat = RotMat(forward, up.Cross(forward), up);
				cs.vel = forward * 1000.f;
				c->SetState(cs);
			}
		if (tick % 8 == 0)
			for (Car* c : cars) {
				CarControls cc;
				cc.throttle = next_axis();
				cc.steer = next_axis();
				cc.pitch = next_axis();
				cc.yaw = next_axis();
				cc.roll = next_axis();
				cc.jump = next_bit(6);
				cc.boost = next_bit(3);
				cc.handbrake = next_bit(8);
				if (onWalls) {
					cc.throttle = 1;
					cc.pitch = cc.yaw = cc.roll = 0;
					cc.jump = cc.handbrake = false;
				}
				c->controls = cc;
			}
		arena->Step(1);
		if (tick % 120 == 119) {
			for (Car* c : cars) checksum = hash_phys(checksum, c->GetState());
			checksum = hash_phys(checksum, arena->ball->GetState());
		}
	}
	double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	std::printf("cars=%zu ticks=%d time=%.3fs ticks/s=%.0f checksum=%016llx\n", cars.size(), ticks, dt, ticks / dt,
	            (unsigned long long)checksum);
#ifdef RS_PROFILE
	uint64_t total = 0;
	for (int i = 0; i < 4; i++) total += RSProf::acc[i];
	for (int i = 0; i < 32; i++)
		if (RSProf::names[i])
			std::printf("%-34s %5.1f%%\n", RSProf::names[i], 100.0 * RSProf::acc[i] / total);
#endif
	delete arena;
	return 0;
}
