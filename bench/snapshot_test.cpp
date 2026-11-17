// Checks that Arena::LoadSnapshot() makes an arena continue exactly as it did from the snapshot, after it went on
// differently in between, at many points of random 1v1-3v3 games. Also times saving and loading.
//   snapshot_test <meshes> [after_ticks=600]
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "RocketSim.h"
using namespace RocketSim;

static uint32_t rngState = 12345;
static float axis() { rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5; return (float)(rngState % 3) - 1; }

static void setControls(Arena* a, uint32_t seed) {
	rngState = seed;
	for (Car* c : a->GetCars()) {
		CarControls cc;
		cc.throttle = axis(); cc.steer = axis(); cc.pitch = axis(); cc.yaw = axis(); cc.roll = axis();
		cc.jump = axis() > 0; cc.boost = axis() > 0; cc.handbrake = axis() > 0.5f;
		c->controls = cc;
	}
}

struct Frame { std::vector<float> v; };
static Frame capture(Arena* a) {
	Frame f;
	auto add = [&](Vec x) { f.v.push_back(x.x); f.v.push_back(x.y); f.v.push_back(x.z); };
	BallState b = a->ball->GetState();
	add(b.pos); add(b.vel); add(b.angVel);
	for (Car* c : a->GetCars()) {
		CarState s = c->GetState();
		add(s.pos); add(s.vel); add(s.angVel); add(s.rotMat.forward); add(s.rotMat.up);
		f.v.push_back(s.boost); f.v.push_back(s.isDemoed);
	}
	for (BoostPad* p : a->GetBoostPads()) f.v.push_back(p->GetState().isActive);
	return f;
}

int main(int argc, char** argv) {
	int after = argc > 2 ? atoi(argv[2]) : 600;
	RocketSim::Init(argv[1], true);
	int failures = 0, total = 0;
	double saveTime = 0, loadTime = 0; int timed = 0;
	for (int perTeam = 1; perTeam <= 3; perTeam++) {
		for (int warm : { 0, 8, 40, 200, 333, 777, 1600, 3000 }) {
			Arena* a = Arena::Create(GameMode::SOCCAR);
			for (int t = 0; t < 2; t++)
				for (int i = 0; i < perTeam; i++)
					a->AddCar(t ? Team::ORANGE : Team::BLUE);
			a->ResetToRandomKickoff(7);
			for (int i = 0; i < warm; i += 8) { setControls(a, 1000 + i); a->Step(8); }

			ArenaSnapshot snap;
			auto t0 = std::chrono::steady_clock::now();
			a->SaveSnapshot(snap);
			auto t1 = std::chrono::steady_clock::now();

			std::vector<Frame> ref;
			for (int i = 0; i < after; i++) { if (i % 8 == 0) setControls(a, 5000 + i); a->Step(1); ref.push_back(capture(a)); }
			// Go somewhere else, then back
			for (int i = 0; i < 300; i += 8) { setControls(a, 9000 + i); a->Step(8); }

			auto t2 = std::chrono::steady_clock::now();
			a->LoadSnapshot(snap);
			auto t3 = std::chrono::steady_clock::now();
			saveTime += std::chrono::duration<double>(t1 - t0).count(); loadTime += std::chrono::duration<double>(t3 - t2).count(); timed++;

			int firstDiff = -1;
			for (int i = 0; i < after && firstDiff < 0; i++) {
				if (i % 8 == 0) setControls(a, 5000 + i);
				a->Step(1);
				if (capture(a).v != ref[i].v) firstDiff = i + 1;
			}
			total++;
			if (firstDiff >= 0) {
				failures++;
				std::printf("%dv%d snapshot after %d ticks: differs after %d ticks\n", perTeam, perTeam, warm, firstDiff);
			}
			delete a;
		}
	}
	std::printf("%d/%d snapshots match; save %.2f us, load %.2f us\n", total - failures, total, saveTime / timed * 1e6, loadTime / timed * 1e6);
	return failures != 0;
}
