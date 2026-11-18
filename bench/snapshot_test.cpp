// Checks that Arena::LoadSnapshot() makes an arena continue exactly as it did from the snapshot, after it went on
// differently in between, at many points of random 1v1-3v3 games. Also checks loading the snapshot into other
// arenas with the same layout (with other car IDs and another history), several at once from different threads.
// Also times saving and loading.
//   snapshot_test <meshes> [after_ticks=600]
#include <chrono>
#include <thread>
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
	// Car IDs as car indices, so they compare between arenas
	auto index = [&](uint32_t id) {
		int i = 0;
		for (Car* c : a->GetCars()) { if (c->id == id) return (float)i; i++; }
		return id ? -2.f : -1.f;
	};
	f.v.push_back(index(b.lastHitCarID));
	for (Car* c : a->GetCars()) f.v.push_back(index(c->GetState().carContact.otherCarID));
	for (BoostPad* p : a->GetBoostPads()) f.v.push_back(index(p->GetState().prevLockedCarID));
	return f;
}

int main(int argc, char** argv) {
	int after = argc > 2 ? atoi(argv[2]) : 600;
	RocketSim::Init(argv[1], true);
	int failures = 0, total = 0, idFieldsSet = 0, keptPoints = 0;
	double saveTime = 0, loadTime = 0; int timed = 0;
	// Also games whose cars start upside down on the floor, so the car bodies rest on contacts kept across ticks
	for (int upsideDown = 0; upsideDown < 2; upsideDown++)
	for (int perTeam = 1; perTeam <= 3; perTeam++) {
		for (int warm : { 0, 8, 40, 200, 333, 777, 1600, 3000 }) {
			if (upsideDown && warm > 333)
				continue;
			Arena* a = Arena::Create(GameMode::SOCCAR);
			for (int t = 0; t < 2; t++)
				for (int i = 0; i < perTeam; i++)
					a->AddCar(t ? Team::ORANGE : Team::BLUE);
			a->ResetToRandomKickoff(7);
			if (upsideDown) {
				int k = 0;
				for (Car* c : a->GetCars()) {
					CarState s = c->GetState();
					s.pos.z = 45 + 10 * (k++ % 3);
					s.rotMat = Angle(0.3f * k, 0, M_PI).ToRotMat();
					s.isOnGround = false;
					c->SetState(s);
				}
			}
			// Drive at the ball for the kickoff (touches and bumps), then random
			for (int i = 0; i < warm; i += 8) {
				setControls(a, 1000 + i);
				if (upsideDown)
					for (Car* c : a->GetCars()) c->controls = CarControls();
				else if (i < 320)
					for (Car* c : a->GetCars()) { CarControls cc; cc.throttle = 1; cc.boost = true; cc.jump = i >= 200 && i < 216; c->controls = cc; }
				a->Step(8);
			}

			{ // How many of the ID fields the snapshot sees set (for the cross-arena checks to mean something)
				int set = a->ball->GetState().lastHitCarID != 0;
				for (Car* c : a->GetCars()) set += c->GetState().carContact.otherCarID != 0;
				for (BoostPad* p : a->GetBoostPads()) set += p->GetState().prevLockedCarID != 0;
				idFieldsSet += set;
				for (int k = 0; k < a->_bulletWorld.m_rsContacts.size(); k++)
					keptPoints += a->_bulletWorld.m_rsContacts[k].manifold.getNumContacts();
			}
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
				std::printf("%dv%d%s snapshot after %d ticks: differs after %d ticks\n", perTeam, perTeam, upsideDown ? " upside down" : "", warm, firstDiff);
			}

			// Other arenas with the same layout: other car IDs (a car added and removed first) and another game
			// before the load. Loaded and stepped from different threads at once.
			const int numOthers = 4;
			std::vector<Arena*> others;
			for (int k = 0; k < numOthers; k++) {
				Arena* b = Arena::Create(GameMode::SOCCAR);
				for (int extra = 0; extra <= k; extra++)
					b->RemoveCar(b->AddCar(Team::BLUE));
				for (int t = 0; t < 2; t++)
					for (int i = 0; i < perTeam; i++)
						b->AddCar(t ? Team::ORANGE : Team::BLUE);
				b->ResetToRandomKickoff(100 + k);
				for (int i = 0; i < 40 * k; i += 8) { setControls(b, 7000 + i); b->Step(8); }
				others.push_back(b);
			}
			std::vector<int> otherDiff(numOthers, -1);
			std::vector<std::thread> threads;
			for (int k = 0; k < numOthers; k++)
				threads.emplace_back([&, k]() {
					Arena* b = others[k];
					b->LoadSnapshot(snap);
					for (int i = 0; i < after && otherDiff[k] < 0; i++) {
						if (i % 8 == 0) {
							// (Controls from the same sequence, without the shared generator)
							uint32_t s = 5000 + i;
							for (Car* c : b->GetCars()) {
								auto ax = [&]() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (float)(s % 3) - 1; };
								CarControls cc;
								cc.throttle = ax(); cc.steer = ax(); cc.pitch = ax(); cc.yaw = ax(); cc.roll = ax();
								cc.jump = ax() > 0; cc.boost = ax() > 0; cc.handbrake = ax() > 0.5f;
								c->controls = cc;
							}
						}
						b->Step(1);
						if (capture(b).v != ref[i].v) otherDiff[k] = i + 1;
					}
				});
			for (auto& t : threads) t.join();
			for (int k = 0; k < numOthers; k++) {
				total++;
				if (otherDiff[k] >= 0) {
					failures++;
					std::printf("%dv%d snapshot after %d ticks, loaded into another arena (%d): differs after %d ticks\n", perTeam, perTeam, warm, k, otherDiff[k]);
				}
				delete others[k];
			}
			delete a;
		}
	}
	std::printf("(car ID fields set at the snapshots: %d; kept contact points: %d)\n", idFieldsSet, keptPoints);
	std::printf("%d/%d snapshots match; save %.2f us, load %.2f us\n", total - failures, total, saveTime / timed * 1e6, loadTime / timed * 1e6);
	return failures != 0;
}
