// Checks that Arena::Clone() continues exactly like the original, cloning at many points of a random game.
//   clone_test <meshes> [after_ticks=600]
#include <cstdio>
#include <cstdlib>
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

static bool same(Arena* a, Arena* b) {
	auto eq = [](Vec x, Vec y) { return x.x == y.x && x.y == y.y && x.z == y.z; };
	BallState ba = a->ball->GetState(), bb = b->ball->GetState();
	if (!eq(ba.pos, bb.pos) || !eq(ba.vel, bb.vel) || !eq(ba.angVel, bb.angVel))
		return false;
	auto ca = a->GetCars(), cb = b->GetCars();
	for (size_t i = 0; i < ca.size(); i++) {
		CarState sa = ca[i]->GetState(), sb = cb[i]->GetState();
		if (!eq(sa.pos, sb.pos) || !eq(sa.vel, sb.vel) || !eq(sa.angVel, sb.angVel) || sa.boost != sb.boost || sa.isDemoed != sb.isDemoed)
			return false;
	}
	return true;
}

int main(int argc, char** argv) {
	int after = argc > 2 ? atoi(argv[2]) : 600;
	RocketSim::Init(argv[1], true);
	int failures = 0, total = 0;
	// Also games whose cars start upside down on the floor, so the car bodies rest on contacts kept across ticks
	for (int upsideDown = 0; upsideDown < 2; upsideDown++)
	for (int perTeam = 1; perTeam <= 3; perTeam++) {
		for (int warm : { 0, 8, 16, 40, 96, 200, 333, 480, 777, 1000, 1600, 2400, 3000, 4000 }) {
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
			for (int i = 0; i < warm; i += 8) {
				setControls(a, 1000 + i);
				if (upsideDown)
					for (Car* c : a->GetCars()) c->controls = CarControls();
				a->Step(8);
			}
			Arena* b = a->Clone(false);
			int firstDiff = -1;
			for (int i = 0; i < after && firstDiff < 0; i += 8) {
				setControls(a, 5000 + i); setControls(b, 5000 + i);
				for (int k = 0; k < 8 && firstDiff < 0; k++) {
					a->Step(1); b->Step(1);
					if (!same(a, b))
						firstDiff = i + k + 1;
				}
			}
			total++;
			if (firstDiff >= 0) {
				failures++;
				std::printf("%dv%d%s cloned after %d ticks: differs after %d ticks\n", perTeam, perTeam, upsideDown ? " upside down" : "", warm, firstDiff);
			}
			delete a;
			delete b;
		}
	}
	std::printf("%d/%d clones match\n", total - failures, total);
	return failures != 0;
}
