#pragma once
#include "../../BaseInc.h"

#include "../Car/Car.h"

#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

#include "../MutatorConfig/MutatorConfig.h"

RS_NS_START

struct BoostPadConfig {
	Vec pos;
	bool isBig = false;

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};
#define BOOSTPADCONFIG_SERIALIZATION_FIELDS \
pos, isBig

struct BoostPadState {
	bool isActive = true;
	float cooldown = 0;

	Car* curLockedCar = nullptr;
	uint32_t prevLockedCarID = 0;

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};
#define BOOSTPAD_SERIALIZATION_FIELDS \
isActive, cooldown, prevLockedCarID

class alignas(64) BoostPad {
public:
	BoostPadConfig config;

	Vec _posBT;
	btVector3 _boxHalfBT;

	BoostPadState _internalState;

	BoostPadState GetState() const { return _internalState; }
	void SetState(const BoostPadState& state) { _internalState = state; }

	// For construction by Arena
	static BoostPad* _AllocBoostPad();
	void _Setup(const BoostPadConfig& config);

	// Puts the car on the pad if it touches it (tested at its position half a tick back)
	void _CheckCollide(Car* car, float tickTime);

	void _PreTickUpdate(float tickTime);
	bool _PostTickUpdate(float tickTime, const MutatorConfig& mutatorConfig);
private:
	BoostPad() {}
};

RS_NS_END
