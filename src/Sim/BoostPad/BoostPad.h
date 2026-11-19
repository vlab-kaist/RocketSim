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

	// The arena's bit for this pad: set while the pad may have something to update each tick (see Arena::Step).
	// Writes to the state go through SetState() (or are followed by _MarkBusy()).
	uint64_t* _busyWord = nullptr;
	uint64_t _busyBit = 0;
	void _MarkBusy() { if (_busyWord) *_busyWord |= _busyBit; }
	// Active, no cooldown and no car on it, this tick or the last: its tick updates change nothing
	bool _IsIdle() const {
		return _internalState.isActive && _internalState.cooldown == 0 && !_internalState.curLockedCar && !_internalState.prevLockedCarID;
	}

	BoostPadState GetState() const { return _internalState; }
	void SetState(const BoostPadState& state) { _internalState = state; _MarkBusy(); }

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
