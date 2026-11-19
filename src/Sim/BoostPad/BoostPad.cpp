#include "BoostPad.h"

#include "../../RLConst.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"

RS_NS_START

void BoostPadConfig::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(
		BOOSTPADCONFIG_SERIALIZATION_FIELDS
	);
}

void BoostPadConfig::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(
		BOOSTPADCONFIG_SERIALIZATION_FIELDS
	);
}

void BoostPadState::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(
		BOOSTPAD_SERIALIZATION_FIELDS
	);
}

void BoostPadState::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(
		BOOSTPAD_SERIALIZATION_FIELDS
	);
}

BoostPad* BoostPad::_AllocBoostPad() {
	return new BoostPad();
}

void BoostPad::_Setup(const BoostPadConfig& config) {
	this->config = config;

	this->_posBT = config.pos * UU_TO_BT;

	{
		using namespace RLConst::BoostPads;

		float boxRad = (config.isBig ? BOX_RAD_BIG : BOX_RAD_SMALL) * UU_TO_BT;
		this->_boxHalfBT = btVector3(boxRad, boxRad, BOX_HEIGHT * UU_TO_BT);
	}
}

void BoostPad::_PreTickUpdate(float tickTime) {

	if (_internalState.cooldown > 0) {
		_internalState.cooldown = RS_MAX(_internalState.cooldown - tickTime, 0);
	}

	_internalState.isActive = (_internalState.cooldown == 0);

	_internalState.curLockedCar = NULL;
}

// Whether the car's hitbox (an oriented box) overlaps the pad's box, by separating axes
static bool HitboxOverlapsBox(const Car* car, const btVector3& carPosBT, const btVector3& padPosBT, const btVector3& padHalfBT) {
	const btMatrix3x3& rot = car->_rigidBody.getWorldTransform().getBasis();
	btVector3 carAxes[3] = { rot.getColumn(0), rot.getColumn(1), rot.getColumn(2) };
	btVector3 carHalf = (btVector3)car->config.hitboxSize * (0.5f * UU_TO_BT);
	btVector3 delta = carPosBT + rot * ((btVector3)car->config.hitboxPosOffset * UU_TO_BT) - padPosBT;
	const btVector3 worldAxes[3] = { btVector3(1, 0, 0), btVector3(0, 1, 0), btVector3(0, 0, 1) };

	btVector3 axes[15];
	int n = 0;
	for (int i = 0; i < 3; i++)
		axes[n++] = worldAxes[i];
	for (int i = 0; i < 3; i++)
		axes[n++] = carAxes[i];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			axes[n++] = carAxes[i].cross(worldAxes[j]);

	for (const btVector3& axis : axes) {
		if (axis.length2() <= FLT_EPSILON)
			continue;
		float padRadius = padHalfBT.dot(btVector3(fabsf(axis.x()), fabsf(axis.y()), fabsf(axis.z())));
		float carRadius = carHalf.dot(btVector3(fabsf(carAxes[0].dot(axis)), fabsf(carAxes[1].dot(axis)), fabsf(carAxes[2].dot(axis))));
		if (fabsf(delta.dot(axis)) > padRadius + carRadius)
			return false;
	}
	return true;
}

void BoostPad::_CheckCollide(Car* car, float tickTime) {
	using namespace RLConst::BoostPads;

	// The car's position half a tick back (as RocketSim v3, which matched the game's pickup ticks)
	btVector3 carPosBT = car->_rigidBody.getWorldTransform().m_origin - car->_rigidBody.m_linearVelocity * (0.5f * tickTime);

	float rad = (config.isBig ? CYL_RAD_BIG : CYL_RAD_SMALL) * UU_TO_BT;
	btVector3 delta = carPosBT - _posBT;
	bool inCylinder = (delta.x() * delta.x() + delta.y() * delta.y() < rad * rad) && fabsf(delta.z()) <= CYL_HEIGHT * UU_TO_BT;

	// A car already on the pad stays on it while its hitbox overlaps the pad's box
	bool stays = (_internalState.prevLockedCarID == car->id);
	bool colliding = inCylinder || (stays && HitboxOverlapsBox(car, carPosBT, _posBT, _boxHalfBT));

	// The car already on the pad keeps it, otherwise the first car to arrive gets it
	if (colliding && (stays || !_internalState.curLockedCar)) {
		_internalState.curLockedCar = car;
		_MarkBusy();
	}
}

bool BoostPad::_PostTickUpdate(float tickTime, const MutatorConfig& mutatorConfig) {
	using namespace RLConst::BoostPads;

	bool pickedUp = false;
	uint32_t lockedCarID = 0;
	if (_internalState.curLockedCar) {
		lockedCarID = _internalState.curLockedCar->id;

		// (A car with full boost stays on the pad without taking it)
		Car* car = _internalState.curLockedCar;
		if (_internalState.isActive && car->_internalState.boost < RLConst::BOOST_MAX) {
			float boostToAdd = config.isBig ? BOOST_AMOUNT_BIG : BOOST_AMOUNT_SMALL;
			_internalState.curLockedCar->_internalState.boost = RS_MIN(_internalState.curLockedCar->_internalState.boost + boostToAdd, RLConst::BOOST_MAX);

			_internalState.isActive = false;
			_internalState.cooldown = config.isBig ? mutatorConfig.boostPadCooldown_Big : mutatorConfig.boostPadCooldown_Small;

			pickedUp = true;
		}
	}

	_internalState.prevLockedCarID = lockedCarID;

	return pickedUp;
}

RS_NS_END
