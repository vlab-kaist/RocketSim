#include "Car.h"
#include "../../RSProf.h"
#include "../../RLConst.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "../CollisionMasks.h"

RS_NS_START

// Update our internal state from bullet and return it
CarState Car::GetState() {
	if (!_internalState.isDemoed) {
		_internalState.pos = _rigidBody.getWorldTransform().m_origin * BT_TO_UU;

		// NOTE: rotMat already updated at the start of Car::_PostTickUpdate()

		_internalState.vel = _rigidBody.m_linearVelocity * BT_TO_UU;

		_internalState.angVel = _rigidBody.m_angularVelocity;
	}

	return _internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) {
	btTransform rbTransform;
	rbTransform.setOrigin(state.pos * UU_TO_BT);
	rbTransform.setBasis(state.rotMat);
	_rigidBody.getWorldTransform() = rbTransform;
	_rigidBody.m_linearVelocity = state.vel * UU_TO_BT;
	_rigidBody.m_angularVelocity = state.angVel;
	_rigidBody.updateInertiaTensor();

	_velocityImpulseCache = { 0, 0, 0 };

	_internalState = state;
	_internalState.tickCountSinceUpdate = 0;

	// The tick logic reads the wheel contacts of the last vehicle update, so trace them for the new state
	if (_bulletVehicle.m_dynamicsWorld && !_internalState.isDemoed)
		_UpdateWheelContacts(true);
}

void Car::_UpdateWheelContacts(bool trace) {
	if (trace)
		_bulletVehicle.updateVehicleFirst(0);

	int numWheelsInContact = 0;
	for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
		bool inContact = _bulletVehicle.m_wheelInfo[i].m_raycastInfo.m_isInContact;
		_internalState.wheelsWithContact[i] = inContact;
		numWheelsInContact += inContact;
	}
	_internalState.isOnGround = numWheelsInContact >= 3;
}

void Car::Demolish(float respawnDelay) {
	_internalState.isDemoed = true;
	_internalState.demoRespawnTimer = respawnDelay;
}

void Car::Respawn(GameMode gameMode, int seed, float boostAmount) {
	using namespace RLConst;

	CarState newState = CarState();

	int spawnPosIndex = Math::RandInt(0, CAR_RESPAWN_LOCATION_AMOUNT, seed);
	CarSpawnPos spawnPos = ((gameMode == GameMode::HOOPS) ? CAR_RESPAWN_LOCATIONS_HOOPS : CAR_RESPAWN_LOCATIONS_SOCCAR)[spawnPosIndex];

	newState.pos = Vec(spawnPos.x, spawnPos.y * (team == Team::BLUE ? 1 : -1), CAR_RESPAWN_Z);
	newState.rotMat = Angle(spawnPos.yawAng + (team == Team::BLUE ? 0 : M_PI), 0.f, 0.f).ToRotMat();

	newState.boost = boostAmount;
	this->SetState(newState);
}

// Byte quantization of an analog input, as the game sends it (see Arena.cpp)
static float QuantizeAxisInput(float x) {
	float clamped = RS_CLAMP(x, -1.f, 1.f);
	float y = (clamped < 0) ? RS_MAX(clamped * 128.f, -128.f) : RS_MIN(clamped * 127.f, 127.f);
	float w = ((y + 128.f) + (y + 128.f)) + 0.5f;
	int byte = (((int)nearbyintf(w)) >> 1) & 0xFF;
	float s = (float)byte - 128.f;
	return (byte < 0x80) ? (s * (1.f / 128.f)) : (s / 127.f);
}

void Car::_LimitVels() {
	using namespace RLConst;
	btVector3& vel = _rigidBody.m_linearVelocity;
	btVector3& angVel = _rigidBody.m_angularVelocity;
	if (vel.length2() > (CAR_MAX_SPEED * UU_TO_BT) * (CAR_MAX_SPEED * UU_TO_BT))
		vel = vel.normalized() * (CAR_MAX_SPEED * UU_TO_BT);
	if (angVel.length2() > (CAR_MAX_ANG_SPEED * CAR_MAX_ANG_SPEED))
		angVel = angVel.normalized() * CAR_MAX_ANG_SPEED;
}

void Car::_ApplyFlipZDamp() {
	using namespace RLConst;
	if (_internalState.isDemoed || _internalState.isOnGround || !_internalState.isFlipping)
		return;
	float flipTime = _internalState.flipTime;
	if (flipTime < FLIP_Z_DAMP_START || flipTime > FLIP_TORQUE_TIME)
		return;
	if (_rigidBody.m_linearVelocity.z() < 0 || flipTime < FLIP_Z_DAMP_END)
		_rigidBody.m_linearVelocity.z() *= 1 - FLIP_Z_DAMP_120;
}

void Car::_PreTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig) {
	using namespace RLConst;

#ifndef RS_MAX_SPEED
	// Fix inputs
	controls.ClampFix();
#endif

	assert(_bulletVehicle.getNumWheels() == 4 || _bulletVehicle.getNumWheels() == 3);

	{ // Update simulation state
		if (_internalState.isDemoed) {
			_internalState.demoRespawnTimer = RS_MAX(_internalState.demoRespawnTimer - tickTime, 0);
			if (_internalState.demoRespawnTimer == 0)
				Respawn(gameMode, -1, mutatorConfig.carSpawnBoostAmount);

			// Disable rigidbody simulation
			_rigidBody.m_activationState1 = DISABLE_SIMULATION;
			_rigidBody.m_collisionFlags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;

			// Don't bother updating anything
		} else {
			// Prevent the car's RB from becoming inactive
			_rigidBody.m_activationState1 = ACTIVE_TAG;
			_rigidBody.m_collisionFlags &= ~btCollisionObject::CF_NO_CONTACT_RESPONSE;
		}
	}

	if (_internalState.isDemoed)
		return; // No other updates need to occur

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	// As in the game (and RocketSim v3), the tick logic uses the wheel contacts found by the
	// previous tick's vehicle update. The wheels are traced again at the end of this tick.
	int numWheelsInContact = 0, numWorldWheels = 0;
	for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
		numWheelsInContact += _internalState.wheelsWithContact[i];
		numWorldWheels += _internalState.wheelsWithContact[i] && _bulletVehicle.m_wheelInfo[i].m_isInContactWithWorld;
	}

	float forwardSpeed_UU = _bulletVehicle.getForwardSpeed() * BT_TO_UU;
	{ RS_PROF(4, "  car logic (wheels..boost)");
	_UpdateWheels(tickTime, mutatorConfig, numWheelsInContact, forwardSpeed_UU);

	if (_internalState.isOnGround) {
		if (_internalState.isFlipping)
			_internalState.flipTime = 0;
		_internalState.isFlipping = false;
	}

	bool wasIsJumping = _internalState.isJumping;
	_UpdateJump(tickTime, mutatorConfig, jumpPressed);
	_UpdateAutoFlip(tickTime, mutatorConfig, jumpPressed);

	bool prevIsFlipping = _internalState.isFlipping;
	float prevFlipTime = _internalState.flipTime;
	bool flipEnded = _UpdateDoubleJumpOrFlip(tickTime, mutatorConfig, jumpPressed, forwardSpeed_UU, wasIsJumping);

	if (!_internalState.isOnGround)
		_UpdateAirTorque(tickTime, mutatorConfig, numWheelsInContact, prevIsFlipping, prevFlipTime);

	// Auto-roll counts the wheels that touched the world last tick, and skips the tick a flip ends
	if (controls.throttle != 0 && !_internalState.isFlipping && !flipEnded && numWorldWheels < 4
		&& (numWorldWheels > 0 || _internalState.worldContact.hasContact))
		_UpdateAutoRoll(tickTime, mutatorConfig, numWheelsInContact);

	_internalState.worldContact.hasContact = false;

	_UpdateBoost(tickTime, mutatorConfig, forwardSpeed_UU);
	}

	// Vehicle update: trace the wheels, set their friction from the new contacts, then apply suspension and friction
	{ RS_PROF(5, "  vehicle: trace");
	_bulletVehicle.updateVehicleFirst(tickTime);
	}
	RS_PROF(6, "  vehicle: friction+suspension");
	_UpdateWheelContacts(false);
	_UpdateWheelFriction();
	if (_internalState.isOnGround == false) {
		int freshContacts = 0;
		for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++)
			freshContacts += _internalState.wheelsWithContact[i];
		if (freshContacts < 3)
			for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++)
				_bulletVehicle.m_wheelInfo[i].m_engineForce /= 4;
	}
	_bulletVehicle.updateVehicleSecond(tickTime);
}

void Car::_PostTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig) {

	if (_internalState.isDemoed)
		return;

	_internalState.rotMat = _rigidBody.getWorldTransform().m_basis;

	{ // Update supersonic: kept between the maintain speed and the start speed for at most SUPERSONIC_MAINTAIN_MAX_TIME
		using namespace RLConst;
		float speedSquared = (_rigidBody.m_linearVelocity * BT_TO_UU).length2();
		float startSq = SUPERSONIC_START_SPEED * SUPERSONIC_START_SPEED;
		float maintainSq = SUPERSONIC_MAINTAIN_MIN_SPEED * SUPERSONIC_MAINTAIN_MIN_SPEED;

		if (_internalState.isSupersonic) {
			if (speedSquared >= startSq) {
				_internalState.supersonicTime = 0;
			} else if (speedSquared < maintainSq) {
				_internalState.isSupersonic = false;
				_internalState.supersonicTime = 0;
			} else {
				_internalState.supersonicTime += tickTime;
				if (_internalState.supersonicTime >= SUPERSONIC_MAINTAIN_MAX_TIME) {
					_internalState.isSupersonic = false;
					_internalState.supersonicTime = 0;
				}
			}
		} else {
			_internalState.isSupersonic = speedSquared >= startSq;
			_internalState.supersonicTime = 0;
		}
	}

	{ // Re-arm the jump once the car has landed
		constexpr uint32_t JUMP_SETTLED_REARM_TICKS = 15;
		if (_internalState.hasJumped && !_internalState.isJumping && _internalState.isOnGround && _internalState.airTime > 0) {
			// Not while all touching wheels are still extending (the car is leaving the ground)
			int numContacts = 0;
			bool allExtending = true;
			for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
				auto& wheel = _bulletVehicle.m_wheelInfo[i];
				if (wheel.m_raycastInfo.m_isInContact) {
					numContacts++;
					allExtending &= wheel.m_suspensionRelativeVelocity > 1;
				}
			}
			if (!(numContacts != 0 && allExtending))
				_internalState.hasJumped = false;
		} else if (_internalState.hasJumped && !_internalState.isJumping && _internalState.isOnGround
			&& _internalState.jumpTicks > JUMP_SETTLED_REARM_TICKS) {
			_internalState.hasJumped = false;
		}
	}

	// Update car contact cooldown timer
	if (_internalState.carContact.cooldownTimer > 0)
		_internalState.carContact.cooldownTimer = RS_MAX(_internalState.carContact.cooldownTimer - tickTime, 0);

	_internalState.lastControls = controls;
}

void Car::_FinishPhysicsTick(const MutatorConfig& mutatorConfig) {
	using namespace RLConst;

	if (_internalState.isDemoed)
		return;

	// Add velocity cache
	if (!_velocityImpulseCache.IsZero()) {
		_rigidBody.m_linearVelocity += _velocityImpulseCache;
		_velocityImpulseCache = { 0,0,0 };
	}

	{ // Limit velocities
		btVector3
			vel = _rigidBody.m_linearVelocity,
			angVel = _rigidBody.m_angularVelocity;

		if (vel.length2() > (CAR_MAX_SPEED * UU_TO_BT) * (CAR_MAX_SPEED * UU_TO_BT))
			vel = vel.normalized() * (CAR_MAX_SPEED * UU_TO_BT);

		if (angVel.length2() > (CAR_MAX_ANG_SPEED * CAR_MAX_ANG_SPEED))
			angVel = angVel.normalized() * CAR_MAX_ANG_SPEED;

		_rigidBody.m_linearVelocity = vel;
		_rigidBody.m_angularVelocity = angVel;
	}

	_internalState.tickCountSinceUpdate++;
}

void Car::_BulletSetup(GameMode gameMode, btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig) {
	// Set up rigidbody and collision shapes
	_childHitboxShape = btBoxShape((config.hitboxSize * UU_TO_BT) / 2);
	// btBoxShape subtracts the default margin before it shrinks the margin to fit small boxes,
	// which leaves the box 0.065uu small on every side. Keep the full hitbox size, as the game does.
	_childHitboxShape.setImplicitShapeDimensions((btVector3)(config.hitboxSize * UU_TO_BT / 2) - btVector3(1, 1, 1) * _childHitboxShape.getMargin());
	_compoundShape = btCompoundShape(false, 1);

	btTransform hitboxOffsetTransform = btTransform();
	hitboxOffsetTransform.setIdentity();
	hitboxOffsetTransform.setOrigin(config.hitboxPosOffset * UU_TO_BT);
	_compoundShape.addChildShape(hitboxOffsetTransform, &_childHitboxShape);

	btVector3 localInertia(0, 0, 0);
	_childHitboxShape.calculateLocalInertia(RLConst::CAR_MASS_BT, localInertia);

	btRigidBody::btRigidBodyConstructionInfo rbInfo
		= btRigidBody::btRigidBodyConstructionInfo(RLConst::CAR_MASS_BT, NULL, &_compoundShape, localInertia);

	btTransform carTransform = btTransform();
	carTransform.setIdentity();
	rbInfo.m_startWorldTransform = carTransform;

	_rigidBody = btRigidBody(rbInfo);
	_rigidBody.setUserIndex(BT_USERINFO_TYPE_CAR);
	_rigidBody.setUserPointer(this);

	_rigidBody.m_collisionFlags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK;

	_rigidBody.m_friction = RLConst::CAR_COLLISION_FRICTION;
	_rigidBody.m_restitution = RLConst::CAR_COLLISION_RESTITUTION;

	// Disable gyroscopic force
	_rigidBody.m_rigidbodyFlags = 0;
	
	// We want our car and our suspension rays to collide with the dropshot floor
	int extraCollisionMask = CollisionMasks::DROPSHOT_FLOOR;

	// Add rigidbody to world
	int mask = btBroadphaseProxy::AllFilter;
	if (!mutatorConfig.enableCarCarCollision)
		mask &= ~btBroadphaseProxy::CharacterFilter;
	bulletWorld->addRigidBody(&_rigidBody, btBroadphaseProxy::CharacterFilter, mask | extraCollisionMask);

	{ // Set up actual vehicle stuff
		_bulletVehicleRaycaster = btDefaultVehicleRaycaster(bulletWorld);
		float minHitDist = (config.hitboxSize.z * 0.5f - 2.25f) * UU_TO_BT;
		_bulletVehicleRaycaster.m_minHitDistSq = minHitDist * minHitDist;

		btVehicleRL::btVehicleTuning tuning = btVehicleRL::btVehicleTuning();

		_bulletVehicle = btVehicleRL(tuning, &_rigidBody, &_bulletVehicleRaycaster, bulletWorld, extraCollisionMask);

		// Match RL with X forward, Y right, Z up
		_bulletVehicle.setCoordinateSystem(1, 2, 0);

		// Set up wheel directions with RL coordinate system
		btVector3 wheelDirectionCS(0, 0, -1), wheelAxleCS(0, -1, 0);

		{ // Set up wheels
			int numWheels = 4;//config.threeWheels ? 3 : 4;
			for (int i = 0; i < numWheels; i++) {
				bool front = i < 2;
				if (numWheels == 3)
					front = i == 0;

				bool left = i % 2;

				float radius = front ? config.frontWheels.wheelRadius : config.backWheels.wheelRadius;
				btVector3 wheelRayStartOffset =
					front ? config.frontWheels.connectionPointOffset : config.backWheels.connectionPointOffset;

				if (left)
					wheelRayStartOffset.y() *= -1;

				float suspensionRestLength =
					front ? config.frontWheels.suspensionRestLength : config.backWheels.suspensionRestLength;

				suspensionRestLength -= RLConst::BTVehicle::MAX_SUSPENSION_TRAVEL;

				_bulletVehicle.addWheel(
					wheelRayStartOffset * UU_TO_BT,
					wheelDirectionCS, wheelAxleCS,
					suspensionRestLength * UU_TO_BT,
					radius * UU_TO_BT, tuning, true);

				{ // Fix wheel info data
					using namespace RLConst::BTVehicle;

					btWheelInfoRL& wheelInfo = _bulletVehicle.m_wheelInfo[i];
					wheelInfo.m_suspensionStiffness = SUSPENSION_STIFFNESS;
					wheelInfo.m_wheelsDampingCompression = WHEELS_DAMPING_COMPRESSION;
					wheelInfo.m_wheelsDampingRelaxation = WHEELS_DAMPING_RELAXATION;
					wheelInfo.m_maxSuspensionTravelCm = (MAX_SUSPENSION_TRAVEL * UU_TO_BT) * 100; // Same for all cars (hopefully)
					wheelInfo.m_maxSuspensionForce = FLT_MAX; // Don't think there's a limit
					wheelInfo.m_bIsFrontWheel = front;
					wheelInfo.m_suspensionForceScale = front ? SUSPENSION_FORCE_SCALE_FRONT : SUSPENSION_FORCE_SCALE_BACK;
				}
			}
		}
	}

	_internalState.boost = mutatorConfig.carSpawnBoostAmount;
}

bool CarState::HasFlipOrJump() const {
	return 
		isOnGround || 
		(!hasFlipped && !hasDoubleJumped && airTimeSinceJump < RLConst::DOUBLEJUMP_MAX_DELAY);
}

bool CarState::HasFlipReset() const {
	return !isOnGround && HasFlipOrJump() && !hasJumped;
}

bool CarState::GotFlipReset() const {
	return !isOnGround && !hasJumped;
}

void CarState::Serialize(DataStreamOut& out) const {
	ballHitInfo.Serialize(out);

	out.WriteMultiple(
		CARSTATE_SERIALIZATION_FIELDS
	);
}

void CarState::Deserialize(DataStreamIn& in) {

	ballHitInfo.Deserialize(in);

	in.ReadMultiple(
		CARSTATE_SERIALIZATION_FIELDS
	);
}

void Car::Serialize(DataStreamOut& out) {
	out.WriteMultiple(CAR_CONTROLS_SERIALIZATION_FIELDS(controls));
	out.WriteMultiple(CAR_CONFIG_SERIALIZATION_FIELDS(config));
	GetState().Serialize(out);
}

void Car::_Deserialize(DataStreamIn& in) {
	in.ReadMultiple(CAR_CONTROLS_SERIALIZATION_FIELDS(controls));
	in.ReadMultiple(CAR_CONFIG_SERIALIZATION_FIELDS(config));
	CarState newState;
	newState.Deserialize(in);
	_internalState = newState;
}

void Car::_UpdateWheels(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact, float forwardSpeed_UU) {
	using namespace RLConst;

	float absForwardSpeed_UU = abs(forwardSpeed_UU);

	bool wheelsHaveWorldContact = false;
	for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++)
		wheelsHaveWorldContact |= _bulletVehicle.m_wheelInfo[i].m_isInContactWithWorld;

	{ // Increase/decrease handbrake value from input
		if (controls.handbrake) {
			_internalState.handbrakeVal += POWERSLIDE_RISE_RATE * tickTime;
		} else {
			_internalState.handbrakeVal -= POWERSLIDE_FALL_RATE * tickTime;
		}
		_internalState.handbrakeVal = RS_CLAMP(_internalState.handbrakeVal, 0, 1);
	}

	float realThrottle = controls.throttle;
	float realBrake = 0;

	{ // Update throttle/brake forces

		float engineThrottle = realThrottle;

		if (controls.handbrake) {
			// Real throttle is unchanged from the input throttle when powersliding
		} else {
			float absThrottle = abs(realThrottle);

			if (absThrottle > THROTTLE_DEADZONE) {
				if (absForwardSpeed_UU > STOPPING_FORWARD_VEL && RS_SGN(realThrottle) != RS_SGN(forwardSpeed_UU)) {
					// Full brake is applied if we are trying to drive in the opposite direction
					realBrake = 1;

					if (absForwardSpeed_UU > BRAKING_NO_THROTTLE_SPEED_THRESH) {
						// Kill actual throttle (we can't throttle and brake at the same time, even backwards)
						engineThrottle = 0;
					}
				}
			} else {
				// No throttle, we are coasting
				engineThrottle = 0;

				// Apply coasting brake, we full-break when coasting very slowly
				bool shouldFullStop = (absForwardSpeed_UU < STOPPING_FORWARD_VEL);
				realBrake = shouldFullStop ? 1 : COASTING_BRAKE_FACTOR;
			}
		}

		float driveSpeedScale = (engineThrottle != 0) ? DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(absForwardSpeed_UU) : 1;
		float driveEngineForce = engineThrottle * (THROTTLE_TORQUE_AMOUNT * UU_TO_BT) * driveSpeedScale;
		float driveBrakeForce = realBrake * (BRAKE_TORQUE_AMOUNT * UU_TO_BT);
		for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
			_bulletVehicle.m_wheelInfo[i].m_engineForce = driveEngineForce;
			_bulletVehicle.m_wheelInfo[i].m_brake = driveBrakeForce;
		}
	}

	{ // Update steering
		float steerAngle = (config.threeWheels ? STEER_ANGLE_FROM_SPEED_CURVE_THREEWHEEL : STEER_ANGLE_FROM_SPEED_CURVE).GetOutput(absForwardSpeed_UU);

		if (_internalState.handbrakeVal) {
			steerAngle +=
				(POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeed_UU) - steerAngle)
				* _internalState.handbrakeVal;
		}

		steerAngle *= controls.steer;
		_bulletVehicle.m_wheelInfo[0].m_steerAngle = steerAngle;
		_bulletVehicle.m_wheelInfo[1].m_steerAngle = steerAngle;
	}

	_realThrottle = realThrottle;

	// Update sticky forces if at least 1 wheel has world contact
	if (wheelsHaveWorldContact) {
		btVector3 upwardsDir = _bulletVehicle.getUpwardsDirFromWheelContacts();

		bool fullStick = (realThrottle != 0) || (absForwardSpeed_UU > STOPPING_FORWARD_VEL);

		float stickyForceScale = config.threeWheels ? 0 : 0.5f;
		if (fullStick)
			stickyForceScale += 1 - abs(upwardsDir.z());

		_rigidBody.applyCentralForce(upwardsDir * stickyForceScale * (GRAVITY_Z * UU_TO_BT) * CAR_MASS_BT);
	}
}

void Car::_UpdateWheelFriction() {
	using namespace RLConst;

	// Steering changed the front wheel axles this tick
	for (int i = 0; i < _bulletVehicle.getNumWheels(); i++)
		_bulletVehicle.updateWheelBasis(i);

	float realThrottle = _realThrottle;
	{
		for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
			auto& wheel = _bulletVehicle.m_wheelInfo[i];
			if (wheel.m_raycastInfo.m_groundObject) {

				btVector3
					vel = _rigidBody.m_linearVelocity,
					angularVel = _rigidBody.m_angularVelocity;

				btVector3
					latDir = wheel.m_worldTransform.getBasis().getColumn(1),
					longDir = latDir.cross(wheel.m_raycastInfo.m_contactNormalWS);

				float frictionCurveInput = 0;

				btVector3 wheelDelta = wheel.m_raycastInfo.m_hardPointWS - _rigidBody.getWorldTransform().m_origin;

				auto crossVec = (angularVel.cross(wheelDelta) + vel) * BT_TO_UU;

				float baseFriction = abs(crossVec.dot(latDir));

				// Significant friction results in lateral slip
				if (baseFriction > 5)
					frictionCurveInput = baseFriction / (abs(crossVec.dot(longDir)) + baseFriction);

				float latFriction = (config.threeWheels ? LAT_FRICTION_CURVE_THREEWHEEL : LAT_FRICTION_CURVE).GetOutput(frictionCurveInput);
				float longFriction = 1;

				if (_internalState.handbrakeVal) {
					float handbrakeAmount = _internalState.handbrakeVal;

					latFriction *= (HANDBRAKE_LAT_FRICTION_FACTOR_CURVE.GetOutput(frictionCurveInput) - 1) * handbrakeAmount + 1;
					longFriction *= (HANDBRAKE_LONG_FRICTION_FACTOR_CURVE.GetOutput(frictionCurveInput) - 1) * handbrakeAmount + 1;
				}

				// Contact on a moving body (the ball or another car) is never sticky
				bool isContactSticky = realThrottle != 0 && ((btCollisionObject*)wheel.m_raycastInfo.m_groundObject)->isStaticObject();

				if (isContactSticky) {
					// Keep current friction values
				} else {
					// Scale friction down with non-sticky friction curve
					float nonStickyScale = NON_STICKY_FRICTION_FACTOR_CURVE.GetOutput(wheel.m_raycastInfo.m_contactNormalWS.z());
					latFriction *= nonStickyScale;
					longFriction *= nonStickyScale;
				}

				wheel.m_latFriction = latFriction;
				wheel.m_longFriction = longFriction;
			}
		}
	}

}

void Car::_UpdateBoost(float tickTime, const MutatorConfig& mutatorConfig, float forwardSpeed_UU) {
	using namespace RLConst;

	// Boost state machine as in RocketSim v3: once started, boosting latches for at least BOOST_MIN_TIME
	// and ends when the boost runs out
	auto applyBoost = [&]() {
		_internalState.isBoosting = true;
		_internalState.boostingTime += tickTime;
		_internalState.timeSinceBoosted = 0;
		_rigidBody.applyCentralForce(
			(_internalState.isOnGround ? mutatorConfig.boostAccelGround : mutatorConfig.boostAccelAir) * UU_TO_BT
			* GetForwardDir() * CAR_MASS_BT
		);
	};
	auto stopBoost = [&]() {
		_internalState.isBoosting = false;
		_internalState.boostingTime = 0;
		_internalState.timeSinceBoosted += tickTime;
		if (mutatorConfig.rechargeBoostEnabled && _internalState.timeSinceBoosted >= mutatorConfig.rechargeBoostDelay)
			_internalState.boost += mutatorConfig.rechargeBoostPerSecond * tickTime;
	};

	if (_internalState.isBoosting) {
		_internalState.boost = RS_MAX(_internalState.boost - mutatorConfig.boostUsedPerSecond * tickTime, 0);
		bool depleted = _internalState.boost == 0;
		bool latchExpired = !controls.boost && _internalState.boostingTime >= BOOST_MIN_TIME;
		if (depleted || latchExpired) {
			stopBoost();
		} else {
			applyBoost();
		}
	} else if (controls.boost && _internalState.boost > 0) {
		applyBoost();
	} else {
		stopBoost();
	}

	_internalState.boost = RS_CLAMP(_internalState.boost, 0.f, RLConst::BOOST_MAX);
}

void Car::_UpdateJump(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed) {
	using namespace RLConst;
	constexpr uint32_t JUMP_MIN_TICKS = 3, JUMP_MAX_TICKS = 24;

	if (!_internalState.hasJumped && _internalState.isOnGround && jumpPressed) {
		_internalState.isJumping = true;
		_internalState.hasJumped = true;
		_internalState.jumpTicks = 0;
		_internalState.jumpHoldBroken = false;
	}

	_internalState.jumpTicks++;

	if (_internalState.isJumping) {
		// Holding must be continuous: once released, the jump ends after the minimum ticks
		if (!controls.jump)
			_internalState.jumpHoldBroken = true;

		_internalState.isJumping = _internalState.jumpTicks <= JUMP_MIN_TICKS
			|| (!_internalState.jumpHoldBroken && controls.jump && _internalState.jumpTicks <= JUMP_MAX_TICKS);
		if (!_internalState.isJumping)
			_internalState.jumpTicks = 1; // Counter restarts when the jump ends
	}
	_internalState.jumpTime = _internalState.isJumping ? (_internalState.jumpTicks * tickTime) : 0;

	if (_internalState.isJumping) {
		btVector3 upDir = _rigidBody.getWorldTransform().getBasis().getColumn(2);
		if (_internalState.jumpTicks == 1) {
			_rigidBody.m_linearVelocity += upDir * mutatorConfig.jumpImmediateForce * UU_TO_BT;
			_LimitVels();
		}
		_rigidBody.applyCentralForce(upDir * mutatorConfig.jumpAccel * UU_TO_BT * mutatorConfig.carMass);
	}
}

void Car::_UpdateAirTorque(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact, bool prevIsFlipping, float prevFlipTime) {
	using namespace RLConst;
	constexpr float FLIP_PITCH_CANCEL_GATE_MIN_TIME = 0.041f;

	btMatrix3x3 basis = _rigidBody.getWorldTransform().getBasis();
	btVector3 forwardDir = basis.getColumn(0), rightDir = basis.getColumn(1), upDir = basis.getColumn(2);
	bool allowDodge = numWheelsInContact < 3;
	bool allowAir = numWheelsInContact == 0;
	btMatrix3x3 inertia = _rigidBody.m_invInertiaTensorWorld.inverse();

	if (_internalState.isFlipping && allowDodge && !_internalState.flipRelTorque.IsZero()) {
		Vec relDodgeTorque = _internalState.flipRelTorque;

		float pitchScale = 1;
		if (relDodgeTorque.y != 0 && controls.pitch != 0 && RS_SGN(relDodgeTorque.y) == RS_SGN(controls.pitch)
			&& prevFlipTime >= FLIP_PITCH_CANCEL_GATE_MIN_TIME)
			pitchScale = 1 - RS_MIN(fabsf(QuantizeAxisInput(-controls.pitch)), 1.f);

		// Each torque component is rounded to whole units before the pitch-cancel scale
		float torqueX = nearbyintf(relDodgeTorque.x * FLIP_TORQUE_X);
		float torqueY = nearbyintf(relDodgeTorque.y * FLIP_TORQUE_Y) * pitchScale;
		btVector3 worldTorque = rightDir * torqueY + forwardDir * torqueX;
		_rigidBody.applyTorque(inertia * worldTorque);
	}

	if (allowAir) {
		Vec inputs = Vec(controls.pitch, controls.yaw, controls.roll);

		float pitchTorqueScale = 1;
		if (!inputs.IsZero() && (prevIsFlipping || _internalState.isFlipping || prevFlipTime < FLIP_PITCHLOCK_EXTRA_TIME))
			pitchTorqueScale = 0;

		// Pitch, yaw and roll axes
		btVector3 dirs[3] = { -rightDir, upDir, -forwardDir };
		float scaled[3] = { inputs.x * pitchTorqueScale, inputs.y, inputs.z };
		float torqueAmounts[3] = { CAR_AIR_CONTROL_TORQUE.x, CAR_AIR_CONTROL_TORQUE.y, CAR_AIR_CONTROL_TORQUE.z };
		float dampAmounts[3] = { CAR_AIR_CONTROL_DAMPING.x, CAR_AIR_CONTROL_DAMPING.y, CAR_AIR_CONTROL_DAMPING.z };
		float retain[3] = { 1 - fabsf(scaled[0]), 1 - fabsf(scaled[1]), 1 };

		btVector3 torque = { 0, 0, 0 }, damping = { 0, 0, 0 };
		btVector3 angVel = _rigidBody.m_angularVelocity;
		for (int i = 0; i < 3; i++) {
			if (!inputs.IsZero())
				torque += dirs[i] * (scaled[i] * torqueAmounts[i]);
			damping += dirs[i] * (dirs[i].dot(angVel) * dampAmounts[i] * retain[i]);
		}
		_rigidBody.applyTorque(inertia * (torque - damping) * CAR_TORQUE_SCALE);

		// While boosting, the game drives the air throttle at full regardless of the input
		float throttleScale = _internalState.isBoosting ? 1.f : controls.throttle;
		if (throttleScale != 0)
			_rigidBody.applyCentralForce(forwardDir * (THROTTLE_AIR_ACCEL * mutatorConfig.carMass) * throttleScale * UU_TO_BT);
	}
}

bool Car::_UpdateDoubleJumpOrFlip(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed, float forwardSpeed_UU, bool wasIsJumping) {
	using namespace RLConst;

	if (_internalState.isOnGround) {
		_internalState.hasDoubleJumped = false;
		_internalState.hasFlipped = false;
		_internalState.airTime = 0;
		_internalState.airTimeSinceJump = 0;
		_internalState.flipTime += tickTime;
		return false;
	}

	_internalState.airTime += tickTime;

	if (_internalState.hasJumped && !wasIsJumping && !_internalState.isJumping) {
		_internalState.airTimeSinceJump += tickTime;
	} else {
		_internalState.airTimeSinceJump = 0;
	}

	if (jumpPressed && _internalState.airTimeSinceJump < DOUBLEJUMP_MAX_DELAY) {
		Vec dodgeInput = Vec(QuantizeAxisInput(-controls.pitch), QuantizeAxisInput(controls.yaw), QuantizeAxisInput(controls.roll));
		float inputMagnitude = fabsf(dodgeInput.x) + fabsf(dodgeInput.y) + fabsf(dodgeInput.z);
		bool isFlipInput = inputMagnitude >= config.dodgeDeadzone;

		bool canUse = (!_internalState.isAutoFlipping && !_internalState.hasDoubleJumped && !_internalState.hasFlipped)
			|| (isFlipInput ? mutatorConfig.unlimitedFlips : mutatorConfig.unlimitedDoubleJumps);

		if (canUse) {
			_internalState.hasJumped = true;

			if (isFlipInput) {
				_internalState.flipTime = 0;
				_internalState.hasFlipped = true;
				_internalState.isFlipping = true;

				Vec dodgeDir = dodgeInput;
				dodgeDir.y += dodgeDir.z;
				if (fabsf(dodgeDir.x) < 0.1f && fabsf(dodgeDir.y) < 0.1f) {
					dodgeDir = Vec(0, 0, 0);
				} else {
					dodgeDir.z = 0;
					dodgeDir = dodgeDir.Normalized();
				}

				Vec deadzoneDodgeDir = Vec(fabsf(dodgeDir.x) < 0.1f ? 0 : dodgeDir.x, fabsf(dodgeDir.y) < 0.1f ? 0 : dodgeDir.y, 0);

				if (deadzoneDodgeDir.LengthSq() > FLT_EPSILON * FLT_EPSILON) {
					_internalState.flipRelTorque = Vec(-dodgeDir.y, dodgeDir.x, 0);

					bool shouldDodgeBackwards;
					if (fabsf(forwardSpeed_UU) < 100.f) {
						shouldDodgeBackwards = std::signbit(deadzoneDodgeDir.x);
					} else {
						shouldDodgeBackwards = std::signbit(deadzoneDodgeDir.x) != std::signbit(forwardSpeed_UU);
					}

					float maxSpeedScaleX = shouldDodgeBackwards ? FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE : FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE;
					float forwardSpeedRatio = fabsf(forwardSpeed_UU) / CAR_MAX_SPEED;

					float mass = mutatorConfig.carMass;
					float dodgeImp = FLIP_INITIAL_VEL_SCALE * mass;
					float dodgeImpX = shouldDodgeBackwards ? nearbyintf(dodgeImp * FLIP_BACKWARD_IMPULSE_SCALE_X) : dodgeImp;
					Vec initialDodgeVel = Vec(deadzoneDodgeDir.x * dodgeImpX, deadzoneDodgeDir.y * dodgeImp, 0);
					initialDodgeVel.x *= ((maxSpeedScaleX - 1) * forwardSpeedRatio) + 1;
					initialDodgeVel.y *= ((FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE - 1) * forwardSpeedRatio) + 1;

					btVector3 forward = _rigidBody.getWorldTransform().getBasis().getColumn(0);
					Vec forwardDir2D = Vec(forward.x(), forward.y(), 0).Normalized();
					if (forwardDir2D.IsZero()) {
						float yaw = atan2f(forward.y(), forward.x());
						forwardDir2D = Vec(cosf(yaw), sinf(yaw), 0);
					}
					Vec rightDir2D = Vec(-forwardDir2D.y, forwardDir2D.x, 0);
					Vec finalDeltaVel = forwardDir2D * initialDodgeVel.x + rightDir2D * initialDodgeVel.y;

					_rigidBody.m_linearVelocity += (finalDeltaVel * UU_TO_BT) * (1 / mass);
					_LimitVels();
				}
			} else {
				btVector3 upDir = _rigidBody.getWorldTransform().getBasis().getColumn(2);
				_rigidBody.m_linearVelocity += upDir * mutatorConfig.jumpImmediateForce * UU_TO_BT;
				_LimitVels();
				_internalState.hasDoubleJumped = true;
			}
		}
	}

	if (_internalState.isFlipping) {
		bool stillFlipping = _internalState.hasFlipped && _internalState.flipTime < FLIP_TORQUE_TIME;
		_internalState.isFlipping = stillFlipping;
		_internalState.flipTime = stillFlipping ? (_internalState.flipTime + tickTime) : tickTime;
		return !stillFlipping;
	}

	_internalState.flipTime += tickTime;
	return false;
}

void Car::_UpdateAutoFlip(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed) {
	using namespace RLConst;
	constexpr float AUTOFLIP_ANG_VEL_DELTA = 1000, AUTOFLIP_UP_DOT_THRESH = -0.966f, AUTOFLIP_THREE_WHEEL_TIME = 0.2f;

	btMatrix3x3 basis = _rigidBody.getWorldTransform().getBasis();
	if (jumpPressed && _internalState.worldContact.hasContact && _internalState.worldContact.contactNormal.z > 0.707f) {
		float roll = -atan2f(basis[2][1], basis[2][2]); // -atan2(right.z, up.z)
		float absRoll = fabsf(roll);

		btVector3 upDir = basis.getColumn(2);
		if (((btVector3)_internalState.worldContact.contactNormal).dot(upDir) < AUTOFLIP_UP_DOT_THRESH) {
			float flipTime = config.threeWheels ? AUTOFLIP_THREE_WHEEL_TIME : CAR_AUTOFLIP_TIME;
			_internalState.autoFlipTimer = (float)((double)flipTime * ((double)absRoll / M_PI));
			_internalState.autoFlipTorqueScale = (roll > 0) ? 1 : -1;
			_internalState.isAutoFlipping = true;

			_rigidBody.m_linearVelocity += upDir * -CAR_AUTOFLIP_IMPULSE * UU_TO_BT;
		}
	}

	if (_internalState.isAutoFlipping) {
		if (_internalState.autoFlipTimer <= 0) {
			_internalState.isAutoFlipping = false;
			_internalState.autoFlipTimer = 0;
		} else {
			_rigidBody.m_angularVelocity += GetForwardDir() * AUTOFLIP_ANG_VEL_DELTA * _internalState.autoFlipTorqueScale;
			_LimitVels();
			_internalState.autoFlipTimer -= tickTime;
		}
	}
}

void Car::_UpdateAutoRoll(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact) {
	using namespace RLConst;

	bool wheelsHaveWorldContact = false;
	for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++)
		wheelsHaveWorldContact |= _bulletVehicle.m_wheelInfo[i].m_isInContactWithWorld;
	if (!wheelsHaveWorldContact && !_internalState.worldContact.hasContact)
		return;

	btVector3 groundUpDir;
	if (wheelsHaveWorldContact && numWheelsInContact > 0) {
		groundUpDir = _bulletVehicle.getUpwardsDirFromWheelContacts();
	} else {
		groundUpDir = _internalState.worldContact.contactNormal;
	}

	btVector3 groundDownDir = -groundUpDir;

	btVector3
		forwardDir = GetForwardDir(),
		rightDir = GetRightDir(),
		upDir = GetUpDir();
	
	btVector3
		crossRightDir = groundUpDir.cross(forwardDir),
		crossForwardDir = groundDownDir.cross(crossRightDir);

	float
		rightTorqueFactor = 1 - RS_CLAMP(rightDir.dot(crossRightDir), 0, 1),
		forwardTorqueFactor = 1 - RS_CLAMP(forwardDir.dot(crossForwardDir), 0, 1);

	Vec
		torqueDirRight = forwardDir * (rightDir.dot(groundUpDir) >= 0 ? -1 : 1),
		torqueDirForward = rightDir * (forwardDir.dot(groundUpDir) >= 0 ? 1 : -1);

	Vec torqueRight = torqueDirRight * rightTorqueFactor;
	Vec torqueForward = torqueDirForward * forwardTorqueFactor;

	_rigidBody.applyCentralForce(groundDownDir * RLConst::CAR_AUTOROLL_FORCE * UU_TO_BT * CAR_MASS_BT);
	_rigidBody.applyTorque(_rigidBody.m_invInertiaTensorWorld.inverse() * (torqueForward + torqueRight) * RLConst::CAR_AUTOROLL_TORQUE);
}

RS_NS_END
