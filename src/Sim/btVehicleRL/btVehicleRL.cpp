#include "btVehicleRL.h"
#include "../../RSProf.h"
#include "../../RLConst.h"
#include "../../BulletLink.h"
#define ROLLING_INFLUENCE_FIX

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/ConstraintSolver/btContactConstraint.h"

RS_NS_START

btVehicleRL::btVehicleRL(
	const btVehicleTuning& tuning, 
	btRigidBody* chassis, btVehicleRaycaster* raycaster, btDynamicsWorld* world, 
	int addedRayCollisionMask
) : m_vehicleRaycaster(raycaster), m_pitchControl(0),  m_dynamicsWorld(world) {
	m_chassisBody = chassis;
	m_indexRightAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;
	m_vehicleRaycaster->addedFilterMask = addedRayCollisionMask;
	defaultInit(tuning);
}

void btVehicleRL::defaultInit(const btVehicleTuning& tuning) {
	(void)tuning;
	m_steeringValue = 0;
}

btVehicleRL::~btVehicleRL() {
}

//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
btWheelInfoRL& btVehicleRL::addWheel(const btVector3& connectionPointCS, const btVector3& wheelDirectionCS0, const btVector3& wheelAxleCS, float suspensionRestLength, float wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel) {
	btWheelInfoConstructionInfo ci;

	ci.m_chassisConnectionCS = connectionPointCS;
	ci.m_wheelDirectionCS = wheelDirectionCS0;
	ci.m_wheelAxleCS = wheelAxleCS;
	ci.m_suspensionRestLength = suspensionRestLength;
	ci.m_wheelRadius = wheelRadius;
	ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
	ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
	ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
	ci.m_frictionSlip = tuning.m_frictionSlip;
	ci.m_bIsFrontWheel = isFrontWheel;
	ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
	ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

	m_wheelInfo.push_back(btWheelInfoRL(ci));

	btWheelInfoRL& wheel = m_wheelInfo[getNumWheels() - 1];

	updateWheelTransformsWS(wheel);
	updateWheelTransform(getNumWheels() - 1);
	return wheel;
}

const btTransform& btVehicleRL::getWheelTransformWS(int wheelIndex) const {
	btAssert(wheelIndex < getNumWheels());
	const btWheelInfoRL& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;
}

// See: I20 or I21
void btVehicleRL::updateWheelTransform(int wheelIndex) {
	updateWheelTransformsWS(m_wheelInfo[wheelIndex]);
	updateWheelBasis(wheelIndex);
}

// Rebuilds the wheel's world transform (steering included) from the current
// world-space wheel info, without touching the contact state
void btVehicleRL::updateWheelBasis(int wheelIndex) {
	btWheelInfoRL& wheel = m_wheelInfo[wheelIndex];
	btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
	btVector3 fwd = up.cross(right);
	fwd = fwd.normalize();
	//	up = right.cross(fwd);
	//	up.normalize();

	btMatrix3x3 basis2;
	basis2[0][m_indexRightAxis] = -right[0];
	basis2[1][m_indexRightAxis] = -right[1];
	basis2[2][m_indexRightAxis] = -right[2];

	basis2[0][m_indexUpAxis] = up[0];
	basis2[1][m_indexUpAxis] = up[1];
	basis2[2][m_indexUpAxis] = up[2];

	basis2[0][m_indexForwardAxis] = fwd[0];
	basis2[1][m_indexForwardAxis] = fwd[1];
	basis2[2][m_indexForwardAxis] = fwd[2];
	if (wheel.m_steerAngle == 0) {
		// No steering rotation (rear wheels, or driving straight)
		wheel.m_worldTransform.setBasis(basis2);
	} else {
		// Same as btQuaternion(up, m_steerAngle), with the trig only redone when the angle changes
		if (wheel.m_steerAngle != wheel.m_cachedSteerAngle) {
			wheel.m_cachedSteerAngle = wheel.m_steerAngle;
			wheel.m_steerSinHalf = btSin(wheel.m_steerAngle * btScalar(0.5));
			wheel.m_steerCosHalf = btCos(wheel.m_steerAngle * btScalar(0.5));
		}
		btScalar s = wheel.m_steerSinHalf / up.length();
		btQuaternion steeringOrn(up.x() * s, up.y() * s, up.z() * s, wheel.m_steerCosHalf);
		wheel.m_worldTransform.setBasis(btMatrix3x3(steeringOrn) * basis2);
	}
	wheel.m_worldTransform.setOrigin(
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength);
}

void btVehicleRL::resetSuspension() {
	int i;
	for (i = 0; i < m_wheelInfo.size(); i++) {
		btWheelInfoRL& wheel = m_wheelInfo[i];
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = 0;

		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = 1;
		wheel.m_extraPushback = 0;
	}
}

// See: I20 or I21
void btVehicleRL::updateWheelTransformsWS(btWheelInfoRL& wheel) {
	wheel.m_raycastInfo.m_isInContact = false;
	wheel.m_isInContactWithWorld = false;

	btTransform chassisTrans = getChassisWorldTransform();
	wheel.m_raycastInfo.m_hardPointWS = chassisTrans(wheel.m_chassisConnectionPointCS);
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

void btVehicleRL::getRay(btWheelInfoRL& wheel, btVector3& source, btVector3& target) {
	updateWheelTransformsWS(wheel);

	float suspensionTravel = wheel.m_maxSuspensionTravelCm / 100;
	// As in RocketSim v3, the ray is not shortened by SUSPENSION_SUBTRACTION (only the pushback threshold is)
	float realRayLength = wheel.getSuspensionRestLength() + suspensionTravel + wheel.m_wheelsRadius;

	// See: I21
	source = wheel.m_raycastInfo.m_hardPointWS;
	target = source + (wheel.m_raycastInfo.m_wheelDirectionWS * realRayLength);
	wheel.m_raycastInfo.m_contactPointWS = target;
	wheel.m_raycastInfo.m_groundObject = NULL;
}

float btVehicleRL::rayCast(btWheelInfoRL& wheel, btCollisionObject* object, const btVehicleRaycaster::btVehicleRaycasterResult& rayResults) {
	float depth = -1;

	float suspensionTravel = wheel.m_maxSuspensionTravelCm / 100;
	float realRayLength = wheel.getSuspensionRestLength() + suspensionTravel + wheel.m_wheelsRadius;

	// See: I23
	if (object) {
		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;
		float fraction = rayResults.m_distFraction;
		depth = realRayLength * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;
		wheel.m_isInContactWithWorld = object->isStaticObject();

		wheel.m_raycastInfo.m_groundObject = object;

		float wheelTraceLenSq = (wheel.m_raycastInfo.m_hardPointWS - wheel.m_raycastInfo.m_contactPointWS).dot(getUpVector());
		wheel.m_raycastInfo.m_suspensionLength = wheelTraceLenSq - wheel.m_wheelsRadius;

		// Only the maximum suspension travel is clamped
		float maxSuspensionLen = wheel.getSuspensionRestLength() + suspensionTravel;
		wheel.m_raycastInfo.m_suspensionLength = RS_MIN(wheel.m_raycastInfo.m_suspensionLength, maxSuspensionLen);

		float denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(getUpVector());

		btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - m_chassisBody->getWorldTransform().m_origin;
		wheel.m_velAtContactPoint = m_chassisBody->getVelocityInLocalPoint(relpos);

		float projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(wheel.m_velAtContactPoint);

		if (denominator > 0.1) {
			float inv = 1 / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		} else {
			// Denominator is too tiny to be meaningful
			wheel.m_suspensionRelativeVelocity = 0;
			wheel.m_clippedInvContactDotSuspension = 10;
		}

		bool isCarHit = object->getUserIndex() == BT_USERINFO_TYPE_CAR;
		if (object->isStaticObject() || isCarHit) { // Compute m_extraPushback when colliding with the world or another car
			float rayPushbackThresh = (wheel.m_suspensionRestLength1 + wheel.m_wheelsRadius) - RLConst::BTVehicle::SUSPENSION_SUBTRACTION;
			if (wheelTraceLenSq < rayPushbackThresh) {

				float wheelTraceDistDelta = wheelTraceLenSq - rayPushbackThresh;
				float collisionResult = resolveSingleCollision(
					m_chassisBody,
					object,
					rayResults.m_hitPointInWorld,
					rayResults.m_hitNormalInWorld,
					m_dynamicsWorld->getSolverInfo(),
					wheelTraceDistDelta,
					false
				);

				wheel.m_extraPushback = collisionResult / getNumWheels();
			}
		}

		// Reactions on a dynamic body under the wheel (as in RocketSim v3), only in a real tick
		if (!object->isStaticObject() && m_applyHitEffects) {
			btRigidBody* hitBody = (btRigidBody*)btRigidBody::upcast(object);
			if (hitBody) {
				btVector3 hitOffset = wheel.m_raycastInfo.m_contactPointWS - hitBody->getWorldTransform().getOrigin();
				if (isCarHit && wheel.m_extraPushback > 0)
					hitBody->applyImpulse(-wheel.m_raycastInfo.m_contactNormalWS * (wheel.m_extraPushback * getNumWheels()), hitOffset);
				hitBody->applyImpulse(-wheel.m_raycastInfo.m_contactNormalWS, hitOffset);
			}
		}

	} else {
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength() + suspensionTravel;
		wheel.m_suspensionRelativeVelocity = 0;
		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = 1;
		wheel.m_extraPushback = 0;
	}

	return depth;
}

const btTransform& btVehicleRL::getChassisWorldTransform() const {
	return getRigidBody()->getCenterOfMassTransform();
}

void btVehicleRL::updateVehicleFirst(float step) {
	// The wheel bases are rebuilt after this tick's steering is set (Car::_UpdateWheelFriction);
	// the traces only need the world-space hard points, which rayCast() updates itself

	// A zero step only refreshes the contacts (after a state set) and must not push anything
	m_applyHitEffects = step > 0;

	// The traces only depend on where the bodies are, which the reactions on hit bodies don't change,
	// so all wheels are traced together, then handled in order
	constexpr int MAX_WHEELS = 8;
	int numWheels = m_wheelInfo.size();
	btAssert(numWheels <= MAX_WHEELS);
	btVector3 sources[MAX_WHEELS], targets[MAX_WHEELS];
	btVehicleRaycaster::btVehicleRaycasterResult rayResults[MAX_WHEELS];
	void* hitObjects[MAX_WHEELS];
	{ RS_PROF(28, "    getRay");
	for (int i = 0; i < numWheels; i++)
		getRay(m_wheelInfo[i], sources[i], targets[i]);
	}

	btAssert(m_vehicleRaycaster);
	{ RS_PROF(29, "    castRays");
	m_vehicleRaycaster->castRays(sources, targets, numWheels, m_chassisBody, rayResults, hitObjects);
	}
	RS_PROF(30, "    post");

	for (int i = 0; i < numWheels; i++)
		rayCast(m_wheelInfo[i], (btCollisionObject*)hitObjects[i], rayResults[i]);
}

// The friction impulses are determined here, after the car has set this tick's
// steering, engine force and brake, so controls take effect on the tick they are given
void btVehicleRL::updateVehicleSecond(float step) {
	// Expects the wheel bases to be current (Car::_UpdateWheelFriction rebuilds them)
	updateSuspension(step);
	calcFrictionImpulses(step);
	applyFrictionImpulses(step);
}

void btVehicleRL::setSteeringValue(float steering, int wheel) {
	btAssert(wheel >= 0 && wheel < getNumWheels());

	btWheelInfoRL& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}

float btVehicleRL::getSteeringValue(int wheel) const {
	return getWheelInfo(wheel).m_steering;
}

void btVehicleRL::applyEngineForce(float force, int wheel) {
	btAssert(wheel >= 0 && wheel < getNumWheels());
	btWheelInfoRL& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}

const btWheelInfoRL& btVehicleRL::getWheelInfo(int index) const {
	btAssert((index >= 0) && (index < getNumWheels()));

	return m_wheelInfo[index];
}

btWheelInfoRL& btVehicleRL::getWheelInfo(int index) {
	btAssert((index >= 0) && (index < getNumWheels()));

	return m_wheelInfo[index];
}

void btVehicleRL::setBrake(float brake, int wheelIndex) {
	btAssert((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}

// See: I24
void btVehicleRL::updateSuspension(float deltaTime) {
	
	for (int i = 0; i < getNumWheels(); i++) {
		btWheelInfoRL& wheel_info = m_wheelInfo[i];

		if (wheel_info.m_raycastInfo.m_isInContact) {
			float force =
				(wheel_info.getSuspensionRestLength() - wheel_info.m_raycastInfo.m_suspensionLength)
				* wheel_info.m_suspensionStiffness * wheel_info.m_clippedInvContactDotSuspension;
			
			float dampingVelScale = (wheel_info.m_suspensionRelativeVelocity < 0) ? wheel_info.m_wheelsDampingCompression : wheel_info.m_wheelsDampingRelaxation;

			wheel_info.m_wheelsSuspensionForce = force - (dampingVelScale * wheel_info.m_suspensionRelativeVelocity);
			wheel_info.m_wheelsSuspensionForce *= wheel_info.m_suspensionForceScale;

			// RL never uses downwards suspension forces
			if (wheel_info.m_wheelsSuspensionForce < 0)
				wheel_info.m_wheelsSuspensionForce = 0;

		} else {
			wheel_info.m_wheelsSuspensionForce = 0;
		}
	}

	// Impulses are applied in the game's wheel order (front right, front left, back right, back left)
	static const int COMBINE_ORDER[4] = { 1, 0, 3, 2 };
	for (int k = 0; k < getNumWheels(); k++) {
		btWheelInfoRL& wheel = m_wheelInfo[getNumWheels() == 4 ? COMBINE_ORDER[k] : k];
		if (wheel.m_wheelsSuspensionForce != 0) {
			btVector3 contactPointOffset = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();
			float baseForceScale = (wheel.m_wheelsSuspensionForce * deltaTime) + wheel.m_extraPushback;
			btVector3 force = wheel.m_raycastInfo.m_contactNormalWS * baseForceScale;
			m_chassisBody->applyImpulse(force, contactPointOffset);
		}
	}
}

// See: I25
void btVehicleRL::calcFrictionImpulses(float timeStep) {

	float frictionScale = m_chassisBody->getMass() / 3;

	// Determine impulses
	for (int i = 0; i < m_wheelInfo.size(); i++) {
		btWheelInfoRL& wheel = m_wheelInfo[i];

		btRigidBody* groundObject = (btRigidBody*)wheel.m_raycastInfo.m_groundObject;
		if (groundObject) {
			// Axle direction (includes steering turn), not projected onto the contact plane
			btVector3 axleDir = wheel.m_worldTransform.getBasis().getColumn(m_indexRightAxis).safeNormalized();

			btVector3 surfNormalWS = wheel.m_raycastInfo.m_contactNormalWS;

			// Wheel forwards direction
			btVector3 forwardDir = surfNormalWS.cross(axleDir).safeNormalized();

			float sideImpulse;

			// Get sideways friction force, always against a fixed ground (even on the ball or a car).
			// Same as resolveSingleBilateral() against getFixedBody(), whose terms are all zero, but without
			// touching that shared static body (writing to it from every thread stalls multi-threaded use).
			{
				btVector3 relPos1 = wheel.m_raycastInfo.m_contactPointWS - m_chassisBody->getCenterOfMassPosition();
				btVector3 vel1 = m_chassisBody->getVelocityInLocalPoint(relPos1);
				btVector3 aJ = m_chassisBody->getCenterOfMassTransform().getBasis().transpose() * relPos1.cross(axleDir);
				btVector3 minvJt0 = m_chassisBody->getInvInertiaDiagLocal() * aJ;
				btScalar jacDiagAB = m_chassisBody->getInvMass() + minvJt0.dot(aJ);
				btScalar relVel = axleDir.dot(vel1);
				sideImpulse = btScalar(-0.2) * relVel * (btScalar(1) / jacDiagAB);
			}

			float rollingFriction;
			if (wheel.m_engineForce == 0) {
				if (wheel.m_brake) {
					// Simplified variation of calcRollingFriction()
					// Chassis velocity at the contact, from the wheel trace
					float relVel = wheel.m_velAtContactPoint.dot(forwardDir);

					// This will round off small rolling friction amounts when at sub-80 TPS to prevent stuttering (to an extent)
					// TODO: Improve and clarify
					if (timeStep > (1 / 80.f)) {
						float threshold = -(1 / (timeStep * 150.f)) + 0.8f;
						if (abs(relVel) < threshold)
							relVel = 0;
					}

					// TODO: No idea where this number comes from or how it was calculated lol
					constexpr float ROLLING_FRICTION_SCALE_MAGIC = 113.73963f;

					rollingFriction = RS_CLAMP(-relVel * ROLLING_FRICTION_SCALE_MAGIC, -wheel.m_brake, wheel.m_brake);
				} else {
					// Don't apply friction when driving with no brake
					rollingFriction = 0;
				}
			} else {
				// Engine force already accounts for our mass, so we will cancel out the friction scale multiplication at the end
				rollingFriction = -wheel.m_engineForce / frictionScale;
			}

			btVector3 totalFrictionForce = (forwardDir * rollingFriction * wheel.m_longFriction) + (axleDir * sideImpulse * wheel.m_latFriction);
			wheel.m_impulse = totalFrictionForce * frictionScale;
		} else {
			wheel.m_impulse = { 0,0,0 };
		}
	}
}

// See: I25
void btVehicleRL::applyFrictionImpulses(float timeStep) {
	// Apply impulses
	btVector3 upDir = m_chassisBody->getWorldTransform().getBasis().getColumn(m_indexUpAxis);
	static const int COMBINE_ORDER[4] = { 1, 0, 3, 2 };
	for (int k = 0; k < m_wheelInfo.size(); k++) {
		btWheelInfoRL& wheel = m_wheelInfo[m_wheelInfo.size() == 4 ? COMBINE_ORDER[k] : k];
		if (!wheel.m_impulse.isZero()) {
			btVector3 wheelContactOffset = wheel.m_raycastInfo.m_contactPointWS - m_chassisBody->getWorldTransform().getOrigin();
			float contactUpDot = upDir.dot(wheelContactOffset);
			btVector3 wheelRelPos = wheelContactOffset - upDir * contactUpDot;
			m_chassisBody->applyImpulse(wheel.m_impulse * timeStep, wheelRelPos);
		}
	}
}

btVector3 btVehicleRL::getUpwardsDirFromWheelContacts() {
	btVector3 sumContactDir = btVector3(0, 0, 0);
	for (int i = 0; i < m_wheelInfo.size(); i++)
		if (m_wheelInfo[i].m_raycastInfo.m_isInContact)
			sumContactDir += m_wheelInfo[i].m_raycastInfo.m_contactNormalWS;

	if (sumContactDir == btVector3(0, 0, 0)) {
		// No wheels had world contact, just return basic upward direction
		return this->getUpVector();
	} else {
		// Average normal of contact
		return sumContactDir.safeNormalized();
	}
}

float btVehicleRL::getForwardSpeed() {
	return m_chassisBody->getLinearVelocity().dot(getForwardVector());
}

RS_NS_END