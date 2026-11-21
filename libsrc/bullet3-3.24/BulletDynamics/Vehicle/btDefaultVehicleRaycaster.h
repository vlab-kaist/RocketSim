/*
 * Copyright (c) 2005 Erwin Coumans https://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "../Dynamics/btRigidBody.h"
#include "../ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btDynamicsWorld;
#include "../../LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "../Dynamics/btActionInterface.h"
#include "../../BulletCollision/CollisionDispatch/btCollisionWorld.h"

class btDefaultVehicleRaycaster : public btVehicleRaycaster
{
public:
	btDynamicsWorld* m_dynamicsWorld;

	// RocketSim: hits closer than this (squared) to the ray start are ignored, so that a
	// graze off the contact patch does not hide a farther hit on the same ray
	btScalar m_minHitDistSq = 0;

	// RocketSim: what castRays() keeps for each ray between calls (btCollisionWorld::rsWheelRays()); only an
	// acceleration, the results are the same without it
	btCollisionWorld::RSWheelRayCache m_rsRayCaches[btCollisionWorld::RS_WHEEL_RAYS_MAX];

	btDefaultVehicleRaycaster() {}

	btDefaultVehicleRaycaster(btDynamicsWorld* world)
		: m_dynamicsWorld(world)
	{
	}

	virtual void* castRay(const btVector3& from, const btVector3& to, const btCollisionObject* ignoreObj, btVehicleRaycasterResult& result);
	virtual void castRays(const btVector3* from, const btVector3* to, int numRays, const btCollisionObject* ignoreObj, btVehicleRaycasterResult* results, void** hitObjects);
};

#endif  //BT_RAYCASTVEHICLE_H
