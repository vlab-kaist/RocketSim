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

#include "../../LinearMath/btVector3.h"
#include <new>
#include "btDefaultVehicleRaycaster.h"

#include "../ConstraintSolver/btJacobianEntry.h"
#include "../../LinearMath/btQuaternion.h"
#include "../Dynamics/btDynamicsWorld.h"
#include "btVehicleRaycaster.h"
#include "btWheelInfo.h"
#include "../../LinearMath/btMinMax.h"
#include "../ConstraintSolver/btContactConstraint.h"

#define ROLLING_INFLUENCE_FIX

btRigidBody& btActionInterface::getFixedBody()
{
	static btRigidBody s_fixed(0, 0, 0);
	s_fixed.setMassProps(btScalar(0.), btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
	return s_fixed;
}

namespace
{
struct MinDistRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
	btScalar m_minDistSq;
	MinDistRayResultCallback(const btVector3& from, const btVector3& to, const btCollisionObject* ignoreObj, btScalar minDistSq, int filterGroup)
		: ClosestRayResultCallback(from, to, ignoreObj), m_minDistSq(minDistSq)
	{
		m_collisionFilterGroup = filterGroup;
		m_collisionFilterMask = btBroadphaseProxy::AllFilter;
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		btVector3 hitPoint;
		hitPoint.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
		if ((hitPoint - m_rayFromWorld).length2() < m_minDistSq)
			return m_closestHitFraction;
		return ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
	}

	void* getResult(btVehicleRaycaster::btVehicleRaycasterResult& result) const
	{
		if (hasHit())
		{
			const btRigidBody* body = btRigidBody::upcast(m_collisionObject);
			if (body && body->hasContactResponse())
			{
				result.m_hitPointInWorld = m_hitPointWorld;
				result.m_hitNormalInWorld = m_hitNormalWorld;
				result.m_hitNormalInWorld.normalize();
				result.m_distFraction = m_closestHitFraction;
				return (void*)body;
			}
		}
		return 0;
	}
};
}  // namespace

void* btDefaultVehicleRaycaster::castRay(const btVector3& from, const btVector3& to, const btCollisionObject* ignoreObj, btVehicleRaycasterResult& result)
{
	MinDistRayResultCallback rayCallback(from, to, ignoreObj, m_minHitDistSq, btBroadphaseProxy::CharacterFilter | addedFilterMask);
	m_dynamicsWorld->rayTest(from, to, rayCallback);
	return rayCallback.getResult(result);
}

void btDefaultVehicleRaycaster::castRays(const btVector3* from, const btVector3* to, int numRays, const btCollisionObject* ignoreObj, btVehicleRaycasterResult* results, void** hitObjects)
{
	constexpr int MAX_RAYS = 8;

	// RocketSim: the direct path, with the same results
	if (numRays <= MAX_RAYS)
	{
		btCollisionWorld::RSRayHit hits[MAX_RAYS];
		if (m_dynamicsWorld->rsWheelRays(from, to, numRays, btBroadphaseProxy::CharacterFilter | addedFilterMask, btBroadphaseProxy::AllFilter, ignoreObj, m_minHitDistSq, hits))
		{
			for (int i = 0; i < numRays; i++)
			{
				hitObjects[i] = 0;
				// MinDistRayResultCallback::getResult()
				if (hits[i].object)
				{
					const btRigidBody* body = btRigidBody::upcast(hits[i].object);
					if (body && body->hasContactResponse())
					{
						results[i].m_hitPointInWorld = hits[i].point;
						results[i].m_hitNormalInWorld = hits[i].normal;
						results[i].m_hitNormalInWorld.normalize();
						results[i].m_distFraction = hits[i].fraction;
						hitObjects[i] = (void*)body;
					}
				}
			}
			return;
		}
	}

	if (numRays > MAX_RAYS)
	{
		btVehicleRaycaster::castRays(from, to, numRays, ignoreObj, results, hitObjects);
		return;
	}

	alignas(MinDistRayResultCallback) unsigned char storage[MAX_RAYS][sizeof(MinDistRayResultCallback)];
	btCollisionWorld::RayResultCallback* callbacks[MAX_RAYS];
	for (int i = 0; i < numRays; i++)
		callbacks[i] = new (storage[i]) MinDistRayResultCallback(from[i], to[i], ignoreObj, m_minHitDistSq, btBroadphaseProxy::CharacterFilter | addedFilterMask);

	m_dynamicsWorld->rayTestMulti(from, to, callbacks, numRays);

	for (int i = 0; i < numRays; i++)
	{
		MinDistRayResultCallback* callback = (MinDistRayResultCallback*)callbacks[i];
		hitObjects[i] = callback->getResult(results[i]);
		callback->~MinDistRayResultCallback();
	}
}
