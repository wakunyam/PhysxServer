#pragma once
#include "PxSimulationEventCallback.h"
#include <vector>
#include <mutex>
using namespace physx;

enum FilterGroup {
	ePlayer = (1 << 0),
	eMONSTER = (1 << 1),
	eBULLET = (1 << 2),
	eBACKGROUND = (1 << 3),
	eSTUFF = (1 << 4)
};

struct UserData {
	int id;
	int hp;
	int owner_id;
	FilterGroup objType;
};

class MySimulationEventCallback : public PxSimulationEventCallback
{	
public:
	std::vector<PxActor*> actorContainer;
	std::mutex containerLock;
	std::vector<PxActor*> removedActors;
	std::mutex removedActorsLock;

	// Implements PxSimulationEventCallback
	virtual void							onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs);
	virtual void							onTrigger(PxTriggerPair* pairs, PxU32 count) {};
	virtual void							onConstraintBreak(PxConstraintInfo*, PxU32) {}
	virtual void							onWake(PxActor**, PxU32);
	virtual void							onSleep(PxActor**, PxU32);
	virtual void							onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) {}

	int getContainerSize();
	PxActor* getActor(int id);
};
