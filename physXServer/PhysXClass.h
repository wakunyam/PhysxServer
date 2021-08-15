#pragma once
#include "include/PxPhysicsAPI.h"
#include "MySimulationEventCallback.h"
using namespace physx;

#define PVD_HOST "127.0.0.1"

class PhysXClass
{
	static PhysXClass* instance;
	PhysXClass();
public:
	PxDefaultAllocator		mAllocator;
	PxDefaultErrorCallback	mErrorCallback;

	PxFoundation* mFoundation = NULL;
	PxPhysics* mPhysics = NULL;
	PxCooking* mCooking = NULL;

	PxDefaultCpuDispatcher* mDispatcher = NULL;
	PxScene* mScene = NULL;
	PxScene* sceneTwo = NULL;

	PxMaterial* mMaterial = NULL;

	PxPvd* mPvd = NULL;

	MySimulationEventCallback mSimulationEventCallback;

	static PhysXClass* getInstance();
	~PhysXClass();
	void stepPhysics(float elapsedTime);
};


