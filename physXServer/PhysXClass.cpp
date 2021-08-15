#include "PhysXClass.h"

PhysXClass* PhysXClass::instance = nullptr;

PhysXClass* PhysXClass::getInstance()
{
	if (instance == nullptr)
		instance = new PhysXClass();

	return instance;
}

PxFilterFlags myFilterShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where 
	// the filtermask of A contains the ID of B and vice versa.
	if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eDETECT_CCD_CONTACT | PxPairFlag::eNOTIFY_TOUCH_CCD;

	return PxFilterFlag::eDEFAULT;
}

PhysXClass::PhysXClass()
{
	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mAllocator, mErrorCallback);

	mPvd = PxCreatePvd(*mFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	mPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
	PxInitExtensions(*mPhysics, mPvd);

	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, PxCookingParams(PxTolerancesScale()));

	PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	mDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = mDispatcher;
	sceneDesc.filterShader = myFilterShader;
	sceneDesc.simulationEventCallback = &mSimulationEventCallback;
	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDesc.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDesc.dynamicStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	mScene = mPhysics->createScene(sceneDesc);
	mScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1);
	mScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1);
	sceneTwo = mPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = mScene->getScenePvdClient();
	if (pvdClient) {
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	/*PxRigidStatic* groundPlane = PxCreatePlane(*mPhysics, PxPlane(0, 1, 0, 0), *mMaterial);
	
	mScene->addActor(*groundPlane);*/
}

PhysXClass::~PhysXClass()
{
	if (mScene) {
		mScene->release();
		mScene = NULL;
	}
	if (mDispatcher) {
		mDispatcher->release();
		mDispatcher = NULL;
	}
	if (mPhysics) {
		mPhysics->release();
		mPhysics = NULL;
	}
	if (mPvd) {
		PxPvdTransport* transport = mPvd->getTransport();
		mPvd->release();	mPvd = NULL;
		transport->release();
	}
	if (mFoundation) {
		mFoundation->release();
		mFoundation = NULL;
	}
}

void PhysXClass::stepPhysics(float elapsedTime)
{
	mScene->simulate(elapsedTime);
	sceneTwo->simulate(elapsedTime);

	mScene->fetchResults(true);
	sceneTwo->fetchResults(true);
}
