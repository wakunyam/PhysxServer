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
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}

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

	PxSceneDesc sceneDescMap2(mPhysics->getTolerancesScale());
	sceneDescMap2.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	mDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDescMap2.cpuDispatcher = mDispatcher;
	sceneDescMap2.filterShader = myFilterShader;
	sceneDescMap2.simulationEventCallback = &mSimulationEventCallbackMap2;
	sceneDescMap2.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDescMap2.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDescMap2.dynamicStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	mMap2Scene = mPhysics->createScene(sceneDescMap2);
	mMap2Scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1);
	mMap2Scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1);

	PxSceneDesc sceneDescMap4(mPhysics->getTolerancesScale());
	sceneDescMap4.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	mDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDescMap4.cpuDispatcher = mDispatcher;
	sceneDescMap4.filterShader = myFilterShader;
	sceneDescMap4.simulationEventCallback = &mSimulationEventCallbackMap4;
	sceneDescMap4.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDescMap4.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDescMap4.dynamicStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	mMap4Scene = mPhysics->createScene(sceneDescMap4);

	PxSceneDesc sceneDescMap5(mPhysics->getTolerancesScale());
	sceneDescMap5.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	mDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDescMap5.cpuDispatcher = mDispatcher;
	sceneDescMap5.filterShader = myFilterShader;
	sceneDescMap5.simulationEventCallback = &mSimulationEventCallbackMap5;
	sceneDescMap5.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDescMap5.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDescMap5.dynamicStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	mMap5Scene = mPhysics->createScene(sceneDescMap5);

	PxSceneDesc sceneDescMapFinal(mPhysics->getTolerancesScale());
	sceneDescMapFinal.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	mDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDescMapFinal.cpuDispatcher = mDispatcher;
	sceneDescMapFinal.filterShader = myFilterShader;
	sceneDescMapFinal.simulationEventCallback = &mSimulationEventCallbackMapFinal;
	sceneDescMapFinal.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDescMapFinal.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDescMapFinal.dynamicStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	mMapFinalScene = mPhysics->createScene(sceneDescMapFinal);

	PxPvdSceneClient* pvdClient = mMap2Scene->getScenePvdClient();
	if (pvdClient) {
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	/*PxRigidStatic* groundPlane = PxCreatePlane(*mPhysics, PxPlane(0, 1, 0, 0), *mMaterial);
	
	mMap2Scene->addActor(*groundPlane);*/
}

PhysXClass::~PhysXClass()
{
	if (mMap2Scene) {
		mMap2Scene->release();
		mMap2Scene = NULL;
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

void PhysXClass::stepPhysics(float elapsedTime, int scene)
{
	if (scene == 2) {
		mMap2Scene->simulate(elapsedTime);
		mMap2Scene->fetchResults(true);
	}
	else if (scene == 4) {
		mMap4Scene->simulate(elapsedTime);
		mMap4Scene->fetchResults(true);
	}
	else if (scene == 5) {
		mMap5Scene->simulate(elapsedTime);
		mMap5Scene->fetchResults(true);
	}
	else if (scene == 6) {
		mMapFinalScene->simulate(elapsedTime);
		mMapFinalScene->fetchResults(true);
	}
}
