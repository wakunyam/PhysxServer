#include "MySimulationEventCallback.h"
#include <algorithm>
#include <iostream>
#include "PxPhysicsAPI.h"

void MySimulationEventCallback::onWake(PxActor** actors, PxU32 count)
{
	std::lock_guard<std::mutex> l{ containerLock };
	for (int i = 0; i < count; ++i) {
		actorContainer.emplace_back(actors[i]);
	}
	//std::cout << "container size: " << getContainerSize() << std::endl;
}

void MySimulationEventCallback::onSleep(PxActor** actors, PxU32 count)
{
	std::lock_guard<std::mutex> l{ containerLock };
	for (int i = 0; i < count; ++i) {
		std::remove_if(actorContainer.begin(), actorContainer.end(), [&actors, i](PxActor* actor) {
			return actor == actors[i];
			});
	}
	actorContainer.erase(actorContainer.end() - count, actorContainer.end());
	//std::cout << "container size: " << getContainerSize() << std::endl;
}

int MySimulationEventCallback::getContainerSize()
{
	return actorContainer.size();
}

PxActor* MySimulationEventCallback::getActor(int id)
{
	std::lock_guard<std::mutex> l{ containerLock };
	auto iter = std::find_if(actorContainer.begin(), actorContainer.end(), [id](PxActor* actor) {
		return *(int*)actor->userData == id;
		});
	if (iter != actorContainer.end())
		return *iter;
	else
		return nullptr;
}
