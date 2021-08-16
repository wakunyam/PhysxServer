#include "MySimulationEventCallback.h"
#include <algorithm>
#include <iostream>
#include "PxPhysicsAPI.h"

void MySimulationEventCallback::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
{
    for (PxU32 i = 0; i < nbPairs; i++)
    {
        UserData* userData0 = (UserData*)pairHeader.actors[0]->userData;
        UserData* userData1 = (UserData*)pairHeader.actors[1]->userData;

        if (userData0 == nullptr || userData1 == nullptr) continue;

        if (userData0->objType == FilterGroup::eMONSTER && userData1->objType == FilterGroup::eBULLET) {
            //removedActorsLock.lock();
            userData0->hp -= 1;
            if (userData0->hp <= 0) {
                if (std::find(removedActors.begin(), removedActors.end(), pairHeader.actors[0]) == removedActors.end())
                    removedActors.emplace_back(pairHeader.actors[0]);
            }
            if (std::find(removedActors.begin(), removedActors.end(), pairHeader.actors[1]) == removedActors.end())
                removedActors.emplace_back(pairHeader.actors[1]);
            //removedActorsLock.unlock();
        }
        else if (userData0->objType == FilterGroup::eBULLET) {
            //removedActorsLock.lock();
            if (std::find(removedActors.begin(), removedActors.end(), pairHeader.actors[0]) == removedActors.end())
                removedActors.emplace_back(pairHeader.actors[0]);
            if (userData1->objType == FilterGroup::eMONSTER) {
                userData1->hp -= 1;
                if (userData1->hp <= 0) {
                    if (std::find(removedActors.begin(), removedActors.end(), pairHeader.actors[1]) == removedActors.end())
                        removedActors.emplace_back(pairHeader.actors[1]);
                }
            }
            //removedActorsLock.unlock();
        }
        else if (userData0->objType == FilterGroup::eBACKGROUND && userData1->objType == FilterGroup::eBULLET) {
            //removedActorsLock.lock();
            if (std::find(removedActors.begin(), removedActors.end(), pairHeader.actors[1]) == removedActors.end())
                removedActors.emplace_back(pairHeader.actors[1]);
            //removedActorsLock.unlock();

        }
        else if (userData0->objType == FilterGroup::eSTUFF && userData1->objType == FilterGroup::eBULLET) {
            //removedActorsLock.lock();
            if (std::find(removedActors.begin(), removedActors.end(), pairHeader.actors[1]) == removedActors.end())
                removedActors.emplace_back(pairHeader.actors[1]);
            //removedActorsLock.unlock();
        }
    }
}

void MySimulationEventCallback::onWake(PxActor** actors, PxU32 count)
{
    if (count < 1) return;

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
	//std::lock_guard<std::mutex> l{ containerLock };
	auto iter = std::find_if(actorContainer.begin(), actorContainer.end(), [id](PxActor* actor) {
		return *(int*)actor->userData == id;
		});
	if (iter != actorContainer.end())
		return *iter;
	else
		return nullptr;
}
