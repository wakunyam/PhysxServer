#define _WINSOCK_DEPRECATED_NO_WARNINGS
#pragma comment(lib, "ws2_32")
#include <winsock2.h>
#include <MSWSock.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <conio.h>
#include <chrono>
#include <vector>
#include "PhysXClass.h"
#include "protocol.h"
#include "JsonParser.h"

using namespace std;
using namespace chrono;
using namespace physx;

constexpr int MAX_BUFF_SIZE = 8192;
constexpr int MIN_BUFF_SIZE = 1024;
constexpr int CLIENT_NUM = 2;

struct OVER_EX {
	WSAOVERLAPPED over;
	WSABUF wsabuf;
	char buf[MAX_BUFF_SIZE];
	int id;
};

struct Player {
	PxController* m_body;
	float body_rX, body_rY, body_rZ, body_rW;
	PxRigidDynamic* m_lHand, * m_rHand;
	PxFixedJoint* m_lHandJoint = nullptr, * m_rHandJoint = nullptr;
};

struct CLIENT {
	SOCKET m_sock;
	OVER_EX m_recv_over;
	char* m_packet_start;
	char* m_recv_start;
	volatile bool conected;
	Player map2Player;
	Player map4Player;
	HandState m_lHandState, m_rHandState;
	bool isVR;
	bool isHost;
	int scene_id;
};

CLIENT clients[CLIENT_NUM];
SOCKET g_lSocket;
OVER_EX g_accept_over;
PhysXClass* gInstance;
JsonParser gParser;
PxRigidDynamic** monsters;

volatile int nbUserOfMap2 = 0;
volatile int nbUserOfMap4 = 0;

// 소켓 함수 오류 출력 후 종료
void err_quit(const char* msg)
{
	LPVOID lpMsgBuf;
	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
		NULL, WSAGetLastError(),
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf, 0, NULL);

	MessageBox(NULL, (LPCTSTR)lpMsgBuf, msg, MB_ICONERROR);
	LocalFree(lpMsgBuf);
	exit(1);
}

// 소켓 함수 오류 출력
void err_display(const char* msg)
{
	LPVOID lpMsgBuf;
	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
		NULL, WSAGetLastError(),
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf, 0, NULL);
	printf("[%s] %s", msg, (char*)lpMsgBuf);
	LocalFree(lpMsgBuf);
}

void CALLBACK send_complete(DWORD err, DWORD bytes, LPWSAOVERLAPPED over, DWORD flags);
void CALLBACK recv_complete(DWORD err, DWORD bytes, LPWSAOVERLAPPED over, DWORD flags);

void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask)
{
	PxFilterData filterData;
	filterData.word0 = filterGroup; // word0 = own ID
	filterData.word1 = filterMask;	// word1 = ID mask to filter pairs that trigger a contact callback;
	const PxU32 numShapes = actor->getNbShapes();
	PxShape** shapes = new PxShape * [numShapes];
	//PxShape** shapes = (PxShape**)SAMPLE_ALLOC(sizeof(PxShape*) * numShapes);
	actor->getShapes(shapes, numShapes);
	for (PxU32 i = 0; i < numShapes; i++)
	{
		PxShape* shape = shapes[i];
		shape->setSimulationFilterData(filterData);
	}
	delete[] shapes;

	//SAMPLE_FREE(shapes);
}

void setupFiltering(PxShape* shape, PxU32 filterGroup, PxU32 filterMask)
{
	PxFilterData filterData;
	filterData.word0 = filterGroup; // word0 = own ID
	filterData.word1 = filterMask;	// word1 = ID mask to filter pairs that trigger a contact callback;
	shape->setSimulationFilterData(filterData);
}

void send_packet(int to_client, void* p)
{
	char* buf = reinterpret_cast<char*>(p);
	OVER_EX* send_exover = new OVER_EX;
	send_exover->id = to_client;
	ZeroMemory(&send_exover->over, sizeof(send_exover->over));
	memcpy(send_exover->buf, buf, buf[0]);
	send_exover->wsabuf.buf = send_exover->buf;
	send_exover->wsabuf.len = buf[0];
	DWORD flags = 0;
	WSASend(clients[to_client].m_sock, &send_exover->wsabuf, 1, NULL, flags, &send_exover->over, send_complete);
	//cout << "send " << (int)buf[1] << endl;
}

void send_login_ok_packet(int to_client)
{
	sc_packet_login_ok p;
	p.id = to_client;
	p.size = sizeof(p);
	p.type = SC_PACKET_LOGIN_OK;
	// 임시 좌표
	p.pX = 0;
	p.pY = 0;
	p.pZ = 0;
	p.rX = 0;
	p.rY = 0;
	p.rZ = 0;
	p.rW = 1;

	send_packet(to_client, &p);
}

void send_move_packet(int to_client, int id)
{
	/*sc_packet_move p;
	p.id = id;
	p.size = sizeof(p);
	p.type = SC_PACKET_MOVE;

	PxExtendedVec3 body = clients[id].m_body->getPosition();
	PxTransform t1 = clients[id].m_lHand->getGlobalPose();
	PxTransform t2 = clients[id].m_rHand->getGlobalPose();

	p.body_pX = body.x;
	p.body_pY = body.y;
	p.body_pZ = body.z;
	p.body_rX = clients[id].body_rX;
	p.body_rY = clients[id].body_rY;
	p.body_rZ = clients[id].body_rZ;
	p.body_rW = clients[id].body_rW;

	p.setHandPose(t1, t2);

	if (clients[to_client].conected)
		send_packet(to_client, &p);*/
}

void send_hand_move_packet(int to_client, int id, cs_packet_hand_move* in_p)
{
	sc_packet_hand_move p;
	p.size = sizeof(p);
	p.type = SC_PACKET_HAND_MOVE;

	p.lHand_pX = in_p->lx;
	p.lHand_pY = in_p->ly;
	p.lHand_pZ = in_p->lz;
	p.lHand_rX = in_p->lrx;
	p.lHand_rY = in_p->lry;
	p.lHand_rZ = in_p->lrz;
	p.lHand_rW = in_p->lrw;

	p.rHand_pX = in_p->rx;
	p.rHand_pY = in_p->ry;
	p.rHand_pZ = in_p->rz;
	p.rHand_rX = in_p->rrx;
	p.rHand_rY = in_p->rry;
	p.rHand_rZ = in_p->rrz;
	p.rHand_rW = in_p->rrw;

	if (clients[to_client].conected)
		send_packet(to_client, &p);
}

void send_head_move_packet(int to_client, cs_packet_head_move* in_p)
{
	sc_packet_head_move p;
	p.size = sizeof(p);
	p.type = SC_PACKET_HEAD_MOVE;

	p.x = in_p->x;
	p.y = in_p->y;
	p.z = in_p->z;
	p.rx = in_p->rx;
	p.ry = in_p->ry;
	p.rz = in_p->rz;
	p.rw = in_p->rw;

	if (clients[to_client].conected)
		send_packet(to_client, &p);
}

void send_enter_packet(int to_client, int id)
{
	sc_packet_enter p;
	p.id = id;
	p.size = sizeof(p);
	p.type = SC_PACKET_ENTER;
	if (clients[id].scene_id == 2) {
		PxExtendedVec3 pose = clients[id].map2Player.m_body->getPosition();
		p.pX = pose.x;
		p.pY = pose.y;
		p.pZ = pose.z;
		p.rX = clients[id].map2Player.body_rX;
		p.rY = clients[id].map2Player.body_rY;
		p.rZ = clients[id].map2Player.body_rZ;
		p.rW = clients[id].map2Player.body_rW;
	}
	else if (clients[id].scene_id == 4) {
		PxExtendedVec3 pose = clients[id].map4Player.m_body->getPosition();
		p.pX = pose.x;
		p.pY = pose.y;
		p.pZ = pose.z;
		p.rX = clients[id].map4Player.body_rX;
		p.rY = clients[id].map4Player.body_rY;
		p.rZ = clients[id].map4Player.body_rZ;
		p.rW = clients[id].map4Player.body_rW;
	}

	send_packet(to_client, &p);
}

void send_object_move_packet_map2()
{
	vector<PxActor*> copyContainer;
	//gInstance->mSimulationEventCallbackMap2.containerLock.lock();
	copy(gInstance->mSimulationEventCallbackMap2.actorContainer.begin(), gInstance->mSimulationEventCallbackMap2.actorContainer.end(), back_inserter(copyContainer));
	//gInstance->mSimulationEventCallbackMap2.containerLock.unlock();
	for (auto iter = copyContainer.begin(); iter != copyContainer.end(); ++iter) {
		sc_packet_object_move p;
		p.size = sizeof(p);
		p.type = SC_PACKET_OBJECT_MOVE;
		p.id = *(int*)(*iter)->userData;
		//cout << p.id << endl;
		PxTransform t = reinterpret_cast<PxRigidDynamic*>(*iter)->getGlobalPose();
		p.pX = t.p.x;
		p.pY = t.p.y;
		p.pZ = t.p.z;
		p.rX = t.q.x;
		p.rY = t.q.y;
		p.rZ = t.q.z;
		p.rW = t.q.w;

		for (int i = 0; i < CLIENT_NUM; ++i) {
			if (clients[i].conected)
				if (clients[i].scene_id == 2)
					send_packet(i, &p);
		}
	}
}

void send_object_move_packet_map4()
{
	vector<PxActor*> copyContainer;
	//gInstance->mSimulationEventCallbackMap4.containerLock.lock();
	copy(gInstance->mSimulationEventCallbackMap4.actorContainer.begin(), gInstance->mSimulationEventCallbackMap4.actorContainer.end(), back_inserter(copyContainer));
	//gInstance->mSimulationEventCallbackMap4.containerLock.unlock();
	for (auto iter = copyContainer.begin(); iter != copyContainer.end(); ++iter) {
		sc_packet_object_move p;
		p.size = sizeof(p);
		p.type = SC_PACKET_OBJECT_MOVE;
		p.id = *(int*)(*iter)->userData;
		//cout << p.id << endl;
		PxTransform t = reinterpret_cast<PxRigidDynamic*>(*iter)->getGlobalPose();
		p.pX = t.p.x;
		p.pY = t.p.y;
		p.pZ = t.p.z;
		p.rX = t.q.x;
		p.rY = t.q.y;
		p.rZ = t.q.z;
		p.rW = t.q.w;

		for (int i = 0; i < CLIENT_NUM; ++i) {
			if (clients[i].conected)
				if (clients[i].scene_id == 4)
					send_packet(i, &p);
		}
	}
}

void send_hand_state_packet(int to_client, int id, bool hand, HandState state)
{
	sc_packet_change_hand_state p;
	p.size = sizeof(p);
	p.type = SC_PACKET_CHANGE_HAND_STATE;
	p.id = id;
	p.hand = hand;
	p.state = state;

	if (clients[to_client].conected)
		send_packet(to_client, &p);
}

void send_monster_move(int to_client, cs_packet_monster_move* in_p)
{
	sc_packet_monster_move p;
	p.size = sizeof(p);
	p.type = SC_PACKET_MONSTER_MOVE;
	p.id = in_p->id;
	p.pX = in_p->pX;
	p.pY = in_p->pY;
	p.pZ = in_p->pZ;
	p.rX = in_p->rX;
	p.rY = in_p->rY;
	p.rZ = in_p->rZ;
	p.rW = in_p->rW;

	if (clients[to_client].conected)
		send_packet(to_client, &p);
}

void send_monster_remove(int id)
{
	sc_packet_monster_remove p;
	p.size = sizeof(p);
	p.type = SC_PACKET_MONSTER_REMOVE;
	p.id = id;
	
	for (int i = 0; i < CLIENT_NUM; ++i)
		if (clients[i].conected)
			send_packet(i, &p);
}

constexpr float bulletSpeed = 350;

PxRigidDynamic* createBullet(int scene_id, const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* bullet = PxCreateDynamic(*gInstance->mPhysics, t, geometry, *gInstance->mMaterial, 200.0f);
	bullet->setAngularDamping(0.5f);
	bullet->setLinearVelocity(velocity);
	UserData* userData = new UserData();
	userData->id = 0;
	userData->objType = FilterGroup::eBULLET;
	bullet->userData = userData;
	setupFiltering(bullet, FilterGroup::eBULLET, FilterGroup::eBACKGROUND | FilterGroup::eSTUFF | FilterGroup::eMONSTER);
	bullet->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	if (scene_id == 2)
		gInstance->mMap2Scene->addActor(*bullet);
	else if (scene_id == 4)
		gInstance->mMap4Scene->addActor(*bullet);
	return bullet;
}

void grabTrigger(int id, cs_packet_grab* in_p)
{
	switch (in_p->hand) {
	case LEFT_HAND: {
		switch (clients[id].m_lHandState) {
		case HandState::eDEFAULT: {
			if (in_p->grab) {
				PxRaycastBuffer hit;

				if (clients[id].scene_id == 2) {
					PxVec3 dir = clients[id].map2Player.m_lHand->getGlobalPose().q.rotate(PxVec3(1, 0, 0));
					if (gInstance->mMap2Scene->raycast(clients[id].map2Player.m_lHand->getGlobalPose().p + dir * 0.031, dir, 0.05, hit)) {
						PxRigidActor* blockObject = hit.block.actor;
						UserData* ud = (UserData*)blockObject->userData;
						if (ud == nullptr) break;
						if (ud->objType != FilterGroup::eSTUFF) break;
						if (ud->owner_id != -1) break;
						if (blockObject == (PxRigidActor*)clients[id].map2Player.m_lHand) break;
						clients[id].map2Player.m_lHandJoint = PxFixedJointCreate(*gInstance->mPhysics,
							clients[id].map2Player.m_lHand,
							PxTransform(PxVec3(0.03, 0, 0)),
							blockObject,
							PxTransform(blockObject->getGlobalPose().q.rotateInv(PxVec3(hit.block.position - blockObject->getGlobalPose().p)),
								blockObject->getGlobalPose().q.getConjugate()));
						clients[id].map2Player.m_lHandJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, false);
						clients[id].map2Player.m_lHandJoint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
						ud->owner_id = id;
					}
				}
				else if (clients[id].scene_id == 4) {
					PxVec3 dir = clients[id].map4Player.m_lHand->getGlobalPose().q.rotate(PxVec3(1, 0, 0));
					if (gInstance->mMap4Scene->raycast(clients[id].map4Player.m_lHand->getGlobalPose().p + dir * 0.031, dir, 0.05, hit)) {
						PxRigidActor* blockObject = hit.block.actor;
						UserData* ud = (UserData*)blockObject->userData;
						if (ud == nullptr) break;
						if (ud->objType != FilterGroup::eSTUFF) break;
						if (ud->owner_id != -1) break;
						if (blockObject == (PxRigidActor*)clients[id].map4Player.m_lHand) break;
						clients[id].map4Player.m_lHandJoint = PxFixedJointCreate(*gInstance->mPhysics,
							clients[id].map4Player.m_lHand,
							PxTransform(PxVec3(0.03, 0, 0)),
							blockObject,
							PxTransform(blockObject->getGlobalPose().q.rotateInv(PxVec3(hit.block.position - blockObject->getGlobalPose().p)),
								blockObject->getGlobalPose().q.getConjugate()));
						clients[id].map4Player.m_lHandJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, false);
						clients[id].map4Player.m_lHandJoint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
						ud->owner_id = id;
					}
				}
			}
			else {
				if (clients[id].scene_id == 2) {
					if (clients[id].map2Player.m_lHandJoint) {
						PxRigidActor* actors[2];
						clients[id].map2Player.m_lHandJoint->getActors(actors[0], actors[1]);
						UserData* ud = (UserData*)actors[1]->userData;
						ud->owner_id = -1;
						clients[id].map2Player.m_lHandJoint->release();
						clients[id].map2Player.m_lHandJoint = nullptr;
					}
				}
				else if (clients[id].scene_id == 4) {
					if (clients[id].map4Player.m_lHandJoint) {
						PxRigidActor* actors[2];
						clients[id].map4Player.m_lHandJoint->getActors(actors[0], actors[1]);
						UserData* ud = (UserData*)actors[1]->userData;
						ud->owner_id = -1;
						clients[id].map4Player.m_lHandJoint->release();
						clients[id].map4Player.m_lHandJoint = nullptr;
					}
				}
			}
			break;
		}
		case HandState::eGUN: {
			if (in_p->grab) {
				createBullet(clients[id].scene_id, PxTransform(PxVec3(in_p->pX, in_p->pY, in_p->pZ)), PxSphereGeometry(0.02), PxVec3(in_p->dirX, in_p->dirY, in_p->dirZ) * bulletSpeed);
			}
			break;
		}
		}
		break;
	}
	case RIGHT_HAND: {
		switch (clients[id].m_rHandState) {
		case HandState::eDEFAULT: {
			if (in_p->grab) {
				PxRaycastBuffer hit;

				if (clients[id].scene_id == 2) {
					PxVec3 dir = clients[id].map2Player.m_rHand->getGlobalPose().q.rotate(PxVec3(-1, 0, 0));
					if (gInstance->mMap2Scene->raycast(clients[id].map2Player.m_rHand->getGlobalPose().p + dir * 0.031, dir, 0.05, hit)) {
						PxRigidActor* blockObject = hit.block.actor;
						UserData* ud = (UserData*)blockObject->userData;
						if (ud == nullptr) break;
						if (ud->objType != FilterGroup::eSTUFF) break;
						if (ud->owner_id != -1) break;
						if (blockObject == (PxRigidActor*)clients[id].map2Player.m_rHand) break;
						clients[id].map2Player.m_rHandJoint = PxFixedJointCreate(*gInstance->mPhysics,
							clients[id].map2Player.m_rHand,
							PxTransform(PxVec3(-0.03, 0, 0)),
							blockObject,
							PxTransform(blockObject->getGlobalPose().q.rotateInv(PxVec3(hit.block.position - blockObject->getGlobalPose().p)),
								blockObject->getGlobalPose().q.getConjugate()));
						clients[id].map2Player.m_rHandJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, false);
						clients[id].map2Player.m_rHandJoint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
						ud->owner_id = id;
					}
				}
				else if (clients[id].scene_id == 4) {
					PxVec3 dir = clients[id].map4Player.m_rHand->getGlobalPose().q.rotate(PxVec3(-1, 0, 0));
					if (gInstance->mMap4Scene->raycast(clients[id].map4Player.m_rHand->getGlobalPose().p + dir * 0.031, dir, 0.05, hit)) {
						PxRigidActor* blockObject = hit.block.actor;
						UserData* ud = (UserData*)blockObject->userData;
						if (ud == nullptr) break;
						if (ud->objType != FilterGroup::eSTUFF) break;
						if (ud->owner_id != -1) break;
						if (blockObject == (PxRigidActor*)clients[id].map4Player.m_rHand) break;
						clients[id].map4Player.m_rHandJoint = PxFixedJointCreate(*gInstance->mPhysics,
							clients[id].map4Player.m_rHand,
							PxTransform(PxVec3(-0.03, 0, 0)),
							blockObject,
							PxTransform(blockObject->getGlobalPose().q.rotateInv(PxVec3(hit.block.position - blockObject->getGlobalPose().p)),
								blockObject->getGlobalPose().q.getConjugate()));
						clients[id].map4Player.m_rHandJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, false);
						clients[id].map4Player.m_rHandJoint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
						ud->owner_id = id;
					}
				}
			}
			else {
				if (clients[id].scene_id == 2) {
					if (clients[id].map2Player.m_rHandJoint) {
						PxRigidActor* actors[2];
						clients[id].map2Player.m_rHandJoint->getActors(actors[0], actors[1]);
						UserData* ud = (UserData*)actors[1]->userData;
						ud->owner_id = -1;
						clients[id].map2Player.m_rHandJoint->release();
						clients[id].map2Player.m_rHandJoint = nullptr;
					}
				}
				else if (clients[id].scene_id == 4) {
					if (clients[id].map4Player.m_rHandJoint) {
						PxRigidActor* actors[2];
						clients[id].map4Player.m_rHandJoint->getActors(actors[0], actors[1]);
						UserData* ud = (UserData*)actors[1]->userData;
						ud->owner_id = -1;
						clients[id].map4Player.m_rHandJoint->release();
						clients[id].map4Player.m_rHandJoint = nullptr;
					}
				}
			}
			break;
		}
		case HandState::eGUN: {
			if (in_p->grab) {
				createBullet(clients[id].scene_id, PxTransform(PxVec3(in_p->pX, in_p->pY, in_p->pZ)), PxSphereGeometry(0.02), PxVec3(in_p->dirX, in_p->dirY, in_p->dirZ) * bulletSpeed);
			}
			break;
		}
		}	
		break;
	}
	}
}

void process_packet(int id)
{
	char p_type = clients[id].m_packet_start[1];
	//cout << "recv " << (int)p_type << endl;
	switch (p_type) {
	case CS_PACKET_LOGIN: {
		send_login_ok_packet(id);
		clients[id].conected = true;
		/*cs_packet_login* p = reinterpret_cast<cs_packet_login*>(clients[id].m_packet_start);
		clients[id].isVR = p->isVR;*/
		break;
	}
	case CS_PACKET_MOVE: {
		cs_packet_move* p = reinterpret_cast<cs_packet_move*>(clients[id].m_packet_start);
		/*clients[id].m_body->setPosition(PxExtendedVec3(p->body_pX, p->body_pY, p->body_pZ));
		clients[id].body_rX = p->body_rX;
		clients[id].body_rY = p->body_rY;
		clients[id].body_rZ = p->body_rZ;
		clients[id].body_rW = p->body_rW;

		clients[id].m_lHand->setKinematicTarget(p->getLHandPose());
		clients[id].m_rHand->setKinematicTarget(p->getRHandPose());*/

		/*if (clients[1 - id].conected) {
			send_move_packet(1 - id, id);
		}*/
		break;
	}
	case CS_PACKET_GRAB: {
		cs_packet_grab* p = reinterpret_cast<cs_packet_grab*>(clients[id].m_packet_start);
		grabTrigger(id, p);
		break;
	}
	case CS_PACKET_CHANGE_HAND_STATE: {
		cs_packet_change_hand_state* p = reinterpret_cast<cs_packet_change_hand_state*>(clients[id].m_packet_start);
		if (p->hand == LEFT_HAND) {
			clients[id].m_lHandState = p->state;
		}
		else {
			clients[id].m_rHandState = p->state;
		}
		if (clients[1 - id].conected)
			if (clients[1 - id].scene_id == clients[id].scene_id)
				send_hand_state_packet(1 - id, id, p->hand, p->state);
		break;
	}
	case CS_PACKET_MONSTER_MOVE: {
		cs_packet_monster_move* p = reinterpret_cast<cs_packet_monster_move*>(clients[id].m_packet_start);
		if (monsters[p->id] != nullptr) {

			//monsters[p->id]->setKinematicTarget(PxTransform(PxVec3(p->pX, p->pY, p->pZ), PxQuat(p->rX, p->rY, p->rZ, p->rW)));
			monsters[p->id]->setGlobalPose(PxTransform(PxVec3(p->pX, p->pY, p->pZ), PxQuat(p->rX, p->rY, p->rZ, p->rW)));

			if (clients[1 - id].conected)
				if (clients[1 - id].scene_id == clients[id].scene_id)
					send_monster_move(1 - id, p);
		}
		break;
	}
	case CS_PACKET_HAND_MOVE: {
		cs_packet_hand_move* p = reinterpret_cast<cs_packet_hand_move*>(clients[id].m_packet_start);

		if (clients[1 - id].scene_id == clients[id].scene_id)
			send_hand_move_packet(1 - id, id, p);
		break;
	}
	case CS_PACKET_HEAD_MOVE: {
		cs_packet_head_move* p = reinterpret_cast<cs_packet_head_move*>(clients[id].m_packet_start);

		if (clients[1 - id].scene_id == clients[id].scene_id)
			send_head_move_packet(1 - id, p);
		break;
	}
	case CS_PACKET_SERVER_HAND_MOVE: {
		cs_packet_server_hand_move* p = reinterpret_cast<cs_packet_server_hand_move*>(clients[id].m_packet_start);

		if (clients[id].scene_id == 2) {
			clients[id].map2Player.m_lHand->setKinematicTarget(p->getLHandPose());
			clients[id].map2Player.m_rHand->setKinematicTarget(p->getRHandPose());
		}
		else if (clients[id].scene_id == 4) {
			clients[id].map4Player.m_lHand->setKinematicTarget(p->getLHandPose());
			clients[id].map4Player.m_rHand->setKinematicTarget(p->getRHandPose());
		}
		break;
	}
	case CS_PACKET_CHANGE_MONSTER_STATE: {
		cs_packet_change_monster_state* p = reinterpret_cast<cs_packet_change_monster_state*>(clients[id].m_packet_start);

		sc_packet_change_monster_state packet;
		packet.size = sizeof(packet);
		packet.type = SC_PACKET_CHANGE_MONSTER_STATE;
		packet.monster_id = p->monster_id;
		packet.state = p->state;

		if(clients[1-id].conected)
			send_packet(1 - id, &packet);
	}
	case CS_PACKET_CHANGE_SCENE: {
		cs_packet_change_scene* p = reinterpret_cast<cs_packet_change_scene*>(clients[id].m_packet_start);

		switch (clients[id].scene_id) {
		case 0: {
			clients[id].scene_id = p->scene_id;
			if (p->scene_id == 2) ++nbUserOfMap2;
			else if (p->scene_id == 4) ++nbUserOfMap4;
			break;
		}
		case 2: {
			--nbUserOfMap2;
			switch (p->scene_id) {
			case 4: {
				++nbUserOfMap4;
				break;
			}
			}
			clients[id].scene_id = p->scene_id;
			/*if (clients[1 - id].scene_id == clients[id].scene_id)
				send_enter_packet(1 - id, id);*/
			break;
		}
		case 4: {
			--nbUserOfMap4;
			switch (p->scene_id) {
			case 2: {
				++nbUserOfMap2;
				break;
			}
			}
			clients[id].scene_id = p->scene_id;
			/*if (clients[1 - id].scene_id == clients[id].scene_id)
				send_enter_packet(1 - id, id);*/
			break;
		}
		}

		break;
	}
	default:
		cout << "Unknown Packet type [" << (int)p_type << "] from Client [" << id << "]\n";
		break;
	}
}

void process_recv(int id, DWORD bytes)
{
	char p_size = clients[id].m_packet_start[0];
	char* next_recv_ptr = clients[id].m_recv_start + bytes;
	while (p_size <= next_recv_ptr - clients[id].m_packet_start) {
		process_packet(id);
		clients[id].m_packet_start += p_size;
		if (clients[id].m_packet_start < next_recv_ptr)
			p_size = clients[id].m_packet_start[0];
		else break;
	}

	long long left_data = next_recv_ptr - clients[id].m_packet_start;

	if (MAX_BUFF_SIZE - (next_recv_ptr - clients[id].m_recv_over.buf) < MIN_BUFF_SIZE) {
		memcpy(clients[id].m_recv_over.buf, clients[id].m_packet_start, left_data);
		clients[id].m_packet_start = clients[id].m_recv_over.buf;
		next_recv_ptr = clients[id].m_packet_start + left_data;
	}
	clients[id].m_recv_start = next_recv_ptr;
	ZeroMemory(&clients[id].m_recv_over.over, sizeof(clients[id].m_recv_over.over));
	clients[id].m_recv_over.wsabuf.buf = reinterpret_cast<CHAR*>(next_recv_ptr);
	clients[id].m_recv_over.wsabuf.len = static_cast<ULONG>(MAX_BUFF_SIZE - (next_recv_ptr - clients[id].m_recv_over.buf));
	DWORD flags = 0;
	WSARecv(clients[id].m_sock, &clients[id].m_recv_over.wsabuf, 1, NULL, &flags, &clients[id].m_recv_over.over, recv_complete);
}

void CALLBACK send_complete(DWORD err, DWORD bytes, LPWSAOVERLAPPED over, DWORD flags)
{
	OVER_EX* exover = reinterpret_cast<OVER_EX*>(over);
	delete exover;
	//cout << "send" << endl;
}

void CALLBACK recv_complete(DWORD err, DWORD bytes, LPWSAOVERLAPPED over, DWORD flags)
{
	OVER_EX* exover = reinterpret_cast<OVER_EX*>(over);
	int id = exover->id;
	if (bytes > 0) {
		process_recv(id, bytes);
	}
	else {
		closesocket(clients[id].m_sock);
	}
	//cout << "recv" << endl;
}

void setupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
{
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid. 
	// The following conditions are true for a valid triangle mesh :
	//  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
	//  2. There are no large triangles(within specified PxTolerancesScale.)
	// It is recommended to run a separate validation check in debug/checked builds, see below.

	if (!skipMeshCleanup)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	// If DISABLE_ACTIVE_EDGES_PREDOCOMPUTE is set, the cooking does not compute the active (convex) edges, and instead 
	// marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change 
	// the collision behavior, as all edges of the triangle mesh will now be considered active.
	if (!skipEdgeData)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

PxShape* createBV33TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
	bool skipMeshCleanup, bool skipEdgeData, bool inserted, bool cookingPerformance, bool meshSizePerfTradeoff)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);

	PxCookingParams params = gInstance->mCooking->getParams();

	// Create BVH33 midphase
	params.midphaseDesc = PxMeshMidPhase::eBVH33;

	// setup common cooking params
	setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

	// The COOKING_PERFORMANCE flag for BVH33 midphase enables a fast cooking path at the expense of somewhat lower quality BVH construction.	
	if (cookingPerformance)
		params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eCOOKING_PERFORMANCE;
	else
		params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eSIM_PERFORMANCE;

	// If meshSizePerfTradeoff is set to true, smaller mesh cooked mesh is produced. The mesh size/performance trade-off
	// is controlled by setting the meshSizePerformanceTradeOff from 0.0f (smaller mesh) to 1.0f (larger mesh).
	if (meshSizePerfTradeoff)
	{
		params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.0f;
	}
	else
	{
		// using the default value
		params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.55f;
	}

	gInstance->mCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
	// We should check the validity of provided triangles in debug/checked builds though.
	if (skipMeshCleanup)
	{
		PX_ASSERT(gInstance->mCooking->validateTriangleMesh(meshDesc));
	}
#endif // DEBUG


	PxTriangleMesh* triMesh = NULL;
	PxU32 meshSize = 0;

	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (inserted)
	{
		triMesh = gInstance->mCooking->createTriangleMesh(meshDesc, gInstance->mPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		PxDefaultMemoryOutputStream outBuffer;
		gInstance->mCooking->cookTriangleMesh(meshDesc, outBuffer);

		PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
		triMesh = gInstance->mPhysics->createTriangleMesh(stream);

		meshSize = outBuffer.getSize();
	}

	if (triMesh) {
		PxTriangleMeshGeometry triGeom;
		triGeom.triangleMesh = triMesh;

		PxShape* shape = gInstance->mPhysics->createShape(triGeom, *gInstance->mMaterial);

		return shape;

		/*PxRigidStatic* actor = PxCreateStatic(*gInstance->mPhysics, transform, *shape);

		gInstance->mMap2Scene->addActor(*actor);
		shape->release();*/
	}
	return nullptr;
}

PxShape* createTriangleMesh(const char* fileName,PxVec3 scale)
{
	PxU32 numVertices;
	PxVec3* vertices = nullptr;
	PxU32 numTriangles;
	PxU32* indices = nullptr;

	gParser.parseTriangleMeshFile(fileName, &numVertices, vertices, &numTriangles, indices, scale);

	PxShape* shape = createBV33TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, false, false);
	
	delete[] vertices;
	delete[] indices;

	if (shape)
		return shape;
	return nullptr;
}

void createObjectMap2()
{
	PxShape* vase01Shape = gInstance->mPhysics->createShape(PxBoxGeometry(0.5394133 / 2, 0.8277771 / 2, 0.5394133 / 2), *gInstance->mMaterial);
	setupFiltering(vase01Shape, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	PxShape* vase02Shape = gInstance->mPhysics->createShape(PxBoxGeometry(0.2987156 / 2, 1.080622 / 2, 0.2937585 / 2), *gInstance->mMaterial);
	setupFiltering(vase02Shape, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	PxShape* puzzleHintShape = gInstance->mPhysics->createShape(PxBoxGeometry(1.0 / 2, 0.001 / 2, 0.5 / 2), *gInstance->mMaterial);
	setupFiltering(puzzleHintShape, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	PxShape* puzzleHintShape1 = gInstance->mPhysics->createShape(PxBoxGeometry(0.5 / 2, 0.001 / 2, 1.0 / 2), *gInstance->mMaterial);
	setupFiltering(puzzleHintShape1, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	PxShape* puzzleHintShape2 = gInstance->mPhysics->createShape(PxBoxGeometry(0.01 / 2, 0.5 / 2, 1.0 / 2), *gInstance->mMaterial);
	setupFiltering(puzzleHintShape2, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	PxShape* puzzleShape = gInstance->mPhysics->createShape(PxBoxGeometry(0.4687378 / 2, 1.3590325 / 2, 0.4566437 / 2), *gInstance->mMaterial);
	setupFiltering(puzzleShape, FilterGroup::eSTUFF, FilterGroup::eBULLET);

	PxRigidDynamic* puzzleHint1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-56.983370, -0.199500, 18.864410)), *puzzleHintShape, 5);
	puzzleHint1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud0 = new UserData();
	ud0->id = 0;
	ud0->owner_id = -1;
	ud0->objType = FilterGroup::eSTUFF;
	puzzleHint1->userData = ud0;
	gInstance->mMap2Scene->addActor(*puzzleHint1);

	PxRigidDynamic* puzzleHint2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-9.193359, 0.900500, 22.374420)), *puzzleHintShape1, 5);
	puzzleHint2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud1 = new UserData();
	ud1->id = 1;
	ud1->owner_id = -1;
	ud1->objType = FilterGroup::eSTUFF;
	puzzleHint2->userData = ud1;
	gInstance->mMap2Scene->addActor(*puzzleHint2);

	PxRigidDynamic* puzzleHint3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-11.242850, 0.850000, -27.125580)), *puzzleHintShape2, 5);
	puzzleHint3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud2 = new UserData();
	ud2->id = 2;
	ud2->owner_id = -1;
	ud2->objType = FilterGroup::eSTUFF;
	puzzleHint3->userData = ud2;
	gInstance->mMap2Scene->addActor(*puzzleHint3);

	PxRigidDynamic* vase1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-12.643370, 1.336746, 50.274410)), *vase02Shape, 5);
	vase1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud3 = new UserData();
	ud3->id = 3;
	ud3->owner_id = -1;
	ud3->objType = FilterGroup::eSTUFF;
	vase1->userData = ud3;
	gInstance->mMap2Scene->addActor(*vase1);

	PxRigidDynamic* vase2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-13.243380, 1.336746, 50.274410)), *vase02Shape, 5);
	vase2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud4 = new UserData();
	ud4->id = 4;
	ud4->owner_id = -1;
	ud4->objType = FilterGroup::eSTUFF;
	vase2->userData = ud4;
	gInstance->mMap2Scene->addActor(*vase2);

	PxRigidDynamic* vase3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-15.543370, 1.107166, 50.174410)), *vase01Shape, 5);
	vase3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud5 = new UserData();
	ud5->id = 5;
	ud5->owner_id = -1;
	ud5->objType = FilterGroup::eSTUFF;
	vase3->userData = ud5;
	gInstance->mMap2Scene->addActor(*vase3);

	PxRigidDynamic* vase4 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-4.043365, 0.336748, 65.274410)), *vase02Shape, 5);
	vase4->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud6 = new UserData();
	ud6->id = 6;
	ud6->owner_id = -1;
	ud6->objType = FilterGroup::eSTUFF;
	vase4->userData = ud6;
	gInstance->mMap2Scene->addActor(*vase4);

	PxRigidDynamic* vase5 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-4.143372, 0.107168, 64.174410)), *vase01Shape, 5);
	vase5->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud7 = new UserData();
	ud7->id = 7;
	ud7->owner_id = -1;
	ud7->objType = FilterGroup::eSTUFF;
	vase5->userData = ud7;
	gInstance->mMap2Scene->addActor(*vase5);

	PxRigidDynamic* vase6 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-4.043365, 0.336748, 64.774410)), *vase02Shape, 5);
	vase6->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud8 = new UserData();
	ud8->id = 8;
	ud8->owner_id = -1;
	ud8->objType = FilterGroup::eSTUFF;
	vase6->userData = ud8;
	gInstance->mMap2Scene->addActor(*vase6);

	PxRigidDynamic* puzzle1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-58.491940, 0.128339, 5.500652)), *puzzleShape, 5);
	puzzle1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud9 = new UserData();
	ud9->id = 9;
	ud9->owner_id = -1;
	ud9->objType = FilterGroup::eSTUFF;
	puzzle1->userData = ud9;
	gInstance->mMap2Scene->addActor(*puzzle1);

	PxRigidDynamic* puzzle2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-51.021930, 0.128339, 7.450634)), *puzzleShape, 5);
	puzzle2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud10 = new UserData();
	ud10->id = 10;
	ud10->owner_id = -1;
	ud10->objType = FilterGroup::eSTUFF;
	puzzle2->userData = ud10;
	gInstance->mMap2Scene->addActor(*puzzle2);

	PxRigidDynamic* puzzle3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-54.571940, 0.128339, 6.730633)), *puzzleShape, 5);
	puzzle3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud11 = new UserData();
	ud11->id = 11;
	ud11->owner_id = -1;
	ud11->objType = FilterGroup::eSTUFF;
	puzzle3->userData = ud11;
	gInstance->mMap2Scene->addActor(*puzzle3);

	PxRigidDynamic* puzzle4 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-51.621940, 0.058339, 18.470650)), *puzzleShape, 5);
	puzzle4->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud12 = new UserData();
	ud12->id = 12;
	ud12->owner_id = -1;
	ud12->objType = FilterGroup::eSTUFF;
	puzzle4->userData = ud12;
	gInstance->mMap2Scene->addActor(*puzzle4);

	PxRigidDynamic* puzzle5 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-51.231940, 0.088339, 17.490640)), *puzzleShape, 5);
	puzzle5->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud13 = new UserData();
	ud13->id = 13;
	ud13->owner_id = -1;
	ud13->objType = FilterGroup::eSTUFF;
	puzzle5->userData = ud13;
	gInstance->mMap2Scene->addActor(*puzzle5);

	PxRigidDynamic* puzzle6 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-60.321940, 0.078339, 15.250650)), *puzzleShape, 5);
	puzzle6->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud14 = new UserData();
	ud14->id = 14;
	ud14->owner_id = -1;
	ud14->objType = FilterGroup::eSTUFF;
	puzzle6->userData = ud14;
	gInstance->mMap2Scene->addActor(*puzzle6);

	vase01Shape->release();
	vase02Shape->release();
	puzzleHintShape->release();
	puzzleHintShape1->release();
	puzzleHintShape2->release();
	puzzleShape->release();
}

void createMonsterMap2()
{
	PxShape* scorpionShape = gInstance->mPhysics->createShape(PxBoxGeometry(1 * 0.5, 0.3 * 0.5, 1.660121 * 0.5), *gInstance->mMaterial);
	scorpionShape->setLocalPose(PxTransform(PxVec3(0, 0.09999932, -0.4699387)));
	setupFiltering(scorpionShape, FilterGroup::eMONSTER, FilterGroup::eBULLET);

	PxShape* skeletonShape = gInstance->mPhysics->createShape(PxBoxGeometry(3.5 * 0.5 * 0.15, 15 * 0.5 * 0.15, 3.5 * 0.5 * 0.15), *gInstance->mMaterial);
	skeletonShape->setLocalPose(PxTransform(PxVec3(0, 8 * 0.15, 0)));
	setupFiltering(skeletonShape, FilterGroup::eMONSTER, FilterGroup::eBULLET);

	monsters = new PxRigidDynamic * [4];

	monsters[0] = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *scorpionShape, 500);
	//monsters[0]->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	monsters[0]->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	UserData* ud0 = new UserData();
	ud0->id = 0;
	ud0->owner_id = -1;
	ud0->objType = FilterGroup::eMONSTER;
	ud0->hp = 2;
	monsters[0]->userData = ud0;
	gInstance->mMap2Scene->addActor(*monsters[0]);

	monsters[1] = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *skeletonShape, 500);
	//monsters[1]->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	monsters[1]->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	UserData* ud1 = new UserData();
	ud1->id = 1;
	ud1->owner_id = -1;
	ud1->objType = FilterGroup::eMONSTER;
	ud1->hp = 2;
	monsters[1]->userData = ud1;
	gInstance->mMap2Scene->addActor(*monsters[1]);

	monsters[2] = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *skeletonShape, 500);
	//monsters[2]->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	monsters[2]->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	UserData* ud2 = new UserData();
	ud2->id = 2;
	ud2->owner_id = -1;
	ud2->objType = FilterGroup::eMONSTER;
	ud2->hp = 2;
	monsters[2]->userData = ud2;
	gInstance->mMap2Scene->addActor(*monsters[2]);

	monsters[3] = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *scorpionShape, 500);
	//monsters[3]->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	monsters[3]->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	UserData* ud3 = new UserData();
	ud3->id = 3;
	ud3->owner_id = -1;
	ud3->objType = FilterGroup::eMONSTER;
	ud3->hp = 2;
	monsters[3]->userData = ud3;
	gInstance->mMap2Scene->addActor(*monsters[3]);
}

void createEnvironmentMap2()
{
	PxShape* floor = gInstance->mPhysics->createShape(PxBoxGeometry(3.758223 * 0.687897, 0.718908 * 0.687897, 4.980401 * 0.6878971), *gInstance->mMaterial);
	setupFiltering(floor, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> floors;
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-5.891041, -1.114556, -22.530620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-4.373831, -1.143186, -28.730610)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.341066, -0.618056, 58.869380)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-14.041070, -0.618057, 52.469380)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(5.158958, -1.143187, -35.330620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-7.041069, -1.064557, -6.980629)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(5.358972, -1.143186, -28.730610)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-5.641038, -1.114556, -12.330620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-22.041060, -0.718066, 39.869380)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-2.241058, -1.064558, -7.130623)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(0.558932, -1.143186, -28.730610)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-25.541060, -1.218068, 23.169360)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.341047, -1.143186, -28.730610)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-4.373831, -1.143188, -35.330620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.141035, -1.143187, -35.330620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-5.641038, -1.114556, -18.980610)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-22.041060, -1.118060, 34.569390)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-7.741056, -1.214556, -0.680610)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.791071, -1.114556, -10.580620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-21.441050, -1.218069, 16.769370)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.441072, -0.618054, 71.069390)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-20.741060, -1.218070, 10.269370)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-18.841050, -0.618057, 52.469380)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-14.041070, -0.618056, 58.869380)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-30.841060, -1.218069, 16.769370)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.791071, -1.114560, -22.530620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(2.858963, -1.114556, -13.330620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-25.841060, -1.218070, 10.269370)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-4.441049, -0.618055, 64.669400)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-4.441049, -0.618054, 71.069390)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-23.441060, -0.518051, 52.469380)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-26.041050, -1.218069, 16.769370)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-30.841060, -1.218070, 10.269370)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-12.341050, -1.214556, 1.669396)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.341066, -0.618055, 64.669400)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-22.041050, -0.518052, 46.169400)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(0.258941, -1.143188, -35.330620)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-30.541050, -1.218067, 29.759390)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-20.761170, -1.218067, 29.759390)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.791071, -1.114556, -16.730610)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(2.708947, -1.064557, -7.130623)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-30.541050, -1.218068, 23.169360)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-21.441050, -1.218068, 23.169360)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-25.541060, -1.218067, 29.759390)), *floor));
	for (auto i = floors.begin(); i != floors.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap2Scene->addActor(**i);
	}
	floor->release();

	PxShape* floor2s = gInstance->mPhysics->createShape(PxBoxGeometry(6.143844, 1.809403 * 0.5, 7.119962), *gInstance->mMaterial);
	setupFiltering(floor2s, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* floor2 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.800447, -0.123929, 21.986960)), *floor2s);
	UserData* f2ud = new UserData();
	f2ud->objType = FilterGroup::eBACKGROUND;
	floor2->userData = f2ud;
	gInstance->mMap2Scene->addActor(*floor2);
	floor2s->release();

	PxShape* lowWall = gInstance->mPhysics->createShape(PxBoxGeometry(9.993652 * 0.3554256, 3.502743 * 0.3554256, 2.433008 * 0.3554256), *gInstance->mMaterial);
	setupFiltering(lowWall, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* lowWall1 = gInstance->mPhysics->createShape(PxBoxGeometry(9.993652 * 0.3554256, 3.502743 * 0.3554256, 2.433008), *gInstance->mMaterial);
	setupFiltering(lowWall1, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* lowWall2 = gInstance->mPhysics->createShape(PxBoxGeometry(9.993652 * 0.25, 3.502743 * 0.25, 2.433008 * 0.25), *gInstance->mMaterial);
	setupFiltering(lowWall2, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> lowWalls;
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-58.154770, -1.456249, 21.361720), PxQuat(0, 0.4907745, 0, 0.8712866)), *lowWall1));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.184440, -0.759763, -29.343510), PxQuat(0, 0.7071068, 0, 0.7071068)), *lowWall));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.184440, -0.759764, -36.043500), PxQuat(0, 0.7071068, 0, 0.7071068)), *lowWall));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-14.125430, -0.459732, 49.933330)), *lowWall));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-27.230740, -0.253092, 31.087360)), *lowWall2));
	for (auto i = lowWalls.begin(); i != lowWalls.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap2Scene->addActor(**i);
	}
	lowWall->release();
	lowWall1->release();
	lowWall2->release();

	PxShape* faceWall = gInstance->mPhysics->createShape(PxBoxGeometry(7.061144 * 0.3536491, 6.413379 * 0.3536491, 0.558719 * 0.3536491), *gInstance->mMaterial);
	setupFiltering(faceWall, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* faceWall2 = gInstance->mPhysics->createShape(PxBoxGeometry(7.061144 * 0.5, 6.413379 * 0.5, 0.558719 * 0.5), *gInstance->mMaterial);
	setupFiltering(faceWall2, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> faceWalls;
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-20.032130, 1.264999, 41.723050), PxQuat(0, -0.6759633, 0, 0.7369353)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-19.632130, 1.265000, 46.223050), PxQuat(0, -0.6759633, 0, 0.7369353)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-24.632140, 1.264999, 42.123040), PxQuat(0, -0.6759633, 0, 0.7369353)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-3.421472, 1.179983, 71.217920), PxQuat(0, 0.717774, 0, 0.6962762)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-3.621487, 1.179982, 66.417900), PxQuat(0, 0.717774, 0, 0.6962762)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-1.320430, 1.179981, 63.476190), PxQuat(0, 0.1657964, 0, 0.98616)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-45.741440, -0.165425, 13.240280), PxQuat(0, -0.1779951, 0, 0.9840314)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-42.182320, -0.165425, 14.641560), PxQuat(0, -0.1996416, 0, 0.979869)), *faceWall));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(5.931244, 2.078884, -25.784020)), *faceWall2));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-7.468738, 2.175289, -25.784020)), *faceWall2));
	faceWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-13.868740, 2.175289, -25.784020)), *faceWall2));
	for (auto i = faceWalls.begin(); i != faceWalls.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap2Scene->addActor(**i);
	}
	faceWall->release();
	faceWall2->release();

	PxShape* column = gInstance->mPhysics->createShape(PxBoxGeometry(3.346645 * 0.275378, 6.738008 * 0.275378, 4.355217 * 0.275378), *gInstance->mMaterial);
	setupFiltering(column, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> columns;
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-32.449540, 0.741323, 10.731020)), *column));
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-12.429860, 0.664695, 48.366620)), *column));
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-14.018170, 0.664695, 48.366620)), *column));
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-15.618160, 0.664695, 48.366620)), *column));
	for (auto i = columns.begin(); i != columns.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap2Scene->addActor(**i);
	}
	column->release();

	PxShape* stairShape = createTriangleMesh("meshData\\stairMeshData.json", PxVec3(1, 1, 1));
	setupFiltering(stairShape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* stair1 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.021679 * 2, -0.4976641 * 2, 8.18721 * 2)), *stairShape);
	UserData* st1ud = new UserData();
	st1ud->objType = FilterGroup::eBACKGROUND;
	stair1->userData = st1ud;
	gInstance->mMap2Scene->addActor(*stair1);
	PxRigidStatic* stair2 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.021679 * 2, -0.4976632 * 2, 13.68721 * 2)), *stairShape);
	UserData* st2ud = new UserData();
	st2ud->objType = FilterGroup::eBACKGROUND;
	stair2->userData = st2ud;
	gInstance->mMap2Scene->addActor(*stair2);
	stairShape->release();

	PxShape* pillar = gInstance->mPhysics->createShape(PxCapsuleGeometry(0.75 * 0.25 * 2, 4.5 * 0.25 * 2), *gInstance->mMaterial);
	setupFiltering(pillar, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	pillar->setLocalPose(PxTransform(PxQuat(PxHalfPi, PxVec3(0, 0, 1))));
	PxRigidStatic* pillar1 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.943360, 2.692040, 27.274410)), *pillar);
	UserData* p1ud = new UserData();
	p1ud->objType = FilterGroup::eBACKGROUND;
	pillar1->userData = p1ud;
	gInstance->mMap2Scene->addActor(*pillar1);
	PxRigidStatic* pillar2 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.943360, 2.692039, 16.574400)), *pillar);
	UserData* p2ud = new UserData();
	p2ud->objType = FilterGroup::eBACKGROUND;
	pillar2->userData = p2ud;
	gInstance->mMap2Scene->addActor(*pillar2);
	pillar->release();

	PxShape* caveShape = createTriangleMesh("meshData\\caveMeshData.json", PxVec3(1, 1, 1));
	setupFiltering(caveShape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* cave = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-27.47168 * 2, -0.6667469 * 2, 6.93721 * 2), PxQuat(0, 0.6156477, 0, 0.7880215)), *caveShape);
	UserData* caveud = new UserData();
	caveud->objType = FilterGroup::eBACKGROUND;
	cave->userData = caveud;
	gInstance->mMap2Scene->addActor(*cave);
	caveShape->release();

	PxShape* caveCeilingShape = gInstance->mPhysics->createShape(PxBoxGeometry(20 * 0.5, 0.05, 20 * 0.5), *gInstance->mMaterial);
	setupFiltering(caveCeilingShape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* caveCeiling = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-27.47168 * 2, -0.6667469 * 2 + 6.56052, 6.93721 * 2), PxQuat(0, 0.6156477, 0, 0.7880215)), *caveCeilingShape);
	UserData* cavecud = new UserData();
	cavecud->objType = FilterGroup::eBACKGROUND;
	caveCeiling->userData = cavecud;
	gInstance->mMap2Scene->addActor(*caveCeiling);
	caveCeilingShape->release();

	PxShape* archDoorShape = createTriangleMesh("meshData\\archDoorMeshData.json", PxVec3(1, 1, 1));
	PxRigidStatic* archDoor = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.4157949 * 2, -0.4176045 * 2, -12.75075 * 2)), *archDoorShape);
	setupFiltering(archDoor, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	UserData* archud = new UserData();
	archud->objType = FilterGroup::eBACKGROUND;
	archDoor->userData = archud;
	gInstance->mMap2Scene->addActor(*archDoor);
	archDoorShape->release();

	PxShape* archDoorWallShape = createTriangleMesh("meshData\\archDoorWallMeshData.json", PxVec3(1, 1, 1));
	PxRigidStatic* archDoorWall = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.4231954 * 2, -0.4162923 * 2, -12.75581 * 2)), *archDoorWallShape);
	setupFiltering(archDoorWall, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	UserData* archwud = new UserData();
	archwud->objType = FilterGroup::eBACKGROUND;
	archDoorWall->userData = archwud;
	gInstance->mMap2Scene->addActor(*archDoorWall);
	archDoorWallShape->release();
}

void createObjectMap4()
{
	PxShape* mirrorShape = gInstance->mPhysics->createShape(PxBoxGeometry(5.5 * 0.5 * 0.2, 0.5 * 0.5 * 0.2, 5.5 * 0.5 * 0.2), *gInstance->mMaterial);
	setupFiltering(mirrorShape, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	PxRigidDynamic* mirror0 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-0.410000, 1.833001, 3.030000)), *mirrorShape, 5);
	mirror0->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud0 = new UserData();
	ud0->id = 0;
	ud0->objType = FilterGroup::eSTUFF;
	ud0->owner_id = -1;
	mirror0->userData = ud0;
	gInstance->mMap4Scene->addActor(*mirror0);
}

void createEnvironmentMap4()
{
	PxShape* floor = gInstance->mPhysics->createShape(PxBoxGeometry(3.758223 * 0.687897, 0.718908 * 0.687897, 4.980401 * 0.6878971), *gInstance->mMaterial);
	setupFiltering(floor, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> floors;
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-31.277690, -0.754556, 35.854960), PxQuat(0.000000, 0.000000, 0.000000, 1.000000)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-32.087690, -0.754556, 29.794960), PxQuat(0.000000, 0.000000, 0.000000, 1.000000)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-52.238680, -0.764556, 24.097650), PxQuat(0.000000, -0.608761, 0.000000, 0.793353)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-56.598680, -0.774556, 25.457650), PxQuat(0.000000, -0.608761, 0.000000, 0.793353)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-15.707440, -0.884556, 18.799020), PxQuat(0.000000, 0.812529, 0.000000, 0.582921)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-10.777440, -0.824556, 16.299020), PxQuat(0.000000, 0.812529, 0.000000, 0.582921)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-33.637700, -0.754556, 11.704960), PxQuat(0.000000, 0.000000, 0.000000, 1.000000)), *floor));
	floors.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-33.237690, -0.904556, 6.624957), PxQuat(0.000000, 0.000000, 0.000000, 1.000000)), *floor));
	for (auto i = floors.begin(); i != floors.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap4Scene->addActor(**i);
	}
	floor->release();

	PxShape* highWall = gInstance->mPhysics->createShape(PxBoxGeometry(7.057033 * 0.35, 6.53896 * 0.35, 1.202788 * 0.35), *gInstance->mMaterial);
	setupFiltering(highWall, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> highWalls;
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-36.115550, 1.989806, 63.202080), PxQuat(0.000000, 0.000000, 0.000000, 1.000000)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-26.970700, 1.989806, 62.759090), PxQuat(0.000000, 0.072980, 0.000000, 0.997333)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-28.597920, 1.530691, 35.119540), PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-28.597920, 1.530692, 30.449550), PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-34.417920, 1.530691, 35.279550), PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-34.417920, 1.530692, 30.609550), PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-25.237920, 1.590695, 14.419550), PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-25.237920, 5.830697, 14.419550), PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *highWall));
	highWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-19.037920, 1.590693, 14.409550), PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *highWall));
	for (auto i = highWalls.begin(); i != highWalls.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap4Scene->addActor(**i);
	}
	highWall->release();

	PxShape* cubeShape = gInstance->mPhysics->createShape(PxBoxGeometry(1 * 0.5 * 600.0001 * 0.015 * 2, 0.1 * 0.5 * 33.33334 * 0.015 * 2, 1 * 0.5 * 580.0001 * 0.015 * 2), *gInstance->mMaterial);
	setupFiltering(cubeShape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* cube = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-33.250000, -0.570000, -5.390001)), *cubeShape);
	UserData* cubeud = new UserData();
	cubeud->objType = FilterGroup::eBACKGROUND;
	cube->userData = cubeud;
	gInstance->mMap4Scene->addActor(*cube);

	PxShape* lowWallShape = gInstance->mPhysics->createShape(PxBoxGeometry(9.993656 * 23.69504 * 0.015, 3.502743 * 23.69504 * 0.015, 2.433009 * 23.69504 * 0.015), *gInstance->mMaterial);
	setupFiltering(lowWallShape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* lowWall = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-41.284250, -1.186249, -8.757929),PxQuat(0.000000, 0.707107, 0.000000, 0.707107)), *lowWallShape);
	UserData* lwud = new UserData();
	lwud->objType = FilterGroup::eBACKGROUND;
	lowWall->userData = lwud;
	gInstance->mMap4Scene->addActor(*lowWall);

	PxShape* pCube004Shape = createTriangleMesh("meshData\\pCube004MeshData.json", PxVec3(3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube004Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube009Shape = createTriangleMesh("meshData\\pCube009MeshData.json", PxVec3(1.0369 * 3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube009Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube012Shape = createTriangleMesh("meshData\\pCube012MeshData.json", PxVec3(3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube012Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube013Shape = createTriangleMesh("meshData\\pCube013MeshData.json", PxVec3(3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube013Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube014Shape = createTriangleMesh("meshData\\pCube014MeshData.json", PxVec3(3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube014Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube015Shape0 = createTriangleMesh("meshData\\pCube015MeshData.json", PxVec3(3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube015Shape0, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube015Shape1 = createTriangleMesh("meshData\\pCube015MeshData.json", PxVec3(3.f / 5, 1.09 * 3.f / 5, 3.f / 5));
	setupFiltering(pCube015Shape1, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube015Shape2 = createTriangleMesh("meshData\\pCube015MeshData.json", PxVec3(3.f / 5, 1.1122 * 3.f / 5, 1.1539 * 3.f / 5));
	setupFiltering(pCube015Shape2, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube016Shape = createTriangleMesh("meshData\\pCube016MeshData.json", PxVec3(3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube016Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxShape* pCube017Shape = createTriangleMesh("meshData\\pCube017MeshData.json", PxVec3(3.f / 5, 3.f / 5, 3.f / 5));
	setupFiltering(pCube017Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> pCubes;
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.450000, -0.520000, -8.680001), PxQuat(0, 0, 0, 1)), *pCube004Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.740000, -0.520000, -8.680003), PxQuat(0, 0, 0, 1)), *pCube004Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-33.484640, -0.520000, -0.310000), PxQuat(0, 0, 0, 1)), *pCube009Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-33.440000, -0.520000, -0.310000), PxQuat(0, 0, 0, 1)), *pCube012Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.400000, -0.520000, -8.700000), PxQuat(0, 0, 0, 1)), *pCube013Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.610000, -0.520000, -8.700000), PxQuat(0, 0, 0, 1)), *pCube013Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-40.990000, -0.520000, -2.060000), PxQuat(0.000000, 1.000000, 0.000000, 0.000000)), *pCube013Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.530000, -0.520000, -0.240000), PxQuat(0, 0, 0, 1)), *pCube014Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.530000, -0.520000, -8.340001), PxQuat(0, 0, 0, 1)), *pCube014Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.510000, -0.520000, -0.310000), PxQuat(0, 0, 0, 1)), *pCube015Shape0));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-25.300000, -0.520000, -1.880000), PxQuat(0.000000, -0.707107, 0.000000, 0.707107)), *pCube015Shape0));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.510000, -0.520000, -8.530000), PxQuat(0, 0, 0, 1)), *pCube015Shape0));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.290000, 8.419999, -0.545597), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.250000, 8.419999, -4.415597), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.190000, 8.419999, -8.435598), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-28.320000, 8.260004, -0.545597), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-28.280000, 8.260005, -4.415597), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-28.220000, 8.260005, -8.435597), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.190000, 8.419999, -12.520000), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-28.220000, 8.260005, -12.520000), PxQuat(0.000000, 0.000000, 0.713440, -0.700716)), *pCube015Shape1));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.320000, 8.480004, -1.310000), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.990000, 8.419999, -1.420000), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.280000, 8.480005, -5.180002), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.950000, 8.419999, -5.290002), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.220000, 8.480006, -9.200004), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.890000, 8.419999, -9.310003), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-29.220000, 8.480006, -13.284400), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.890000, 8.419999, -13.394400), PxQuat(0.498475, -0.507527, -0.501411, 0.492469)), *pCube015Shape2));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.730000, -0.520000, -8.530000), PxQuat(0, 0, 0, 1)), *pCube016Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.730000, -0.520000, -0.200000), PxQuat(0, 0, 0, 1)), *pCube016Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.760000, -0.520000, -8.739999), PxQuat(0, 0, 0, 1)), *pCube017Shape));
	pCubes.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-37.760000, -0.520000, -0.250000), PxQuat(0, 0, 0, 1)), *pCube017Shape));
	for (auto i = pCubes.begin(); i != pCubes.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap4Scene->addActor(**i);
	}
	pCube004Shape->release();
	pCube009Shape->release();
	pCube012Shape->release();
	pCube013Shape->release();
	pCube014Shape->release();
	pCube015Shape0->release();
	pCube015Shape1->release();
	pCube015Shape2->release();
	pCube016Shape->release();
	pCube017Shape->release();

	PxShape* cylinderShape = gInstance->mPhysics->createShape(PxCapsuleGeometry(1 * 33 * 0.015 * 2, 3.5 * 33 * 0.015 * 2), *gInstance->mMaterial);
	cylinderShape->setLocalPose(PxTransform(PxQuat(PxHalfPi, PxVec3(0, 0, 1))));
	setupFiltering(cylinderShape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* cylinder = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-33.170000, 0.940000, -5.560003)), *cylinderShape);
	UserData* cylinderud = new UserData();
	cylinderud->objType = FilterGroup::eBACKGROUND;
	cylinder->userData = cylinderud;
	gInstance->mMap4Scene->addActor(*cylinder);

	PxShape* floor2Shape = gInstance->mPhysics->createShape(PxBoxGeometry(6.143843 * 0.4348328, 1.809403 * 0.4348328, 7.119961 * 0.4348328), *gInstance->mMaterial);
	setupFiltering(floor2Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	vector<PxRigidStatic*> floor2s;
	floor2s.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(3.330992, -0.062493, 3.631158)), *floor2Shape));
	floor2s.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-1.519008, -0.062493, 3.631158)), *floor2Shape));
	floor2s.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-6.399007, -0.062493, 3.631158)), *floor2Shape));
	floor2s.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-1.519008, -0.722493, 6.571158)), *floor2Shape));
	for (auto i = floor2s.begin(); i != floor2s.end(); ++i) {
		UserData* ud = new UserData();
		ud->objType = FilterGroup::eBACKGROUND;
		(*i)->userData = ud;
		gInstance->mMap4Scene->addActor(**i);
	}
	floor2Shape->release();

	PxShape* pillarShape = gInstance->mPhysics->createShape(PxBoxGeometry(5.476431 * 0.15, 3.681873 * 0.15, 5.483719 * 0.15), *gInstance->mMaterial);
	setupFiltering(pillarShape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* pillar = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.379687, 1.234654, 2.939684)), *pillarShape);
	UserData* pillarud = new UserData();
	pillarud->objType = FilterGroup::eBACKGROUND;
	pillar->userData = pillarud;
	gInstance->mMap4Scene->addActor(*pillar);
}

int main()
{
	gInstance = PhysXClass::getInstance();

	PxShape* map2Shape = createTriangleMesh("meshData\\map2MeshData.json", PxVec3(1, 1, 1));
	setupFiltering(map2Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* map2 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *map2Shape);
	UserData* map2ud = new UserData();
	map2ud->objType = FilterGroup::eBACKGROUND;
	map2->userData = map2ud;
	map2Shape->release();
	gInstance->mMap2Scene->addActor(*map2);
	createEnvironmentMap2();
	createObjectMap2();
	createMonsterMap2();

	PxShape* map45Shape = createTriangleMesh("meshData\\map45MeshData.json", PxVec3(1, 1, 1));
	setupFiltering(map45Shape, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	PxRigidStatic* map4 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *map45Shape);
	UserData* map4ud = new UserData();
	map4ud->objType = FilterGroup::eBACKGROUND;
	map4->userData = map4ud;
	map45Shape->release();
	gInstance->mMap4Scene->addActor(*map4);
	createEnvironmentMap4();
	createObjectMap4();

	/*PxShape* navShape = createTriangleMesh("meshData\\map2NavMeshData.json", PxVec3(10, 10, 10));
	PxRigidStatic* nav = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *navShape);
	gInstance->mMap2Scene->addActor(*nav);*/

	cout << "맵 생성 완료\n";

	/*PxRigidStatic* groundPlane = PxCreatePlane(*gInstance->mPhysics, PxPlane(0, 1, 0, 0), *gInstance->mMaterial);
	gInstance->mMap2Scene->addActor(*groundPlane);

	PxRigidDynamic* box = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0.5, 2.57)), PxBoxGeometry(0.5, 0.25, 1.0), *gInstance->mMaterial, 5);
	box->setLinearDamping(1.f);
	box->setAngularDamping(1.f);
	box->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud = new UserData();
	ud->id = 0;
	ud->objType = FilterGroup::eSTUFF;
	ud->owner_id = -1;
	box->userData = ud;
	gInstance->mMap2Scene->addActor(*box);*/

	int retval;

	// 윈속 초기화
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		return 1;

	// socket()
	g_lSocket = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED);
	if (g_lSocket == INVALID_SOCKET) err_quit("socket()");

	// bind()
	SOCKADDR_IN serveraddr;
	ZeroMemory(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serveraddr.sin_port = htons(SERVER_PORT);
	retval = bind(g_lSocket, (SOCKADDR*)&serveraddr, sizeof(serveraddr));
	if (retval == SOCKET_ERROR) err_quit("bind()");

	// listen()
	retval = listen(g_lSocket, SOMAXCONN);
	if (retval == SOCKET_ERROR) err_quit("listen()");

	// 데이터 통신에 사용할 변수
	SOCKADDR_IN clientaddr;
	int addrlen;

	for (int i = 0; i < CLIENT_NUM; ++i) {
		ZeroMemory(&clients[i].m_recv_over.over, sizeof(clients[i].m_recv_over.over));
		clients[i].m_recv_over.wsabuf.buf = clients[i].m_recv_over.buf;
		clients[i].m_recv_over.wsabuf.len = MAX_BUFF_SIZE;
		clients[i].m_packet_start = clients[i].m_recv_over.buf;
		clients[i].m_recv_start = clients[i].m_recv_over.buf;
		clients[i].conected = false;
		clients[i].scene_id = 0;
	}

	PxControllerManager* manager = PxCreateControllerManager(*gInstance->mMap2Scene);

	PxCapsuleControllerDesc bodyDesc;
	bodyDesc.contactOffset = 0.05f;
	bodyDesc.height = 0.75f;
	bodyDesc.radius = 0.125f;
	bodyDesc.stepOffset = 0.2f;
	bodyDesc.material = gInstance->mMaterial;
	bodyDesc.position = PxExtendedVec3(0, 0.75, 0);
	bodyDesc.density = 5;

	PxCapsuleControllerDesc headDesc;
	headDesc.radius = 0.0625;

	addrlen = sizeof(clientaddr);
	for (int i = 0; i < CLIENT_NUM; ++i) {
		clients[i].m_recv_over.id = i;
		clients[i].m_sock = accept(g_lSocket, (SOCKADDR*)&clientaddr, &addrlen);
		err_display("accept: ");
		printf("\n클라이언트 접속: IP 주소=%s,포트 번호=%d\n",
			inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port));

		clients[i].map2Player.m_body = manager->createController(bodyDesc);
		PxShape* lHand = gInstance->mPhysics->createShape(PxBoxGeometry(0.06/2, 0.135/2, 0.3/2), *gInstance->mMaterial);
		clients[i].map2Player.m_lHand = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *lHand, 5);
		clients[i].map2Player.m_lHand->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gInstance->mMap2Scene->addActor(*clients[i].map2Player.m_lHand);
		lHand->release();

		PxShape* rHand = gInstance->mPhysics->createShape(PxBoxGeometry(0.06/2, 0.135/2, 0.3/2), *gInstance->mMaterial);
		clients[i].map2Player.m_rHand = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *rHand, 5);
		clients[i].map2Player.m_rHand->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gInstance->mMap2Scene->addActor(*clients[i].map2Player.m_rHand);
		rHand->release();

		DWORD flags = 0;
		WSARecv(clients[i].m_sock, &clients[i].m_recv_over.wsabuf, 1, NULL, &flags, &clients[i].m_recv_over.over, recv_complete);
		/*if (WSAGetLastError() != WSA_IO_PENDING)
			err_display("wsarecv: ");*/
	}

	while (true) {
		if (clients[0].conected && clients[1].conected) break;
		SleepEx(1, true);
	}

	send_enter_packet(0, 1);
	send_enter_packet(1, 0);

	bool loop = true;

	auto start_t_m2 = high_resolution_clock::now();
	auto end_t_m2 = high_resolution_clock::now();
	auto start_t_m4 = high_resolution_clock::now();
	auto end_t_m4 = high_resolution_clock::now();
	while (loop) {
		if (_kbhit()) {
			switch (_getch())
			{
			case 27:
				loop = false;
				break;
			default:
				break;
			}
		}

		if (nbUserOfMap2 > 0) {
			end_t_m2 = high_resolution_clock::now();
			auto elapsedTime = duration_cast<milliseconds>(end_t_m2 - start_t_m2).count();
			float et = elapsedTime * 0.001;
			if (et >= 1.f / 60) {
				start_t_m2 = end_t_m2;
				gInstance->stepPhysics(et, 2);
				send_object_move_packet_map2();
				//gInstance->mSimulationEventCallbackMap2.removedActorsLock.lock();
				if (!gInstance->mSimulationEventCallbackMap2.removedActors.empty()) {
					for (auto iter = gInstance->mSimulationEventCallbackMap2.removedActors.begin(); iter != gInstance->mSimulationEventCallbackMap2.removedActors.end(); ++iter) {
						// 제거된 몬스터 패킷 전송
						UserData* ud = (UserData*)(*iter)->userData;
						if (ud->objType == FilterGroup::eMONSTER) {
							send_monster_remove(ud->id);
						}
						delete ud;
						(*iter)->release();
					}
					gInstance->mSimulationEventCallbackMap2.removedActors.clear();
				}
				//gInstance->mSimulationEventCallbackMap2.removedActorsLock.unlock();
			}
		}
		else {
			start_t_m2 = high_resolution_clock::now();
		}

		if (nbUserOfMap4 > 0) {
			end_t_m4 = high_resolution_clock::now();
			auto elapsedTime = duration_cast<milliseconds>(end_t_m4 - start_t_m4).count();
			float et = elapsedTime * 0.001;
			if (et >= 1.f / 60) {
				start_t_m4 = end_t_m4;
				gInstance->stepPhysics(et, 4);
				send_object_move_packet_map4();
				//gInstance->mSimulationEventCallbackMap4.removedActorsLock.lock();
				if (!gInstance->mSimulationEventCallbackMap4.removedActors.empty()) {
					for (auto iter = gInstance->mSimulationEventCallbackMap4.removedActors.begin(); iter != gInstance->mSimulationEventCallbackMap4.removedActors.end(); ++iter) {
						// 제거된 몬스터 패킷 전송
						UserData* ud = (UserData*)(*iter)->userData;
						if (ud->objType == FilterGroup::eMONSTER) {
							send_monster_remove(ud->id);
						}
						delete ud;
						(*iter)->release();
					}
					gInstance->mSimulationEventCallbackMap2.removedActors.clear();
				}
				//gInstance->mSimulationEventCallbackMap2.removedActorsLock.unlock();
			}
		}
		else {
			start_t_m4 = high_resolution_clock::now();
		}

		
		SleepEx(1, true);
	}

	// closesocket()
	closesocket(g_lSocket);

	// 윈속 종료
	WSACleanup();
	return 0;
}