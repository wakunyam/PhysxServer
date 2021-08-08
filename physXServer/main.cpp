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

constexpr int MAX_BUFF_SIZE = 4096;
constexpr int MIN_BUFF_SIZE = 1024;
constexpr int CLIENT_NUM = 2;

struct OVER_EX {
	WSAOVERLAPPED over;
	WSABUF wsabuf;
	char buf[MAX_BUFF_SIZE];
	int id;
};

struct CLIENT {
	SOCKET m_sock;
	OVER_EX m_recv_over;
	char* m_packet_start;
	char* m_recv_start;
	volatile bool conected;
	PxController* m_body;
	float body_rX, body_rY, body_rZ, body_rW;
	PxRigidDynamic* m_lHand, * m_rHand;
	PxFixedJoint* m_lHandJoint = nullptr, * m_rHandJoint = nullptr;
	bool isVR;

	/*PxController* m_lHand;
	float lHand_rX, lHand_rY, lHand_rZ, lHand_rW;
	PxController* m_rHand;
	float rHand_rX, rHand_rY, rHand_rZ, rHand_rW;*/
};

CLIENT clients[CLIENT_NUM];
SOCKET g_lSocket;
OVER_EX g_accept_over;
PhysXClass* gInstance;
JsonParser gParser;

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
	sc_packet_move p;
	p.id = id;
	p.size = sizeof(p);
	p.type = SC_PACKET_MOVE;

	PxExtendedVec3 body = clients[id].m_body->getPosition();
	PxTransform t1 = clients[id].m_lHand->getGlobalPose();
	PxTransform t2 = clients[id].m_rHand->getGlobalPose();
	/*PxExtendedVec3 lHand = clients[id].m_lHand->getPosition();
	PxExtendedVec3 rHand = clients[id].m_rHand->getPosition();*/

	p.body_pX = body.x;
	p.body_pY = body.y;
	p.body_pZ = body.z;
	p.body_rX = clients[id].body_rX;
	p.body_rY = clients[id].body_rY;
	p.body_rZ = clients[id].body_rZ;
	p.body_rW = clients[id].body_rW;

	p.setHandPose(t1, t2);

	/*p.lHand_pX = lHand.x;
	p.lHand_pY = lHand.y;
	p.lHand_pZ = lHand.z;
	p.lHand_rX = clients[id].lHand_rX;
	p.lHand_rY = clients[id].lHand_rY;
	p.lHand_rZ = clients[id].lHand_rZ;
	p.lHand_rW = clients[id].lHand_rW;

	p.rHand_pX = rHand.x;
	p.rHand_pY = rHand.y;
	p.rHand_pZ = rHand.z;
	p.rHand_rX = clients[id].rHand_rX;
	p.rHand_rY = clients[id].rHand_rY;
	p.rHand_rZ = clients[id].rHand_rZ;
	p.rHand_rW = clients[id].rHand_rW;*/

	send_packet(to_client, &p);
}

void send_enter_packet(int to_client, int id)
{
	sc_packet_enter p;
	p.id = id;
	p.size = sizeof(p);
	p.type = SC_PACKET_ENTER;
	PxExtendedVec3 pose = clients[id].m_body->getPosition();
	p.pX = pose.x;
	p.pY = pose.y;
	p.pZ = pose.z;
	p.rX = clients[id].body_rX;
	p.rY = clients[id].body_rY;
	p.rZ = clients[id].body_rZ;
	p.rW = clients[id].body_rW;

	send_packet(to_client, &p);
}

void send_object_move_packet()
{
	vector<PxActor*> copyContainer;
	gInstance->mSimulationEventCallback.containerLock.lock();
	copy(gInstance->mSimulationEventCallback.actorContainer.begin(), gInstance->mSimulationEventCallback.actorContainer.end(), back_inserter(copyContainer));
	gInstance->mSimulationEventCallback.containerLock.unlock();
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
			send_packet(i, &p);
		}
	}
}

void grabObject(int id, bool hand, bool grab)
{
	switch (hand) {
	case LEFT_HAND: {
		if (grab) {
			PxRaycastBuffer hit;

			PxVec3 dir = clients[id].m_lHand->getGlobalPose().q.rotate(PxVec3(1, 0, 0));
			if (gInstance->mScene->raycast(clients[id].m_lHand->getGlobalPose().p + clients[id].m_lHand->getCMassLocalPose().p + dir * 0.015, dir, 0.05, hit)) {
				PxRigidActor* blockObject = hit.block.actor;
				if (blockObject == (PxRigidActor*)clients[id].m_lHand) break;
				clients[id].m_lHandJoint = PxFixedJointCreate(*gInstance->mPhysics,
					clients[id].m_lHand,
					PxTransform(PxVec3(0.01, 0, 0) + clients[id].m_lHand->getCMassLocalPose().p),
					blockObject,
					PxTransform(blockObject->getGlobalPose().q.rotateInv(PxVec3(hit.block.position - blockObject->getGlobalPose().p)),
						blockObject->getGlobalPose().q.getConjugate()));
				clients[id].m_lHandJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, false);
				clients[id].m_lHandJoint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
			}
		}
		else {
			if (clients[id].m_lHandJoint) {
				clients[id].m_lHandJoint->release();
				clients[id].m_lHandJoint = nullptr;
			}
		}
		break;
	}
	case RIGHT_HAND: {
		if (grab) {
			PxRaycastBuffer hit;

			PxVec3 dir = clients[id].m_rHand->getGlobalPose().q.rotate(PxVec3(-1, 0, 0));

			if (gInstance->mScene->raycast(clients[id].m_rHand->getGlobalPose().p + clients[id].m_rHand->getCMassLocalPose().p + dir * 0.015, dir, 0.05, hit)) {
				PxRigidActor* blockObject = hit.block.actor;
				if (blockObject == (PxRigidActor*)clients[id].m_rHand) break;
				clients[id].m_rHandJoint = PxFixedJointCreate(*gInstance->mPhysics,
					clients[id].m_rHand, 
					PxTransform(PxVec3(-0.01, 0, 0) + clients[id].m_rHand->getCMassLocalPose().p),
					blockObject, 
					PxTransform(blockObject->getGlobalPose().q.rotateInv(PxVec3(hit.block.position - blockObject->getGlobalPose().p)), 
						blockObject->getGlobalPose().q.getConjugate()));
				clients[id].m_rHandJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, false);
				clients[id].m_rHandJoint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
			}
		}
		else {
			if (clients[id].m_rHandJoint) {
				clients[id].m_rHandJoint->release();
				clients[id].m_rHandJoint = nullptr;
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
		clients[id].m_body->setPosition(PxExtendedVec3(p->body_pX, p->body_pY, p->body_pZ));
		clients[id].body_rX = p->body_rX;
		clients[id].body_rY = p->body_rY;
		clients[id].body_rZ = p->body_rZ;
		clients[id].body_rW = p->body_rW;

		clients[id].m_lHand->setKinematicTarget(p->getLHandPose());
		clients[id].m_rHand->setKinematicTarget(p->getRHandPose());

		send_move_packet(CLIENT_NUM - 1 - id, id);
		break;
	}
	case CS_PACKET_GRAB: {
		cs_packet_grab* p = reinterpret_cast<cs_packet_grab*>(clients[id].m_packet_start);
		grabObject(id, p->hand, p->grab);
		break;
	}
	default:
		cout << "Unknown Packet type [" << p_type << "] from Client [" << id << "]\n";
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

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	/*PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	UserData* userData = new UserData();
	userData->id = 0;
	userData->objType = ObjectType::eBULLET;
	dynamic->userData = userData;
	setupFiltering(dynamic, FilterGroup::eBULLET, FilterGroup::eBACKGROUND | FilterGroup::eSTUFF);
	dynamic->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
	gScene->addActor(*dynamic);
	return dynamic;*/
}

void createObject()
{
	PxShape* vase01Shape = gInstance->mPhysics->createShape(PxBoxGeometry(0.5394133/2, 0.8277771/2, 0.5394133/2), *gInstance->mMaterial);
	PxShape* vase02Shape = gInstance->mPhysics->createShape(PxBoxGeometry(0.2987156/2, 1.080622/2, 0.2937585 /2), *gInstance->mMaterial);
	PxShape* puzzleHintShape = gInstance->mPhysics->createShape(PxBoxGeometry(1.0/2, 0.001/2, 0.5 / 2), *gInstance->mMaterial);
	PxShape* puzzleHintShape1 = gInstance->mPhysics->createShape(PxBoxGeometry(0.5 / 2, 0.001 / 2, 1.0 / 2), *gInstance->mMaterial);
	PxShape* puzzleHintShape2 = gInstance->mPhysics->createShape(PxBoxGeometry(0.01 / 2, 0.5 / 2, 1.0 / 2), *gInstance->mMaterial);
	PxShape* puzzleShape = gInstance->mPhysics->createShape(PxBoxGeometry(0.4687378/2, 1.3590325/2, 0.4566437/2), *gInstance->mMaterial);

	PxRigidDynamic* puzzleHint1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-56.983370, -0.199500, 18.864410)), *puzzleHintShape, 5);
	puzzleHint1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud0 = new UserData();
	ud0->id = 0;
	ud0->objType = FilterGroup::eSTUFF;
	puzzleHint1->userData = ud0;
	setupFiltering(puzzleHint1, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzleHint1);

	PxRigidDynamic* puzzleHint2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-9.193359, 0.900500, 22.374420)), *puzzleHintShape1, 5);
	puzzleHint2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud1 = new UserData();
	ud1->id = 1;
	ud1->objType = FilterGroup::eSTUFF;
	puzzleHint2->userData = ud1;
	setupFiltering(puzzleHint2, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzleHint2);

	PxRigidDynamic* puzzleHint3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-11.242850, 0.850000, -27.125580)), *puzzleHintShape2, 5);
	puzzleHint3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud2 = new UserData();
	ud2->id = 2;
	ud2->objType = FilterGroup::eSTUFF;
	puzzleHint3->userData = ud2;
	setupFiltering(puzzleHint3, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzleHint3);

	PxRigidDynamic* vase1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-12.643370, 1.336746, 50.274410)), *vase02Shape, 5);
	vase1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud3 = new UserData();
	ud3->id = 3;
	ud3->objType = FilterGroup::eSTUFF;
	vase1->userData = ud3;
	setupFiltering(vase1, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*vase1);

	PxRigidDynamic* vase2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-13.243380, 1.336746, 50.274410)), *vase02Shape, 5);
	vase2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud4 = new UserData();
	ud4->id = 4;
	ud4->objType = FilterGroup::eSTUFF;
	vase2->userData = ud4;
	setupFiltering(vase2, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*vase2);

	PxRigidDynamic* vase3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-15.543370, 1.107166, 50.174410)), *vase01Shape, 5);
	vase3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud5 = new UserData();
	ud5->id = 5;
	ud5->objType = FilterGroup::eSTUFF;
	vase3->userData = ud5;
	setupFiltering(vase3, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*vase3);

	PxRigidDynamic* vase4 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-4.043365, 0.336748, 65.274410)), *vase02Shape, 5);
	vase4->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud6 = new UserData();
	ud6->id = 6;
	ud6->objType = FilterGroup::eSTUFF;
	vase4->userData = ud6;
	setupFiltering(vase4, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*vase4);

	PxRigidDynamic* vase5 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-4.143372, 0.107168, 64.174410)), *vase01Shape, 5);
	vase5->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud7 = new UserData();
	ud7->id = 7;
	ud7->objType = FilterGroup::eSTUFF;
	vase5->userData = ud7;
	setupFiltering(vase5, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*vase5);

	PxRigidDynamic* vase6 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-4.043365, 0.336748, 64.774410)), *vase02Shape, 5);
	vase6->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud8 = new UserData();
	ud8->id = 8;
	ud8->objType = FilterGroup::eSTUFF;
	vase6->userData = ud8;
	setupFiltering(vase6, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*vase6);

	PxRigidDynamic* puzzle1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-58.491940, 0.128339, 5.500652)), *puzzleShape, 5);
	puzzle1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud9 = new UserData();
	ud9->id = 9;
	ud9->objType = FilterGroup::eSTUFF;
	puzzle1->userData = ud9;
	setupFiltering(puzzle1, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzle1);

	PxRigidDynamic* puzzle2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-51.021930, 0.128339, 7.450634)), *puzzleShape, 5);
	puzzle2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud10 = new UserData();
	ud10->id = 10;
	ud10->objType = FilterGroup::eSTUFF;
	puzzle2->userData = ud10;
	setupFiltering(puzzle2, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzle2);

	PxRigidDynamic* puzzle3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-54.571940, 0.128339, 6.730633)), *puzzleShape, 5);
	puzzle3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud11 = new UserData();
	ud11->id = 11;
	ud11->objType = FilterGroup::eSTUFF;
	puzzle3->userData = ud11;
	setupFiltering(puzzle3, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzle3);

	PxRigidDynamic* puzzle4 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-51.621940, 0.058339, 18.470650)), *puzzleShape, 5);
	puzzle4->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud12 = new UserData();
	ud12->id = 12;
	ud12->objType = FilterGroup::eSTUFF;
	puzzle4->userData = ud12;
	setupFiltering(puzzle4, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzle4);

	PxRigidDynamic* puzzle5 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-51.231940, 0.088339, 17.490640)), *puzzleShape, 5);
	puzzle5->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud13 = new UserData();
	ud13->id = 13;
	ud13->objType = FilterGroup::eSTUFF;
	puzzle5->userData = ud13;
	setupFiltering(puzzle5, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzle5);

	PxRigidDynamic* puzzle6 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(-60.321940, 0.078339, 15.250650)), *puzzleShape, 5);
	puzzle6->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	UserData* ud14 = new UserData();
	ud14->id = 14;
	ud14->objType = FilterGroup::eSTUFF;
	puzzle6->userData = ud14;
	setupFiltering(puzzle6, FilterGroup::eSTUFF, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*puzzle6);
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

		gInstance->mScene->addActor(*actor);
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

void createEnvironment()
{
	PxShape* floor = gInstance->mPhysics->createShape(PxBoxGeometry(3.758223 * 0.687897, 0.718908 * 0.687897, 4.980401 * 0.6878971), *gInstance->mMaterial);
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
		setupFiltering(*i, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
		gInstance->mScene->addActor(**i);
	}

	PxShape* floor2s = gInstance->mPhysics->createShape(PxBoxGeometry(6.143844, 1.809403 * 0.5, 7.119962), *gInstance->mMaterial);
	PxRigidStatic* floor2 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.800447, -0.123929, 21.986960)), *floor2s);
	setupFiltering(floor2, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*floor2);

	PxShape* lowWall = gInstance->mPhysics->createShape(PxBoxGeometry(9.993652 * 0.3554256, 3.502743 * 0.3554256, 2.433008 * 0.3554256), *gInstance->mMaterial);
	PxShape* lowWall1 = gInstance->mPhysics->createShape(PxBoxGeometry(9.993652 * 0.3554256, 3.502743 * 0.3554256, 2.433008), *gInstance->mMaterial);
	PxShape* lowWall2 = gInstance->mPhysics->createShape(PxBoxGeometry(9.993652 * 0.25, 3.502743 * 0.25, 2.433008 * 0.25), *gInstance->mMaterial);
	vector<PxRigidStatic*> lowWalls;
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-58.154770, -1.456249, 21.361720), PxQuat(0, 0.4907745, 0, 0.8712866)), *lowWall1));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.184440, -0.759763, -29.343510), PxQuat(0, 0.7071068, 0, 0.7071068)), *lowWall));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.184440, -0.759764, -36.043500), PxQuat(0, 0.7071068, 0, 0.7071068)), *lowWall));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-14.125430, -0.459732, 49.933330)), *lowWall));
	lowWalls.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-27.230740, -0.253092, 31.087360)), *lowWall2));
	for (auto i = lowWalls.begin(); i != lowWalls.end(); ++i) {
		setupFiltering(*i, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
		gInstance->mScene->addActor(**i);
	}

	PxShape* faceWall = gInstance->mPhysics->createShape(PxBoxGeometry(7.061144 * 0.3536491, 6.413379 * 0.3536491, 0.558719 * 0.3536491), *gInstance->mMaterial);
	PxShape* faceWall2 = gInstance->mPhysics->createShape(PxBoxGeometry(7.061144 * 0.5, 6.413379 * 0.5, 0.558719 * 0.5), *gInstance->mMaterial);
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
		setupFiltering(*i, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
		gInstance->mScene->addActor(**i);
	}

	PxShape* column = gInstance->mPhysics->createShape(PxBoxGeometry(3.346645 * 0.275378, 6.738008 * 0.275378, 4.355217 * 0.275378), *gInstance->mMaterial);
	vector<PxRigidStatic*> columns;
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-32.449540, 0.741323, 10.731020)), *column));
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-12.429860, 0.664695, 48.366620)), *column));
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-14.018170, 0.664695, 48.366620)), *column));
	columns.emplace_back(PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-15.618160, 0.664695, 48.366620)), *column));
	for (auto i = columns.begin(); i != columns.end(); ++i) {
		setupFiltering(*i, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
		gInstance->mScene->addActor(**i);
	}

	PxShape* stairShape = createTriangleMesh("meshData\\stairMeshData.json", PxVec3(1, 1, 1));
	PxRigidStatic* stair1 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.021679 * 2, -0.4976641 * 2, 8.18721 * 2)), *stairShape);
	setupFiltering(stair1, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*stair1);
	PxRigidStatic* stair2 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-9.021679 * 2, -0.4976632 * 2, 13.68721 * 2)), *stairShape);
	setupFiltering(stair2, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*stair2);

	PxShape* pillar = gInstance->mPhysics->createShape(PxCapsuleGeometry(0.75 * 0.25 * 2, 4.5 * 0.25 * 2), *gInstance->mMaterial);
	pillar->setLocalPose(PxTransform(PxQuat(PxHalfPi, PxVec3(0, 0, 1))));
	PxRigidStatic* pillar1 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.943360, 2.692040, 27.274410)), *pillar);
	setupFiltering(pillar1, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*pillar1);
	PxRigidStatic* pillar2 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-11.943360, 2.692039, 16.574400)), *pillar);
	setupFiltering(pillar2, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*pillar2);

	PxShape* caveShape = createTriangleMesh("meshData\\caveMeshData.json", PxVec3(1, 1, 1));
	PxRigidStatic* cave = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-27.47168 * 2, -0.6667469 * 2, 6.93721 * 2), PxQuat(0, 0.6156477, 0, 0.7880215)), *caveShape);
	setupFiltering(cave, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*cave);
	PxShape* caveCeilingShape = gInstance->mPhysics->createShape(PxBoxGeometry(20 * 0.5, 0.05, 20 * 0.5), *gInstance->mMaterial);
	PxRigidStatic* caveCeiling = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-27.47168 * 2, -0.6667469 * 2 + 6.56052, 6.93721 * 2), PxQuat(0, 0.6156477, 0, 0.7880215)), *caveCeilingShape);
	setupFiltering(caveCeiling, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*caveCeiling);

	PxShape* archDoorShape = createTriangleMesh("meshData\\archDoorMeshData.json", PxVec3(1, 1, 1));
	PxRigidStatic* archDoor = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.4157949 * 2, -0.4176045 * 2, -12.75075 * 2)), *archDoorShape);
	setupFiltering(archDoor, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*archDoor);
	PxShape* archDoorWallShape = createTriangleMesh("meshData\\archDoorWallMeshData.json", PxVec3(1, 1, 1));
	PxRigidStatic* archDoorWall = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(-0.4231954 * 2, -0.4162923 * 2, -12.75581 * 2)), *archDoorWallShape);
	setupFiltering(archDoorWall, FilterGroup::eBACKGROUND, FilterGroup::eBULLET);
	gInstance->mScene->addActor(*archDoorWall);
}

int main()
{
	gInstance = PhysXClass::getInstance();

	createObject();
	createEnvironment();
	PxShape* map1Shape = createTriangleMesh("meshData\\shellMeshData.json", PxVec3(1, 1, 1));
	PxRigidStatic* map1 = PxCreateStatic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *map1Shape);
	gInstance->mScene->addActor(*map1);

	cout << "맵 생성 완료\n";

	/*PxRigidStatic* groundPlane = PxCreatePlane(*gInstance->mPhysics, PxPlane(0, 1, 0, 0), *gInstance->mMaterial);
	gInstance->mScene->addActor(*groundPlane);

	PxRigidDynamic* box = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0.5, 2.57)), PxBoxGeometry(0.5, 0.25, 1.0), *gInstance->mMaterial, 5);
	box->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	int* ud = new int;
	*ud = 0;
	box->userData = (void*)ud;
	gInstance->mScene->addActor(*box);*/

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
	}

	PxControllerManager* manager = PxCreateControllerManager(*gInstance->mScene);

	PxCapsuleControllerDesc bodyDesc;
	bodyDesc.contactOffset = 0.05f;
	bodyDesc.height = 1.5f;
	bodyDesc.radius = 0.25f;
	bodyDesc.stepOffset = 0.2f;
	bodyDesc.material = gInstance->mMaterial;
	bodyDesc.position = PxExtendedVec3(0, 0.75, 0);
	bodyDesc.density = 5;

	addrlen = sizeof(clientaddr);
	for (int i = 0; i < CLIENT_NUM; ++i) {
		clients[i].m_recv_over.id = i;
		clients[i].m_sock = accept(g_lSocket, (SOCKADDR*)&clientaddr, &addrlen);
		err_display("accept: ");
		printf("\n클라이언트 접속: IP 주소=%s,포트 번호=%d\n",
			inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port));

		clients[i].m_body = manager->createController(bodyDesc);
		PxShape* lHand = gInstance->mPhysics->createShape(PxBoxGeometry(0.01, 0.04, 0.095), *gInstance->mMaterial);
		lHand->setLocalPose(PxTransform(PxVec3(-0.03, -0.015, -0.095), PxQuat(PxHalfPi / 3, PxVec3(1, 0, 0))));
		clients[i].m_lHand = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *lHand, 5);
		//clients[i].m_lHand = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), PxBoxGeometry(0.04, 0.095, 0.01), *gInstance->mMaterial, 5);
		clients[i].m_lHand->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gInstance->mScene->addActor(*clients[i].m_lHand);
		lHand->release();
		PxShape* rHand = gInstance->mPhysics->createShape(PxBoxGeometry(0.01, 0.04, 0.095), *gInstance->mMaterial);
		rHand->setLocalPose(PxTransform(PxVec3(0.03, -0.015, -0.095), PxQuat(PxHalfPi / 3, PxVec3(1, 0, 0))));
		clients[i].m_rHand = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), *rHand, 5);
		//clients[i].m_rHand = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0, 0)), PxBoxGeometry(0.04, 0.095, 0.01), *gInstance->mMaterial, 5);
		clients[i].m_rHand->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gInstance->mScene->addActor(*clients[i].m_rHand);
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

	auto start_t = high_resolution_clock::now();
	auto end_t = high_resolution_clock::now();
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

		end_t = high_resolution_clock::now();
		auto elapsedTime = duration_cast<milliseconds>(end_t - start_t).count();
		float et = elapsedTime * 0.001;
		if (et >= 1.f / 60) {
			start_t = end_t;
			gInstance->stepPhysics(et);
			send_object_move_packet();
			gInstance->mSimulationEventCallback.removedActorsLock.lock();
			if (gInstance->mSimulationEventCallback.removedActors.empty()) {
				for (auto iter = gInstance->mSimulationEventCallback.removedActors.begin(); iter != gInstance->mSimulationEventCallback.removedActors.end(); ++iter)
					(*iter)->release();
				gInstance->mSimulationEventCallback.removedActors.clear();
			}
			gInstance->mSimulationEventCallback.removedActorsLock.unlock();
		}
		SleepEx(1, true);
	}

	// closesocket()
	closesocket(g_lSocket);

	// 윈속 종료
	WSACleanup();
	return 0;
}