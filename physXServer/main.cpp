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
constexpr int CLIENT_NUM = 1;

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
	std::lock_guard<std::mutex> l{ gInstance->mSimulationEventCallback.containerLock };
	for (auto iter = gInstance->mSimulationEventCallback.actorContainer.begin(); iter != gInstance->mSimulationEventCallback.actorContainer.end(); ++iter) {
		sc_packet_object_move p;
		p.id = *(int*)(*iter)->userData;
		p.size = sizeof(p);
		p.type = SC_PACKET_OBJECT_MOVE;
		PxTransform t = reinterpret_cast<PxRigidDynamic*>(*iter)->getGlobalPose();
		p.pX = t.p.x;
		p.pY = t.p.y;
		p.pZ = t.p.z;
		p.rX = t.q.x;
		p.rY = t.q.y;
		p.rZ = t.q.z;
		p.rW = t.q.w;

		for (int i = 0; i < CLIENT_NUM; ++i)
			send_packet(i, &p);
	}
}

void grabObject(int id, bool hand, bool grab)
{
	switch (hand) {
	case LEFT_HAND: {
		if (grab) {
			PxRaycastBuffer hit;

			PxVec3 dir = clients[id].m_lHand->getGlobalPose().q.rotate(PxVec3(1, 0, 0));

			/*PxVec3 v(1, 0, 0);
			PxQuat q = clients[id].m_lHand->getGlobalPose().q;
			PxVec3 u(q.x, q.y, q.z);
			PxReal s = q.w;
			PxVec3 vp = 2 * u.dot(v) * u
				+ (s * s - u.dot(u)) * v
				+ 2 * s * u.cross(v);*/

			if (gInstance->mScene->raycast(clients[id].m_lHand->getGlobalPose().p + dir * 0.011, dir, 0.05, hit)) {
				PxRigidActor* blockObject = hit.block.actor;
				clients[id].m_lHandJoint = PxFixedJointCreate(*gInstance->mPhysics, clients[id].m_lHand, PxTransform(PxVec3(-0.01, 0, 0)), blockObject, PxTransform(blockObject->getGlobalPose().p - hit.block.position));
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

			/*PxVec3 v(-1, 0, 0);
			PxQuat q = clients[id].m_lHand->getGlobalPose().q;
			PxVec3 u(q.x, q.y, q.z);
			PxReal s = q.w;
			PxVec3 vp = 2 * u.dot(v) * u
				+ (s * s - u.dot(u)) * v
				+ 2 * s * u.cross(v);*/

			if (gInstance->mScene->raycast(clients[id].m_rHand->getGlobalPose().p + dir * 0.011, dir, 0.05, hit)) {
				PxRigidActor* blockObject = hit.block.actor;
				clients[id].m_rHandJoint = PxFixedJointCreate(*gInstance->mPhysics, clients[id].m_rHand, PxTransform(PxVec3(-0.01, 0, 0)), blockObject, PxTransform(hit.block.position - blockObject->getGlobalPose().p));
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

		/*clients[id].m_lHand->setPosition(PxExtendedVec3(p->lHand_pX, p->lHand_pY, p->lHand_pZ));
		clients[id].lHand_rX = p->lHand_rX;
		clients[id].lHand_rY = p->lHand_rY;
		clients[id].lHand_rZ = p->lHand_rZ;
		clients[id].lHand_rW = p->lHand_rW;

		clients[id].m_rHand->setPosition(PxExtendedVec3(p->rHand_pX, p->rHand_pY, p->rHand_pZ));
		clients[id].rHand_rX = p->rHand_rX;
		clients[id].rHand_rY = p->rHand_rY;
		clients[id].rHand_rZ = p->rHand_rZ;
		clients[id].rHand_rW = p->rHand_rW;*/

		//send_move_packet(id, id);
		if (clients[CLIENT_NUM - 1 - id].conected)
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

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent, PhysXClass& instance)
{
	PxShape* shape = instance.mPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *instance.mMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = instance.mPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			instance.mScene->addActor(*body);
		}
	}
	shape->release();
}

void createObject()
{
	int* id = new int[15];

	PxShape* vase01Shape = gInstance->mPhysics->createShape(PxBoxGeometry(0.5394133/2, 0.8277771/2, 0.5394133/2), *gInstance->mMaterial);
	PxShape* vase02Shape = gInstance->mPhysics->createShape(PxBoxGeometry(0.2987156/2, 0.2937585/2, 1.080622/2), *gInstance->mMaterial);
	PxShape* puzzleHintShape = gInstance->mPhysics->createShape(PxBoxGeometry(1.0/2, 0.5/2, 0.001/2), *gInstance->mMaterial);
	PxShape* puzzleShape = gInstance->mPhysics->createShape(PxBoxGeometry(0.4687378/2, 1.3590325/2, 0.4566437/2), *gInstance->mMaterial);

	PxRigidDynamic* puzzleHint1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(219.56, 3.5, 283.74), PxQuat(PxHalfPi, PxVec3(1, 0, 0))), *puzzleHintShape, 5);
	puzzleHint1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[0] = 0;
	puzzleHint1->userData = (void*)&id[0];
	gInstance->mScene->addActor(*puzzleHint1);

	PxRigidDynamic* puzzleHint2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(267.6, 4.6, 287), PxQuat(PxHalfPi, PxVec3(1, 0, 0))), *puzzleHintShape, 5);
	puzzleHint2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[1] = 1;
	puzzleHint2->userData = (void*)&id[1];
	gInstance->mScene->addActor(*puzzleHint2);

	PxRigidDynamic* puzzleHint3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(265.8, 4.8, 237.5), PxQuat(0.5, -0.5, -0.5, 0.5)), *puzzleHintShape, 5);
	puzzleHint3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[2] = 2;
	puzzleHint3->userData = (void*)&id[2];
	gInstance->mScene->addActor(*puzzleHint3);

	PxRigidDynamic* vase1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(264.4, 4.496517, 315.4), PxQuat(-0.0040726, 0.7070951, 0.7070951, 0.0040726)), *vase02Shape, 5);
	vase1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[3] = 3;
	vase1->userData = (void*)&id[3];
	gInstance->mScene->addActor(*vase1);

	PxRigidDynamic* vase2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(263.8, 4.496517, 315.4), PxQuat(-0.0040726, 0.7070951, 0.7070951, 0.0040726)), *vase02Shape, 5);
	vase2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[4] = 4;
	vase2->userData = (void*)&id[4];
	gInstance->mScene->addActor(*vase2);

	PxRigidDynamic* vase3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(261.5, 4.396511, 315.3)), *vase01Shape, 5);
	vase3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[5] = 5;
	vase3->userData = (void*)&id[5];
	gInstance->mScene->addActor(*vase3);

	PxRigidDynamic* vase4 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(273, 3.496519, 330.4), PxQuat(-0.5, 0.5, 0.5, 0.5)), *vase02Shape, 5);
	vase4->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[6] = 6;
	vase4->userData = (void*)&id[6];
	gInstance->mScene->addActor(*vase4);

	PxRigidDynamic* vase5 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(272.9, 3.396513, 329.3), PxQuat(0, -0.7071068, 0, 0.7071068)), *vase01Shape, 5);
	vase5->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[7] = 7;
	vase5->userData = (void*)&id[7];
	gInstance->mScene->addActor(*vase5);

	PxRigidDynamic* vase6 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(273, 3.496519, 329.9), PxQuat(-0.5, 0.5, 0.5, 0.5)), *vase02Shape, 5);
	vase6->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[8] = 8;
	vase6->userData = (void*)&id[8];
	gInstance->mScene->addActor(*vase6);

	PxRigidDynamic* puzzle1 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(212.88, 2.69, 277.48)), *puzzleShape, 5);
	puzzle1->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[9] = 9;
	puzzle1->userData = (void*)&id[9];
	gInstance->mScene->addActor(*puzzle1);

	PxRigidDynamic* puzzle2 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(220.35, 2.69, 279.43)), *puzzleShape, 5);
	puzzle2->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[10] = 10;
	puzzle2->userData = (void*)&id[10];
	gInstance->mScene->addActor(*puzzle2);

	PxRigidDynamic* puzzle3 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(216.8, 2.69, 278.71)), *puzzleShape, 5);
	puzzle3->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[11] = 11;
	puzzle3->userData = (void*)&id[11];
	gInstance->mScene->addActor(*puzzle3);

	PxRigidDynamic* puzzle4 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(219.75, 2.62, 290.45)), *puzzleShape, 5);
	puzzle4->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[12] = 12;
	puzzle4->userData = (void*)&id[12];
	gInstance->mScene->addActor(*puzzle4);

	PxRigidDynamic* puzzle5 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(220.14, 2.65, 289.47)), *puzzleShape, 5);
	puzzle5->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[13] = 13;
	puzzle5->userData = (void*)&id[13];
	gInstance->mScene->addActor(*puzzle5);

	PxRigidDynamic* puzzle6 = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(211.05, 2.64, 287.23)), *puzzleShape, 5);
	puzzle6->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	id[14] = 14;
	puzzle6->userData = (void*)&id[14];
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

void createBV33TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
	bool skipMeshCleanup, bool skipEdgeData, bool inserted, bool cookingPerformance, bool meshSizePerfTradeoff, const PxTransform& transform)
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
		//PxRigidStatic* actor = gInstance->mPhysics->createRigidStatic(transform);
		PxShape* shape = gInstance->mPhysics->createShape(triGeom, *gInstance->mMaterial);
		shape->setLocalPose(PxTransform(PxVec3(-6.973256/2, -0.394945/2, 21.30691/2)));
		PxRigidStatic* actor = PxCreateStatic(*gInstance->mPhysics, transform, *shape);
		//PxRigidActorExt::createExclusiveShape(*actor, triGeom, *gInstance->mMaterial);
		gInstance->mScene->addActor(*actor);
		shape->release();
	}
}

void createMap(const char* fileName, const PxTransform& transform)
{
	PxU32 numVertices;
	PxVec3* vertices = nullptr;
	PxU32 numTriangles;
	PxU32* indices = nullptr;

	gParser.parseTriangleMeshFile(fileName, &numVertices, vertices, &numTriangles, indices);

	createBV33TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, false, false, transform);

	delete[] vertices;
	delete[] indices;
}

int main()
{
	gInstance = PhysXClass::getInstance();

	/*createObject();
	createMap("meshData.json", PxTransform(PxVec3(277.0434, 3.7, 265.1256)));*/

	PxRigidStatic* groundPlane = PxCreatePlane(*gInstance->mPhysics, PxPlane(0, 1, 0, 0), *gInstance->mMaterial);

	gInstance->mScene->addActor(*groundPlane);

	PxRigidDynamic* box = PxCreateDynamic(*gInstance->mPhysics, PxTransform(PxVec3(0, 0.5, 2.57)), PxBoxGeometry(0.5, 0.5, 0.5), *gInstance->mMaterial, 5);
	box->setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, true);
	int* ud = new int;
	*ud = 0;
	box->userData = (void*)ud;
	gInstance->mScene->addActor(*box);

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

	// 비동기 accept를 어떻게 할지 향후 생각
	// 1. 쓰레드를 사용해 accept와 worker 분리
	/*SOCKET cSocket = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED);
	g_accept_over.wsabuf.len = static_cast<ULONG>(cSocket);
	ZeroMemory(&g_accept_over.over, sizeof(g_accept_over.over));*/

	PxControllerManager* manager = PxCreateControllerManager(*gInstance->mScene);

	PxCapsuleControllerDesc bodyDesc;
	bodyDesc.contactOffset = 0.05f;
	bodyDesc.height = 1.5f;
	bodyDesc.radius = 0.25f;
	bodyDesc.stepOffset = 0.2f;
	bodyDesc.material = gInstance->mMaterial;
	bodyDesc.position = PxExtendedVec3(0, 0.75, 0);
	bodyDesc.density = 5;

	//while (1) {
		// accept()
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
	//}

	/*while (clients[0].conected == false && clients[1].conected == false);
	send_enter_packet(0, 1);
	send_enter_packet(1, 0);*/

	/*while (!all_ready) {
		if (clients[0].conected && clients[1].conected) all_ready = true;
		SleepEx(1, true);
	}*/

	//send_enter_packet(0, 1);
	//send_enter_packet(1, 0);

	bool loop = true;


	/*for (auto& c : clients) {
		c.m_controller = manager->createController(desc);
		c.m_controller->setFootPosition(PxExtendedVec3(0, 0, 0));
	}*/

	PxReal stackZ = 0.0f;

	float x, z;
	x = z = 0;
	auto start_t = high_resolution_clock::now();
	while (loop) {
		if (_kbhit()) {
			switch (_getch())
			{
			case 27:
				loop = false;
				break;
			case ' ': {
				//createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f, *gInstance);
				break;
			}
			default:
				break;
			}
		}
		/*PxVec3 o(c->getFootPosition().x, c->getFootPosition().y, c->getFootPosition().z);
		PxVec3 ud(0, -1, 0);
		PxReal d(0.001);
		PxRaycastBuffer hit;
		instance->mScene->raycast(o, ud, d, hit);

		if (!hit.hasAnyHits()) {
			c->move(PxVec3(0, -1, 0), 0.1, 0, PxControllerFilters());
			if (c->getFootPosition().y < PxExtended(0))
				c->setFootPosition(PxExtendedVec3(c->getFootPosition().x, 0, c->getFootPosition().z));
		}*/
		/*if (c->getFootPosition().y > PxExtended(0)) {
			c->move(PxVec3(0, -1, 0), 0.1, 0, PxControllerFilters());
			if (c->getFootPosition().y < PxExtended(0))
				c->setFootPosition(PxExtendedVec3(c->getFootPosition().x, 0, c->getFootPosition().z));
		}*/
		auto end_t = high_resolution_clock::now();
		SleepEx(1, true);
		auto exec_t = duration_cast<milliseconds>(end_t - start_t).count();
		if (exec_t > 1000 / 60) {
			start_t = end_t;
			gInstance->stepPhysics();
			send_object_move_packet();
		}
	}

	// closesocket()
	closesocket(g_lSocket);

	// 윈속 종료
	WSACleanup();
	return 0;
}