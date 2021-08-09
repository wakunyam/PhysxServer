#pragma once
constexpr int SERVER_PORT = 9000;
constexpr int MAX_ID_LEN = 10;

enum HandState {
	eDEFAULT = 0,
	eGUN = 1
};

constexpr bool LEFT_HAND = true;
constexpr bool RIGHT_HAND = false;

#pragma pack(push, 1)
constexpr char SC_PACKET_LOGIN_OK = 0;
constexpr char SC_PACKET_MOVE = 1;
constexpr char SC_PACKET_ENTER = 2;
constexpr char SC_PACKET_GRAB = 3;
constexpr char SC_PACKET_OBJECT_MOVE = 4;
constexpr char SC_PACKET_CHANGE_HAND_STATE = 5;
constexpr char SC_PACKET_MONSTER_MOVE = 6;
constexpr char SC_PACKET_MONSTER_REMOVE = 7;

constexpr char CS_PACKET_LOGIN = 0;
constexpr char CS_PACKET_MOVE = 1;
constexpr char CS_PACKET_GRAB = 2;
constexpr char CS_PACKET_CHANGE_HAND_STATE = 3;
constexpr char CS_PACKET_MONSTER_MOVE = 4;

struct sc_packet_login_ok {
	char size;
	char type;
	int id;
	float pX, pY, pZ;
	float rX, rY, rZ, rW;
};

struct sc_packet_move {
	char size;
	char type;
	int id;
	float body_pX, body_pY, body_pZ;
	float body_rX, body_rY, body_rZ, body_rW;
	float lHand_pX, lHand_pY, lHand_pZ;
	float lHand_rX, lHand_rY, lHand_rZ, lHand_rW;
	float rHand_pX, rHand_pY, rHand_pZ;
	float rHand_rX, rHand_rY, rHand_rZ, rHand_rW;

	void setHandPose(const PxTransform& lHand, const PxTransform& rHand) {
		lHand_pX = lHand.p.x;
		lHand_pY = lHand.p.y;
		lHand_pZ = lHand.p.z;
		lHand_rX = lHand.q.x;
		lHand_rY = lHand.q.y;
		lHand_rZ = lHand.q.z;
		lHand_rW = lHand.q.w;

		rHand_pX = rHand.p.x;
		rHand_pY = rHand.p.y;
		rHand_pZ = rHand.p.z;
		rHand_rX = rHand.q.x;
		rHand_rY = rHand.q.y;
		rHand_rZ = rHand.q.z;
		rHand_rW = rHand.q.w;
	}
};

struct sc_packet_enter {
	char size;
	char type;
	int id;
	char name[MAX_ID_LEN];
	float pX, pY, pZ;
	float rX, rY, rZ, rW;
};

struct sc_packet_grab {
	char size;
	char type;
	int id;
	bool hand;
	bool grab;
};

struct sc_packet_object_move {
	char size;
	char type;
	int id;
	float pX, pY, pZ;
	float rX, rY, rZ, rW;
};

struct sc_packet_change_hand_state {
	char size;
	char type;
	int id;
	bool hand;
	HandState state;
};

struct sc_packet_monster_move {
	char size;
	char type;
	int id;
	float pX, pY, pZ;
	float rX, rY, rZ, rW;
};

struct sc_packet_monster_remove {
	char size;
	char type;
	int id;
};

struct cs_packet_login {
	char size;
	char type;
	//bool isVR;
};

struct cs_packet_move {
	char size;
	char type;
	float body_pX, body_pY, body_pZ;
	float body_rX, body_rY, body_rZ, body_rW;
	float lHand_pX, lHand_pY, lHand_pZ;
	float lHand_rX, lHand_rY, lHand_rZ, lHand_rW;
	float rHand_pX, rHand_pY, rHand_pZ;
	float rHand_rX, rHand_rY, rHand_rZ, rHand_rW;

	PxTransform getLHandPose() {
		return PxTransform(PxVec3(lHand_pX, lHand_pY, lHand_pZ), PxQuat(lHand_rX, lHand_rY, lHand_rZ, lHand_rW));
	}

	PxTransform getRHandPose() {
		return PxTransform(PxVec3(rHand_pX, rHand_pY, rHand_pZ), PxQuat(rHand_rX, rHand_rY, rHand_rZ, rHand_rW));
	}
};

struct cs_packet_grab {
	char size;
	char type;
	bool hand;
	bool grab;
	float pX, pY, pZ;
	float dirX, dirY, dirZ;
};

struct cs_packet_change_hand_state {
	char size;
	char type;
	bool hand;
	HandState state;
};

struct cs_packet_monster_move {
	char size;
	char type;
	int id;
	float pX, pY, pZ;
	float rX, rY, rZ, rW;
};
#pragma pack(pop)