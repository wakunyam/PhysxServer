#pragma once
constexpr int SERVER_PORT = 9000;
constexpr int MAX_ID_LEN = 10;

enum HandState {
	eDEFAULT = 0,
	eGUN,
	eTORCHLIGHT,
	eGEM4
};

enum EnemyState {
	Idle,
	Finding,
	Chasing,
	Attacking,
	Dead
};

constexpr bool LEFT_HAND = true;
constexpr bool RIGHT_HAND = false;

constexpr int bits = 10;

struct compressed_quaternion {
	uint32_t largest : 2;
	uint32_t integer_a : bits;
	uint32_t integer_b : bits;
	uint32_t integer_c : bits;

	void load(float x, float y, float z, float w) {
		const float minimum = -1.0f / 1.414214f;       // 1.0f / sqrt(2)
		const float maximum = +1.0f / 1.414214f;

		const float scale = float((1 << bits) - 1);

		const float abs_x = fabs(x);
		const float abs_y = fabs(y);
		const float abs_z = fabs(z);
		const float abs_w = fabs(w);

		largest = 0;
		float largest_value = abs_x;

		if (abs_y > largest_value)
		{
			largest = 1;
			largest_value = abs_y;
		}

		if (abs_z > largest_value)
		{
			largest = 2;
			largest_value = abs_z;
		}

		if (abs_w > largest_value)
		{
			largest = 3;
			largest_value = abs_w;
		}

		float a = 0;
		float b = 0;
		float c = 0;

		switch (largest)
		{
		case 0:
			if (x >= 0)
			{
				a = y;
				b = z;
				c = w;
			}
			else
			{
				a = -y;
				b = -z;
				c = -w;
			}
			break;

		case 1:
			if (y >= 0)
			{
				a = x;
				b = z;
				c = w;
			}
			else
			{
				a = -x;
				b = -z;
				c = -w;
			}
			break;

		case 2:
			if (z >= 0)
			{
				a = x;
				b = y;
				c = w;
			}
			else
			{
				a = -x;
				b = -y;
				c = -w;
			}
			break;

		case 3:
			if (w >= 0)
			{
				a = x;
				b = y;
				c = z;
			}
			else
			{
				a = -x;
				b = -y;
				c = -z;
			}
			break;
		}

		const float normal_a = (a - minimum) / (maximum - minimum);
		const float normal_b = (b - minimum) / (maximum - minimum);
		const float normal_c = (c - minimum) / (maximum - minimum);

		integer_a = floor(normal_a * scale + 0.5f);
		integer_b = floor(normal_b * scale + 0.5f);
		integer_c = floor(normal_c * scale + 0.5f);
	}

	void save(float& x, float& y, float& z, float& w) const
	{
		const float minimum = -1.0f / 1.414214f;
		const float maximum = +1.0f / 1.414214f;

		const float scale = float((1 << bits) - 1);

		const float inverse_scale = 1.0f / scale;

		const float a = integer_a * inverse_scale * (maximum - minimum) + minimum;
		const float b = integer_b * inverse_scale * (maximum - minimum) + minimum;
		const float c = integer_c * inverse_scale * (maximum - minimum) + minimum;

		switch (largest)
		{
		case 0:
		{
			x = sqrtf(1 - a * a - b * b - c * c);
			y = a;
			z = b;
			w = c;
		}
		break;

		case 1:
		{
			x = a;
			y = sqrtf(1 - a * a - b * b - c * c);
			z = b;
			w = c;
		}
		break;

		case 2:
		{
			x = a;
			y = b;
			z = sqrtf(1 - a * a - b * b - c * c);
			w = c;
		}
		break;

		case 3:
		{
			x = a;
			y = b;
			z = c;
			w = sqrtf(1 - a * a - b * b - c * c);
		}
		break;
		}
	}
};

#pragma pack(push, 1)
constexpr char SC_PACKET_LOGIN_OK = 0;
constexpr char SC_PACKET_MOVE = 1;
constexpr char SC_PACKET_ENTER = 2;
constexpr char SC_PACKET_GRAB = 3;
constexpr char SC_PACKET_OBJECT_MOVE = 4;
constexpr char SC_PACKET_CHANGE_HAND_STATE = 5;
constexpr char SC_PACKET_MONSTER_MOVE = 6;
constexpr char SC_PACKET_MONSTER_REMOVE = 7;
constexpr char SC_PACKET_HAND_MOVE = 8;
constexpr char SC_PACKET_HEAD_MOVE = 9;
constexpr char SC_PACKET_CHANGE_MONSTER_STATE = 10;
constexpr char SC_PACKET_MAP2_CLEAR = 11;
constexpr char SC_PACKET_Y_POS = 12;
constexpr char SC_PACKET_CHANGE_SCENE = 13;
constexpr char SC_PACKET_GET_GEM = 14;

constexpr char CS_PACKET_LOGIN = 0;
constexpr char CS_PACKET_MOVE = 1;
constexpr char CS_PACKET_GRAB = 2;
constexpr char CS_PACKET_CHANGE_HAND_STATE = 3;
constexpr char CS_PACKET_MONSTER_MOVE = 4;
constexpr char CS_PACKET_HAND_MOVE = 5;
constexpr char CS_PACKET_HEAD_MOVE = 6;
constexpr char CS_PACKET_SERVER_HAND_MOVE = 7;
constexpr char CS_PACKET_CHANGE_MONSTER_STATE = 8;
constexpr char CS_PACKET_CHANGE_SCENE = 9;
constexpr char CS_PACKET_Y_POS = 10;
constexpr char CS_PACKET_MAP4_CLEAR = 11;

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
	compressed_quaternion quat;
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
	compressed_quaternion quat;
};

struct sc_packet_monster_remove {
	char size;
	char type;
	int id;
};

struct sc_packet_hand_move {
	char size;
	char type;
	float lHand_pX, lHand_pY, lHand_pZ;
	compressed_quaternion lHandQuat;
	float rHand_pX, rHand_pY, rHand_pZ;
	compressed_quaternion rHandQuat;

	void setHandPose(const PxTransform& lHand, const PxTransform& rHand) {
		lHand_pX = lHand.p.x;
		lHand_pY = lHand.p.y;
		lHand_pZ = lHand.p.z;

		rHand_pX = rHand.p.x;
		rHand_pY = rHand.p.y;
		rHand_pZ = rHand.p.z;
	}
};

struct sc_packet_head_move {
	char size;
	char type;
	float x, y, z;
	compressed_quaternion quat;
};

struct sc_packet_change_monster_state {
	char size;
	char type;
	int monster_id;
	EnemyState state;
};

struct sc_packet_map2_clear {
	char size;
	char type;
};

struct sc_packet_y_pos {
	char size;
	char type;
	float y;
};

struct sc_packet_change_scene {
	char size;
	char type;
};

struct sc_packet_get_gem {
	char size;
	char type;
	int owner_id;
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
	compressed_quaternion quat;
};

struct cs_packet_hand_move {
	char size;
	char type;
	float lx, ly, lz;
	compressed_quaternion lQuat;
	float rx, ry, rz;
	compressed_quaternion rQuat;
};

struct cs_packet_head_move {
	char size;
	char type;
	float x, y, z;
	compressed_quaternion quat;
};

struct cs_packet_server_hand_move {
	char size;
	char type;

	float lHand_pX, lHand_pY, lHand_pZ;
	compressed_quaternion lHandQuat;
	float rHand_pX, rHand_pY, rHand_pZ;
	compressed_quaternion rHandQuat;

	PxTransform getLHandPose() {
		return PxTransform(PxVec3(lHand_pX, lHand_pY, lHand_pZ));
	}

	PxTransform getRHandPose() {
		return PxTransform(PxVec3(rHand_pX, rHand_pY, rHand_pZ));
	}
};

struct cs_packet_change_monster_state {
	char size;
	char type;
	int monster_id;
	EnemyState state;
};

struct cs_packet_change_scene {
	char size;
	char type;
	int scene_id;
};

struct cs_packet_y_pos {
	char size;
	char type;
	float y;
};

struct cs_packet_map4_clear {
	char size;
	char type;
};
#pragma pack(pop)