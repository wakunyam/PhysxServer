#pragma once
#include "json/json.h"
#include "PxPhysicsAPI.h"

using namespace Json;
using namespace physx;

class JsonParser
{
	CharReaderBuilder builder;
	JSONCPP_STRING errs;
	Value root;
public:
	JsonParser();
	void parseTriangleMeshFile(const char* fileName, PxU32* numVertices, PxVec3*& vertices, PxU32* numTriangles, PxU32*& indices);
};

