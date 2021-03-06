#include "JsonParser.h"
#include <fstream>
#include <iostream>

using namespace std;

JsonParser::JsonParser()
{
	builder["collectComments"] = true;
}

void JsonParser::parseTriangleMeshFile(const char* fileName, PxU32* numVertices, PxVec3*& vertices, PxU32* numTriangles, PxU32*& indices,PxVec3 scale)
{
	ifstream in(fileName, ios::binary);

	if (!parseFromStream(builder, in, &root, &errs)) {
		cout << errs << endl;
		return;
	}

	Value verticesValue = root["vertices"];
	*numVertices = verticesValue["numVertices"].asUInt();
	vertices = new PxVec3[*numVertices];
	Value vertexValue = verticesValue["vertex"];
	for (unsigned int i = 0; i < *numVertices; ++i) {
		vertices[i].x = vertexValue[i]["x"].asFloat() * 0.01 * scale.x;
		vertices[i].y = vertexValue[i]["y"].asFloat() * 0.01 * scale.y;
		vertices[i].z = vertexValue[i]["z"].asFloat() * 0.01 * scale.z;
	}

	Value indicesValue = root["indices"];
	*numTriangles = indicesValue["numTriangles"].asUInt();
	indices = new PxU32[*numTriangles * 3];
	Value indexValue = indicesValue["index"];
	for (unsigned int i = 0; i < *numTriangles * 3; ++i) {
		indices[i] = indexValue[i].asUInt();
	}
}
