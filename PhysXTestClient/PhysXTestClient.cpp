// PhysXTestClient.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>
#include <vector>

#include "OBJ_Loader.h"
#include "extensions/PxDefaultAllocator.h"
#include "extensions/PxDefaultErrorCallback.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxAllocator.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "cooking/PxCooking.h"
#include "common/PxTolerancesScale.h"
#include "PxTriangleMeshGeometry.h"
#include "PxGeometryQuery.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"

#include "PxTriangleMeshCreator.h"

typedef unsigned int uint;

static physx::PxDefaultAllocator		gAllocator;
static physx::PxDefaultErrorCallback	gErrorCallback;
static physx::PxFoundation* gFoundation = NULL;

void loadObjToScene(const std::string& filename, std::vector<uint>& indices, std::vector<float>& vertices) {
	objl::Loader Loader;

	// Load .obj File
	bool loadout = Loader.LoadFileSimple(filename);
	std::cout << "current filename: " << filename << std::endl;


	if (loadout)
	{
		// Copy one of the loaded meshes to be our current mesh
		objl::Mesh curMesh = Loader.LoadedMeshes[0];

		std::chrono::steady_clock::time_point loadMeshBeginTime = std::chrono::steady_clock::now();

		for (int j = 0; j < curMesh.Vertices.size(); j++)
		{
			vertices.emplace_back(curMesh.Vertices[j].Position.X);
			vertices.emplace_back(curMesh.Vertices[j].Position.Y);
			vertices.emplace_back(curMesh.Vertices[j].Position.Z);
		}

		for (int j = 0; j < curMesh.Indices.size(); j++)
		{
			indices.emplace_back(curMesh.Indices[j]);
		}

		//physx::PxTriangleMeshDesc meshDesc;
		//meshDesc.points.count = curMesh.Vertices.size();
		//meshDesc.points.stride = 3 * sizeof(float);
		//meshDesc.points.data = vertexData;

		//meshDesc.triangles.count = facesSize;
		//meshDesc.triangles.stride = 3 * sizeof(PxU32);
		//meshDesc.triangles.data = indexData;

		//PxTolerancesScale scale;
		//PxCookingParams params(scale);

		//PxTriangleMesh* res = MeshOverlap::PxTriangleMeshCreator::createTriangleMesh(params, meshDesc);

		return;
	}
	else
	{
		std::cerr << "OBJ Load error!" << std::endl;
	}
	return;
}

void initPhysics() {
	const uint PX_PHYSICS_VERSION = 84082944;
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	const int model01 = 1;
	const int model02 = 2;

	//const std::string modelPath01 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_0" + std::to_string(model01) + ".obj";
	//const std::string modelPath02 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_0" + std::to_string(model02) + ".obj";

	//const std::string modelPath01 = "D:\\Models\\OBJ\\CarModelList\\Model_" + std::to_string(model01) + ".obj";
	//const std::string modelPath02 = "D:\\Models\\OBJ\\CarModelList\\Model_" + std::to_string(model02) + ".obj";

	const std::string modelPath01 = "D:\\ProjectNew\\physX\\PhysX\\physx\\client\\Models\\Model_Box_Close_0" + std::to_string(model01) + ".obj";
	const std::string modelPath02 = "D:\\ProjectNew\\physX\\PhysX\\physx\\client\\Models\\Model_Box_Close_0" + std::to_string(model02) + ".obj";

	std::vector<uint> modelIndices01;
	std::vector<float> modelVertices01;

	std::vector<uint> modelIndices02;
	std::vector<float> modelVertices02;

	loadObjToScene(modelPath01, modelIndices01, modelVertices01);
	loadObjToScene(modelPath02, modelIndices02, modelVertices02);

	physx::PxTriangleMeshDesc meshDesc;

	meshDesc.points.count = modelVertices01.size() / 3;
	meshDesc.points.stride = 3 * sizeof(float);
	meshDesc.points.data = modelVertices01.data();

	meshDesc.triangles.count = modelIndices01.size() / 3;
	meshDesc.triangles.stride = 3 * sizeof(physx::PxU32);
	meshDesc.triangles.data = modelIndices01.data();

	physx::PxTolerancesScale scale;
	physx::PxCookingParams params(scale);

	physx::PxTriangleMesh* triangleMesh1 = PxCreateTriangleMesh(params, meshDesc);
	//physx::PxTriangleMesh* triangleMesh1 = MeshOverlap::PxTriangleMeshCreator::createTriangleMesh(params, meshDesc);
	physx::PxTriangleMesh* triangleMesh2 = PxCreateTriangleMesh(params, meshDesc);

	physx::PxTriangleMeshGeometry geometry01 = physx::PxTriangleMeshGeometry(triangleMesh1);
	physx::PxTriangleMeshGeometry geometry02 = physx::PxTriangleMeshGeometry(triangleMesh2);

	bool isOverlapping = physx::PxGeometryQuery::overlap(geometry01,
		physx::PxTransform(physx::PxVec3(2.5f, 0, 0.f)),
		geometry02,
		physx::PxTransform(physx::PxVec3(0, 0, 0.f)));

	std::cout << "The two mesh is overlapping? " << isOverlapping << std::endl;


	std::cout << "Init physics called!" << std::endl;
}

void cleanupPhysics()
{
	PX_RELEASE(gFoundation);

	printf("SnippetHelloWorld done.\n");
}

int main()
{
	initPhysics();

	cleanupPhysics();

	std::cout << "Hello World!\n";
}

