// PhysXTestClient.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>
#include <vector>

#include "OBJ_Loader.h"
#include "MeshOverlapTest.h"

typedef unsigned int uint;

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

		return;
	}
	else
	{
		std::cerr << "OBJ Load error!" << std::endl;
	}
	return;
}

void setUp() {
	const int model01 = 1;
	const int model02 = 2;

	//const std::string modelPath01 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_0" + std::to_string(model01) + ".obj";
	//const std::string modelPath02 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_0" + std::to_string(model02) + ".obj";

	//const std::string modelPath01 = "D:\\Models\\OBJ\\CarModelList\\Model_0" + std::to_string(model01) + ".obj";
	//const std::string modelPath02 = "D:\\Models\\OBJ\\CarModelList\\Model_0" + std::to_string(model02) + ".obj";

	//const std::string modelPath01 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_Box_Close_0" + std::to_string(model01) + ".obj";
	//const std::string modelPath02 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_Box_Close_0" + std::to_string(model02) + ".obj";

	const std::string modelPath01 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_Tri_Close_0" + std::to_string(model01) + ".obj";
	const std::string modelPath02 = "D:\\Models\\OBJ\\BoolTestObjects\\Model_Tri_Close_0" + std::to_string(model02) + ".obj";


	std::vector<uint> modelIndices01;
	std::vector<float> modelVertices01;

	std::vector<uint> modelIndices02;
	std::vector<float> modelVertices02;

	loadObjToScene(modelPath01, modelIndices01, modelVertices01);
	loadObjToScene(modelPath02, modelIndices02, modelVertices02);

	MeshOverlap::MeshOverlapTest meshOverlapTest = MeshOverlap::MeshOverlapTest();

	meshOverlapTest.initPhysics();

	meshOverlapTest.overlap(modelVertices01, modelIndices01, 
							modelVertices02, modelIndices02,
							MeshOverlap::OverlapTransform(0.f, -0.1f, 0.f),
							MeshOverlap::OverlapTransform(0.f, 0.f, 0.f));

	modelIndices01.clear();
	modelIndices02.clear();
	modelVertices01.clear();
	modelVertices02.clear();

	meshOverlapTest.releasePhysics();
}

int main()
{
	setUp();
	std::cout << "Hello World!\n";
}

