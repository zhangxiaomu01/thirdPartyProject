#pragma once

#include <vector>

#include "extensions/PxDefaultAllocator.h"
#include "extensions/PxDefaultErrorCallback.h"
#include "foundation/Px.h"
#include "foundation/PxFoundation.h"
#include "geometry/PxGeometry.h"

namespace MeshOverlap {
	typedef unsigned int uint;

	struct OverlapTransform {
		float x = 0.f;
		float y = 0.f;
		float z = 0.f;

		OverlapTransform(float newX, float newY, float newZ) : x(newX), y(newY), z(newZ) {}
	};

	class MeshOverlapTest {
	public:
		MeshOverlapTest() = default;

		void initPhysics();

		void releasePhysics();

		bool overlap(uint vertexCount01, float* vertices01, uint indexCount01, uint* indices01, 
			uint vertexCount02, float* vertices02, uint indexCount02, uint* indices02,
			OverlapTransform transform01 = { 0.f, 0.f, 0.f }, OverlapTransform transform02 = { 0.f, 0.f, 0.f });

		bool overlap(std::vector<float>& vertices01, 
			std::vector<uint>& indices01, 
			std::vector<float>& vertices02, 
			std::vector<uint>& indices02,
			OverlapTransform transform01 = { 0.f, 0.f, 0.f },
			OverlapTransform transform02 = { 0.f, 0.f, 0.f });

	private:
		physx::PxDefaultAllocator		gAllocator;
		physx::PxDefaultErrorCallback	gErrorCallback;
		physx::PxFoundation* gFoundation = nullptr;

		const uint PX_PHYSICS_VERSION = 84082944;

		bool meshMeshOverlap(const physx::PxGeometry& geom0, const physx::PxTransform& pose0,
			const physx::PxGeometry& geom1, const physx::PxTransform& pose1);
	};
}
