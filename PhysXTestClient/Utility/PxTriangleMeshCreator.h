#pragma once

#include "geometry/PxTriangleMesh.h"
#include "cooking/PxCooking.h"
#include "foundation/PxFPU.h"
#include "cooking/GuCookingTriangleMesh.h"
#include "common/PxInsertionCallback.h"

/// <summary>
/// 从PxCreateTriangleMesh(params, meshDesc)摘出来的核心创建triangleMesh的代码。具体见
/// ..\PhysX\physx\source\geomutils\src\cooking\GuCookingTriangleMesh.cpp line 1347
/// </summary>
namespace MeshOverlap {
	using namespace physx;

	class PxTriangleMeshCreator {
	public:
		static PxTriangleMesh* createTriangleMesh(const PxCookingParams& params, const PxTriangleMeshDesc& desc, 
			PxInsertionCallback& insertionCallback = *PxGetStandaloneInsertionCallback(), 
			PxTriangleMeshCookingResult::Enum* condition = nullptr)
		{
			struct Local
			{
				static PxTriangleMesh* createTriangleMesh(const PxCookingParams& cookingParams_, TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc_, PxInsertionCallback& insertionCallback_, PxTriangleMeshCookingResult::Enum* condition_)
				{
					// cooking code does lots of float bitwise reinterpretation that generates exceptions
					//PX_FPU_GUARD;

					if (condition_)
						*condition_ = PxTriangleMeshCookingResult::eSUCCESS;
					if (!builder.loadFromDesc(desc_, condition_, false))
						return NULL;

					// check if the indices can be moved from 32bits to 16bits
					if (!(cookingParams_.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES))
						builder.checkMeshIndicesSize();

					// bvh33已经弃用
					PxConcreteType::Enum type = PxConcreteType::eTRIANGLE_MESH_BVH34;

					return static_cast<PxTriangleMesh*>(insertionCallback_.buildObjectFromData(type, &builder.getMeshData()));
				}
			};
			BV4TriangleMeshBuilder builder(params);
			return Local::createTriangleMesh(params, builder, desc, insertionCallback, condition);
		}
	};
}