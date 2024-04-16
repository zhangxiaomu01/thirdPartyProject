#include "MeshOverlapTest.h"

#include <iostream>

#include "foundation/PxAllocator.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "cooking/PxCooking.h"
#include "common/PxTolerancesScale.h"
#include "PxTriangleMeshCreator.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxGeometryQuery.h"
#include "geometry/PxMeshQuery.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"
#include "mesh/GuTriangleMesh.h"
#include "mesh/GuMidphaseInterface.h"

namespace MeshOverlap {

	void MeshOverlapTest::initPhysics()
	{
		gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	}
	void MeshOverlapTest::releasePhysics()
	{
		PX_RELEASE(gFoundation);
	}
	
	bool MeshOverlapTest::overlap(uint vertexCount01, 
								 float* vertices01, 
								 uint indexCount01, 
								 uint* indices01,
								 uint vertexCount02, 
								 float* vertices02, 
								 uint indexCount02, 
								 uint* indices02, 
								 OverlapTransform transform01, 
								 OverlapTransform transform02)
	{
		physx::PxTolerancesScale scale;
		physx::PxCookingParams params(scale);

		physx::PxTriangleMeshDesc meshDesc01;
		meshDesc01.points.count = vertexCount01 / 3;
		meshDesc01.points.stride = 3 * sizeof(float);
		meshDesc01.points.data = vertices01;

		meshDesc01.triangles.count = indexCount01 / 3;
		meshDesc01.triangles.stride = 3 * sizeof(physx::PxU32);
		meshDesc01.triangles.data = indices01;

		physx::PxTriangleMeshDesc meshDesc02;
		meshDesc02.points.count = vertexCount02 / 3;
		meshDesc02.points.stride = 3 * sizeof(float);
		meshDesc02.points.data = vertices02;

		meshDesc02.triangles.count = indexCount02 / 3;
		meshDesc02.triangles.stride = 3 * sizeof(physx::PxU32);
		meshDesc02.triangles.data = indices02;

		physx::PxTriangleMesh* triangleMesh1 = MeshOverlap::PxTriangleMeshCreator::createTriangleMesh(params, meshDesc01);
		physx::PxTriangleMesh* triangleMesh2 = MeshOverlap::PxTriangleMeshCreator::createTriangleMesh(params, meshDesc02);

		physx::PxTriangleMeshGeometry geometry01 = physx::PxTriangleMeshGeometry(triangleMesh1);
		physx::PxTriangleMeshGeometry geometry02 = physx::PxTriangleMeshGeometry(triangleMesh2);

		bool isOverlapping = meshMeshOverlap(geometry01,
			physx::PxTransform(physx::PxVec3(transform01.x, transform01.y, transform01.z)),
			geometry02,
			physx::PxTransform(physx::PxVec3(transform02.x, transform02.y, transform02.z)));

		std::cout << "The two mesh is overlapping? " << isOverlapping << std::endl;

		return isOverlapping;
	}
	bool MeshOverlapTest::overlap(std::vector<float>& vertices01, 
		std::vector<uint>& indices01, 
		std::vector<float>& vertices02, 
		std::vector<uint>& indices02, 
		OverlapTransform transform01,
		OverlapTransform transform02)
	{

		return overlap(vertices01.size(), 
						vertices01.data(), 
						indices01.size(), 
						indices01.data(), 
						vertices02.size(), 
						vertices02.data(), 
						indices02.size(), 
						indices02.data(),
						transform01,
						transform02);
	}

	bool MeshOverlapTest::meshMeshOverlap(const physx::PxGeometry& geom0,
		const physx::PxTransform& pose0,
		const physx::PxGeometry& geom1,
		const physx::PxTransform& pose1)
	{

		PX_ASSERT(geom0.getType() == PxGeometryType::eTRIANGLEMESH);
		PX_ASSERT(geom1.getType() == PxGeometryType::eTRIANGLEMESH);

		const PxTriangleMeshGeometry& meshGeom0 = static_cast<const PxTriangleMeshGeometry&>(geom0);
		const PxTriangleMeshGeometry& meshGeom1 = static_cast<const PxTriangleMeshGeometry&>(geom1);

		const physx::Gu::TriangleMesh* tm0 = static_cast<const physx::Gu::TriangleMesh*>(meshGeom0.triangleMesh);
		const physx::Gu::TriangleMesh* tm1 = static_cast<const physx::Gu::TriangleMesh*>(meshGeom1.triangleMesh);

		// PT: only implemented for BV4
		if (!tm0 || !tm1 || tm0->getConcreteType() != PxConcreteType::eTRIANGLE_MESH_BVH34 || tm1->getConcreteType() != PxConcreteType::eTRIANGLE_MESH_BVH34)
			return PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxGeometryQuery::overlap(): only available between two BVH34 triangles meshes.");

		class AnyHitReportCallback : public PxReportCallback<PxGeomIndexPair>
		{
		public:
			AnyHitReportCallback()
			{
				mCapacity = 1;
			}

			virtual	bool	flushResults(PxU32, const PxGeomIndexPair*)
			{
				return false;
			}
		};

		AnyHitReportCallback callback;

		// PT: ...so we don't need a table like for the other ops, just go straight to BV4
		return physx::Gu::intersectMeshVsMesh_BV4(callback, *tm0, pose0, meshGeom0.scale, 
			*tm1, pose1, meshGeom1.scale, physx::PxMeshMeshQueryFlag::eDEFAULT, 0.0f);

	}
}