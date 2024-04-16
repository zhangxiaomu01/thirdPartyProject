#pragma once

#include "mesh/GuTriangleMesh.h"
#include "mesh/GuBV4_Common.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxReportCallback.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxGeometryHit.h"
#include "geometry/PxMeshQuery.h"

using namespace physx;
using namespace physx::aos;
using namespace physx::Gu;

#ifndef EDGE_EDGE_TEST(V0, U0, U1)
#define EDGE_EDGE_TEST(V0, U0, U1)						\
	Bx = U0[i0] - U1[i0];								\
	By = U0[i1] - U1[i1];								\
	Cx = V0[i0] - U0[i0];								\
	Cy = V0[i1] - U0[i1];								\
	f  = Ay*Bx - Ax*By;									\
	d  = By*Cx - Bx*Cy;									\
	if((f>0.0f && d>=0.0f && d<=f) || (f<0.0f && d<=0.0f && d>=f))	\
	{													\
		const float e=Ax*Cy - Ay*Cx;					\
		if(f>0.0f)										\
		{												\
			if(e>=0.0f && e<=f) return 1;				\
		}												\
		else											\
		{												\
			if(e<=0.0f && e>=f) return 1;				\
		}												\
	}
#endif // !1



#ifndef EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)
#define EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)		\
{														\
	float Bx,By,Cx,Cy,d,f;								\
	const float Ax = V1[i0] - V0[i0];					\
	const float Ay = V1[i1] - V0[i1];					\
	/* test edge U0,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U0, U1);							\
	/* test edge U1,U2 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U1, U2);							\
	/* test edge U2,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U2, U0);							\
}
#endif // !1

#ifndef POINT_IN_TRI(V0, U0, U1, U2)
#define POINT_IN_TRI(V0, U0, U1, U2)					\
{														\
	/* is T1 completly inside T2? */					\
	/* check if V0 is inside tri(U0,U1,U2) */			\
	float a  = U1[i1] - U0[i1];							\
	float b  = -(U1[i0] - U0[i0]);						\
	float c  = -a*U0[i0] - b*U0[i1];					\
	float d0 = a*V0[i0] + b*V0[i1] + c;					\
														\
	a  = U2[i1] - U1[i1];								\
	b  = -(U2[i0] - U1[i0]);							\
	c  = -a*U1[i0] - b*U1[i1];							\
	const float d1 = a*V0[i0] + b*V0[i1] + c;			\
														\
	a  = U0[i1] - U2[i1];								\
	b  = -(U0[i0] - U2[i0]);							\
	c  = -a*U2[i0] - b*U2[i1];							\
	const float d2 = a*V0[i0] + b*V0[i1] + c;			\
	if(d0*d1>0.0f)										\
	{													\
		if(d0*d2>0.0f) return 1;						\
	}													\
}
#endif // !POINT_IN_TRI(V0, U0, U1, U2)



static PxU32 CoplanarTriTri(const PxVec3& n, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& u0, const PxVec3& u1, const PxVec3& u2)
{
	int i0, i1;
	{
		float A[3];
		/* first project onto an axis-aligned plane, that maximizes the area */
		/* of the triangles, compute indices: i0,i1. */
		A[0] = fabsf(n.x);
		A[1] = fabsf(n.y);
		A[2] = fabsf(n.z);
		if (A[0] > A[1])
		{
			if (A[0] > A[2])
			{
				i0 = 1;      /* A[0] is greatest */
				i1 = 2;
			}
			else
			{
				i0 = 0;      /* A[2] is greatest */
				i1 = 1;
			}
		}
		else   /* A[0]<=A[1] */
		{
			if (A[2] > A[1])
			{
				i0 = 0;      /* A[2] is greatest */
				i1 = 1;
			}
			else
			{
				i0 = 0;      /* A[1] is greatest */
				i1 = 2;
			}
		}
	}

	/* test all edges of triangle 1 against the edges of triangle 2 */
	EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);

	/* finally, test if tri1 is totally contained in tri2 or vice versa */
	POINT_IN_TRI(v0, u0, u1, u2);
	POINT_IN_TRI(u0, v0, v1, v2);

	return 0;
}

#ifndef BV4_ALIGN16(x)
#define BV4_ALIGN16(x)	PX_ALIGN_PREFIX(16)	x PX_ALIGN_SUFFIX(16)
#endif // !

#ifndef IEEE_1_0
#define IEEE_1_0	0x3f800000	
#endif // !IEEE_1_0

#ifndef NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, A, B, C, X0, X1)
#define NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, A, B, C, X0, X1)	\
{																						\
	if(D0D1>0.0f)																		\
	{																					\
		/* here we know that D0D2<=0.0 */												\
		/* that is D0, D1 are on the same side, D2 on the other or on the plane */		\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else if(D0D2>0.0f)																	\
	{																					\
		/* here we know that d0d1<=0.0 */												\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D1*D2>0.0f || D0!=0.0f)														\
	{																					\
		/* here we know that d0d1<=0.0 or that D0!=0.0 */								\
		A=VV0; B=(VV1 - VV0)*D0; C=(VV2 - VV0)*D0; X0=D0 - D1; X1=D0 - D2;				\
	}																					\
	else if(D1!=0.0f)																	\
	{																					\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D2!=0.0f)																	\
	{																					\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else																				\
	{																					\
		/* triangles are coplanar */													\
		return ignoreCoplanar ? 0 : CoplanarTriTri(N1, V0, V1, V2, U0, U1, U2);			\
	}																					\
}
#endif // !1

#define OPC_SLABS_GET_MIN_MAX(i)																	\
		const VecI32V minVi = I4LoadXYZW(node->mX[i].mMin, node->mY[i].mMin, node->mZ[i].mMin, 0);		\
		const Vec4V minCoeffV = V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x);				\
		Vec4V minV = V4Mul(Vec4V_From_VecI32V(minVi), minCoeffV);										\
		const VecI32V maxVi = I4LoadXYZW(node->mX[i].mMax, node->mY[i].mMax, node->mZ[i].mMax, 0);		\
		const Vec4V maxCoeffV = V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x);				\
		Vec4V maxV = V4Mul(Vec4V_From_VecI32V(maxVi), maxCoeffV);										\

#define OPC_SLABS_GET_CEQ(i)									\
		OPC_SLABS_GET_MIN_MAX(i)									\
		const FloatV HalfV = FLoad(0.5f);							\
		const Vec4V centerV = V4Scale(V4Add(maxV, minV), HalfV);	\
		const Vec4V extentsV = V4Scale(V4Sub(maxV, minV), HalfV);

#ifndef OPC_SLABS_GET_CENQ(i)	
#define OPC_SLABS_GET_CENQ(i)																\
		const FloatV HalfV = FLoad(0.5f);														\
		const Vec4V minV = V4LoadXYZW(node->mMinX[i], node->mMinY[i], node->mMinZ[i], 0.0f);	\
		const Vec4V maxV = V4LoadXYZW(node->mMaxX[i], node->mMaxY[i], node->mMaxZ[i], 0.0f);	\
		const Vec4V centerV = V4Scale(V4Add(maxV, minV), HalfV);								\
		const Vec4V extentsV = V4Scale(V4Sub(maxV, minV), HalfV);
#endif // !1



#ifndef SORT(a,b)
#define SORT(a,b)			\
	if(a>b)					\
	{						\
		const float _c=a;	\
		a=b;				\
		b=_c;				\
	}
#endif // !


namespace MeshOverlap {

	class BV4TriangleMesh : public TriangleMesh
	{
	public:
		virtual const char* getConcreteTypeName()	const { return "PxBVH34TriangleMesh"; }
		// PX_SERIALIZATION
		BV4TriangleMesh(PxBaseFlags baseFlags) : TriangleMesh(baseFlags), mMeshInterface(PxEmpty), mBV4Tree(PxEmpty) {}
		PX_PHYSX_COMMON_API	virtual void					exportExtraData(PxSerializationContext& ctx);
		void					importExtraData(PxDeserializationContext&);
		PX_PHYSX_COMMON_API	static	TriangleMesh* createObject(PxU8*& address, PxDeserializationContext& context);
		PX_PHYSX_COMMON_API	static	void					getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION
		BV4TriangleMesh(MeshFactory* factory, TriangleMeshData& data);
		virtual							~BV4TriangleMesh() {}

		virtual	PxMeshMidPhase::Enum	getMidphaseID()			const { return PxMeshMidPhase::eBVH34; }

		virtual PxVec3* getVerticesForModification();
		virtual PxBounds3				refitBVH();

		PX_PHYSX_COMMON_API									BV4TriangleMesh(const PxTriangleMeshInternalData& data);
		virtual	bool					getInternalData(PxTriangleMeshInternalData&, bool)	const;

		PX_FORCE_INLINE				const Gu::BV4Tree& getBV4Tree()			const { return mBV4Tree; }
	private:
		Gu::SourceMesh			mMeshInterface;
		Gu::BV4Tree				mBV4Tree;
	};

	template<class ParamsT>
	PX_FORCE_INLINE void precomputeData(ParamsT* PX_RESTRICT dst, PxMat33* PX_RESTRICT absRot, const PxMat33* PX_RESTRICT boxToModelR)
	{
		// Precompute absolute box-to-model rotation matrix
		dst->mPreca0_PaddedAligned.x = boxToModelR->column0.x;
		dst->mPreca0_PaddedAligned.y = boxToModelR->column1.y;
		dst->mPreca0_PaddedAligned.z = boxToModelR->column2.z;

		dst->mPreca1_PaddedAligned.x = boxToModelR->column0.y;
		dst->mPreca1_PaddedAligned.y = boxToModelR->column1.z;
		dst->mPreca1_PaddedAligned.z = boxToModelR->column2.x;

		dst->mPreca2_PaddedAligned.x = boxToModelR->column0.z;
		dst->mPreca2_PaddedAligned.y = boxToModelR->column1.x;
		dst->mPreca2_PaddedAligned.z = boxToModelR->column2.y;

		// Epsilon value prevents floating-point inaccuracies (strategy borrowed from RAPID)
		const PxReal epsilon = 1e-6f;
		absRot->column0.x = dst->mPreca0b_PaddedAligned.x = epsilon + fabsf(boxToModelR->column0.x);
		absRot->column0.y = dst->mPreca1b_PaddedAligned.x = epsilon + fabsf(boxToModelR->column0.y);
		absRot->column0.z = dst->mPreca2b_PaddedAligned.x = epsilon + fabsf(boxToModelR->column0.z);

		absRot->column1.x = dst->mPreca2b_PaddedAligned.y = epsilon + fabsf(boxToModelR->column1.x);
		absRot->column1.y = dst->mPreca0b_PaddedAligned.y = epsilon + fabsf(boxToModelR->column1.y);
		absRot->column1.z = dst->mPreca1b_PaddedAligned.y = epsilon + fabsf(boxToModelR->column1.z);

		absRot->column2.x = dst->mPreca1b_PaddedAligned.z = epsilon + fabsf(boxToModelR->column2.x);
		absRot->column2.y = dst->mPreca2b_PaddedAligned.z = epsilon + fabsf(boxToModelR->column2.y);
		absRot->column2.z = dst->mPreca0b_PaddedAligned.z = epsilon + fabsf(boxToModelR->column2.z);
	}

	template<class ParamsT>
	PX_FORCE_INLINE	void setupBoxData(ParamsT* PX_RESTRICT dst, const PxVec3& extents, const PxMat33* PX_RESTRICT mAR)
	{
		dst->mBoxExtents_PaddedAligned = extents;

		const float Ex = extents.x;
		const float Ey = extents.y;
		const float Ez = extents.z;
		dst->mBB_PaddedAligned.x = Ex * mAR->column0.x + Ey * mAR->column1.x + Ez * mAR->column2.x;
		dst->mBB_PaddedAligned.y = Ex * mAR->column0.y + Ey * mAR->column1.y + Ez * mAR->column2.y;
		dst->mBB_PaddedAligned.z = Ex * mAR->column0.z + Ey * mAR->column1.z + Ez * mAR->column2.z;
	}

	namespace
	{
		PX_ALIGN_PREFIX(16)
			struct TriangleData
		{
			PxVec3p	mV0, mV1, mV2;
			PxVec3p	mXXX, mYYY, mZZZ;
			//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
			PxVec3	mNormal;
			float	mD;
			//#endif
			PX_FORCE_INLINE	void	init(const PxVec3& V0, const PxVec3& V1, const PxVec3& V2)
			{
				// 45 lines of asm (x64)
				const aos::Vec4V V0V = V4LoadU(&V0.x);
				const aos::Vec4V V1V = V4LoadU(&V1.x);
				const aos::Vec4V V2V = V4LoadU(&V2.x);

				//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
				const Vec4V E1V = V4Sub(V1V, V0V);
				const Vec4V E2V = V4Sub(V2V, V0V);
				const Vec4V NV = V4Cross(E1V, E2V);
				const FloatV dV = FNeg(V4Dot3(NV, V0V));
				//#endif
				V4StoreA(V0V, &mV0.x);
				V4StoreA(V1V, &mV1.x);
				V4StoreA(V2V, &mV2.x);
				//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
				V4StoreA(NV, &mNormal.x);
				FStore(dV, &mD);

				const Vec4V tri_xs = V4LoadXYZW(V0.x, V1.x, V2.x, 0.0f);
				const Vec4V tri_ys = V4LoadXYZW(V0.y, V1.y, V2.y, 0.0f);
				const Vec4V tri_zs = V4LoadXYZW(V0.z, V1.z, V2.z, 0.0f);
				V4StoreA(tri_xs, &mXXX.x);
				V4StoreA(tri_ys, &mYYY.x);
				V4StoreA(tri_zs, &mZZZ.x);

			}
		}PX_ALIGN_SUFFIX(16);
	}

	enum TriVsTriImpl
	{
		TRI_TRI_MOLLER_REGULAR,	// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
		TRI_TRI_MOLLER_NEW,		// Alternative implementation in Gu
		TRI_TRI_NEW_SAT,		// "New" SAT-based implementation
		TRI_TRI_NEW_SAT_SIMD,	// "New" SAT-based implementation using SIMD
	};

	//Based on the paper A Fast Triangle-Triangle Intersection Test by T. Moeller
	//http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
	struct Interval
	{
		PxReal min;
		PxReal max;
		PxVec3 minPoint;
		PxVec3 maxPoint;

		PX_FORCE_INLINE Interval() : min(FLT_MAX), max(-FLT_MAX), minPoint(PxVec3(NAN)), maxPoint(PxVec3(NAN)) { }

		PX_FORCE_INLINE static bool overlapOrTouch(const Interval& a, const Interval& b)
		{
			return !(a.min > b.max || b.min > a.max);
		}

		PX_FORCE_INLINE static Interval intersection(const Interval& a, const Interval& b)
		{
			Interval result;
			if (!overlapOrTouch(a, b))
				return result;

			if (a.min > b.min)
			{
				result.min = a.min;
				result.minPoint = a.minPoint;
			}
			else
			{
				result.min = b.min;
				result.minPoint = b.minPoint;
			}

			if (a.max < b.max)
			{
				result.max = a.max;
				result.maxPoint = a.maxPoint;
			}
			else
			{
				result.max = b.max;
				result.maxPoint = b.maxPoint;
			}
			return result;
		}

		PX_FORCE_INLINE void include(PxReal d, const PxVec3& p)
		{
			if (d < min) { min = d; minPoint = p; }
			if (d > max) { max = d; maxPoint = p; }
		}
	};

	struct TriVsTriParams;

	struct MeshMeshParams;

	class LeafFunction_MeshMesh;

	typedef bool (*trisVsTrisFunction)(const TriVsTriParams& params,
		PxU32 nb0, PxU32 startPrim0, const TriangleData* data0,
		PxU32 nb1, PxU32 startPrim1, const TriangleData* data1,
		bool& abort);

	struct TriVsTriParams
	{
		PX_FORCE_INLINE	TriVsTriParams(trisVsTrisFunction leafFunc, PxReportCallback<PxGeomIndexPair>& callback, float tolerance, bool mustFlip, bool ignoreCoplanar) :
			mLeafFunction(leafFunc),
			mCallback(callback),
			mTolerance(tolerance),
			mMustFlip(mustFlip),
			mIgnoreCoplanar(ignoreCoplanar)
		{
		}

		const trisVsTrisFunction			mLeafFunction;
		PxReportCallback<PxGeomIndexPair>& mCallback;
		const float							mTolerance;
		const bool							mMustFlip;
		const bool							mIgnoreCoplanar;

		PX_NOCOPY(TriVsTriParams)
	};

	struct OBBTestParams	// Data needed to perform the OBB-OBB overlap test
	{
		BV4_ALIGN16(PxVec3p	mCenterOrMinCoeff_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mExtentsOrMaxCoeff_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mTBoxToModel_PaddedAligned);		//!< Translation from obb space to model space
		BV4_ALIGN16(PxVec3p	mBB_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mBoxExtents_PaddedAligned);

		BV4_ALIGN16(PxVec3p	mPreca0_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mPreca1_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mPreca2_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mPreca0b_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mPreca1b_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mPreca2b_PaddedAligned);

		PX_FORCE_INLINE	void	precomputeBoxData(const PxVec3& extents, const PxMat33* PX_RESTRICT box_to_model)
		{
			PxMat33	absRot;	//!< Absolute rotation matrix
			precomputeData(this, &absRot, box_to_model);

			setupBoxData(this, extents, &absRot);
		}
	};

	static void transformV(PxVec3p* PX_RESTRICT dst, const PxVec3* PX_RESTRICT src, const Vec4V& c0, const Vec4V& c1, const Vec4V& c2, const Vec4V& c3)
	{
		const Vec4V vertexV = V4LoadU(&src->x);

		Vec4V ResV = V4Scale(c0, V4GetX(vertexV));
		// PT: TODO: V4ScaleAdd
		ResV = V4Add(ResV, V4Scale(c1, V4GetY(vertexV)));
		ResV = V4Add(ResV, V4Scale(c2, V4GetZ(vertexV)));
		ResV = V4Add(ResV, c3);

		V4StoreU(ResV, &dst->x);
	}
	
	static bool doLeafVsLeaf(const TriVsTriParams& params, const PxU32 prim0, const PxU32 prim1, const SourceMesh* mesh0, const SourceMesh* mesh1, const PxMat44* mat0to1, bool& abort)
	{
		// PT: TODO: revisit this approach, it was fine with the original overlap code but now with the 2 additional queries, not so much
		TriangleData data0[16];
		TriangleData data1[16];

		PxU32 nb0 = 0;
		PxU32 startPrim0;
		{
			PxU32 primIndex0 = prim0;
			PxU32 nbTris0 = getNbPrimitives(primIndex0);
			startPrim0 = primIndex0;
			const PxVec3* verts0 = mesh0->getVerts();
			do
			{
				PX_ASSERT(primIndex0 < mesh0->getNbTriangles());
				PxU32 VRef00, VRef01, VRef02;
				getVertexReferences(VRef00, VRef01, VRef02, primIndex0++, mesh0->getTris32(), mesh0->getTris16());
				PX_ASSERT(VRef00 < mesh0->getNbVertices());
				PX_ASSERT(VRef01 < mesh0->getNbVertices());
				PX_ASSERT(VRef02 < mesh0->getNbVertices());

				if (mat0to1)
				{
					//const PxVec3 p0 = mat0to1->transform(verts0[VRef00]);
					//const PxVec3 p1 = mat0to1->transform(verts0[VRef01]);
					//const PxVec3 p2 = mat0to1->transform(verts0[VRef02]);
					//data0[nb0++].init(p0, p1, p2);

					const Vec4V c0 = V4LoadU(&mat0to1->column0.x);
					const Vec4V c1 = V4LoadU(&mat0to1->column1.x);
					const Vec4V c2 = V4LoadU(&mat0to1->column2.x);
					const Vec4V c3 = V4LoadU(&mat0to1->column3.x);

					PxVec3p p0, p1, p2;
					transformV(&p0, &verts0[VRef00], c0, c1, c2, c3);
					transformV(&p1, &verts0[VRef01], c0, c1, c2, c3);
					transformV(&p2, &verts0[VRef02], c0, c1, c2, c3);

					data0[nb0++].init(p0, p1, p2);
				}
				else
				{
					data0[nb0++].init(verts0[VRef00], verts0[VRef01], verts0[VRef02]);
				}

			} while (nbTris0--);
		}

		PxU32 nb1 = 0;
		PxU32 startPrim1;
		{
			PxU32 primIndex1 = prim1;
			PxU32 nbTris1 = getNbPrimitives(primIndex1);
			startPrim1 = primIndex1;
			const PxVec3* verts1 = mesh1->getVerts();
			do
			{
				PX_ASSERT(primIndex1 < mesh1->getNbTriangles());
				PxU32 VRef10, VRef11, VRef12;
				getVertexReferences(VRef10, VRef11, VRef12, primIndex1++, mesh1->getTris32(), mesh1->getTris16());
				PX_ASSERT(VRef10 < mesh1->getNbVertices());
				PX_ASSERT(VRef11 < mesh1->getNbVertices());
				PX_ASSERT(VRef12 < mesh1->getNbVertices());

				data1[nb1++].init(verts1[VRef10], verts1[VRef11], verts1[VRef12]);

			} while (nbTris1--);
		}

		return (params.mLeafFunction)(params, nb0, startPrim0, data0, nb1, startPrim1, data1, abort);
	}

	class OverlapTest {

	public:
		static bool MeshOverlaps(const PxGeometry& geom0, const PxTransform& pose0,
			const PxGeometry& geom1, const PxTransform& pose1)
		{
			PX_ASSERT(geom0.getType() == PxGeometryType::eTRIANGLEMESH);
			PX_ASSERT(geom1.getType() == PxGeometryType::eTRIANGLEMESH);

			const PxTriangleMeshGeometry& meshGeom0 = static_cast<const PxTriangleMeshGeometry&>(geom0);
			const PxTriangleMeshGeometry& meshGeom1 = static_cast<const PxTriangleMeshGeometry&>(geom1);

			const Gu::TriangleMesh* tm0 = static_cast<const Gu::TriangleMesh*>(meshGeom0.triangleMesh);
			const Gu::TriangleMesh* tm1 = static_cast<const Gu::TriangleMesh*>(meshGeom1.triangleMesh);

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

			//return false;
			// PT: ...so we don't need a table like for the other ops, just go straight to BV4
			return intersectMeshVsMesh_BV4(callback, *tm0, pose0, meshGeom0.scale, *tm1, pose1, meshGeom1.scale, PxMeshMeshQueryFlag::eDEFAULT, 0.0f);
		}

		static bool intersectMeshVsMesh_BV4(PxReportCallback<PxGeomIndexPair>& callback,
			const Gu::TriangleMesh& triMesh0, const PxTransform& meshPose0, const PxMeshScale& meshScale0,
			const Gu::TriangleMesh& triMesh1, const PxTransform& meshPose1, const PxMeshScale& meshScale1,
			PxMeshMeshQueryFlags meshMeshFlags, float tolerance)
		{
			PX_ASSERT(triMesh0.getConcreteType() == PxConcreteType::eTRIANGLE_MESH_BVH34);
			PX_ASSERT(triMesh1.getConcreteType() == PxConcreteType::eTRIANGLE_MESH_BVH34);
			const Gu::BV4Tree& tree0 = static_cast<const BV4TriangleMesh&>(triMesh0).getBV4Tree();
			const Gu::BV4Tree& tree1 = static_cast<const BV4TriangleMesh&>(triMesh1).getBV4Tree();

			const PxTransform t0to1 = meshPose1.transformInv(meshPose0);
			const PxTransform t1to0 = meshPose0.transformInv(meshPose1);

			BV4_ALIGN16(PxMat44 World0to1);
			const PxMat44* TM0to1 = setupWorldMatrix(World0to1, &t0to1.p.x, &t0to1.q.x);

			BV4_ALIGN16(PxMat44 World1to0);
			const PxMat44* TM1to0 = setupWorldMatrix(World1to0, &t1to0.p.x, &t1to0.q.x);

			//if (!meshScale0.isIdentity() || !meshScale1.isIdentity())
			//	return BV4_OverlapMeshVsMesh(callback, tree0, tree1, TM0to1, TM1to0, meshPose0, meshPose1, meshScale0, meshScale1, meshMeshFlags, tolerance) != 0;
			//else
			return BV4_OverlapMeshVsMesh(callback, tree0, tree1, TM0to1, TM1to0, meshMeshFlags, tolerance) != 0;
		}

		static const PxMat44* setupWorldMatrix(PxMat44& world, const float* meshPos, const float* meshRot)
		{
			//	world = PxMat44(PxIdentity);
			setIdentity(world);

			bool isIdt = true;
			if (meshRot)
			{
				const PxU32* Bin = reinterpret_cast<const PxU32*>(meshRot);
				if (Bin[0] != 0 || Bin[1] != 0 || Bin[2] != 0 || Bin[3] != IEEE_1_0)
				{
					//			const PxQuat Q(meshRot[0], meshRot[1], meshRot[2], meshRot[3]);
					//			world = PxMat44(Q);
					setRotation(world, PxQuat(meshRot[0], meshRot[1], meshRot[2], meshRot[3]));
					isIdt = false;
				}
			}

			if (meshPos)
			{
				const PxU32* Bin = reinterpret_cast<const PxU32*>(meshPos);
				if (Bin[0] != 0 || Bin[1] != 0 || Bin[2] != 0)
				{
					//			world.setPosition(PxVec3(meshPos[0], meshPos[1], meshPos[2]));
					world.column3.x = meshPos[0];
					world.column3.y = meshPos[1];
					world.column3.z = meshPos[2];
					isIdt = false;
				}
			}
			return isIdt ? NULL : &world;
		}

		static PX_FORCE_INLINE void setIdentity(PxMat44& m)
		{
			m.column0 = PxVec4(1.0f, 0.0f, 0.0f, 0.0f);
			m.column1 = PxVec4(0.0f, 1.0f, 0.0f, 0.0f);
			m.column2 = PxVec4(0.0f, 0.0f, 1.0f, 0.0f);
			m.column3 = PxVec4(0.0f, 0.0f, 0.0f, 1.0f);
		}

		static PX_FORCE_INLINE void setRotation(PxMat44& m, const PxQuat& q)
		{
			const PxReal x = q.x;
			const PxReal y = q.y;
			const PxReal z = q.z;
			const PxReal w = q.w;

			const PxReal x2 = x + x;
			const PxReal y2 = y + y;
			const PxReal z2 = z + z;

			const PxReal xx = x2 * x;
			const PxReal yy = y2 * y;
			const PxReal zz = z2 * z;

			const PxReal xy = x2 * y;
			const PxReal xz = x2 * z;
			const PxReal xw = x2 * w;

			const PxReal yz = y2 * z;
			const PxReal yw = y2 * w;
			const PxReal zw = z2 * w;

			m.column0 = PxVec4(1.0f - yy - zz, xy + zw, xz - yw, 0.0f);
			m.column1 = PxVec4(xy - zw, 1.0f - xx - zz, yz + xw, 0.0f);
			m.column2 = PxVec4(xz + yw, yz - xw, 1.0f - xx - yy, 0.0f);
		}

		static bool BV4_OverlapMeshVsMesh(
			PxReportCallback<PxGeomIndexPair>& callback,
			const Gu::BV4Tree& tree0, const Gu::BV4Tree& tree1, const PxMat44* mat0to1, const PxMat44* mat1to0, PxMeshMeshQueryFlags meshMeshFlags, float tolerance)
		{
			PxGeomIndexPair stackBuffer[256];
			bool mustResetBuffer;
			if (callback.mBuffer)
			{
				PX_ASSERT(callback.mCapacity);
				mustResetBuffer = false;
			}
			else
			{
				callback.mBuffer = stackBuffer;
				PX_ASSERT(callback.mCapacity <= 256);
				if (callback.mCapacity == 0 || callback.mCapacity > 256)
				{
					callback.mCapacity = 256;
				}
				callback.mSize = 0;
				mustResetBuffer = true;
			}

			const bool ignoreCoplanar = meshMeshFlags & PxMeshMeshQueryFlag::eDISCARD_COPLANAR;
			const trisVsTrisFunction leafFunc = getLeafFunc(meshMeshFlags, tolerance);

			bool status;
			bool abort = false;
			if (!tree0.mNodes && !tree1.mNodes)
			{
				const Gu::SourceMesh* mesh0 = static_cast<const Gu::SourceMesh*>(tree0.mMeshInterface);
				const Gu::SourceMesh* mesh1 = static_cast<const Gu::SourceMesh*>(tree1.mMeshInterface);
				status = doSmallMeshVsSmallMesh(callback, mesh0, mesh1, mat0to1, abort, ignoreCoplanar, leafFunc, tolerance);
			}
			else
			{
				if (tree0.mQuantized)
				{
					if (tree1.mQuantized)
						status = BV4_OverlapMeshVsMeshT<BVDataPackedQ, BVDataPackedQ, BVDataSwizzledQ, BVDataSwizzledQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance); // Path here
					else
						status = BV4_OverlapMeshVsMeshT<BVDataPackedQ, BVDataPackedNQ, BVDataSwizzledQ, BVDataSwizzledNQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance);
				}
				else
				{
					if (tree1.mQuantized)
						status = BV4_OverlapMeshVsMeshT<BVDataPackedNQ, BVDataPackedQ, BVDataSwizzledNQ, BVDataSwizzledQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance);
					else
						status = BV4_OverlapMeshVsMeshT<BVDataPackedNQ, BVDataPackedNQ, BVDataSwizzledNQ, BVDataSwizzledNQ>(callback, tree0, tree1, mat0to1, mat1to0, abort, ignoreCoplanar, leafFunc, tolerance);
				}
			}

			if (!abort)
			{
				const PxU32 currentSize = callback.mSize;
				if (currentSize)
				{
					callback.mSize = 0;
					callback.flushResults(currentSize, callback.mBuffer);
				}
			}

			if (mustResetBuffer)
				callback.mBuffer = NULL;
			return status;

			//return false;
		}

		template<class PackedNodeT0, class PackedNodeT1, class SwizzledNodeT0, class SwizzledNodeT1>
		static bool BV4_OverlapMeshVsMeshT(PxReportCallback<PxGeomIndexPair>& callback, const BV4Tree& tree0, const BV4Tree& tree1, const PxMat44* mat0to1, const PxMat44* mat1to0,
			bool& _abort, bool ignoreCoplanar, trisVsTrisFunction leafFunc, float tolerance)
		{
			const SourceMesh* mesh0 = static_cast<const SourceMesh*>(tree0.mMeshInterface);
			const SourceMesh* mesh1 = static_cast<const SourceMesh*>(tree1.mMeshInterface);

			const PackedNodeT0* PX_RESTRICT node0 = reinterpret_cast<const PackedNodeT0*>(tree0.mNodes);
			const PackedNodeT1* PX_RESTRICT node1 = reinterpret_cast<const PackedNodeT1*>(tree1.mNodes);
			PX_ASSERT(node0 || node1);

			if (!node0 && node1)
			{
				MeshMeshParams ParamsForTree1Traversal(leafFunc, callback, mesh0, mesh1, mat0to1, tree1, false, ignoreCoplanar, tolerance);
				return doSmallMeshVsTree<PackedNodeT1, SwizzledNodeT1>(callback, ParamsForTree1Traversal, node1, mesh0, mesh1, _abort);
			}
			else if (node0 && !node1)
			{
				MeshMeshParams ParamsForTree0Traversal(leafFunc, callback, mesh1, mesh0, mat1to0, tree0, true, ignoreCoplanar, tolerance);
				return doSmallMeshVsTree<PackedNodeT0, SwizzledNodeT0>(callback, ParamsForTree0Traversal, node0, mesh1, mesh0, _abort);
			}
			else
			{
				PX_ASSERT(node0);
				PX_ASSERT(node1);

				MeshMeshParams ParamsForTree1Traversal(leafFunc, callback, mesh0, mesh1, mat0to1, tree1, false, ignoreCoplanar, tolerance);
				MeshMeshParams ParamsForTree0Traversal(leafFunc, callback, mesh1, mesh0, mat1to0, tree0, true, ignoreCoplanar, tolerance);

				BV4_ALIGN16(PxVec3p boxCenter0);
				BV4_ALIGN16(PxVec3p boxExtents0);
				BV4_ALIGN16(PxVec3p boxCenter1);
				BV4_ALIGN16(PxVec3p boxExtents1);

				struct indexPair
				{
					PxU32	index0;
					PxU32	index1;
				};

				PxU32 nb = 1;
				indexPair stack[GU_BV4_STACK_SIZE];
				stack[0].index0 = tree0.mInitData;
				stack[0].index1 = tree1.mInitData;

				bool status = false;

				const PackedNodeT0* const root0 = node0;
				const PackedNodeT1* const root1 = node1;

				do
				{
					const indexPair& childData = stack[--nb];
					node0 = root0 + getChildOffset(childData.index0);
					node1 = root1 + getChildOffset(childData.index1);

					const SwizzledNodeT0* tn0 = reinterpret_cast<const SwizzledNodeT0*>(node0);
					const SwizzledNodeT1* tn1 = reinterpret_cast<const SwizzledNodeT1*>(node1);

					for (PxU32 i = 0; i < 4; i++)
					{
						if (tn0->mData[i] == 0xffffffff)
							continue;

						Vec4V centerV0, extentsV0;
						getNodeBounds(centerV0, extentsV0, tn0, i, &ParamsForTree0Traversal.mCenterOrMinCoeff_PaddedAligned, &ParamsForTree0Traversal.mExtentsOrMaxCoeff_PaddedAligned);
						V4StoreA(centerV0, &boxCenter0.x);
						V4StoreA(extentsV0, &boxExtents0.x);

						ParamsForTree1Traversal.setupForTraversal(boxCenter0, boxExtents0, tolerance);

						for (PxU32 j = 0; j < 4; j++)
						{
							if (tn1->mData[j] == 0xffffffff)
								continue;

							Vec4V centerV1, extentsV1;
							getNodeBounds(centerV1, extentsV1, tn1, j, &ParamsForTree1Traversal.mCenterOrMinCoeff_PaddedAligned, &ParamsForTree1Traversal.mExtentsOrMaxCoeff_PaddedAligned);

							if (BV4_BoxBoxOverlap(centerV1, extentsV1, &ParamsForTree1Traversal))
							{
								const PxU32 isLeaf0 = tn0->isLeaf(i);
								const PxU32 isLeaf1 = tn1->isLeaf(j);

								if (isLeaf0)
								{
									if (isLeaf1)
									{
										bool abort;
										if (doLeafVsLeaf(ParamsForTree1Traversal.mTriVsTriParams, tn0->getPrimitive(i), tn1->getPrimitive(j), mesh0, mesh1, mat0to1, abort))
											status = true;
										if (abort)
											return abortQuery(callback, _abort);
									}
									else
									{
										ParamsForTree1Traversal.mPrimIndex0 = tn0->getPrimitive(i);
										if (doLeafVsNode(root1, tn1, j, &ParamsForTree1Traversal))
											return abortQuery(callback, _abort);
									}
								}
								else
								{
									if (isLeaf1)
									{
										V4StoreA(centerV1, &boxCenter1.x);
										V4StoreA(extentsV1, &boxExtents1.x);

										ParamsForTree0Traversal.setupForTraversal(boxCenter1, boxExtents1, tolerance);

										ParamsForTree0Traversal.mPrimIndex0 = tn1->getPrimitive(j);
										if (doLeafVsNode(root0, tn0, i, &ParamsForTree0Traversal))
											return abortQuery(callback, _abort);
									}
									else
									{
										stack[nb].index0 = tn0->getChildData(i);
										stack[nb].index1 = tn1->getChildData(j);
										nb++;
									}
								}
							}
						}
					}

				} while (nb);

				return status || ParamsForTree0Traversal.mStatus || ParamsForTree1Traversal.mStatus;
			}
		}

		template<class PackedNodeT, class SwizzledNodeT>
		static PX_NOINLINE bool doSmallMeshVsTree(PxReportCallback<PxGeomIndexPair>& callback, MeshMeshParams& params,
			const PackedNodeT* PX_RESTRICT node, const SourceMesh* mesh0, const SourceMesh* mesh1, bool& _abort)
		{
			const PxU32 nbTris = mesh0->getNbTriangles();
			PX_ASSERT(nbTris < 16);

			{
				BV4_ALIGN16(PxVec3p boxCenter);
				BV4_ALIGN16(PxVec3p boxExtents);

				Vec4V centerV, extentsV;
				computeBoundsAroundVertices(centerV, extentsV, mesh0->getNbVertices(), mesh0->getVerts());
				V4StoreA(centerV, &boxCenter.x);
				V4StoreA(extentsV, &boxExtents.x);

				params.setupForTraversal(boxCenter, boxExtents, params.mTriVsTriParams.mTolerance);
				params.mPrimIndex0 = nbTris;
			}

			const PackedNodeT* const root = node;
			const SwizzledNodeT* tn = reinterpret_cast<const SwizzledNodeT*>(node);

			bool status = false;
			for (PxU32 i = 0; i < 4; i++)
			{
				if (tn->mData[i] == 0xffffffff)
					continue;

				Vec4V centerV1, extentsV1;
				getNodeBounds(centerV1, extentsV1, tn, i, &params.mCenterOrMinCoeff_PaddedAligned, &params.mExtentsOrMaxCoeff_PaddedAligned);

				if (BV4_BoxBoxOverlap(centerV1, extentsV1, &params))
				{
					if (tn->isLeaf(i))
					{
						bool abort;
						if (doLeafVsLeaf(params.mTriVsTriParams, nbTris, tn->getPrimitive(i), mesh0, mesh1, params.mMat0to1, abort))
							status = true;
						if (abort)
							return abortQuery(callback, _abort);
					}
					else
					{
						if (doLeafVsNode(root, tn, i, &params))
							return abortQuery(callback, _abort);
					}
				}
			}

			return status || params.mStatus;
		}

		static void computeBoundsAroundVertices(Vec4V& centerV, Vec4V& extentsV, PxU32 nbVerts, const PxVec3* PX_RESTRICT verts)
		{
			Vec4V minV = V4LoadU(&verts[0].x);
			Vec4V maxV = minV;

			for (PxU32 i = 1; i < nbVerts; i++)
			{
				const Vec4V vV = V4LoadU(&verts[i].x);
				minV = V4Min(minV, vV);
				maxV = V4Max(maxV, vV);
			}

			const FloatV HalfV = FLoad(0.5f);
			centerV = V4Scale(V4Add(maxV, minV), HalfV);
			extentsV = V4Scale(V4Sub(maxV, minV), HalfV);
		}

		static PX_FORCE_INLINE void getNodeBounds(Vec4V& centerV, Vec4V& extentsV, const BVDataSwizzledQ* PX_RESTRICT node, PxU32 i, const PxVec3p* PX_RESTRICT centerOrMinCoeff_PaddedAligned, const PxVec3p* PX_RESTRICT extentsOrMaxCoeff_PaddedAligned)
		{
			// Dequantize box0
			//OPC_SLABS_GET_MIN_MAX(tn0, i)
			const VecI32V minVi = I4LoadXYZW(node->mX[i].mMin, node->mY[i].mMin, node->mZ[i].mMin, 0);
			const Vec4V minCoeffV = V4LoadA_Safe(&centerOrMinCoeff_PaddedAligned->x);
			Vec4V minV = V4Mul(Vec4V_From_VecI32V(minVi), minCoeffV);
			const VecI32V maxVi = I4LoadXYZW(node->mX[i].mMax, node->mY[i].mMax, node->mZ[i].mMax, 0);
			const Vec4V maxCoeffV = V4LoadA_Safe(&extentsOrMaxCoeff_PaddedAligned->x);
			Vec4V maxV = V4Mul(Vec4V_From_VecI32V(maxVi), maxCoeffV);

			// OPC_SLABS_GET_CEQ(i)
			const FloatV HalfV = FLoad(0.5f);
			centerV = V4Scale(V4Add(maxV, minV), HalfV);
			extentsV = V4Scale(V4Sub(maxV, minV), HalfV);
		}

		static PX_FORCE_INLINE void getNodeBounds(Vec4V& centerV, Vec4V& extentsV, const BVDataSwizzledNQ* PX_RESTRICT node, PxU32 i, const PxVec3p* PX_RESTRICT /*centerOrMinCoeff_PaddedAligned*/, const PxVec3p* PX_RESTRICT /*extentsOrMaxCoeff_PaddedAligned*/)
		{
			const FloatV HalfV = FLoad(0.5f);
			const Vec4V minV = V4LoadXYZW(node->mMinX[i], node->mMinY[i], node->mMinZ[i], 0.0f);
			const Vec4V maxV = V4LoadXYZW(node->mMaxX[i], node->mMaxY[i], node->mMaxZ[i], 0.0f);
			centerV = V4Scale(V4Add(maxV, minV), HalfV);
			extentsV = V4Scale(V4Sub(maxV, minV), HalfV);
		}

		template<class LeafTestT, int i, class ParamsT>
		static PX_FORCE_INLINE PxIntBool BV4_ProcessNodeNoOrder_SwizzledQ(PxU32* PX_RESTRICT Stack, PxU32& Nb, const BVDataSwizzledQ* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
		{
			OPC_SLABS_GET_CEQ(i)

				if (BV4_BoxBoxOverlap(centerV, extentsV, params))
				{
					if (node->isLeaf(i))
					{
						if (LeafTestT::doLeafTest(params, node->getPrimitive(i)))
							return 1;
					}
					else
						Stack[Nb++] = node->getChildData(i);
				}
			return 0;
		}

		template<class LeafTestT, int i, class ParamsT>
		static PX_FORCE_INLINE PxIntBool BV4_ProcessNodeNoOrder_SwizzledNQ(PxU32* PX_RESTRICT Stack, PxU32& Nb, const BVDataSwizzledNQ* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
		{
			OPC_SLABS_GET_CENQ(i)

				if (BV4_BoxBoxOverlap(centerV, extentsV, params))
				{
					if (node->isLeaf(i))
					{
						if (LeafTestT::doLeafTest(params, node->getPrimitive(i)))
							return 1;
					}
					else
						Stack[Nb++] = node->getChildData(i);
				}
			return 0;
		}

		template<class LeafTestT, class ParamsT>
		static PxIntBool BV4_ProcessStreamSwizzledNoOrderQ(const BVDataPackedQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
		{
			const BVDataPackedQ* root = node;

			PxU32 nb = 1;
			PxU32 stack[GU_BV4_STACK_SIZE];
			stack[0] = initData;

			do
			{
				const PxU32 childData = stack[--nb];
				node = root + getChildOffset(childData);

				const BVDataSwizzledQ* tn = reinterpret_cast<const BVDataSwizzledQ*>(node);

				const PxU32 nodeType = getChildType(childData);

				if (nodeType > 1 && BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 3>(stack, nb, tn, params))
					return 1;
				if (nodeType > 0 && BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 2>(stack, nb, tn, params))
					return 1;
				if (BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 1>(stack, nb, tn, params))
					return 1;
				if (BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 0>(stack, nb, tn, params))
					return 1;

			} while (nb);

			return 0;
		}

		template<class LeafTestT, class ParamsT>
		static PxIntBool BV4_ProcessStreamSwizzledNoOrderNQ(const BVDataPackedNQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
		{
			const BVDataPackedNQ* root = node;

			PxU32 nb = 1;
			PxU32 stack[GU_BV4_STACK_SIZE];
			stack[0] = initData;

			do
			{
				const PxU32 childData = stack[--nb];
				node = root + getChildOffset(childData);

				const BVDataSwizzledNQ* tn = reinterpret_cast<const BVDataSwizzledNQ*>(node);

				const PxU32 nodeType = getChildType(childData);

				if (nodeType > 1 && BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 3>(stack, nb, tn, params))
					return 1;
				if (nodeType > 0 && BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 2>(stack, nb, tn, params))
					return 1;
				if (BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 1>(stack, nb, tn, params))
					return 1;
				if (BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 0>(stack, nb, tn, params))
					return 1;

			} while (nb);

			return 0;
		}

		static PxIntBool doLeafVsNode(const BVDataPackedQ* const PX_RESTRICT root, const BVDataSwizzledQ* PX_RESTRICT node, PxU32 i, MeshMeshParams* PX_RESTRICT params)
		{
			return BV4_ProcessStreamSwizzledNoOrderQ<LeafFunction_MeshMesh, MeshMeshParams>(root, node->getChildData(i), params);
		}

		static PxIntBool doLeafVsNode(const BVDataPackedNQ* const PX_RESTRICT root, const BVDataSwizzledNQ* PX_RESTRICT node, PxU32 i, MeshMeshParams* PX_RESTRICT params)
		{
			return BV4_ProcessStreamSwizzledNoOrderNQ<LeafFunction_MeshMesh, MeshMeshParams>(root, node->getChildData(i), params);
		}

		static PxIntBool BV4_BoxBoxOverlap(const Vec4V boxCenter, const Vec4V extentsV, const OBBTestParams* PX_RESTRICT params)
		{
			const Vec4V TV = V4Sub(V4LoadA_Safe(&params->mTBoxToModel_PaddedAligned.x), boxCenter);
			{
				const Vec4V absTV = V4Abs(TV);
				const BoolV res = V4IsGrtr(absTV, V4Add(extentsV, V4LoadA_Safe(&params->mBB_PaddedAligned.x)));
				const PxU32 test = BGetBitMask(res);
				if (test & 7)
					return 0;
			}

			Vec4V tV;
			{
				const Vec4V T_YZX_V = V4Perm<1, 2, 0, 3>(TV);
				const Vec4V T_ZXY_V = V4Perm<2, 0, 1, 3>(TV);

				tV = V4Mul(TV, V4LoadA_Safe(&params->mPreca0_PaddedAligned.x));
				tV = V4Add(tV, V4Mul(T_YZX_V, V4LoadA_Safe(&params->mPreca1_PaddedAligned.x)));
				tV = V4Add(tV, V4Mul(T_ZXY_V, V4LoadA_Safe(&params->mPreca2_PaddedAligned.x)));
			}

			Vec4V t2V;
			{
				const Vec4V extents_YZX_V = V4Perm<1, 2, 0, 3>(extentsV);
				const Vec4V extents_ZXY_V = V4Perm<2, 0, 1, 3>(extentsV);

				t2V = V4Mul(extentsV, V4LoadA_Safe(&params->mPreca0b_PaddedAligned.x));
				t2V = V4Add(t2V, V4Mul(extents_YZX_V, V4LoadA_Safe(&params->mPreca1b_PaddedAligned.x)));
				t2V = V4Add(t2V, V4Mul(extents_ZXY_V, V4LoadA_Safe(&params->mPreca2b_PaddedAligned.x)));
				t2V = V4Add(t2V, V4LoadA_Safe(&params->mBoxExtents_PaddedAligned.x));
			}

			{
				const Vec4V abstV = V4Abs(tV);
				const BoolV resB = V4IsGrtr(abstV, t2V);
				const PxU32 test = BGetBitMask(resB);
				if (test & 7)
					return 0;
			}
			return 1;
		}

		static PX_NOINLINE bool doSmallMeshVsSmallMesh(PxReportCallback<PxGeomIndexPair>& callback, const SourceMesh* mesh0, const SourceMesh* mesh1, const PxMat44* mat0to1,
			bool& _abort, bool ignoreCoplanar, trisVsTrisFunction leafFunc, float tolerance)
		{
			const PxU32 nbTris0 = mesh0->getNbTriangles();
			PX_ASSERT(nbTris0 < 16);
			const PxU32 nbTris1 = mesh1->getNbTriangles();
			PX_ASSERT(nbTris1 < 16);

			const TriVsTriParams params(leafFunc, callback, tolerance, false, ignoreCoplanar);

			bool abort;
			bool status = false;
			if (doLeafVsLeaf(params, nbTris0, nbTris1, mesh0, mesh1, mat0to1, abort))
				status = true;
			if (abort)
				return abortQuery(callback, _abort);
			return status;
		}

		static bool doLeafVsLeaf(const TriVsTriParams& params, const PxU32 prim0, const PxU32 prim1, const SourceMesh* mesh0, const SourceMesh* mesh1, const PxMat44* mat0to1, bool& abort)
		{
			// PT: TODO: revisit this approach, it was fine with the original overlap code but now with the 2 additional queries, not so much
			TriangleData data0[16];
			TriangleData data1[16];

			PxU32 nb0 = 0;
			PxU32 startPrim0;
			{
				PxU32 primIndex0 = prim0;
				PxU32 nbTris0 = getNbPrimitives(primIndex0);
				startPrim0 = primIndex0;
				const PxVec3* verts0 = mesh0->getVerts();
				do
				{
					PX_ASSERT(primIndex0 < mesh0->getNbTriangles());
					PxU32 VRef00, VRef01, VRef02;
					getVertexReferences(VRef00, VRef01, VRef02, primIndex0++, mesh0->getTris32(), mesh0->getTris16());
					PX_ASSERT(VRef00 < mesh0->getNbVertices());
					PX_ASSERT(VRef01 < mesh0->getNbVertices());
					PX_ASSERT(VRef02 < mesh0->getNbVertices());

					if (mat0to1)
					{
						//const PxVec3 p0 = mat0to1->transform(verts0[VRef00]);
						//const PxVec3 p1 = mat0to1->transform(verts0[VRef01]);
						//const PxVec3 p2 = mat0to1->transform(verts0[VRef02]);
						//data0[nb0++].init(p0, p1, p2);

						const Vec4V c0 = V4LoadU(&mat0to1->column0.x);
						const Vec4V c1 = V4LoadU(&mat0to1->column1.x);
						const Vec4V c2 = V4LoadU(&mat0to1->column2.x);
						const Vec4V c3 = V4LoadU(&mat0to1->column3.x);

						PxVec3p p0, p1, p2;
						transformV(&p0, &verts0[VRef00], c0, c1, c2, c3);
						transformV(&p1, &verts0[VRef01], c0, c1, c2, c3);
						transformV(&p2, &verts0[VRef02], c0, c1, c2, c3);

						data0[nb0++].init(p0, p1, p2);
					}
					else
					{
						data0[nb0++].init(verts0[VRef00], verts0[VRef01], verts0[VRef02]);
					}

				} while (nbTris0--);
			}

			PxU32 nb1 = 0;
			PxU32 startPrim1;
			{
				PxU32 primIndex1 = prim1;
				PxU32 nbTris1 = getNbPrimitives(primIndex1);
				startPrim1 = primIndex1;
				const PxVec3* verts1 = mesh1->getVerts();
				do
				{
					PX_ASSERT(primIndex1 < mesh1->getNbTriangles());
					PxU32 VRef10, VRef11, VRef12;
					getVertexReferences(VRef10, VRef11, VRef12, primIndex1++, mesh1->getTris32(), mesh1->getTris16());
					PX_ASSERT(VRef10 < mesh1->getNbVertices());
					PX_ASSERT(VRef11 < mesh1->getNbVertices());
					PX_ASSERT(VRef12 < mesh1->getNbVertices());

					data1[nb1++].init(verts1[VRef10], verts1[VRef11], verts1[VRef12]);

				} while (nbTris1--);
			}

			return (params.mLeafFunction)(params, nb0, startPrim0, data0, nb1, startPrim1, data1, abort);
		}

		static bool abortQuery(PxReportCallback<PxGeomIndexPair>& callback, bool& abort)
		{
			abort = true;
			callback.mSize = 0;
			return true;
		}

		static trisVsTrisFunction getLeafFunc(PxMeshMeshQueryFlags meshMeshFlags, float tolerance)
		{
			if (tolerance != 0.0f)
				return doTriVsTri_Distance;
			if (meshMeshFlags & PxMeshMeshQueryFlag::eRESERVED1)
				return doTriVsTri_Overlap<TRI_TRI_MOLLER_NEW>;
			if (meshMeshFlags & PxMeshMeshQueryFlag::eRESERVED2)
				return doTriVsTri_Overlap<TRI_TRI_NEW_SAT>;
			if (meshMeshFlags & PxMeshMeshQueryFlag::eRESERVED3)
				return doTriVsTri_Overlap<TRI_TRI_NEW_SAT_SIMD>;
			return doTriVsTri_Overlap<TRI_TRI_MOLLER_REGULAR>;
		}

		template<const TriVsTriImpl impl>
		static bool doTriVsTri_Overlap(const TriVsTriParams& params,
			PxU32 nb0, PxU32 startPrim0, const TriangleData* data0,
			PxU32 nb1, PxU32 startPrim1, const TriangleData* data1,
			bool& abort)
		{
			PX_ASSERT(nb0 <= 16);
			PX_ASSERT(nb1 <= 16);

			PxReportCallback<PxGeomIndexPair>& callback = params.mCallback;
			PxGeomIndexPair* dst = callback.mBuffer;
			PxU32 capacity = callback.mCapacity;
			PxU32 currentSize = callback.mSize;
			PX_ASSERT(currentSize < capacity);

			const bool ignoreCoplanar = params.mIgnoreCoplanar;
			const bool mustFlip = params.mMustFlip;

			bool foundHit = false;
			abort = false;

			for (PxU32 i = 0; i < nb0; i++)
			{
				for (PxU32 j = 0; j < nb1; j++)
				{
					bool ret;
					if (impl == TRI_TRI_MOLLER_REGULAR)
						ret = TriTriOverlap(data0[i], data1[j], ignoreCoplanar);
					else if (impl == TRI_TRI_MOLLER_NEW)
						ret = trianglesIntersect(data0[i].mV0, data0[i].mV1, data0[i].mV2, data1[j].mV0, data1[j].mV1, data1[j].mV2, ignoreCoplanar);
					else if (impl == TRI_TRI_NEW_SAT)
						ret = TriTriSAT(data0[i], data1[j], ignoreCoplanar);
					else if (impl == TRI_TRI_NEW_SAT_SIMD)
						ret = TriTriSAT_SIMD(data0[i], data1[j], ignoreCoplanar);
					else
						ret = false;

					if (ret)
					{
						foundHit = true;
						if (!accumulateResults(callback, dst, capacity, currentSize, startPrim0 + i, startPrim1 + j, mustFlip, abort))
							return true;
					}
				}
			}

			callback.mSize = currentSize;
			return foundHit;
		}

		static bool TriTriSAT(const TriangleData& data0, const TriangleData& data1, bool ignoreCoplanar)
		{
			{
				const PxReal data1_v0_dot_N0 = data1.mV0.dot(data0.mNormal);
				const PxReal data1_v1_dot_N0 = data1.mV1.dot(data0.mNormal);
				const PxReal data1_v2_dot_N0 = data1.mV2.dot(data0.mNormal);

				const PxReal p1ToA = data1_v0_dot_N0 + data0.mD;
				const PxReal p1ToB = data1_v1_dot_N0 + data0.mD;
				const PxReal p1ToC = data1_v2_dot_N0 + data0.mD;

				const PxReal tolerance = 1e-8f;

				if (PxAbs(p1ToA) < tolerance && PxAbs(p1ToB) < tolerance && PxAbs(p1ToC) < tolerance)
				{
					return ignoreCoplanar ? false : CoplanarTriTri(data0.mNormal, data0.mV0, data0.mV1, data0.mV2,
						data1.mV0, data1.mV1, data1.mV2) != 0;
				}

				if ((p1ToA > 0) == (p1ToB > 0) && (p1ToA > 0) == (p1ToC > 0))
				{
					return false; //All points of triangle 2 on same side of triangle 1 -> no intersection
				}
			}

			{
				const PxReal data0_v0_dot_N1 = data0.mV0.dot(data1.mNormal);
				const PxReal data0_v1_dot_N1 = data0.mV1.dot(data1.mNormal);
				const PxReal data0_v2_dot_N1 = data0.mV2.dot(data1.mNormal);

				const PxReal p2ToA = data0_v0_dot_N1 + data1.mD;
				const PxReal p2ToB = data0_v1_dot_N1 + data1.mD;
				const PxReal p2ToC = data0_v2_dot_N1 + data1.mD;

				if ((p2ToA > 0) == (p2ToB > 0) && (p2ToA > 0) == (p2ToC > 0))
					return false; //All points of triangle 1 on same side of triangle 2 -> no intersection	
			}

			{
				const PxVec3 edge1_01 = data1.mV0 - data1.mV1;
				const PxVec3 edge1_12 = data1.mV1 - data1.mV2;
				const PxVec3 edge1_20 = data1.mV2 - data1.mV0;

				{
					const PxVec3 edge0_01 = data0.mV0 - data0.mV1;

					if (!testEdges(data0, data1, edge0_01, edge1_01))
						return false;
					if (!testEdges(data0, data1, edge0_01, edge1_12))
						return false;
					if (!testEdges(data0, data1, edge0_01, edge1_20))
						return false;
				}

				{
					const PxVec3 edge0_12 = data0.mV1 - data0.mV2;

					if (!testEdges(data0, data1, edge0_12, edge1_01))
						return false;
					if (!testEdges(data0, data1, edge0_12, edge1_12))
						return false;
					if (!testEdges(data0, data1, edge0_12, edge1_20))
						return false;
				}

				{
					const PxVec3 edge0_20 = data0.mV2 - data0.mV0;

					if (!testEdges(data0, data1, edge0_20, edge1_01))
						return false;
					if (!testEdges(data0, data1, edge0_20, edge1_12))
						return false;
					if (!testEdges(data0, data1, edge0_20, edge1_20))
						return false;
				}
			}

			return true;
		}

		static bool TriTriSAT_SIMD(const TriangleData& data0, const TriangleData& data1, bool ignoreCoplanar)
		{
			const Vec4V tri1_xs = V4LoadA(&data1.mXXX.x);
			const Vec4V tri1_ys = V4LoadA(&data1.mYYY.x);
			const Vec4V tri1_zs = V4LoadA(&data1.mZZZ.x);
			{
				const Vec4V tri0_normal_x = V4Load(data0.mNormal.x);
				const Vec4V tri0_normal_y = V4Load(data0.mNormal.y);
				const Vec4V tri0_normal_z = V4Load(data0.mNormal.z);

				Vec4V tri1_dot_N0 = V4Mul(tri1_xs, tri0_normal_x);
				// PT: TODO: V4MulAdd
				tri1_dot_N0 = V4Add(tri1_dot_N0, V4Mul(tri1_ys, tri0_normal_y));
				tri1_dot_N0 = V4Add(tri1_dot_N0, V4Mul(tri1_zs, tri0_normal_z));

				const Vec4V p1ToABC = V4Add(tri1_dot_N0, V4Load(data0.mD));
				if (V4AllGrtrOrEq3(V4Load(1e-8f), V4Abs(p1ToABC)))
				{
					return ignoreCoplanar ? false : CoplanarTriTri(data0.mNormal, data0.mV0, data0.mV1, data0.mV2,
						data1.mV0, data1.mV1, data1.mV2) != 0;
				}

				PxU32 mm = BGetBitMask(V4IsGrtr(p1ToABC, V4Zero()));
				if ((mm & 0x7) == 0x7)
					return false;
				mm = BGetBitMask(V4IsGrtrOrEq(V4Zero(), p1ToABC));
				if ((mm & 0x7) == 0x7)
					return false;
			}

			{
				const Vec4V tri0_xs = V4LoadA(&data0.mXXX.x);
				const Vec4V tri0_ys = V4LoadA(&data0.mYYY.x);
				const Vec4V tri0_zs = V4LoadA(&data0.mZZZ.x);

				const Vec4V tri1_normal_x = V4Load(data1.mNormal.x);
				const Vec4V tri1_normal_y = V4Load(data1.mNormal.y);
				const Vec4V tri1_normal_z = V4Load(data1.mNormal.z);

				Vec4V tri0_dot_N1 = V4Mul(tri0_xs, tri1_normal_x);
				// PT: TODO: V4MulAdd
				tri0_dot_N1 = V4Add(tri0_dot_N1, V4Mul(tri0_ys, tri1_normal_y));
				tri0_dot_N1 = V4Add(tri0_dot_N1, V4Mul(tri0_zs, tri1_normal_z));

				const Vec4V p2ToABC = V4Add(tri0_dot_N1, V4Load(data1.mD));

				PxU32 mm = BGetBitMask(V4IsGrtr(p2ToABC, V4Zero()));
				if ((mm & 0x7) == 0x7)
					return false; //All points of triangle 1 on same side of triangle 2 -> no intersection	
				mm = BGetBitMask(V4IsGrtrOrEq(V4Zero(), p2ToABC));
				if ((mm & 0x7) == 0x7)
					return false; //All points of triangle 1 on same side of triangle 2 -> no intersection	
			}

			{
				const Vec4V tri1_xs_shuffled = V4PermYZXW(tri1_xs);
				const Vec4V tri1_ys_shuffled = V4PermYZXW(tri1_ys);
				const Vec4V tri1_zs_shuffled = V4PermYZXW(tri1_zs);

				const Vec4V edge1_xs = V4Sub(tri1_xs, tri1_xs_shuffled);
				const Vec4V edge1_ys = V4Sub(tri1_ys, tri1_ys_shuffled);
				const Vec4V edge1_zs = V4Sub(tri1_zs, tri1_zs_shuffled);

				const Vec4V data0_V0 = V4LoadA(&data0.mV0.x);
				const Vec4V data0_V1 = V4LoadA(&data0.mV1.x);
				const Vec4V data0_V2 = V4LoadA(&data0.mV2.x);

				const Vec4V edge0_01 = V4Sub(data0_V0, data0_V1);
				if (!testEdges4(data0, data1,
					V4GetX(edge0_01), V4GetY(edge0_01), V4GetZ(edge0_01),
					edge1_xs, edge1_ys, edge1_zs))
					return false;

				const Vec4V edge0_12 = V4Sub(data0_V1, data0_V2);
				if (!testEdges4(data0, data1,
					V4GetX(edge0_12), V4GetY(edge0_12), V4GetZ(edge0_12),
					edge1_xs, edge1_ys, edge1_zs))
					return false;

				const Vec4V edge0_20 = V4Sub(data0_V2, data0_V0);
				if (!testEdges4(data0, data1,
					V4GetX(edge0_20), V4GetY(edge0_20), V4GetZ(edge0_20),
					edge1_xs, edge1_ys, edge1_zs))
					return false;
			}

			return true;
		}

		static bool testEdges(const TriangleData& tri0, const TriangleData& tri1,
			const PxVec3& edge0, const PxVec3& edge1)
		{
			PxVec3 cp = edge0.cross(edge1);
			if (!isAlmostZero(cp))
			{
				if (!testSepAxis(cp, tri0, tri1))
					return false;
			}
			return true;
		}

		static PX_FORCE_INLINE bool testEdges4(const TriangleData& data0, const TriangleData& data1,
			const FloatV& edge0_x,
			const FloatV& edge0_y,
			const FloatV& edge0_z,
			const Vec4V& edge1_xs,
			const Vec4V& edge1_ys,
			const Vec4V& edge1_zs
		)
		{
			const Vec4V axis_xs = V4Sub(V4Scale(edge1_zs, edge0_y), V4Scale(edge1_ys, edge0_z));
			const Vec4V axis_ys = V4Sub(V4Scale(edge1_xs, edge0_z), V4Scale(edge1_zs, edge0_x));
			const Vec4V axis_zs = V4Sub(V4Scale(edge1_ys, edge0_x), V4Scale(edge1_xs, edge0_y));

			const Vec4V eps = V4Load(1e-6f);

			Vec4V maxV = V4Max(axis_ys, axis_xs);
			maxV = V4Max(axis_zs, maxV);
			Vec4V minV = V4Min(axis_ys, axis_xs);
			minV = V4Min(axis_zs, minV);
			maxV = V4Max(V4Neg(minV), maxV);

			BoolV cmpV = V4IsGrtr(maxV, eps);
			const PxU32 mask = BGetBitMask(cmpV) & 0x7;

			return testSepAxis(data0, data1, axis_xs, axis_ys, axis_zs, mask);
		}

		static PX_FORCE_INLINE bool testSepAxis(
			const TriangleData& data0,
			const TriangleData& data1,
			const Vec4V& axesX,	// axis0x axis1x axis2x axis3x
			const Vec4V& axesY,	// axis0y axis1y axis2y axis3y
			const Vec4V& axesZ,	// axis0z axis1z axis2z axis3z
			PxU32 mask
		)
		{
			Vec4V min0, max0;
			projectTriangle4(data0, axesX, axesY, axesZ, min0, max0);

			Vec4V min1, max1;
			projectTriangle4(data1, axesX, axesY, axesZ, min1, max1);

			if (V4AnyGrtrX(min1, max0, mask)
				|| V4AnyGrtrX(min0, max1, mask))
				return false;

			return true;
		}

		static PX_FORCE_INLINE void projectTriangle4(
			const TriangleData& data,
			const Vec4V& axesX,	// axis0x axis1x axis2x axis3x
			const Vec4V& axesY,	// axis0y axis1y axis2y axis3y
			const Vec4V& axesZ,	// axis0z axis1z axis2z axis3z
			Vec4V& min4,
			Vec4V& max4
		)
		{
			Vec4V dp0_4 = V4Mul(V4Load(data.mV0.x), axesX);
			dp0_4 = V4MulAdd(V4Load(data.mV0.y), axesY, dp0_4);	//dp0_4 = V4Add(dp0_4, V4Mul(V4Load(data.mV0.y), axesY));
			dp0_4 = V4MulAdd(V4Load(data.mV0.z), axesZ, dp0_4);	//dp0_4 = V4Add(dp0_4, V4Mul(V4Load(data.mV0.z), axesZ));

			Vec4V dp1_4 = V4Mul(V4Load(data.mV1.x), axesX);
			dp1_4 = V4MulAdd(V4Load(data.mV1.y), axesY, dp1_4);	//dp1_4 = V4Add(dp1_4, V4Mul(V4Load(data.mV1.y), axesY));
			dp1_4 = V4MulAdd(V4Load(data.mV1.z), axesZ, dp1_4);	//dp1_4 = V4Add(dp1_4, V4Mul(V4Load(data.mV1.z), axesZ));

			Vec4V dp2_4 = V4Mul(V4Load(data.mV2.x), axesX);
			dp2_4 = V4MulAdd(V4Load(data.mV2.y), axesY, dp2_4);	//dp2_4 = V4Add(dp2_4, V4Mul(V4Load(data.mV2.y), axesY));
			dp2_4 = V4MulAdd(V4Load(data.mV2.z), axesZ, dp2_4);	//dp2_4 = V4Add(dp2_4, V4Mul(V4Load(data.mV2.z), axesZ));

			min4 = V4Min(V4Min(dp0_4, dp1_4), dp2_4);
			max4 = V4Max(V4Max(dp0_4, dp1_4), dp2_4);
		}

		static PX_FORCE_INLINE PxU32 V4AnyGrtrX(const Vec4V a, const Vec4V b, PxU32 mask)
		{
			const PxU32 moveMask = BGetBitMask(V4IsGrtr(a, b));
			return moveMask & mask;
		}

		static bool testSepAxis(const PxVec3& axis, const TriangleData& triangle0, const TriangleData& triangle1)
		{
			float min0, max0;
			projectTriangle(axis, triangle0, min0, max0);

			float min1, max1;
			projectTriangle(axis, triangle1, min1, max1);

			if (max0 < min1 || max1 < min0)
				return false;

			return true;
		}

		static void projectTriangle(const PxVec3& axis, const TriangleData& triangle, float& min, float& max)
		{
			const float dp0 = triangle.mV0.dot(axis);
			const float dp1 = triangle.mV1.dot(axis);
			min = PxMin(dp0, dp1);
			max = PxMax(dp0, dp1);

			const float dp2 = triangle.mV2.dot(axis);
			min = PxMin(min, dp2);
			max = PxMax(max, dp2);
		}

		static bool isAlmostZero(const PxVec3& v)
		{
			if (PxAbs(v.x) > 1e-6f || PxAbs(v.y) > 1e-6f || PxAbs(v.z) > 1e-6f)
				return false;
			return true;
		}

		static bool doTriVsTri_Distance(const TriVsTriParams& params,
			PxU32 nb0, PxU32 startPrim0, const TriangleData* data0,
			PxU32 nb1, PxU32 startPrim1, const TriangleData* data1,
			bool& abort)
		{
			PX_ASSERT(nb0 <= 16);
			PX_ASSERT(nb1 <= 16);

			PxReportCallback<PxGeomIndexPair>& callback = params.mCallback;
			PxGeomIndexPair* dst = callback.mBuffer;
			PxU32 capacity = callback.mCapacity;
			PxU32 currentSize = callback.mSize;
			PX_ASSERT(currentSize < capacity);

			const bool mustFlip = params.mMustFlip;

			bool foundHit = false;
			abort = false;

			const float toleranceSquared = params.mTolerance * params.mTolerance;
			for (PxU32 i = 0; i < nb0; i++)
			{
				// PT: TODO: improve this
				const PxVec3p pp[3] = { data0[i].mV0, data0[i].mV1, data0[i].mV2 };
				for (PxU32 j = 0; j < nb1; j++)
				{
					PxVec3 cp, cq;

					// PT: TODO: improve this
					const PxVec3p qq[3] = { data1[j].mV0, data1[j].mV1, data1[j].mV2 };

					const float d = distanceTriangleTriangleSquared(cp, cq, pp, qq);
					if (d <= toleranceSquared)
					{
						foundHit = true;
						// PT: TODO: this is not enough here
						if (!accumulateResults(callback, dst, capacity, currentSize, startPrim0 + i, startPrim1 + j, mustFlip, abort))
							return true;
					}
				}
			}

			callback.mSize = currentSize;
			return foundHit;
		}

		static bool accumulateResults(PxReportCallback<PxGeomIndexPair>& callback, PxGeomIndexPair*& dst, PxU32& capacity, PxU32& currentSize, PxU32 primIndex0, PxU32 primIndex1, bool mustFlip, bool& abort)
		{
			dst[currentSize].id0 = mustFlip ? primIndex1 : primIndex0;
			dst[currentSize].id1 = mustFlip ? primIndex0 : primIndex1;
			currentSize++;
			if (currentSize == capacity)
			{
				callback.mSize = 0;
				if (!callback.flushResults(currentSize, dst))
				{
					abort = true;
					return false;
				}
				dst = callback.mBuffer;
				capacity = callback.mCapacity;
				currentSize = callback.mSize;
			}
			return true;
		}

		static float distanceTriangleTriangleSquared(PxVec3& cp, PxVec3& cq, const PxVec3p p[3], const PxVec3p q[3])
		{
			PxVec3p Sv[3];
			V4StoreU(V4Sub(V4LoadU(&p[1].x), V4LoadU(&p[0].x)), &Sv[0].x);
			V4StoreU(V4Sub(V4LoadU(&p[2].x), V4LoadU(&p[1].x)), &Sv[1].x);
			V4StoreU(V4Sub(V4LoadU(&p[0].x), V4LoadU(&p[2].x)), &Sv[2].x);

			PxVec3p Tv[3];
			V4StoreU(V4Sub(V4LoadU(&q[1].x), V4LoadU(&q[0].x)), &Tv[0].x);
			V4StoreU(V4Sub(V4LoadU(&q[2].x), V4LoadU(&q[1].x)), &Tv[1].x);
			V4StoreU(V4Sub(V4LoadU(&q[0].x), V4LoadU(&q[2].x)), &Tv[2].x);

			PxVec3 minP, minQ;
			bool shown_disjoint = false;

			float mindd = PX_MAX_F32;

			for (PxU32 i = 0; i < 3; i++)
			{
				for (PxU32 j = 0; j < 3; j++)
				{
					edgeEdgeDist(cp, cq, p[i], Sv[i], q[j], Tv[j]);
					const PxVec3 V = cq - cp;
					const float dd = V.dot(V);

					if (dd <= mindd)
					{
						minP = cp;
						minQ = cq;
						mindd = dd;

						PxU32 id = i + 2;
						if (id >= 3)
							id -= 3;
						PxVec3 Z = p[id] - cp;
						float a = Z.dot(V);
						id = j + 2;
						if (id >= 3)
							id -= 3;
						Z = q[id] - cq;
						float b = Z.dot(V);

						if ((a <= 0.0f) && (b >= 0.0f))
							return V.dot(V);

						if (a <= 0.0f)	a = 0.0f;
						else	if (b > 0.0f)	b = 0.0f;

						if ((mindd - a + b) > 0.0f)
							shown_disjoint = true;
					}
				}
			}

			PxVec3 Sn = Sv[0].cross(Sv[1]);
			float Snl = Sn.dot(Sn);

			if (Snl > 1e-15f)
			{
				const PxVec3 Tp((p[0] - q[0]).dot(Sn),
					(p[0] - q[1]).dot(Sn),
					(p[0] - q[2]).dot(Sn));

				int index = -1;
				if ((Tp[0] > 0.0f) && (Tp[1] > 0.0f) && (Tp[2] > 0.0f))
				{
					if (Tp[0] < Tp[1])		index = 0; else index = 1;
					if (Tp[2] < Tp[index])	index = 2;
				}
				else if ((Tp[0] < 0.0f) && (Tp[1] < 0.0f) && (Tp[2] < 0.0f))
				{
					if (Tp[0] > Tp[1])		index = 0; else index = 1;
					if (Tp[2] > Tp[index])	index = 2;
				}

				if (index >= 0)
				{
					shown_disjoint = true;

					const PxVec3& qIndex = q[index];

					PxVec3 V = qIndex - p[0];
					PxVec3 Z = Sn.cross(Sv[0]);
					if (V.dot(Z) > 0.0f)
					{
						V = qIndex - p[1];
						Z = Sn.cross(Sv[1]);
						if (V.dot(Z) > 0.0f)
						{
							V = qIndex - p[2];
							Z = Sn.cross(Sv[2]);
							if (V.dot(Z) > 0.0f)
							{
								cp = qIndex + Sn * Tp[index] / Snl;
								cq = qIndex;
								return (cp - cq).magnitudeSquared();
							}
						}
					}
				}
			}

			PxVec3 Tn = Tv[0].cross(Tv[1]);
			float Tnl = Tn.dot(Tn);

			if (Tnl > 1e-15f)
			{
				const PxVec3 Sp((q[0] - p[0]).dot(Tn),
					(q[0] - p[1]).dot(Tn),
					(q[0] - p[2]).dot(Tn));

				int index = -1;
				if ((Sp[0] > 0.0f) && (Sp[1] > 0.0f) && (Sp[2] > 0.0f))
				{
					if (Sp[0] < Sp[1])		index = 0; else index = 1;
					if (Sp[2] < Sp[index])	index = 2;
				}
				else if ((Sp[0] < 0.0f) && (Sp[1] < 0.0f) && (Sp[2] < 0.0f))
				{
					if (Sp[0] > Sp[1])		index = 0; else index = 1;
					if (Sp[2] > Sp[index])	index = 2;
				}

				if (index >= 0)
				{
					shown_disjoint = true;

					const PxVec3& pIndex = p[index];

					PxVec3 V = pIndex - q[0];
					PxVec3 Z = Tn.cross(Tv[0]);
					if (V.dot(Z) > 0.0f)
					{
						V = pIndex - q[1];
						Z = Tn.cross(Tv[1]);
						if (V.dot(Z) > 0.0f)
						{
							V = pIndex - q[2];
							Z = Tn.cross(Tv[2]);
							if (V.dot(Z) > 0.0f)
							{
								cp = pIndex;
								cq = pIndex + Tn * Sp[index] / Tnl;
								return (cp - cq).magnitudeSquared();
							}
						}
					}
				}
			}

			if (shown_disjoint)
			{
				cp = minP;
				cq = minQ;
				return mindd;
			}
			else return 0.0f;
		}

		static void edgeEdgeDist(PxVec3& x, PxVec3& y,				// closest points
			const PxVec3& p, const PxVec3& a,	// seg 1 origin, vector
			const PxVec3& q, const PxVec3& b)	// seg 2 origin, vector
		{
			const PxVec3 T = q - p;
			const PxReal ADotA = a.dot(a);
			const PxReal BDotB = b.dot(b);
			const PxReal ADotB = a.dot(b);
			const PxReal ADotT = a.dot(T);
			const PxReal BDotT = b.dot(T);

			// t parameterizes ray (p, a)
			// u parameterizes ray (q, b)

			// Compute t for the closest point on ray (p, a) to ray (q, b)
			const PxReal Denom = ADotA * BDotB - ADotB * ADotB;

			PxReal t;	// We will clamp result so t is on the segment (p, a)
			if (Denom != 0.0f)
				t = PxClamp((ADotT * BDotB - BDotT * ADotB) / Denom, 0.0f, 1.0f);
			else
				t = 0.0f;

			// find u for point on ray (q, b) closest to point at t
			PxReal u;
			if (BDotB != 0.0f)
			{
				u = (t * ADotB - BDotT) / BDotB;

				// if u is on segment (q, b), t and u correspond to closest points, otherwise, clamp u, recompute and clamp t
				if (u < 0.0f)
				{
					u = 0.0f;
					if (ADotA != 0.0f)
						t = PxClamp(ADotT / ADotA, 0.0f, 1.0f);
					else
						t = 0.0f;
				}
				else if (u > 1.0f)
				{
					u = 1.0f;
					if (ADotA != 0.0f)
						t = PxClamp((ADotB + ADotT) / ADotA, 0.0f, 1.0f);
					else
						t = 0.0f;
				}
			}
			else
			{
				u = 0.0f;
				if (ADotA != 0.0f)
					t = PxClamp(ADotT / ADotA, 0.0f, 1.0f);
				else
					t = 0.0f;
			}

			x = p + a * t;
			y = q + b * u;
		}


		//#ifndef USE_GU_TRI_TRI_OVERLAP_FUNCTION
		static PxU32 TriTriOverlap(const TriangleData& data0, const TriangleData& data1, bool ignoreCoplanar)
		{
			const PxVec3& V0 = data0.mV0;
			const PxVec3& V1 = data0.mV1;
			const PxVec3& V2 = data0.mV2;
			const PxVec3& U0 = data1.mV0;
			const PxVec3& U1 = data1.mV1;
			const PxVec3& U2 = data1.mV2;

			const PxVec3& N1 = data0.mNormal;
			float du0, du1, du2, du0du1, du0du2;
			{
				const float d1 = data0.mD;

				// Put U0,U1,U2 into plane equation 1 to compute signed distances to the plane
				du0 = N1.dot(U0) + d1;
				du1 = N1.dot(U1) + d1;
				du2 = N1.dot(U2) + d1;

				// Coplanarity robustness check
#ifdef OPC_TRITRI_EPSILON_TEST
				if (fabsf(du0) < LOCAL_EPSILON) du0 = 0.0f;
				if (fabsf(du1) < LOCAL_EPSILON) du1 = 0.0f;
				if (fabsf(du2) < LOCAL_EPSILON) du2 = 0.0f;
#endif
				du0du1 = du0 * du1;
				du0du2 = du0 * du2;

				if (du0du1 > 0.0f && du0du2 > 0.0f)	// same sign on all of them + not equal 0 ?
					return 0;					// no intersection occurs
			}

			const PxVec3& N2 = data1.mNormal;
			float dv0, dv1, dv2, dv0dv1, dv0dv2;
			{
				const float d2 = data1.mD;

				// put V0,V1,V2 into plane equation 2
				dv0 = N2.dot(V0) + d2;
				dv1 = N2.dot(V1) + d2;
				dv2 = N2.dot(V2) + d2;

#ifdef OPC_TRITRI_EPSILON_TEST
				if (fabsf(dv0) < LOCAL_EPSILON) dv0 = 0.0f;
				if (fabsf(dv1) < LOCAL_EPSILON) dv1 = 0.0f;
				if (fabsf(dv2) < LOCAL_EPSILON) dv2 = 0.0f;
#endif

				dv0dv1 = dv0 * dv1;
				dv0dv2 = dv0 * dv2;

				if (dv0dv1 > 0.0f && dv0dv2 > 0.0f)	// same sign on all of them + not equal 0 ?
					return 0;					// no intersection occurs
			}

			// Compute direction of intersection line
			// Compute and index to the largest component of D
			short index = 0;
			{
				const PxVec3 D = N1.cross(N2);

				float max = fabsf(D[0]);
				const float bb = fabsf(D[1]);
				const float cc = fabsf(D[2]);
				if (bb > max)
				{
					max = bb;
					index = 1;
				}
				if (cc > max)
				{
					max = cc;
					index = 2;
				}
			}

			// This is the simplified projection onto L
			const float vp0 = V0[index];
			const float vp1 = V1[index];
			const float vp2 = V2[index];

			const float up0 = U0[index];
			const float up1 = U1[index];
			const float up2 = U2[index];

			// Compute interval for triangle 1
			float a, b, c, x0, x1;
			NEWCOMPUTE_INTERVALS(vp0, vp1, vp2, dv0, dv1, dv2, dv0dv1, dv0dv2, a, b, c, x0, x1);

			// Compute interval for triangle 2
			float d, e, f, y0, y1;
			NEWCOMPUTE_INTERVALS(up0, up1, up2, du0, du1, du2, du0du1, du0du2, d, e, f, y0, y1);

			const float xx = x0 * x1;
			const float yy = y0 * y1;
			const float xxyy = xx * yy;

			float isect1[2], isect2[2];

			float tmp = a * xxyy;
			isect1[0] = tmp + b * x1 * yy;
			isect1[1] = tmp + c * x0 * yy;

			tmp = d * xxyy;
			isect2[0] = tmp + e * xx * y1;
			isect2[1] = tmp + f * xx * y0;

			SORT(isect1[0], isect1[1]);
			SORT(isect2[0], isect2[1]);

			if (isect1[1] < isect2[0] || isect2[1] < isect1[0])
				return 0;
			return 1;
		}
		//#endif

		static bool trianglesIntersect(const PxVec3& a1, const PxVec3& b1, const PxVec3& c1, const PxVec3& a2, const PxVec3& b2, const PxVec3& c2/*, Segment* intersection*/, bool ignoreCoplanar)
		{
			const PxReal tolerance = 1e-8f;

			const PxPlane p1(a1, b1, c1);
			const PxReal p1ToA = p1.distance(a2);
			const PxReal p1ToB = p1.distance(b2);
			const PxReal p1ToC = p1.distance(c2);

			if (PxAbs(p1ToA) < tolerance && PxAbs(p1ToB) < tolerance && PxAbs(p1ToC) < tolerance)
				return ignoreCoplanar ? false : trianglesIntersectCoplanar(p1, a1, b1, c1, a2, b2, c2); //Coplanar triangles

			if ((p1ToA > 0) == (p1ToB > 0) && (p1ToA > 0) == (p1ToC > 0))
				return false; //All points of triangle 2 on same side of triangle 1 -> no intersection

			const PxPlane p2(a2, b2, c2);
			const PxReal p2ToA = p2.distance(a1);
			const PxReal p2ToB = p2.distance(b1);
			const PxReal p2ToC = p2.distance(c1);

			if ((p2ToA > 0) == (p2ToB > 0) && (p2ToA > 0) == (p2ToC > 0))
				return false; //All points of triangle 1 on same side of triangle 2 -> no intersection	

			PxVec3 intersectionDirection = p1.n.cross(p2.n);
			const PxReal l2 = intersectionDirection.magnitudeSquared();
			intersectionDirection *= 1.0f / PxSqrt(l2);

			const Interval i1 = computeInterval(p2ToA, p2ToB, p2ToC, a1, b1, c1, intersectionDirection);
			const Interval i2 = computeInterval(p1ToA, p1ToB, p1ToC, a2, b2, c2, intersectionDirection);

			if (Interval::overlapOrTouch(i1, i2))
			{
				/*if (intersection)
				{
					const Interval i = Interval::intersection(i1, i2);
					intersection->p0 = i.minPoint;
					intersection->p1 = i.maxPoint;
				}*/
				return true;
			}
			return false;
		}

		static Interval computeInterval(PxReal distanceA, PxReal distanceB, PxReal distanceC, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& dir)
		{
			Interval i;

			const bool bA = distanceA > 0;
			const bool bB = distanceB > 0;
			const bool bC = distanceC > 0;
			distanceA = PxAbs(distanceA);
			distanceB = PxAbs(distanceB);
			distanceC = PxAbs(distanceC);

			if (bA != bB)
			{
				const PxVec3 p = (distanceA / (distanceA + distanceB)) * b + (distanceB / (distanceA + distanceB)) * a;
				i.include(dir.dot(p), p);
			}
			if (bA != bC)
			{
				const PxVec3 p = (distanceA / (distanceA + distanceC)) * c + (distanceC / (distanceA + distanceC)) * a;
				i.include(dir.dot(p), p);
			}
			if (bB != bC)
			{
				const PxVec3 p = (distanceB / (distanceB + distanceC)) * c + (distanceC / (distanceB + distanceC)) * b;
				i.include(dir.dot(p), p);
			}

			return i;
		}

		static bool trianglesIntersectCoplanar(const PxPlane& p1, const PxVec3& a1, const PxVec3& b1, const PxVec3& c1, const PxVec3& a2, const PxVec3& b2, const PxVec3& c2)
		{
			PxU32 x = 0;
			PxU32 y = 0;
			getProjectionIndices(p1.n, x, y);

			const PxReal third = (1.0f / 3.0f);

			//A bit of the computations done inside the following functions could be shared but it's kept simple since the 
			//difference is not very big and the coplanar case is not expected to be the most common case
			if (linesIntersect(a1, b1, a2, b2, x, y) || linesIntersect(a1, b1, b2, c2, x, y) || linesIntersect(a1, b1, c2, a2, x, y) ||
				linesIntersect(b1, c1, a2, b2, x, y) || linesIntersect(b1, c1, b2, c2, x, y) || linesIntersect(b1, c1, c2, a2, x, y) ||
				linesIntersect(c1, a1, a2, b2, x, y) || linesIntersect(c1, a1, b2, c2, x, y) || linesIntersect(c1, a1, c2, a2, x, y) ||
				pointInTriangle(a1, b1, c1, third * (a2 + b2 + c2), x, y) || pointInTriangle(a2, b2, c2, third * (a1 + b1 + c1), x, y))
				return true;

			return false;
		}

		static PxReal pointInTriangle(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& point, PxU32 x, PxU32 y)
		{
			const PxReal ab = orient2d(a, b, point, x, y);
			const PxReal bc = orient2d(b, c, point, x, y);
			const PxReal ca = orient2d(c, a, point, x, y);

			if ((ab >= 0) == (bc >= 0) && (ab >= 0) == (ca >= 0))
				return true;

			return false;
		}

		static void getProjectionIndices(PxVec3 normal, PxU32& x, PxU32& y)
		{
			normal.x = PxAbs(normal.x);
			normal.y = PxAbs(normal.y);
			normal.z = PxAbs(normal.z);

			if (normal.x >= normal.y && normal.x >= normal.z)
			{
				//x is the dominant normal direction
				x = 1;
				y = 2;
			}
			else if (normal.y >= normal.x && normal.y >= normal.z)
			{
				//y is the dominant normal direction
				x = 2;
				y = 0;
			}
			else
			{
				//z is the dominant normal direction
				x = 0;
				y = 1;
			}
		}

		static PxReal linesIntersect(const PxVec3& startA, const PxVec3& endA, const PxVec3& startB, const PxVec3& endB, PxU32 x, PxU32 y)
		{
			const PxReal aaS = orient2d(startA, endA, startB, x, y);
			const PxReal aaE = orient2d(startA, endA, endB, x, y);

			if ((aaS >= 0) == (aaE >= 0))
				return false;

			const PxReal bbS = orient2d(startB, endB, startA, x, y);
			const PxReal bbE = orient2d(startB, endB, endA, x, y);

			if ((bbS >= 0) == (bbE >= 0))
				return false;

			return true;
		}

		static PxReal orient2d(const PxVec3& a, const PxVec3& b, const PxVec3& c, PxU32 x, PxU32 y)
		{
			return (a[y] - c[y]) * (b[x] - c[x]) - (a[x] - c[x]) * (b[y] - c[y]);
		}
	};

	struct MeshMeshParams : OBBTestParams
	{
		PX_FORCE_INLINE	MeshMeshParams(trisVsTrisFunction leafFunc, PxReportCallback<PxGeomIndexPair>& callback, const SourceMesh* mesh0, const SourceMesh* mesh1, const PxMat44* mat0to1, const BV4Tree& tree,
			bool mustFlip, bool ignoreCoplanar, float tolerance) :
			mTriVsTriParams(leafFunc, callback, tolerance, mustFlip, ignoreCoplanar),
			mMesh0(mesh0),
			mMesh1(mesh1),
			mMat0to1(mat0to1),
			mStatus(false)
		{
			V4StoreA_Safe(V4LoadU_Safe(&tree.mCenterOrMinCoeff.x), &mCenterOrMinCoeff_PaddedAligned.x);
			V4StoreA_Safe(V4LoadU_Safe(&tree.mExtentsOrMaxCoeff.x), &mExtentsOrMaxCoeff_PaddedAligned.x);

			PxMat33 mLocalBox_rot;
			if (mat0to1)
				mLocalBox_rot = PxMat33(PxVec3(mat0to1->column0.x, mat0to1->column0.y, mat0to1->column0.z),
					PxVec3(mat0to1->column1.x, mat0to1->column1.y, mat0to1->column1.z),
					PxVec3(mat0to1->column2.x, mat0to1->column2.y, mat0to1->column2.z));
			else
				mLocalBox_rot = PxMat33(PxIdentity);

			precomputeData(this, &mAbsRot, &mLocalBox_rot);
		}

		void	setupForTraversal(const PxVec3p& center, const PxVec3p& extents, float tolerance)
		{
			if (mMat0to1)
			{
				const Vec4V c0 = V4LoadU(&mMat0to1->column0.x);
				const Vec4V c1 = V4LoadU(&mMat0to1->column1.x);
				const Vec4V c2 = V4LoadU(&mMat0to1->column2.x);
				const Vec4V c3 = V4LoadU(&mMat0to1->column3.x);
				transformV(&mTBoxToModel_PaddedAligned, &center, c0, c1, c2, c3);
			}
			else
				mTBoxToModel_PaddedAligned = center;

			setupBoxData(this, extents + PxVec3(tolerance), &mAbsRot);
		}

		PxMat33						mAbsRot;	//!< Absolute rotation matrix
		const TriVsTriParams		mTriVsTriParams;
		const SourceMesh* const	mMesh0;
		const SourceMesh* const	mMesh1;
		const PxMat44* const	mMat0to1;
		PxU32						mPrimIndex0;
		bool						mStatus;

		PX_NOCOPY(MeshMeshParams)
	};

	class LeafFunction_MeshMesh
	{
	public:
		static PX_FORCE_INLINE PxIntBool doLeafTest(MeshMeshParams* PX_RESTRICT params, PxU32 primIndex1)
		{
			bool abort;
			if (doLeafVsLeaf(params->mTriVsTriParams, params->mPrimIndex0, primIndex1, params->mMesh0, params->mMesh1, params->mMat0to1, abort))
				params->mStatus = true;
			return PxIntBool(abort);
		}
	};
}