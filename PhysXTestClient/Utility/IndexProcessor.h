#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <tuple>
#include <cmath>

#include <assert.h>

#ifndef uint
typedef unsigned int uint;
#endif

namespace MeshProcessing {
	struct CorkTriMesh
	{
		uint    n_triangles;
		uint    n_vertices;
		uint* triangles;
		float* vertices;
	};

	class IndexProcessor {
	public:
		static void simplifyMesh(CorkTriMesh* in, CorkTriMesh* out);

	private:

		// A struct to represent a 3D vertex.
		struct Vertex {
			float x, y, z;
		};

		// A hash function for the Vertex struct.
		struct VertexHash {
			size_t operator()(const Vertex& vertex) const {
				return std::hash<float>()(vertex.x) ^ std::hash<float>()(vertex.y) ^ std::hash<float>()(vertex.z);
			}
		};

		// An equality comparison struct for the Vertex struct.
		struct VertexEqual {
			bool operator()(const Vertex& lhs, const Vertex& rhs) const {
				return CompareDoubles(lhs.x, rhs.x) && CompareDoubles(lhs.y, rhs.y) && CompareDoubles(lhs.z, rhs.z);
			}
		};

		static bool CompareDoubles(double x, double y, double epsilon = 1e-8);

		static bool IsSameVertices(Vertex& v0, Vertex& v1);

		static void removeTriangleWithDuplicateVertices(std::vector<uint>& indices, std::vector<Vertex>& vertices);
	};

}
