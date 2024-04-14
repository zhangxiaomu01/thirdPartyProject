#include "IndexProcessor.h"

namespace MeshProcessing {

		void IndexProcessor::simplifyMesh(CorkTriMesh* in, CorkTriMesh* out)
		{
			unsigned int numVertices = in->n_vertices;
			unsigned int numIndices = in->n_triangles * 3;
			unsigned int* indices = in->triangles;
			float* vertices = in->vertices;

			std::unordered_map<Vertex, int, VertexHash, VertexEqual> vertexToIndex;
			std::vector<Vertex> uniqueVertices;
			std::vector<unsigned int> updatedIndices(numIndices);


			for (uint i = 0; i < numVertices; ++i) {
				Vertex vertex = Vertex();
				vertex.x = vertices[i * 3];
				vertex.y = vertices[i * 3 + 1];
				vertex.z = vertices[i * 3 + 2];
				auto it = vertexToIndex.find(vertex);
				if (it == vertexToIndex.end()) {
					size_t newIndex = uniqueVertices.size();
					uniqueVertices.emplace_back(vertex);
					vertexToIndex[vertex] = newIndex;
				}
			}

			for (uint i = 0; i < numIndices; ++i) {
				unsigned int index = indices[i];

				Vertex vertex = {
					vertices[index * 3],
					vertices[index * 3 + 1],
					vertices[index * 3 + 2]
				};
				updatedIndices[i] = vertexToIndex[vertex];
			}

			removeTriangleWithDuplicateVertices(updatedIndices, uniqueVertices);

			out->n_vertices = uniqueVertices.size();
			out->n_triangles = updatedIndices.size() / 3;

			unsigned int numVerticeFloats = out->n_vertices * 3;
			out->vertices = new float[numVerticeFloats];
			out->triangles = new unsigned[numIndices];
			for (unsigned int i = 0; i < numVerticeFloats; i += 3) {
				unsigned int idx = i / 3;
				out->vertices[i] = uniqueVertices[idx].x;
				out->vertices[i + 1] = uniqueVertices[idx].y;
				out->vertices[i + 2] = uniqueVertices[idx].z;
			}

			for (unsigned int i = 0; i < numIndices; ++i) {
				out->triangles[i] = updatedIndices[i];
			}
		}

		bool IndexProcessor::CompareDoubles(double x, double y, double epsilon)
		{
			if (fabs(x - y) < epsilon)
				return true; //they are same
			return false; //they are not same
		}

		bool IndexProcessor::IsSameVertices(Vertex& v0, Vertex& v1)
		{
			if (CompareDoubles(v0.x, v1.x)
				&& CompareDoubles(v0.y, v1.y)
				&& CompareDoubles(v0.z, v1.z)) {
				return true;
			}
			return false;
		}

		void IndexProcessor::removeTriangleWithDuplicateVertices(std::vector<uint>& indices, std::vector<Vertex>& vertices)
		{
			uint indexCount = indices.size();
			uint vertexCount = vertices.size();
			std::vector<uint> newIndex;

			for (uint i = 0; i < indexCount; i += 3) {
				uint idx0 = indices[i];
				uint idx1 = indices[i + 1];
				uint idx2 = indices[i + 2];

				Vertex v0 = { 0.f, 0.f, 0.f };
				v0.x = vertices[idx0].x;
				v0.y = vertices[idx0].y;
				v0.z = vertices[idx0].z;

				Vertex v1 = { 0.f, 0.f, 0.f };
				v1.x = vertices[idx1].x;
				v1.y = vertices[idx1].y;
				v1.z = vertices[idx1].z;

				Vertex v2 = { 0.f, 0.f, 0.f };
				v2.x = vertices[idx2].x;
				v2.y = vertices[idx2].y;
				v2.z = vertices[idx2].z;

				if (IsSameVertices(v0, v1) || IsSameVertices(v0, v2) || IsSameVertices(v1, v2)) {
					continue;
				}
				else {
					newIndex.push_back(idx0);
					newIndex.push_back(idx1);
					newIndex.push_back(idx2);
				}
			}

			std::swap(newIndex, indices);
		}

}
