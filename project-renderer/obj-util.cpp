#include "obj-util.h"
#include "map"
#include <iostream>


//===========================================
//
//				  structs
//
//===========================================


struct VertexInfo {
	glm::vec3 pos;		// 3-D position coords _(x, y, z)
	glm::vec2 uvs;		// 2-D UV coords _(u, v)
	glm::vec3 norm;		// 3-D normal _(x, y, z)

	// for the use of map
	bool operator<(const VertexInfo that) const {
		return memcmp((void*)this, (void*)&that, sizeof(VertexInfo)) > 0;
	};
};

struct Vec3Comparator {
	glm::vec3 pos;		// 3-D position coords _(x, y, z)

	// for the use of map
	bool operator<(const Vec3Comparator that) const {
		return memcmp((void*)this, (void*)&that, sizeof(Vec3Comparator)) > 0;
	};
};

struct HalfEdge {
	unsigned int head;
	unsigned int tail;

	// for the use of map
	bool operator<(const HalfEdge that) const {
		return memcmp((void*)this, (void*)&that, sizeof(HalfEdge)) > 0;
	};
};




//========================================
//
//				local funcs 
//
//========================================


bool isVertexExist(
	VertexInfo& vertex,
	std::map<VertexInfo, unsigned int>& VerticesMap,
	unsigned int& result
) 
{
	std::map<VertexInfo, unsigned int>::iterator it = VerticesMap.find(vertex);
	if (it == VerticesMap.end()) {
		return false;
	}
	else {
		result = it->second;
		return true;
	}
}

bool isPositionExist(
	Vec3Comparator& comparator,
	std::map<Vec3Comparator, unsigned int>& PosMap,
	unsigned int& result
)
{
	auto iter = PosMap.find(comparator);

	if (iter == PosMap.end()) {
		return false;
	}
	else {
		result = iter->second;
		return true;
	}
}

int lookForOtherHalf(
	HalfEdge& halfEdge,
	std::map<HalfEdge, unsigned int>& EdgeMap
)
{
	auto iter = EdgeMap.find(halfEdge);

	if (iter == EdgeMap.end()) {
		return -1;
	}
	else {
		return iter->second;
	}
}






//===============================================
//
//				global funcs 
//
//===============================================


namespace ObjUtils
{
	//===============================================
	// 
	//				for rendering
	// 
	//===============================================
	void generate_VBO(
		Objmodel& model,
		std::vector<unsigned int>& out_indices,
		std::vector<glm::vec3>& out_vertices,
		std::vector<glm::vec2>& out_uvs,
		std::vector<glm::vec3>& out_normals
	)
	{
		std::map<VertexInfo, unsigned int> VerticesMap;

		// For each input vertex
		for (unsigned int i = 0; i < model.vertices.size(); i++)
		{

			VertexInfo vertex = { model.vertices[i], model.uvs[i], model.normals[i] };

			unsigned int index;

			if (isVertexExist(vertex, VerticesMap, index))
			{ // A similar vertex is already in the VBO, use it instead !
				out_indices.push_back(index);
			}

			else
			{ // If not, it needs to be added in the output data.
				out_vertices.push_back(model.vertices[i]);
				out_uvs.push_back(model.uvs[i]);
				out_normals.push_back(model.normals[i]);
				unsigned int newindex = (unsigned int)out_vertices.size() - 1;
				out_indices.push_back(newindex);
				VerticesMap[vertex] = newindex;
			}
		}
	}





	//===============================================
	// 
	//			for geometry processing
	// 
	//===============================================

	void remove_duplicate_pos(
		Objmodel& model,
		std::vector<unsigned int>& indices,
		std::vector<glm::vec3>& positions
	)
	{
		std::map<Vec3Comparator, unsigned int> PosMap;

		for (auto& vert : model.vertices)
		{	
			Vec3Comparator comp = { vert };
			unsigned int index = -1;
			bool posExists = isPositionExist(comp, PosMap, index);

			if (posExists)
			{
				indices.push_back(index);
			}

			else
			{
				positions.push_back(vert);
				unsigned int newindex = (unsigned int)positions.size() - 1;
				indices.push_back(newindex);
				PosMap[comp] = newindex;
			}
		}
	}

	void remove_duplicate_pos(
		Objmodel& model
	)
	{
		std::map<Vec3Comparator, unsigned int> PosMap;

		std::vector<unsigned int> indices;
		std::vector<glm::vec3> positions;

		for (auto& vert : model.vertices)
		{
			Vec3Comparator comp = { vert };
			unsigned int index = -1;
			bool posExists = isPositionExist(comp, PosMap, index);

			if (posExists)
			{
				indices.push_back(index);
			}

			else
			{
				positions.push_back(vert);
				unsigned int newindex = (unsigned int)positions.size() - 1;
				indices.push_back(newindex);
				PosMap[comp] = newindex;
			}
		}

		model.indices = indices;
		model.vertices = positions;
	}

	void generate_otherHalves(
		std::vector<unsigned int>& indices,
		std::vector<int>& otherHalves,
		unsigned short vertexPerFace
	)
	{
		std::map<HalfEdge, unsigned int> edgeMap;
		otherHalves.resize(indices.size(), -1);

		for (unsigned int head = 0; head < indices.size(); ++head)
		{
			HalfEdge halfEdge {0, 0};
			unsigned int remainder = head % vertexPerFace;
			if (remainder == vertexPerFace - 1)
			{
				halfEdge.head = indices[head];
				halfEdge.tail = indices[head - (vertexPerFace - 1)];
			}
			else
			{
				halfEdge.head = indices[head];
				halfEdge.tail = indices[head + 1];
			}
			edgeMap[halfEdge] = head;

			//std::cout << head << " edge: " << halfEdge.head << " - " << halfEdge.tail << std::endl;

		}


		for (auto iter = edgeMap.begin(); iter != edgeMap.end(); ++iter)
		{
			HalfEdge otherHalf{ iter->first.tail, iter->first.head };
			if (otherHalves[iter->second] != -1) continue;
			otherHalves[iter->second] = lookForOtherHalf(otherHalf, edgeMap);
		}

		return;
	}

	void obj_triangulate(Objmodel& model)
	{
		std::vector<unsigned int> new_indices;		
		const unsigned int original_size = model.indices.size();
		new_indices.reserve(original_size * 1.5f);

		for (unsigned int id = 0; id < original_size; id += 4)
		{
			new_indices.push_back(model.indices[id]);
			new_indices.push_back(model.indices[id+1]);
			new_indices.push_back(model.indices[id+2]);
												
			new_indices.push_back(model.indices[id+2]);
			new_indices.push_back(model.indices[id+3]);
			new_indices.push_back(model.indices[id]);
		}

		model.indices = new_indices;
	}

	void calculate_normals(Objmodel& model)
	{
		model.normals.clear();
		const unsigned int index_size = model.indices.size();
		model.normals.reserve(index_size);

		for (unsigned int id = 0; id < index_size; id += 3)
		{
			glm::vec3 v0v1 = model.get_vert_by_index_id(id + 1) - model.get_vert_by_index_id(id);
			glm::vec3 v1v2 = model.get_vert_by_index_id(id + 2) - model.get_vert_by_index_id(id + 1);
			glm::vec3 normal = glm::normalize(glm::cross(v0v1, v1v2));

			for (unsigned int i = 0; i < 3; ++i)
			{
				model.normals.push_back(normal);
			}
		}
	}


}

