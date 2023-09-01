#pragma once
#include "Objmodel.h"
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
	);


	//===============================================
	// 
	//			for geometry processing
	// 
	//===============================================
	void remove_duplicate_pos(
		Objmodel& model
	);

	void generate_otherHalves(
		std::vector<unsigned int>& indices,
		std::vector<int>& otherHalves,
		unsigned short vertexPerFace
	);

	void obj_triangulate(Objmodel& model);

	void calculate_normals(Objmodel& model);
}

