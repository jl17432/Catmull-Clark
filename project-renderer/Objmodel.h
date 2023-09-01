#pragma once

#include <vector>
#include <glm/glm.hpp>

#include <fstream>

class Objmodel
{

	enum MeshMode
	{
		QUADS,
		TRIANGLES,
		UNDEFINED
	};

public:
	Objmodel();
	Objmodel(char* path);


	std::vector<unsigned int> indices;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals;

	MeshMode meshMode = UNDEFINED;


	glm::vec3 getVertex(unsigned int index) { return glm::vec3(vertices[index]); }

	glm::vec2 getUVs(unsigned int index) { return glm::vec2(uvs[index]); }

	glm::vec3 getNormals(unsigned int index) { return glm::vec3(normals[index]); }

	glm::vec3 get_vert_by_index_id(unsigned int index);

	void writeObj(std::ofstream& file);



private:

	bool loadObjAssimp(
		char* path,
		std::vector<unsigned int>& indices,
		std::vector<glm::vec3>& vertices,
		std::vector<glm::vec2>& uvs,
		std::vector<glm::vec3>& normals
	);

	


};

