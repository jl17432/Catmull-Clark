#include "Objmodel.h"

#include <iostream>
#include <iomanip>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h> 



Objmodel::Objmodel(char* path)
{

	if (loadObjAssimp(path, indices, vertices, uvs,	normals	) == false)
	{
		std::cout << "fail to read this obj file: " << path << std::endl;
	}



}

bool Objmodel::loadObjAssimp(char* path, std::vector<unsigned int>& indices, std::vector<glm::vec3>& vertices, std::vector<glm::vec2>& uvs, std::vector<glm::vec3>& normals)
{
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(path, 0);
	if (!scene) {
		fprintf(stderr, importer.GetErrorString());
		getchar();
		return false;
	}
	const aiMesh* mesh = scene->mMeshes[0]; // always use the 1st mesh 

	// Fill vertices positions
	vertices.reserve(mesh->mNumVertices);
	for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
		aiVector3D pos = mesh->mVertices[i];
		vertices.push_back(glm::vec3(pos.x, pos.y, pos.z));
	}

	// Fill vertices texture coordinates
	//uvs.reserve(mesh->mNumVertices);
	//for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
	//	aiVector3D UVW = mesh->mTextureCoords[0][i]; // Assume only 1 set of UV coords; AssImp supports 8 UV sets.
	//	uvs.push_back(glm::vec2(UVW.x, UVW.y));
	//}

	// Fill vertices normals
	normals.reserve(mesh->mNumVertices);
	for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
		aiVector3D n = mesh->mNormals[i];
		normals.push_back(glm::vec3(n.x, n.y, n.z));
	}


	// Fill face indices
	if (mesh->mFaces[0].mNumIndices == 3) { 
		meshMode = TRIANGLES; 

		indices.reserve(3 * mesh->mNumFaces);
		for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
			// Assume the model has only triangles.
			indices.push_back(mesh->mFaces[i].mIndices[0]);
			indices.push_back(mesh->mFaces[i].mIndices[1]);
			indices.push_back(mesh->mFaces[i].mIndices[2]);
		}
	}
	else if (mesh->mFaces[0].mNumIndices == 4) { 
		meshMode = QUADS; 
	
		indices.reserve(4 * mesh->mNumFaces);
		for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
			// Assume the model has only triangles.
			indices.push_back(mesh->mFaces[i].mIndices[0]);
			indices.push_back(mesh->mFaces[i].mIndices[1]);
			indices.push_back(mesh->mFaces[i].mIndices[2]);
			indices.push_back(mesh->mFaces[i].mIndices[3]);
		}
	}
	else { meshMode = UNDEFINED; return false; }

	

	// The "scene" pointer will be deleted automatically by "importer"
	return true;
}


glm::vec3 Objmodel::get_vert_by_index_id(unsigned int index)
{
	return getVertex(indices[index]);
}

void Objmodel::writeObj(std::ofstream& file)
{

	// write vertex
	file << std::fixed;
	file << std::setprecision(6);
	for (auto& vertex : vertices)
	{
		file << "v " << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
		//file << "(" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << std::endl;
	}
	// write texture points...if there is any...
	//if (textures.size() > 0)
	//{
	//	for (auto& texture : textures)
	//	{
	//		file << "vt " << texture.x << " " << texture.y << std::endl;
	//	}
	//}
	// 
	// write normals...if there is any...
	//if (normals.size() > 0)
	//{
	//	for (auto& normal : normals)
	//	{
	//		file << "vn " << normal.x << " " << normal.y << " " << normal.z << endl;
	//	}
	//}
	// 
	// write faces

	for (unsigned int id = 0; id < indices.size(); id += 4)
	{
		std::string output_line = "f ";

		// for the 3 vertices
		for (int i = 0; i < 4; i++)
		{
			output_line += std::to_string(indices[id + i] + 1);

			output_line += "/ ";
		}
		//finished one face
		file << output_line << std::endl;
	}
	std::cout << "finished writing file! " << std::endl;
}