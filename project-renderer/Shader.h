/*
*	The code in this file was been modified based on the code comes from
*	OpenGL Tutorial, original source code can be found at:
*	https://github.com/opengl-tutorials/ogl/blob/master/common/shader.h
*
*	The owner of the site who wrote the tutorial owns the copy right to this code
*/


#pragma once
#include<glad/glad.h>

namespace CAT_Render
{
	GLuint LoadShaders(const char* vertex_file_path, const char* fragment_file_path);
}
