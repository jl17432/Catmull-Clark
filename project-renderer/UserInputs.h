/*
*	The code in this file was been modified based on the code comes from
*	OpenGL Tutorial, original source code can be found at:
*	https://github.com/opengl-tutorials/ogl/blob/master/common/controls.h
*
*	The owner of the site who wrote the tutorial owns the copy right to this code
*/
#pragma once
#include <glm/glm.hpp>

void computeMatricesFromInputs(GLFWwindow* window);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();