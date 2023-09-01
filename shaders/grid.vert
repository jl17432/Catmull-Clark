#version 330 core

const int numColumns = 20;
const int numRows = 20;

uniform mat4 MVP;


void main()
{
    int totalLines = numColumns + numRows;
    int lineID = gl_VertexID / 2;  
    bool isVertical = lineID < numColumns; 

    float pos = isVertical ? float(lineID) / float(numColumns - 1) : float(lineID - numColumns) / float(numRows - 1);
    pos = -10.0 + 20.0 * pos; 
    
    float x = isVertical ? pos : (gl_VertexID % 2 == 0 ? -10.0 : 10.0);
    float z = isVertical ? (gl_VertexID % 2 == 0 ? -10.0 : 10.0) : pos;

    gl_Position = MVP * vec4(x, 0.0, z, 1.0);
}