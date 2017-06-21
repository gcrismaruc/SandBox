#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <vector>
#include <string>
#include <io.h>
#include <sys/types.h>
#include <fstream>
#include <GL\glew.h>
#include <GL\GL.h>
#include <GL\freeglut.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2\core\types.hpp"

#define BUFFER_OFFSET(i) ((char*)NULL + (i))
#define WIDTH 640
#define HEIGHT 480


int idWindowHandle;
GLhandleARB idProgram;
GLhandleARB idVao;
GLhandleARB idVbo[2]; // positions and colors
GLhandleARB idEbo;
GLhandleARB idVShader;
GLhandleARB idFShader;
GLhandleARB idAttributePosition;
GLhandleARB idAttributeZCoordinate;


GLhandleARB idAttributeColor;
GLhandleARB idUniformColor;
GLhandleARB pmatrix_location;
GLhandleARB rmatrix_location;
GLhandleARB smatrix_location;

GLhandleARB projMatrix_location;
GLhandleARB depthProjMatrix_location;

GLuint	 imageTextureBufferID;	// We have to create a buffer to hold the image. However, it WON'T go in the vertex buffer
GLuint	 imageTextureCoordID;	// The ID of the "texCoord" variable in the shader
GLuint	 imageTextureID;			// The ID of the "texture" variable in the shader
GLubyte* imageData;		// This will contain the raw color information from the file

GLuint	 gradientTextureBufferID;
GLuint	 gradientTextureCoordID;
GLuint	 gradientTextureID;
GLfloat *zCoordinate;


cv::Mat imgKinect;
unsigned char* image;


int vertexIndices[] = {
	0, 1, 2, 3
};

GLuint * vertexIndices1;
GLfloat *vertexPositions1;
GLfloat *textureCoord1;


GLfloat textureCoord[] = {
	0.0f, 0.0f,
	640.0f, 0.0f,
	640.0f, 480.0f,
	0.0f, 480.0f
};

GLfloat vertexPositions[] = {
	0.5f, -1.0f, -1.0f,
	0.5f, 1.0f, -1.0f,
	1.0f, 0.5f, -1.0f,
	-1.0f, 0.5f, -1.0f
};


GLfloat vertexColors[] = {
	1.0f, 0.0f, 0.0f, 1.0f,
	0.0f, 1.0f, 0.0f, 1.0f,
	0.0f, 0.0f, 1.0f, 1.0f,
	0.2f, 1.0f, 0.5f, 1.0f,
};
long lengthVertexColors = 12;

GLfloat pmatrix[] = {
	2.0f / 640, 0.0f, 0.0f, 1.0f,
	0.0f, 2.0f / 480, 0.0f, 1.0f,
	0.0f, 0.0f, -2.0f, -1.0f,
	0.0f, 0.0f, -1.0f, 1.0f
};

GLfloat rmatrix[] = {
	cos(10), 0.0f, sin(10), 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	-sin(10), 0.0f, cos(10), 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f
};

GLfloat smatrix[] = {
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, cos(10), sin(10), 0.0f,
	0.0f, -sin(10), cos(10), 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f
};

float angle = 0.0f;

//GLfloat projectionMatrix[] = {
//	-1092.394907f,       33.347613f, - 1394.285630f,     1065.236367f,
//	- 60.737744f, - 1147.162938f,     361.694681f,      261.100936f,
//	- 1320878.218000f,    14797.158894f, - 828151.565903f, - 3732695.275730f,
//	0.000000f,        0.000000f, - 1.000000f,        0.000000f
//};

GLfloat projectionMatrix[] = {
	3.60291f, -0.0682034f, -0.635952f, -58.0176f,
	0.0946639f, 4.77029f, 1.30154f, 122.229f,
	0.234703f, 0.04387f, -1.55057f, -198.061f,
	0.149602f, 0.0279632f, -0.988351f, -262.8f
};

//
//0.000625707, -0.000011845, -0.000110444, -0.010075743,
//0.000016440, 0.000828442, 0.000226035, 0.021227144,
//-0.000000432, -0.000000081, 0.000002856, 0.000759492,
//-0.227169875, -0.199334218, 0.133034288, 17.107157905

GLfloat depthProjectionMatrix[] = {
	0.000173667, 0.0f, 0.0f, 0.0f,
	0.0f, 0.000173667f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, -0.00000289f,
	-0.055573332f, -0.041679999f, -0.1f, 0.00315323f
};

GLhandleARB ebo;
GLhandleARB vao;
GLhandleARB vbo;

