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
GLubyte *colors;

GLubyte *red, *green, *blue, *opacity;

GLubyte *vertexColors;

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

GLfloat *projectionMatrix;

GLfloat depthProjectionMatrix[] = {
	0.000173667, 0.0f, 0.0f, 0.0f,
	0.0f, 0.000173667f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, -0.00000289f,
	-0.055573332f, -0.041679999f, -0.1f, 0.00315323f
};

GLhandleARB ebo;
GLhandleARB vao;
GLhandleARB vbo;

