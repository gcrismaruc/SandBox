#include <iostream>
#include <stdint.h>
#include <list>
#include <opencv2\opencv.hpp>
#include <fstream>
#include <GL\glew.h>
#include <GL\GL.h>
#include <GL\freeglut.h>
#include <thread>
#include <ctime>

#include "DepthMap.h"
#include "Calibration.h"
#include "Aquisition.h"
#include "Header.h"

#define WIDTH 640
#define HEIGHT 480

Aquisition *aq;
Calibration *c;
cv::Mat currentCenter(4, 1, CV_32FC1);
cv::Mat projection_matrix(4, 4, CV_32FC1, &projectionMatrix);
cv::Mat transposed_projection;

bool renderSurface;
long nr_Frame = 0;


string textFileRead(char *fn) {

	std::ifstream ifile(fn);
	std::string filetext;

	while (ifile.good()) {
		std::string line;
		std::getline(ifile, line);
		filetext.append(line + "\n");
	}

	return filetext;
}
void addVertexPositions(){

	vertexIndices1 = new GLuint[WIDTH * HEIGHT * 2];
	vertexPositions1 = new GLfloat[WIDTH * HEIGHT * 2];
	textureCoord1 = new GLfloat[WIDTH * HEIGHT * 2];
	zCoordinate = new GLfloat[WIDTH * HEIGHT];
	vertexColors = new GLubyte[WIDTH * HEIGHT * 4];

	cv::Mat colorsMat = cv::imread("map2.png", CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_COLOR);

	int size = (int)colorsMat.total() * colorsMat.channels();
	red = new GLubyte[20];
	green = new GLubyte[20];
	blue = new GLubyte[20];
	opacity = new GLubyte[20];
	
	//memcpy(colors, colorsMat.data, size * sizeof(GLubyte));
	int indexColor = 0;
	cv::Vec4b c;
	for (int i = 0; i < size; i+=4) {		
		blue[indexColor] = colorsMat.data[i];
		green[indexColor] = colorsMat.data[i + 1];
		red[indexColor] = colorsMat.data[i + 2];
		opacity[indexColor] = colorsMat.data[i + 3];
		indexColor++;
	}
	
	
	cout << "size = " << size;
	cout << "type = " << colorsMat.type();


	long long index = 0;
	long zIndex = 0;

	long long colorIdx = 0;

	for (double y = 0; y < HEIGHT; y++) {
		for (double x = 0; x < WIDTH; x++) {
			// Set the template vertex' position to the pixel center's position: 
			vertexPositions1[index] = GLfloat(x + 0.5);
			vertexPositions1[index + 1] = GLfloat(y + 0.5);
			index += 2;

			zCoordinate[zIndex] = GLfloat(0);
			zIndex++;

			vertexColors[colorIdx] = 0;
			vertexColors[colorIdx + 1] = 0;
			vertexColors[colorIdx + 2] = 0;
			vertexColors[colorIdx + 3] = 0;
			colorIdx += 4;
		}
	}

	index = 0;
	for (unsigned int y = 1; y < HEIGHT; ++y) {
		for (unsigned int x = 0; x < WIDTH; ++x) {
			vertexIndices1[index] = GLfloat(y * WIDTH + x);
			vertexIndices1[index + 1] = GLfloat((y - 1) * WIDTH + x);

			index += 2;
		}
	}

	index = 0;
	for (unsigned int y = 0; y < HEIGHT; ++y) {
		for (unsigned int x = 0; x < WIDTH; ++x) {
			textureCoord1[index] = GLfloat(x);
			textureCoord1[index + 1] = GLfloat(y);
			index += 2;
		}
	}

	std::cout << "Done!";
}

void loadCalibration(){
	projectionMatrix = new GLfloat[16];
	ifstream file("calibration.txt");
	GLfloat val;
	for (int i = 0; i < 16; i++){
			file >> val;
			projectionMatrix[i] = val;
			cout << projectionMatrix[i]<<", \n";
	}
}

void tasta(unsigned char key, int xmouse, int ymouse)
{
	switch (key){
	case 'f':
		glutFullScreen();
		break;

	default:
		break;
	}
}

GLuint getAttributeLocation(GLuint idProgram, std::string varName, std::string shaderType) {
	const char *positionAttributeName = varName.c_str();
	GLuint attributeIdLocation = (GLuint)glGetAttribLocation(idProgram, positionAttributeName);
	if (attributeIdLocation == -1) {
		std::cout << "Eroare getAttributeLocation + tipShader =" <<shaderType<<" varName ="<< varName << (int)idAttributePosition;
	}

	return attributeIdLocation;
}

void setShaders(char* vertexFileName, char* fragmentFileName){

	string vertexShaderCode = textFileRead(vertexFileName);
	const char* vv = vertexShaderCode.c_str();
	GLint const vertexShaderLength = vertexShaderCode.size();

	idVShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(idVShader, 1, &vv, NULL);
	glCompileShader(idVShader);
	
	GLint Result = GL_FALSE;
	int InfoLogLength;

	glGetShaderiv(idVShader, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(idVShader, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (InfoLogLength > 0){
		std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
		glGetShaderInfoLog(idVShader, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
		printf("%s\n", &VertexShaderErrorMessage[0]);
	}

	string fragmentShaderCode = textFileRead(fragmentFileName);
	const char* ff = fragmentShaderCode.c_str();
	GLint const fragmentShaderLength = fragmentShaderCode.size();

	idFShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(idFShader, 1, &ff, &fragmentShaderLength);
	glCompileShader(idFShader);
	std::cout << "idFShader " << idFShader << "shader length " << fragmentShaderLength << endl;

	glGetShaderiv(idFShader, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(idFShader, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (InfoLogLength > 0){
		std::vector<char> FragmentShaderErrorMessage(InfoLogLength + 1);
		glGetShaderInfoLog(idFShader, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
		printf("%s\n", &FragmentShaderErrorMessage[0]);
	}


	idProgram = glCreateProgram();

	glAttachShader(idProgram, idFShader);
	glAttachShader(idProgram, idVShader);
	glLinkProgram(idProgram);

	GLint linkStatus;
	glGetProgramiv(idProgram, GL_LINK_STATUS, &linkStatus);
	char m_linked = (linkStatus == GL_TRUE);

	if (!m_linked)
	{
		GLint logLength;
		glGetProgramiv(idProgram, GL_INFO_LOG_LENGTH, &logLength);

		char* log = new char[logLength];
		GLsizei length;
		glGetProgramInfoLog(idProgram, logLength, &length, log);

		std::cout << log;
		delete[] log;
	}

	if (renderSurface){
		idAttributePosition = getAttributeLocation(idProgram, "in_Position", "vertex shader");
		idAttributeZCoordinate = getAttributeLocation(idProgram, "in_ZCoordinate", "vertex shader");
		idAttributeColor = getAttributeLocation(idProgram, "in_Color", "fragment shader");

		int width, height;

		cv::flip(imgKinect, imgKinect, 0);

		glEnable(GL_TEXTURE_RECTANGLE);					// Turn on texturing
		glGenTextures(2, &imageTextureBufferID);				// Create an ID for a texture buffer
		glBindTexture(GL_TEXTURE_RECTANGLE, imageTextureBufferID);	// Bind that buffer so we can then fill it (in next line)
		glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RED, WIDTH, HEIGHT, 0, GL_RED, GL_UNSIGNED_SHORT, aq->Update());

		imageTextureCoordID = glGetAttribLocation(idProgram, "s_vTexCoord");

		glTexParameterf(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);	// Set the preferences
		glTexParameterf(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
		glTexParameterf(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST);


		imageTextureID = glGetUniformLocation(idProgram, "texture");
		glActiveTexture(GL_TEXTURE0);				// Turn on texture unit 0
		glUniform1i(imageTextureID, 0);

		projMatrix_location = glGetUniformLocation(idProgram, "projMatrix");
		depthProjMatrix_location = glGetUniformLocation(idProgram, "depthProjMatrix");
	}
	glUseProgram(idProgram);
}


/******************************* CALLBACKS ***********************/
void reshape(int w, int h) {
	glViewport(0, 0, w, h);
}

void idle() {
	glutPostRedisplay();
}

void updateVertexPosition(float x, float y) {
	vertexPositions[0] = x;
	vertexPositions[3] = x;
	vertexPositions[7] = y;
	vertexPositions[10] = y;
}

void displayCalibration() {
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	c->updateCenterOfCurrentCircle();
	currentCenter.at<float>(0, 0) = c->currentXCenter;
	currentCenter.at<float>(1, 0) = c->currentYCenter;
	currentCenter.at<float>(2, 0) = -c->currentZCenter;
	currentCenter.at<float>(3, 0) = 1;

	cout << "Aici: x=" << currentCenter.at<float>(0, 0) << "y = " << currentCenter.at<float>(1, 0)
		<< " z = " << currentCenter.at<float>(2, 0) << endl;

	cv::Mat new_vertex = projection_matrix * currentCenter;

	float x = new_vertex.at<float>(0, 0) / new_vertex.at<float>(3, 0);
	float y = -new_vertex.at<float>(1, 0) / new_vertex.at<float>(3, 0);

	updateVertexPosition(x, y);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * 3 * sizeof(GLfloat), vertexPositions);
	
	glDrawElements(GL_LINES, 4, GL_UNSIGNED_INT, 0);
	glutSwapBuffers();
	glutPostRedisplay();
}

int UpdateZAndTextureBuffer(Aquisition *aq) {

	try {
		long long zIndex = 0;
		Mat depthMat(HEIGHT, WIDTH, CV_16UC1, aq->Update()); // from kinect
		if (depthMat.ptr() != NULL) {
			blur(depthMat, depthMat, cv::Size(13, 13));

			cv::flip(depthMat, depthMat, 0);
			cv::flip(depthMat, depthMat, 1);

			cv::Mat test;
			depthMat.copyTo(test);

			long long index = 0;
			for (int y = 0; y < 480; y++)
			{
				for (int x = 0; x < 640; x++)
				{
					ushort z = depthMat.at<ushort>(cv::Point(x, y));
					zCoordinate[zIndex] = GLfloat(z);
					zIndex++;

					int den = z - 850;
					if (den <= 0){
						den = 0;
					}

					if (z >= 992) {
						den = 190;
					}

					int idx = den / 10;
					if (den < 130 && den % 10 == 9 ){
						vertexColors[index] = 0;
						vertexColors[index + 1] = 0;
						vertexColors[index + 2] = 0;
						vertexColors[index + 3] = 1;
					} else {
						vertexColors[index] = red[idx];
						vertexColors[index + 1] = green[idx];
						vertexColors[index + 2] = blue[idx];
						vertexColors[index + 3] = opacity[idx];
					}

			
					index += 4;
				}
			}

			glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RED, WIDTH, HEIGHT, 0, GL_RED, GL_UNSIGNED_SHORT, depthMat.ptr());
			depthMat.release();
			return 1;
		}
		else {
			std::cout << "mat null" << endl;
			return -1;
		}
	}
	catch (int a){
		std::cout << "eroare" << endl;
		return -1;
	}
}

void display() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUniformMatrix4fv(projMatrix_location, 1, GL_TRUE, projectionMatrix);
	glUniformMatrix4fv(depthProjMatrix_location, 1, GL_TRUE, depthProjectionMatrix);

	if (UpdateZAndTextureBuffer(aq) == 1) {
		glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat), WIDTH * HEIGHT * sizeof(GLfloat), zCoordinate);
		glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat)+WIDTH * HEIGHT * 2 * sizeof(GLfloat), WIDTH * HEIGHT * 4 * sizeof(GLubyte), vertexColors);
	}

	GLuint* indexPtr = 0;
	for (unsigned int y = 1; y < HEIGHT; ++y, indexPtr += WIDTH * 2){
		glDrawElements(GL_QUAD_STRIP, WIDTH * 2, GL_UNSIGNED_INT, indexPtr);
	}

	glutSwapBuffers();
	glutPostRedisplay();
	
}

void cpu(){
	imgKinect = cv::imread("calibration\\calibration20.png", CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);

	std::ifstream image("buffer.txt");

	int index = 0;
	float *img = new float[640 * 480];
	float a, b, c;
	for (int y = 0; y < 480; y++){
		for (int x = 0; x < 640; x++){
			image >> b >> c >> a;
			img[index++] = a;
		}
	}

	cv::Mat projM = cv::Mat(4, 4, CV_32F, &projectionMatrix);
	cv::Mat depthM = cv::Mat(4, 4, CV_32F, &depthProjectionMatrix);



	std::ofstream f("rezultat.txt");
	std::ofstream g("bufferGenerat.txt");

	index = 0;

	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++){
			cv::Mat xyzOmogen = cv::Mat(4, 1, CV_32FC1);
			cv::Mat xyzOmogen1 = cv::Mat(4, 1, CV_32FC1);

			float zInMM = imgKinect.at<ushort>(cv::Point(x, y));
			double xInMM = (x - WIDTH / 2.0f) * (320.0f / WIDTH) * 3.501e-3f * zInMM;
			float yInMM = -(y - HEIGHT / 2.0f) * (240.0f / HEIGHT) * 3.501e-3f * zInMM;
			index++;

			xyzOmogen.at<float>(0, 0) = xInMM;
			xyzOmogen.at<float>(1, 0) = yInMM;
			xyzOmogen.at<float>(2, 0) = zInMM;
			xyzOmogen.at<float>(3, 0) = 1.0;

			cv::Mat depthTranspose, projectionTranspose;
			cv::transpose(depthM, depthTranspose);
			cv::transpose(projM, projectionTranspose);

			cv::Mat UV1 = projM * xyzOmogen;


			f << " x = " << xyzOmogen.at<float>(0, 0) << " y = " << xyzOmogen.at<float>(1, 0) << " z = " << xyzOmogen.at<float>(2, 0)
				<< " xFinal = " << UV1.at<float>(0, 0) << " yFinal = " << UV1.at<float>(1, 0) << "  factor scalare = " << UV1.at<float>(3, 0) << " xFinalScalat = " << UV1.at<float>(0, 0) / UV1.at<float>(3, 0)
				<< " yFinalScalat = " << UV1.at<float>(1, 0) / UV1.at<float>(3, 0) << std::endl;

			g << UV1.at<float>(0, 0) / UV1.at<float>(3, 0) << " " << UV1.at<float>(1, 0) / UV1.at<float>(3, 0) << std::endl;

		}
	}

	f.close();
	g.close();
}


void setOpenGLAfterCalibration(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA);
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_POLYGON_BIT);


	glutInitContextVersion(4, 2);
	glutInitContextProfile(GLUT_CORE_PROFILE);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);

	glutInitWindowSize(WIDTH, HEIGHT);

	glutCreateWindow("Hello");

	glutReshapeFunc(reshape);
	glutDisplayFunc(displayCalibration);
	glutKeyboardFunc(tasta);

	glClearColor(0.0f, 1.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glewExperimental = GL_TRUE;

	GLenum glew_status;
	glew_status = glewInit();

	if (glew_status != GLEW_OK) {
		std::cout << "GLEW error: ";
	}

	if (!GLEW_VERSION_4_0) {
		std::cout << "No support for OpenGL 4.0 found : ";
	}

	setShaders("calibrationVertexShader.vs", "calibrationFragmentShader.fgs");

	//vertices buffer
	//generez nume pentru array
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);

	glBufferData(GL_ARRAY_BUFFER, 4 * 3 * sizeof(GLfloat), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * 3 * sizeof(GLfloat), vertexPositions);
	
	glVertexAttribPointer(idAttributePosition, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glEnableVertexAttribArray(idAttributePosition);

	//indices buffer
	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 4 * 2 * sizeof(GLfloat), vertexIndices, GL_STATIC_DRAW);

	glutMainLoop();

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(2, idVbo);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glUseProgram(0);
	glDeleteProgram(idProgram);
}

void setOpenGL(int argc, char** argv){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA);
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_POLYGON_BIT);


	glutInitContextVersion(4, 2);
	glutInitContextProfile(GLUT_CORE_PROFILE);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);

	glutInitWindowSize(WIDTH, HEIGHT);

	glutCreateWindow("Hello");

	//glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutKeyboardFunc(tasta);


	glClearColor(0.1, 0.2, 0.4, 1);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glewExperimental = GL_TRUE;

	GLenum glew_status;
	glew_status = glewInit();

	if (glew_status != GLEW_OK) {
		std::cout << "GLEW error: ";
	}

	if (!GLEW_VERSION_4_0) {
		std::cout << "No support for OpenGL 4.0 found : ";
	}

	addVertexPositions();
	setShaders("vertexShader.vs", "fragmentShader.fgs");

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);

	glBufferData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat)
		+WIDTH * HEIGHT * 2 * sizeof(GLfloat)+4 * WIDTH * HEIGHT * sizeof(GLubyte), NULL, GL_STATIC_DRAW);

	glBufferSubData(GL_ARRAY_BUFFER, 0, WIDTH * HEIGHT * 2 * sizeof(GLfloat), vertexPositions1);
	glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat), WIDTH * HEIGHT * sizeof(GLfloat), zCoordinate);
	glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat), WIDTH * HEIGHT * 2 * sizeof(GLfloat), textureCoord1);
	
	
	glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat)+WIDTH * HEIGHT * 2 * sizeof(GLfloat), WIDTH * HEIGHT * 4 * sizeof(GLubyte), vertexColors);
	

	glVertexAttribPointer(idAttributePosition, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glVertexAttribPointer(idAttributeZCoordinate, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(WIDTH * HEIGHT * 2 * sizeof(GLfloat)));

	//load text coords
	glVertexAttribPointer(imageTextureCoordID, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat)));
	glVertexAttribPointer(idAttributeColor, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, BUFFER_OFFSET(WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat)+WIDTH * HEIGHT * 2 * sizeof(GLfloat)));

	glEnableVertexAttribArray(imageTextureCoordID);
	glEnableVertexAttribArray(idAttributePosition);
	glEnableVertexAttribArray(idAttributeZCoordinate);
	glEnableVertexAttribArray(idAttributeColor);

	//indices buffer
	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat), vertexIndices1, GL_STATIC_DRAW);
	//glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(vertexIndices), vertexIndices);
	
	

	glutMainLoop();

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(2, idVbo);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glUseProgram(0);
	glDeleteProgram(idProgram);
}

int main(int argc, char ** argv){

	aq = new Aquisition();
	const int eventCount = 1;
    HANDLE hEvents[eventCount];

	hEvents[0] = aq->m_hNextDepthFrameEvent;
	HRESULT hr = aq->CreateFirstConnected();

	c = new Calibration(aq);

	std::cout << "\n\tCalibration 1 \n \tAfterCalibtration opengl 2\n\tRenderDepth 3\nYourOption... ";

	int option;
	cin >> option;

	switch (option){
	case 1:
		c->genereazaPuncteLaPozitiiSpecificate();
		c->readConfig();
		c->getFrames();
		
		c->getCoef();
		break;
	case 2:
		loadCalibration();
		projection_matrix = cv::Mat(4, 4, CV_32FC1, projectionMatrix);
		cout << projection_matrix << endl;
		setOpenGLAfterCalibration(argc, argv);
		break;

	case 3:
		loadCalibration();
		renderSurface = true;
		setOpenGL(argc, argv);
		break;

	default:
		break;

	}	
	return 0;
}