#include <iostream>
#include <stdint.h>
#include <list>
#include <opencv2\opencv.hpp>
#include <fstream>
#include <GL\glew.h>
#include <GL\GL.h>
#include <GL\freeglut.h>
#include <thread>
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


	long long index = 0;
	long zIndex = 0;

	//for (double y = -1; y <= 1; y += 0.00416666666667) {
	//	for (double x = -1; x <= 1; x += 0.003125) {
	//		// Set the template vertex' position to the pixel center's position: 
	//		vertexPositions1[index] = GLfloat(-x);
	//		vertexPositions1[index + 1] = GLfloat(-y);
	//		vertexPositions1[index + 2] = GLfloat(0);
	//		index += 3;
	//	}
	//}

	for (double y = 0; y < 480; y++) {
		for (double x = 0; x < 640; x++) {
			// Set the template vertex' position to the pixel center's position: 
			vertexPositions1[index] = GLfloat(x + 0.5);
			vertexPositions1[index + 1] = GLfloat(y + 0.5);
			index += 2;

			zCoordinate[zIndex] = GLfloat(0);
			zIndex++;
		}
	}

	//float vx, vy,vz;
	//ifstream file("bufferGenerat.txt");

	//for (double y = 0; y < 480 ; y++) {
	//	for (double x = 0; x < 640; x++) {
	//		// Set the template vertex' position to the pixel center's position: 
	//		file >> vx >> vy;
	//		vertexPositions1[index] = GLfloat(vx);
	//		vertexPositions1[index + 1] = GLfloat(vy);
	//		vertexPositions1[index + 2] = GLfloat(0.99);
	//		index += 3;
	//	}
	//}

	//index = 0;
	//ofstream g("vertexPosition1.txt");
	//for (double y = 0; y < 480; y++) {
	//	for (double x = 0; x < 640; x++) {
	//		// Set the template vertex' position to the pixel center's position: 
	//		g << vertexPositions1[index] << " "<<vertexPositions1[index + 1] << " "<< vertexPositions1[index + 2]<<std::endl;
	//		index += 3;
	//	}
	//}
	//g.close();

	index = 0;
	for (unsigned int y = 1; y < HEIGHT; ++y) {
		for (unsigned int x = 0; x < WIDTH; ++x) {
			// Set the template vertex' position to the pixel center's position:
			vertexIndices1[index] = GLfloat(y * WIDTH + x);
			vertexIndices1[index + 1] = GLfloat((y - 1) * WIDTH + x);

			index += 2;
		}
	}

	/*vertexIndices1 = new GLuint[WIDTH * HEIGHT * 2];
	vertexPositions1 = new GLfloat[WIDTH * HEIGHT * 3];
	textureCoord1 = new GLfloat[WIDTH * HEIGHT * 2];

	long long index = 0;

	for (double y = -1; y <= 1; y += 0.00416666666667) {
	for (double x = -1; x <= 1; x += 0.003125) {
	// Set the template vertex' position to the pixel center's position:
	vertexPositions1[index] = GLfloat(x);
	vertexPositions1[index + 1] = GLfloat(y);
	vertexPositions1[index + 2] = GLfloat(0);
	index += 3;
	}
	}

	ifstream f("buffer.txt");
	float x1, y1, z;
	for (unsigned int y = 1; y < HEIGHT; ++y) {
	for (unsigned int x = 0; x < WIDTH; ++x) {
	f >> x1 >> y1 >> z;
	vertexPositions1[index] = GLfloat(x1);
	vertexPositions1[index + 1] = GLfloat(y1);
	vertexPositions1[index + 2] = GLfloat(z);
	//cout << z << " ";
	index += 3;

	}
	}

	f.close();*/

	index = 0;
	for (unsigned int y = 0; y < HEIGHT; ++y) {
		for (unsigned int x = 0; x < WIDTH; ++x) {
			// Set the template vertex' position to the pixel center's position: 
			textureCoord1[index] = GLfloat(x);
			textureCoord1[index + 1] = GLfloat(y);
			index += 2;
		}
	}

	/*ofstream f("vertexPositions.txt");

	for (float x = -1; x <= 1; x += 0.003125){
	vertexPositions1[index] = GLfloat(x);
	vertexPositions1[index + 1] = GLfloat(-0.5);
	vertexPositions1[index + 2] = GLfloat(0);

	f << vertexPositions1[index] << " " << vertexPositions1[index + 1] << " " << vertexPositions1[index + 2] << "\n";
	index += 3;
	}
	for (float x = -1; x <= 1; x += 0.003125){
	vertexPositions1[index] = GLfloat(x);
	vertexPositions1[index + 1] = GLfloat(0.5);
	vertexPositions1[index + 2] = GLfloat(0);

	f << vertexPositions1[index] << " " << vertexPositions1[index + 1] << " " << vertexPositions1[index + 2] << "\n";
	index += 3;
	}
	f.close();
	index = 0;

	ofstream g("trianglesIndices.txt");

	for (unsigned int x = 0; x < WIDTH; ++x) {
	vertexIndices1[index] = GLfloat(WIDTH + x);
	vertexIndices1[index + 1] = GLfloat(x);
	g << vertexIndices1[index] << " " << vertexIndices1[index + 1] << "\n";

	index += 2;
	}
	g.close();


	ofstream g1("indecsi.txt");


	for (int idx = 0; idx < WIDTH * 2; idx += 2){
	g1 << "| " << vertexPositions1[3*vertexIndices1[idx]] << ", " << vertexPositions1[3*vertexIndices1[idx] + 1] << ", " << vertexPositions1[3*vertexIndices1[idx] + 2] << ", | "
	"| " << vertexPositions1[3*vertexIndices1[idx+1]] << ", " << vertexPositions1[3*vertexIndices1[idx+1] + 1] << ", " << vertexPositions1[3*vertexIndices1[idx+1] + 2] << ", |\n";
	}
	g1.close();
	*/

	/*index = 0;
	ofstream f("vertexPositions.txt");

	for (unsigned int y = 0; y < HEIGHT; ++y) {
	for (unsigned int x = 0; x < WIDTH; ++x) {
	f << vertexPositions1[index] << " " << vertexPositions1[index + 1] << " " << vertexPositions1[index + 2] << "\n";
	index += 3;
	}
	}


	index = 0;
	ofstream g("trianglesIndices.txt");

	for (unsigned int y = 1; y < HEIGHT; ++y) {
	for (unsigned int x = 0; x < WIDTH; ++x) {
	g << vertexIndices1[index] << " " << vertexIndices1[index + 1] <<"\n";
	index += 2;
	}
	}

	f.close();
	g.close();*/

	std::cout << "Done!";
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

void setShaders(char* vertexFileName, char* fragmentFileName){

	string vertexShaderCode = textFileRead(vertexFileName);
	const char* vv = vertexShaderCode.c_str();
	GLint const vertexShaderLength = vertexShaderCode.size();

	idVShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(idVShader, 1, &vv, NULL);
	glCompileShader(idVShader);
	std::cout << "idVShader " << idVShader << "shader length " << vv << endl;

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

	//glUniform4fv(glGetUniformLocation(idProgram, "in_Color"), 1, mycolor);

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

	const char *positionAttributeName = "in_Position";
	idAttributePosition = (GLuint)glGetAttribLocation(idProgram, positionAttributeName);
	if (idAttributePosition == -1) {
		std::cout << "Error! Vertex getAttribLocation. Error code = \n" << (int)idAttributePosition;
	}

	const char *zCoordinateAttributeName = "in_ZCoordinate";
	idAttributeZCoordinate = (GLuint)glGetAttribLocation(idProgram, zCoordinateAttributeName);
	if (idAttributeZCoordinate == -1) {
		std::cout << "Error! Vertex getAttribLocation. in_zCoordinate Error code = \n" << (int)idAttributeZCoordinate;
	}

	

	//imagine kinect
	int width, height;
	//imgKinect = cv::imread("calibration\\calibration20.png", CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);

	

	//cv::flip(imgKinect, imgKinect,0);
	//cv::flip(imgKinect, imgKinect,1);
	//std::cout << type2str(imgKinect.type())<<endl;
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

	pmatrix_location = glGetUniformLocation(idProgram, "pmatrix");
	rmatrix_location = glGetUniformLocation(idProgram, "rmatrix");
	smatrix_location = glGetUniformLocation(idProgram, "smatrix");

	projMatrix_location = glGetUniformLocation(idProgram, "projMatrix");
	depthProjMatrix_location = glGetUniformLocation(idProgram, "depthProjMatrix");

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
	
	cout << "display\n";
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	c->updateCenterOfCurrentCircle();
	currentCenter.at<float>(0, 0) = c->currentXCenter;
	currentCenter.at<float>(1, 0) = c->currentYCenter;
	currentCenter.at<float>(2, 0) = -c->currentZCenter;
	currentCenter.at<float>(3, 0) = 1;

	cout << currentCenter << endl;

	cv::Mat new_vertex = projection_matrix * currentCenter;

	cout << new_vertex << endl;

	float x = new_vertex.at<float>(0, 0) / new_vertex.at<float>(3, 0);
	float y = -new_vertex.at<float>(1, 0) / new_vertex.at<float>(3, 0);

	cout << x << "   " << y << endl;

	updateVertexPosition(x, y);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * 3 * sizeof(GLfloat), vertexPositions);
	
	glDrawElements(GL_LINES, 4, GL_UNSIGNED_INT, 0);
	glutSwapBuffers();
	glutPostRedisplay();

}

int UpdateZAndTextureBuffer(Aquisition *aq) {

	try {
		long long zIndex = 0;
		//USHORT *vec = aq->Update();
		Mat depthMat(HEIGHT, WIDTH, CV_16UC1, aq->Update()); // from kinect
		
		cv::flip(depthMat, depthMat, 0);
		cv::flip(depthMat, depthMat, 1);
		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
			//	ushort z = vec[zIndex];
				ushort z = depthMat.at<ushort>(cv::Point(x, y));
				zCoordinate[zIndex] = GLfloat(z);
				zIndex++;
			}
		}

		glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RED, WIDTH, HEIGHT, 0, GL_RED, GL_UNSIGNED_SHORT, depthMat.ptr());

		return 1;
	}
	catch (int a){
		std::cout << "eroare" << endl;
		return -1;
	}
}

void display() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUniformMatrix4fv(pmatrix_location, 1, GL_TRUE, pmatrix);
	glUniformMatrix4fv(rmatrix_location, 1, GL_TRUE, rmatrix);
	glUniformMatrix4fv(smatrix_location, 1, GL_TRUE, smatrix);

	glUniformMatrix4fv(projMatrix_location, 1, GL_TRUE, projectionMatrix);
	glUniformMatrix4fv(depthProjMatrix_location, 1, GL_TRUE, depthProjectionMatrix);

	if (UpdateZAndTextureBuffer(aq) == 1) {
		glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat), WIDTH * HEIGHT * sizeof(GLfloat), zCoordinate);
	}

	GLuint* indexPtr = 0;
	for (unsigned int y = 1; y < HEIGHT; ++y, indexPtr += WIDTH * 2){
		glDrawElements(GL_QUAD_STRIP, WIDTH * 2, GL_UNSIGNED_INT, indexPtr);
	}

	//glDrawElements(GL_LINES, HEIGHT * WIDTH * 2, GL_UNSIGNED_INT, 0);

	/*GLuint* idx = GLuint(0);
	//idx += 1;
	glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);
	*/
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

			/*xInMM /= 1000;
			yInMM /= 1000;
			zInMM /= 1000;*/

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

	//glutIdleFunc(idle);
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
	//fac "activ" noul array -> vao, si il creeaza daca este cazul
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

	glBufferData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat)+WIDTH * HEIGHT * 2 * sizeof(GLfloat), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, WIDTH * HEIGHT * 2 * sizeof(GLfloat), vertexPositions1);
	glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat), WIDTH * HEIGHT * sizeof(GLfloat), zCoordinate);
	glBufferSubData(GL_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat), WIDTH * HEIGHT * 2 * sizeof(GLfloat), textureCoord1);

	glVertexAttribPointer(idAttributePosition, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glVertexAttribPointer(idAttributeZCoordinate, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(WIDTH * HEIGHT * 2 * sizeof(GLfloat)));

	//load text coords
	glVertexAttribPointer(imageTextureCoordID, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(WIDTH * HEIGHT * 2 * sizeof(GLfloat)+WIDTH * HEIGHT * sizeof(GLfloat)));

	glEnableVertexAttribArray(imageTextureCoordID);
	glEnableVertexAttribArray(idAttributePosition);
	glEnableVertexAttribArray(idAttributeZCoordinate);

	//indices buffer
	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, WIDTH * HEIGHT * 2 * sizeof(GLfloat), vertexIndices1, GL_STATIC_DRAW);
	//glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(vertexIndices), vertexIndices);
	
	
	//std::thread first(UpdateZAndTextureBuffer, aq);

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

	//MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);
	//DepthMap *dm = new DepthMap(aq);
	//dm->createPalette(dm->palette, dm->paletteSize);
	//while(true) {
	//	dm->ColorTheImage(dm->aquisition.Update());
	//}

	c = new Calibration(aq);


	std::cout << "\n\tCalibration 1 \n \tAfterCalibtration opengl 2\n\tRenderDepth 3\nYourOption... ";

	int option;
	cin >> option;


	cv::Mat depthPrj = cv::Mat(4, 4, CV_32FC1, &depthProjectionMatrix);
	cv::transpose(depthPrj, transposed_projection);
	cout << depthPrj << endl << transposed_projection << endl;


	switch (option){
	case 1:
		c->savePointsForAxes();
		c->readConfig();
		c->getFrames();
		
		c->getCoef();

		//c->solveSVD();
		

		//c->solvePnP();

		//c->zhangEstimation();
		break;
	case 2:
		
		cv::transpose(depthPrj, transposed_projection);
		cout << depthPrj << endl << transposed_projection << endl;
		setOpenGLAfterCalibration(argc, argv);
		break;

	case 3:
		
		setOpenGL(argc, argv);
		break;

	default:
		break;

	}	


	/*while (true){
		Mat depthMat(HEIGHT, WIDTH, CV_16UC1, aq->Update()); // from kinect
		Mat depthf(HEIGHT, WIDTH, CV_8UC1, depthMat.ptr());

		//depthMat.convertTo(depthf, CV_16UC1, 255.0 / 2048.0);
		imshow("original-depth", depthf);

		const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
		Mat temp, temp2;

		// 1 step - downsize for performance, use a smaller version of depth image
		Mat small_depthf;
		resize(depthf, small_depthf, Size(), 0.2, 0.2);

		// 2 step - inpaint only the masked "unknown" pixels
		cv::inpaint(small_depthf, (small_depthf == noDepth), temp, 5.0, INPAINT_TELEA);

		// 3 step - upscale to original size and replace inpainted regions in original depth image
		resize(temp, temp2, depthf.size());
		temp2.copyTo(depthf, (depthf == noDepth)); // add to the original signal

		
		cv::Mat test = cv::Mat(WIDTH, HEIGHT, CV_16UC1);
		
		

		//depthf.convertTo(test, CV_16UC1, 2048.0 / 255.0);


		imshow("depth-inpaint", test); // show results
		waitKey(1);
	}*/
	Mat kernel;
	Point anchor;
	double delta;
	int ddepth;
	int kernel_size = 25;
	anchor = Point(-1, -1);
	delta = 10;
	ddepth = -1;

	while (true){
	
		Mat depthMat(HEIGHT, WIDTH, CV_16UC1, aq->Update()); // from kinect
		Mat depthMatOut(HEIGHT, WIDTH, CV_16UC1);
		
		//kernel_size = 3 + 2 * (ind % 5);
		kernel = Mat::ones(kernel_size, kernel_size, CV_32F) / (float)(kernel_size*kernel_size);

		filter2D(depthMat, depthMatOut, ddepth, kernel, anchor, delta, BORDER_DEFAULT);

		imshow("test", depthMatOut);
		waitKey(1);
	}
	return 0;
}