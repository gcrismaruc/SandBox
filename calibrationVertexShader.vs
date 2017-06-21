#version 430
#extension GL_ARB_texture_rectangle : enable

in vec3 in_Position;

out vec4 color;

void main()
{	

	vec4 vertexDic = vec4(in_Position, 1.0);

	color = vec4(1,0,0,0);
	gl_Position =  vertexDic;
}