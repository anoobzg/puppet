#version 430 core

out vec4 frag_color;
uniform vec4 color = vec4(1.0, 0.0, 0.0, 1.0);
void main()
{	
	frag_color = color;
}