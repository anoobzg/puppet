#version 430 core

out vec4 fragment_color;
uniform vec4 color = vec4(125.0/255.0, 221.0/255.0, 169.0/255.0, 0.1);
void main()
{
	fragment_color = color;
}
