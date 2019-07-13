#version 430 core

out vec4 fragment_color;
uniform vec4 color;
void main()
{
	fragment_color = color;
}
