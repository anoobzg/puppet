#version 430 core

out vec4 frag_color;
uniform vec4 upper_color;
uniform vec4 lower_color;
uniform float divide;

in float d;
void main()
{	
	if(d >= divide)
		frag_color = upper_color;
	else
		frag_color = lower_color;
}