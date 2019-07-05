#version 430 core

out vec4 frag_color;

in vec2 texcoord;
in vec4 color;
uniform sampler2D font;

void main()
{	
	float r = texture2D(font, texcoord).r;
	frag_color = color * r;
}