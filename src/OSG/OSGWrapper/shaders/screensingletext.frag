#version 430 core

out vec4 frag_color;

in vec2 texcoord;
uniform vec4 color = vec4(1.0, 1.0, 1.0, 1.0);
uniform sampler2D font;

void main()
{	
	float r = texture2D(font, texcoord).r;
	frag_color = color * r;
	//frag_color = vec4(texcoord, 0.0, 1.0);
}