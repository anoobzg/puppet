#version 430 core

out vec4 frag_color;
uniform sampler2D texture;

in vec2 texcoord;
in vec4 color;
void main()
{	
	frag_color = texture2D(texture, texcoord);;
	//frag_color = vec4(texcoord, 0.0, 1.0);
}