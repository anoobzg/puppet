#version 430 core

out vec4 frag_color;

in vec2 texcoord;
uniform vec4 color = vec4(1.0, 0.0, 0.0, 1.0);
uniform sampler2D texture;
uniform float alpha;

void main()
{	
	vec4 tcolor = texture2D(texture, texcoord);
	float a = tcolor.a;
	if(a == 1.0) a = alpha;
	frag_color = vec4(tcolor.xyz, a);
}