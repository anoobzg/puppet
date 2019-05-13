#version 430 core

out vec4 frag_color;

in vec2 tex_coord;

uniform sampler2D tex;
void main()
{	
	vec4 color = vec4(1.0, 1.0, 1.0, 1.0);
	if(tex_coord.x < 0.0 || tex_coord.y < 0.0)
		frag_color = vec4(0.0, 0.0, 0.0, 0.0);
	else
		frag_color = color * texture(tex, tex_coord).r;
}