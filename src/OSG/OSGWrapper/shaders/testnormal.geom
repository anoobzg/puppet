#version 430 core
layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vec3 position;
in vec3 normal;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main()
{
	mat4 m = projection_matrix * view_matrix * model_matrix;
	
	gl_Position = m * vec4(position, 1.0);
	EmitVertex();
	gl_Position = m * vec4(position + vec3(100.0, 0.0, 0.0), 1.0);
	EmitVertex();
	EndPrimitive();
}