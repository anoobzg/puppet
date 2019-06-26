#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float osg_SimulationTime;
uniform mat4 icp_matrix;
uniform float time;

out vec3 view_direction;
out vec3 normal;
out vec4 color;
out float shine;

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix * icp_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	
	color = vec4(0.0, 1.0, 0.0, 1.0);
	
	float slope = -10.0;
	shine = clamp((slope * (osg_SimulationTime - time) + 1.0), 0.0, 1.0);
}