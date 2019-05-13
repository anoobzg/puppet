#version 430 core

out vec4 fragment_color;

uniform uint mesh_id = 0;  

void main( void )
{				
	uint primitive_id = gl_PrimitiveID + 1;
	
	uint a = primitive_id%256;
	uint b = (primitive_id/(1<<8))%256;
	uint g = (primitive_id/(1<<16))%256;
	uint r = mesh_id%256;
	vec4 color = vec4(float(r)/255.0, float(g)/255.0, float(b)/255.0, float(a)/255.0);
	
	fragment_color = color;
}
