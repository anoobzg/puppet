#version 430 core

out vec4 fragment_color;

in vec3 view_direction;
in vec3 normal;
in vec2 coord;

uniform sampler2D tex;
uniform vec4 front_ambient = vec4(0.7, 0.7, 0.7, 1.0);
uniform vec4 front_diffuse = vec4(0.3, 0.3, 0.3, 1.0);
uniform vec4 back_ambient = vec4(0.3, 0.3, 0.3, 1.0);
uniform vec4 back_diffuse = vec4(0.3, 0.3, 0.3, 1.0);
uniform vec4 specular = vec4(0.5, 0.5, 0.5, 1.0);
uniform float specular_power = 128.0;
uniform vec3 light_direction = vec3(0.0, 0.0, 1.0);

void main( void )
{
	vec4 core_color = texture2D(tex, coord);
	vec3 flight_direction = normalize(light_direction);
	vec3 fnormal 		  =	normalize(normal);
	vec4 ambient_color 	  = front_ambient;
	vec4 diffuse_color    = front_diffuse;
	vec4 specular_color   = specular;
	
	if(!gl_FrontFacing)
	{
		fnormal.z  = - fnormal.z;
		ambient_color = back_ambient;
		diffuse_color = back_diffuse;
		specular_color = vec4(0.0, 0.0, 0.0, 1.0);
	    
		core_color = vec4(0.6, 0.6, 0.6, 1.0);
	}
	
	float NdotL 		  = dot(fnormal, flight_direction);
	ambient_color 		  = ambient_color * core_color;
	diffuse_color         = diffuse_color * core_color;
	vec3 freflection      = normalize(((2.0 * fnormal) * NdotL) - flight_direction);
	vec3 fview_direction  = normalize(view_direction);
	float RdotV           = max(0.0, dot(freflection, fview_direction)); 
	vec4 total_diffuse    = NdotL * diffuse_color;
	vec4 total_specular   = specular_color * pow( RdotV, 70.0);
	core_color = ambient_color + total_diffuse + total_specular;
	fragment_color = vec4(core_color.rgb, 1.0);
}
