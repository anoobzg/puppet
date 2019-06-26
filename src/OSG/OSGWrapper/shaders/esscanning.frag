#version 430 core

out vec4 fragment_color;

in vec3 view_direction;
in vec3 normal;
in vec4 color;
in float shine;

uniform vec4 ambient = vec4(0.3, 0.3, 0.3, 1.0);
uniform vec4 diffuse = vec4(0.3, 0.3, 0.3, 1.0);
uniform vec4 specular = vec4(0.5, 0.5, 0.5, 1.0);
uniform float specular_power = 600.0;
uniform vec3 light_direction = vec3(0.0, 0.0, 1.0);

void main( void )
{	
	vec4 ambient_color 	  = ambient * color;
	vec3 flight_direction = normalize(light_direction);
	vec3 fnormal 		  =	normalize(normal);
	float NdotL 		  = dot(fnormal, flight_direction);
	
	vec4 diffuse_color    = NdotL * diffuse * color;
	
	vec3 freflection      = normalize(((2.0 * fnormal) * NdotL) - flight_direction);
	vec3 fview_direction  = normalize(view_direction);
	float RdotV           = max(0.0, dot(freflection, fview_direction)); 
	
	vec4 specular_color   = specular * pow( RdotV, specular_power);

	fragment_color = ambient_color + diffuse_color + specular_color + shine * vec4(1.0, 1.0, 1.0, 1.0);
}
