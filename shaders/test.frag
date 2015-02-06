#version 400

in vec3 position_eye, normal_eye, camera_ray_eye, col_choice_frag;
uniform mat4 view;

//Fixed point light properties
vec3 light_position_world = vec3(0.0, 2.0, 5.0);
vec3 Ls = vec3(0.2, 0.2, 0.2); // specular light colour
vec3 Ld = vec3(1.0, 1.0, 1.0); // diffuser light colour
vec3 La = vec3(0.3, 0.3, 0.3); // ambient light colour

//Surface reflectance
vec3 Ks = vec3(0.2, 0.2, 0.2); // fully reflects specular light
vec3 Kd = col_choice_frag;
vec3 Kpicked = vec3(1.0, 0.0, 0.0); //triangle was picked
vec3 Ka = vec3(0.8, 0.8, 0.8); // fully reflect ambient light

float specular_exponent = 100.0; // specular power
out vec4 frag_colour;

void main()
{
  //Raise light position to eye space
  vec3 light_position_eye = vec3(view * vec4(light_position_world, 1.0));
 
  //Calculate diffuse shading (Lambert)
  vec3 distance_to_light_eye = light_position_eye - position_eye;
  vec3 direction_to_light_eye = normalize(distance_to_light_eye);
  float dot_prod = dot(direction_to_light_eye, normal_eye);
  dot_prod = max(dot_prod, 0.0);

  //Calculate specular shading
  //Phong 
  //vec3 reflection_eye = reflect(-direction_to_light_eye, normal_eye);
  //Blinn
  vec3 surface_to_viewer_eye = normalize(-position_eye);
  vec3 half_way_eye = normalize(surface_to_viewer_eye + direction_to_light_eye);

  //less efficient than half vector
  //float dot_prod_specular = dot(reflection_eye, surface_to_viewer_eye);
  float dot_prod_specular = dot(half_way_eye, normal_eye);
  dot_prod_specular = max(dot_prod_specular, 0.0);
  float specular_factor = pow(dot_prod_specular, specular_exponent);
  
  //intersection
  float dot_prod_intersection = dot(camera_ray_eye, normal_eye);
  dot_prod_intersection = max(dot_prod_intersection, 0.0);
  if (dot_prod_intersection < 0.5)
  {
      Ks = Kpicked;
  }

  //ambient intensity
  vec3 Ia = La * Ka;

  //diffuse intensity
  vec3 Id = Ld * Kd * dot_prod; //final diffuse intensity

  //specular intensity
  vec3 Is = Ls * Ks * specular_factor; //final specular intensity 

  //frag_colour = vec4(Is + Id + Ia, 1.0);
  frag_colour = vec4(Id + Ia, 1.0);
}
