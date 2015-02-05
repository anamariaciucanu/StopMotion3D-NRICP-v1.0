#version 400

layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_normal;
//layout(location = 2) in vec3 camera_ray;

uniform mat4 model, view, proj; 
uniform vec3 col_choice_vert;
out vec3 position_eye, normal_eye, camera_ray_eye, col_choice_frag;

void main()
{
  position_eye = vec3(view * model * vec4(vertex_position, 1.0));
  normal_eye = vec3(view * model * vec4(vertex_normal, 0.0));
  gl_Position = proj * view * model * vec4(vertex_position, 1.0);
 // camera_ray_eye = camera_ray;
  col_choice_frag = col_choice_vert;
}


