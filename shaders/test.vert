#version 400

layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_normal;
//layout(location = 2) in vec3 camera_ray_eye;
in int gl_VertexID;
uniform int vert_index;
uniform mat4 model, view, proj; 
uniform vec3 col_choice_vert;
uniform vec3 col_picked_vert;
out vec3 position_eye, normal_eye, col_choice_frag;


void main()
{
  gl_PointSize = 5.0;
  position_eye = vec3(view * model * vec4(vertex_position, 1.0));
  normal_eye = vec3(view * model * vec4(vertex_normal, 0.0));
  gl_Position = proj * view * model * vec4(vertex_position, 1.0);

  if(gl_VertexID == vert_index)
  {
   gl_PointSize = 10.0;
   col_choice_frag = col_picked_vert;
  }
  else
  {
   col_choice_frag = col_choice_vert;
  }
}


