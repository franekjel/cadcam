#version 420 core
layout(location = 0) in vec3 pos;

out vec3 wpos;

uniform mat4 m_trans;
uniform mat4 m_view;
uniform mat4 m_proj;

void main(){
    gl_PointSize = 4.0f;
    gl_Position = m_proj * m_view * m_trans * vec4(pos, 1.0);
    wpos = (m_trans * vec4(pos, 1.0)).xyz;
}
