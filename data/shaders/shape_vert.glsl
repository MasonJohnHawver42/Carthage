#version 430 core
layout (location = 0) in vec3 aPos;   // the position variable has attribute position 0

struct Instance {
    mat4 model;
    vec4 color;
};

layout(std430, binding = 1) readonly buffer ssbo1 {
    Instance instances[];
};

out vec4 ourColor;

uniform int offset;
uniform mat4 VP;

void main()
{
    Instance data = instances[gl_InstanceID + offset];
    gl_Position = VP * data.model * vec4(aPos, 1.0);
    ourColor = data.color; // set ourColor to the input color we got from the vertex data
}     