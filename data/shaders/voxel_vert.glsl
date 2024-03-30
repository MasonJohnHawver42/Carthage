#version 430 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoords;
layout (location = 2) in uint aFaceId;
layout (location = 3) in uint aFace; 

out vec2 TexCord;

uniform mat4 VP;
uniform mat4 M;

float ao_vals[] = {
    0.0, 0.2, 0.2, .5
};

flat out vec4 ao_data;
flat out vec3 pos;

void main()
{
    uint x = (aFace >> 10) & 31;
    uint y = (aFace >> 5) & 31;
    uint z = (aFace & 31);
    uint light = (aFace >> 15) & 255;

    TexCord = aTexCoords;

    ao_data[0] = ao_vals[light & 3u];
    ao_data[1] = ao_vals[(light >> 4u) & 3u];
    ao_data[2] = ao_vals[(light >> 2u) & 3u];
    ao_data[3] = ao_vals[(light >> 6u) & 3u];

    pos = vec3(x, y, z);

    gl_Position = VP * ((M * vec4(aPos, 1.0f)) + vec4(x, y, z, 0.0) + vec4(0, 15, -33, 0));
}  
