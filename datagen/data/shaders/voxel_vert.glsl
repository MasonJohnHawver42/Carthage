#version 460 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoords;
layout (location = 2) in uint aFaceId;
layout (location = 3) in uint aFace; 

layout(std430, binding = 5) buffer chunk_sbbo {
    float chunks_floats[];
};

layout(std430, binding = 4) buffer draw_ssbo {
    uint draw_indexs[];
};

const vec3 normal_vecs[6] = vec3[6](
    vec3(-1.0, 0.0, 0.0),
    vec3(1.0, 0.0, 0.0),
    vec3(0.0, -1.0, 0.0),
    vec3(0.0, 1.0, 0.0),
    vec3(0.0, 0.0, -1.0),
    vec3(0.0, 0.0, 1.0)
);

const mat4 normal_mats[6] = mat4[6](
    mat4(
        0.0, 1.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 1.0, 1.0
    ),

    mat4(
        0.0, 1.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 1.0
    ),

    mat4(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    ),

    mat4(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 1.0, 1.0, 1.0
    ),

    mat4(
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    ),

    mat4(
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        1.0, 0.0, 1.0, 1.0
    )
);

out vec2 TexCord;

uniform mat4 VP;
// uniform mat4 M;

uniform float scale;
uniform int index_data0;

// uniform vec3 ChunkPos;
// uniform vec3 normal;

float ao_vals[] = {
    0.0, 0.2, 0.2, .5
};

flat out vec4 ao_data;
flat out vec3 pos;

void main()
{

    uint index_data = draw_indexs[index_data0];
    uint normal = index_data & 7;
    uint chunk_index = index_data >> 3;

    mat4 M = normal_mats[normal];
    vec3 ChunkPos = vec3(chunks_floats[chunk_index * 3 + 0], chunks_floats[chunk_index * 3 + 1], chunks_floats[chunk_index * 3 + 2]);

    // ChunkPos = vec3(0, 0, 0);

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

    float scale0 = .1;
    mat4 scl = mat4(scale, 0, 0, 0,
                    0, scale, 0, 0,
                    0, 0, scale, 0,
                    ChunkPos.x, ChunkPos.y, ChunkPos.z, 1);

    gl_Position = VP * scl * ((M * vec4(aPos, 1.0f)) + vec4(x, y, z, 0.0));
}  
