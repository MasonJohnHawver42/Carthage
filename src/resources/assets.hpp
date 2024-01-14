#pragma once

namespace res 
{

    struct Image 
    {

    };

    struct Vertex 
    {
        float pos[3];
        float normal[3];
        float uv[2];
    };


    struct Mesh 
    {
        Vertex* verts;
        unsigned int* indicies;
    };

}