#include <iostream>
#include <unistd.h>

#include "core/allocators.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <vector>
#include <unordered_map>

struct Vertex 
{
    float x, y, z;
    float nx, ny, nz;
    float tx, ty;
};

struct Mesh 
{
    float* data;
    unsigned int* indicies;
};

struct Material 
{
    char* name;
};

struct Model 
{
    struct 
    {
        Mesh mesh;
        Material mat;
    }* objects;
}

int main(void) {

    std::string inputfile = std::string(MY_DATA_DIR) + "models/sponza/sponza.obj";
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = std::string(MY_DATA_DIR) + "models/sponza"; // Path to material files

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(inputfile, reader_config)) {
    if (!reader.Error().empty()) {
        std::cerr << "TinyObjReader: " << reader.Error();
    }
        exit(1);
    }

    if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();


    for (size_t m = 0; m < materials.size(); m++) 
    {

        std::vector<Vertex> verts();
        std::vector<unsigned int> inds();

        std::unordered_map<std::tuple<int, int, int>, unsigned int> vert_map();
        unsigned int vert_index = 0;

        for (size_t s = 0; s < shapes.size(); s++) 
        {
            size_t index_offset = 0;

            for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) 
            {
                size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
                if(shapes[s].mesh.material_ids[f] == m && fv == 3) 
                {

                    //compute triangle normal
                    //latter compute bitangent and tangent
                    float nx, ny, nz;
                    nx = 0.0f;
                    ny = 0.0f;
                    nz = 0.0f;

                    for (size_t v = 0; v < fv; v++) 
                    {
                        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                        std::tuple<int, int, int> hash = std::make_tuple(idx.vertex_index, idx.normal_index, idx.texcoord_index);
                        auto it = vert_map.find(hash);
                        if (it != vert_map.end() && idx.normal_index >= 0 && idx.texcoord_index >= 0) 
                        {
                            inds.push_back(vert_map[hash]);
                        }
                        else 
                        {
                            Vertex vert;
                            vert.x = attrib.vertices[3*size_t(idx.vertex_index)+0];
                            vert.y = attrib.vertices[3*size_t(idx.vertex_index)+1];
                            vert.z = attrib.vertices[3*size_t(idx.vertex_index)+2];

                            // Check if `normal_index` is zero or positive. negative = no normal data
                            if (idx.normal_index >= 0) {
                                vert.nx = attrib.normals[3*size_t(idx.normal_index)+0];
                                vert.ny = attrib.normals[3*size_t(idx.normal_index)+1];
                                vert.nz = attrib.normals[3*size_t(idx.normal_index)+2];
                            }
                            else 
                            {
                                vert.nx = nx;
                                vert.ny = ny;
                                vert.nz = nz;
                            }

                            // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                            if (idx.texcoord_index >= 0) {
                                vert.tx = attrib.texcoords[2*size_t(idx.texcoord_index)+0];
                                vert.ty = attrib.texcoords[2*size_t(idx.texcoord_index)+1];
                            }
                            else 
                            {
                                vert.tx = 0.0f;
                                vert.ty = 0.0f;
                            }

                            verts.push_back(vert);
                            inds.push_back(vert_index);
                            vert_index++;
                        }
                    }

                }
                
                index_offset += fv;

            }


        } 

        

    }

    return 0;
}