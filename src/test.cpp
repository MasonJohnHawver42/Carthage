
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <vector>
#include <unordered_map>
#include <tuple>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <cstring>
#include <limits>

#include "resources/resources.hpp"

struct Vertex 
{
    float x, y, z;
    float nx, ny, nz;
    float tx, ty;
};

void convert_path(char* fn) 
{
    int i = 0;
    while (fn[i] != '\0') 
    {
        if (fn[i] == '\\') { fn[i] = '/'; }
        i++;
    }
}
 
int convert_object(const char* in_fn, const char* in_dir, const char* out_fn) 
{
    std::string inputfn = std::string(MY_DATA_DIR) + in_fn;
    std::string outputfn = std::string(MY_DATA_DIR) + out_fn;

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = std::string(MY_DATA_DIR) + in_dir; // Path to material files

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(inputfn, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        return 1;
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    res::Model model;

    model.aabb_min[0] = std::numeric_limits<float>::infinity();
    model.aabb_min[1] = std::numeric_limits<float>::infinity();
    model.aabb_min[2] = std::numeric_limits<float>::infinity();

    model.aabb_max[0] = -std::numeric_limits<float>::infinity();
    model.aabb_max[1] = -std::numeric_limits<float>::infinity();
    model.aabb_max[2] = -std::numeric_limits<float>::infinity();

    std::vector<Vertex> verts;
    std::vector<unsigned int> inds;
    for (size_t m = 0; m < materials.size(); m++) 
    {
        std::unordered_map<size_t, unsigned int> vert_map;

        model.m_meshes[m].m_offset = inds.size();
        model.m_meshes[m].m_matindex = m;

        for (int ti = 0; ti < 3; ti++) 
        {
            model.m_matpool[m].color[ti] = materials[m].diffuse[ti];
        }

        std::strcpy(model.m_matpool[m].diffuse_fn, in_dir);
        std::strcat(model.m_matpool[m].diffuse_fn, materials[m].diffuse_texname.c_str());
        convert_path(model.m_matpool[m].diffuse_fn);
        std::cout << model.m_matpool[m].diffuse_fn << std::endl;
        // std::cout << model.m_matpool[m].color[0] << " " << model.m_matpool[m].color[1] << " " << model.m_matpool[m].color[2] << " " << m << std::endl;

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
                        size_t hash = idx.vertex_index + (idx.normal_index * attrib.vertices.size()) + (idx.texcoord_index * attrib.vertices.size() * attrib.normals.size());
                        
                        auto it = vert_map.find(hash);
                        if (it != vert_map.end()) 
                        {
                            inds.push_back(vert_map[hash]);
                        }
                        else 
                        {
                            Vertex vert;
                            vert.x = attrib.vertices[3*size_t(idx.vertex_index)+0];
                            vert.y = attrib.vertices[3*size_t(idx.vertex_index)+1];
                            vert.z = attrib.vertices[3*size_t(idx.vertex_index)+2];

                            model.aabb_min[0] = std::min(model.aabb_min[0], vert.x);
                            model.aabb_min[1] = std::min(model.aabb_min[1], vert.y);
                            model.aabb_min[2] = std::min(model.aabb_min[2], vert.z);

                            model.aabb_max[0] = std::max(model.aabb_max[0], vert.x);
                            model.aabb_max[1] = std::max(model.aabb_max[1], vert.y);
                            model.aabb_max[2] = std::max(model.aabb_max[2], vert.z);

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

                            inds.push_back(verts.size());
                            vert_map[hash] = verts.size();
                            verts.push_back(vert);

                            // std::cout << idx.vertex_index << " " << vert.x << " " << vert.y << " " << vert.z << " " << verts.size() - 1 << std::endl;
                        }

                    }

                    // std::cout << std::endl;
                }
                
                index_offset += fv;

            }


        } 

        model.m_meshes[m].m_count = inds.size() - model.m_meshes[m].m_offset;
    }

    float* data = (float*)malloc(verts.size() * sizeof(float) * 8);
    unsigned int* inds_data = (unsigned int*)malloc(inds.size() * sizeof(unsigned int));

    std::copy(inds.begin(), inds.end(), inds_data);
    for (size_t i = 0; i < verts.size(); i++) 
    {
        data[(i * 8) + 0] = verts[i].x;
        data[(i * 8) + 1] = verts[i].y;
        data[(i * 8) + 2] = verts[i].z;
        data[(i * 8) + 3] = verts[i].nx;
        data[(i * 8) + 4] = verts[i].ny;
        data[(i * 8) + 5] = verts[i].nz;
        data[(i * 8) + 6] = verts[i].tx;
        data[(i * 8) + 7] = verts[i].ty;
    }

    model.data = data;
    model.indicies = inds_data;

    model.vc = verts.size();
    model.ic = inds.size();
    model.mesh_count = materials.size();
    model.mat_count = materials.size();

    std::ofstream fs(outputfn, std::ios::out | std::ios::binary );

    if (!fs) {
        std::cerr << "Error opening the file." << std::endl;
        return 1;
    }

    std::cout << model.vc << " " << model.ic << std::endl;

    fs.write((const char*)(&model.vc), sizeof(unsigned int));
    fs.write((const char*)(&model.ic), sizeof(unsigned int));

    fs.write((const char*)(&model.mesh_count), sizeof(unsigned int));
    fs.write((const char*)(&model.mat_count), sizeof(unsigned int));

    fs.write((const char*)(model.m_matpool), sizeof(res::Material) * 64);
    fs.write((const char*)(model.m_meshes), sizeof(res::Mesh) * 64);

    fs.write((const char*)(model.data), sizeof(float) * model.vc * 8);
    fs.write((const char*)(model.indicies), sizeof(unsigned int) * model.ic);

    fs.write((const char*)(model.aabb_min), sizeof(float) * 3);
    fs.write((const char*)(model.aabb_max), sizeof(float) * 3);

    fs.close();

    //free
    delete[] model.data;
    delete[] model.indicies;

    return 0;
}

int main(void) 
{

    int res0 = convert_object("models/bunny/bunny.obj", "models/bunny/", "binary/bunny.bin");
    int res1 = convert_object("models/sponza/sponza.obj", "models/sponza/", "binary/sponza.bin");
    int res2 = convert_object("models/cube/cube.obj", "models/cube/", "binary/cube.bin");
    int res3 = convert_object("models/035_power_drill/google_64k/textured.obj", "models/035_power_drill/google_64k/", "binary/drill.bin");
    int res4 = convert_object("models/005_tomato_soup_can/google_64k/textured.obj", "models/005_tomato_soup_can/google_64k/", "binary/can.bin");

    return 0;
}