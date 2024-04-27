#include <glad/glad.h>

#include "graphics/primitives.hpp"
#include "graphics/device.hpp"

#include <iostream>

namespace gfx 
{

    //program

    void create_shader(ShaderStage stage, const char* source, Shader* shader) 
    {
        unsigned int shader_id;
        switch(stage) 
        {
            case VERTEX:
                shader_id = glCreateShader(GL_VERTEX_SHADER); break;
            case FRAGMENT:
                shader_id = glCreateShader(GL_FRAGMENT_SHADER); break;
            default:
                return; //call error on future logging sys
        }

        glShaderSource(shader_id, 1, &source, NULL);
        glCompileShader(shader_id);

        int gl_success;
        glGetShaderiv(shader_id, GL_COMPILE_STATUS, &gl_success);

        if(!gl_success)
        {
            char infoLog[512]; //need a logging system
            glGetShaderInfoLog(shader_id, 512, NULL, infoLog);
            std::cout << "Shader: " << source << std::endl;
            std::cout << infoLog << std::endl;
            return; //call error on future logging sys
        }

        *shader = shader_id;
    }

    void create_program(const char* vert_source, const char* frag_source, Program* program) 
    {
        Shader vert, frag;
        create_shader(VERTEX, vert_source, &vert);
        create_shader(FRAGMENT, frag_source, &frag);
        
        unsigned int shaderProgram;
        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vert);
        glAttachShader(shaderProgram, frag);
        glLinkProgram(shaderProgram);

        int gl_success;
        glGetProgramiv(shaderProgram, GL_LINK_STATUS, &gl_success);
        if(!gl_success) {
            char infoLog[512];
            glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
            std::cout << "Program : " << vert_source << " " << frag_source << std::endl;
            std::cout << infoLog << std::endl;
            return; //call error on future logging sys
        }

        glDeleteShader(vert);
        glDeleteShader(frag);

        *program = shaderProgram;
    }

    void bind_program(Program program) 
    {
        glUseProgram(program);
    }
    
    void set_uniform_int(const char* name, int value, Program program) 
    {
        glUniform1i(glGetUniformLocation(program, name), (int)value);
    }

    void set_uniform_float(const char* name, float value, Program program) 
    {
        glUniform1f(glGetUniformLocation(program, name), value);
    }

    void set_uniform_mat4(const char* name, float* value, Program program) 
    {
        glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, value);
    }

    void set_uniform_vec4(const char* name, float* value, Program program) 
    {
        glUniform4fv(glGetUniformLocation(program, name), 1, value); 
    }

    void set_uniform_vec3(const char* name, float* value, Program program)
    {
        glUniform3fv(glGetUniformLocation(program, name), 1, value); 
    }

    void set_uniform_vec2(const char* name, float* value, Program program) 
    {
        glUniform2fv(glGetUniformLocation(program, name), 1, value); 
    }

    void free_program(Program* program) 
    {
        glDeleteProgram(*program);
        *program = 0;
    }

    //texture

    GLint get_wrap(WrapConfig w) 
    {
        switch (w) 
        {
            case REPEAT:
                return GL_REPEAT;
            case CLAMP:
                return GL_CLAMP_TO_EDGE;
        }

        return GL_REPEAT;
    }

    GLint get_maxfilter(FilterConfig f) 
    {
        switch (f) 
        {
            case NEAREST:
                return GL_NEAREST;
            
            case LINEAR:
                return GL_LINEAR;
        }

        return GL_LINEAR;
    }

    GLint get_minfilter(FilterConfig min, FilterConfig mipmap) 
    {
        switch (mipmap) 
        {
            case NEAREST:
                switch (min) 
                {
                    case NEAREST:
                        return GL_NEAREST_MIPMAP_NEAREST;
                    
                    case LINEAR:
                        return GL_NEAREST_MIPMAP_LINEAR;
                }
            
            case LINEAR:
                switch (min) 
                {
                    case NEAREST:
                        return GL_LINEAR_MIPMAP_NEAREST;
                    
                    case LINEAR:
                        return GL_LINEAR_MIPMAP_LINEAR;
                }
        }

        return GL_LINEAR_MIPMAP_LINEAR;
    }

    void create_texture2d(WrapConfig xw, WrapConfig yw, FilterConfig max, FilterConfig min, FilterConfig mipmap, Texture2D* tex)
    {
        unsigned int texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, get_wrap(xw));	
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, get_wrap(yw));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, get_minfilter(min, mipmap));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, get_maxfilter(max));

        tex->id = texture;
        tex->m_xwrap = xw;
        tex->m_ywrap = yw;
        tex->m_max = max;
        tex->m_min = min;
        tex->m_mipmap = mipmap;
    }

    void bind_texture2d(Texture2D* tex) 
    {
        glBindTexture(GL_TEXTURE_2D, tex->id);
    }

    void free_texture2d(Texture2D* tex) 
    {
        glDeleteTextures(1, &tex->id);
    }
    
    void load_texture2d(int width, int height, int nc, unsigned char* data, Texture2D* tex) 
    {

        if (nc != 1 && nc != 3 && nc != 4) { return; }

        bind_texture2d(tex);

        GLint pf;
        if (nc == 1) { pf = GL_RED; }
        if (nc == 3) { pf = GL_RGB; }
        if (nc == 4)  { pf = GL_RGBA; }

        glTexImage2D(GL_TEXTURE_2D, 0, pf, width, height, 0, pf, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        tex->width = width;
        tex->height = height;
    }

    void default_texture2d(Texture2D* tex) 
    {
        create_texture2d(WrapConfig::REPEAT, WrapConfig::REPEAT, FilterConfig::NEAREST, FilterConfig::NEAREST, FilterConfig::NEAREST, tex);
        
        unsigned char image[8][8];
        unsigned char* data = (unsigned char *)image;
        for (int i = 0; i < 8 * 8; i++) 
        {
            image[i / 8][i % 8] = (i + i / 8) % 2 ? 0 : 255;
        }

        load_texture2d(8, 8, 1, data, tex);
        // printf("Default Texture %d\n", tex->id);
    }

    void activate_texture2d(Texture2D* tex) 
    {
        // printf("%d\n", tex->id);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex->id);
    }

    //util

    void clear() 
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void enable_depth() 
    {
        glEnable(GL_DEPTH_TEST);
    }

    //model

    void create_model(float* vs, unsigned int* ind, unsigned int vc, unsigned int ic, Model* model) 
    {
        unsigned int vao, vbo, ebo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);
        
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 8 * vc, vs, GL_STATIC_DRAW); 

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * ic, ind, GL_STATIC_DRAW); 

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); //position
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float))); glEnableVertexAttribArray(1); //normals
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float))); glEnableVertexAttribArray(2); //tex-coords

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        model->m_vao = vao;
        model->m_vbo = vbo;
        model->m_ebo = ebo;

        model->mesh_count = 0;
        model->mat_count = 0;
    }

    void add_mesh_model(Mesh* mesh, Model* model)
    {
        if (model->mesh_count >= 64) { return; }
        Mesh* mesh_dest = model->m_meshes + model->mesh_count;
        *mesh_dest = *mesh;
        model->mesh_count++;
    }

    void bind_model(Model* model) 
    {
        glBindVertexArray(model->m_vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model->m_ebo);
    }

    void draw_mesh(Program program, Model* model, unsigned int i, gfx::Material* material) 
    {
        // set_uniform_int("ourTexture", 0, program);
        set_uniform_vec3("Color", material->color, program);

        glDrawElements(GL_TRIANGLES, model->m_meshes[i].m_count, GL_UNSIGNED_INT, (void*)(sizeof(unsigned int) * model->m_meshes[i].m_offset));
    }

    void unbind_model() 
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }


    void free_model(Model* model) 
    {
        glDeleteBuffers(1, &model->m_vbo);
        glDeleteBuffers(1, &model->m_ebo);
        glDeleteVertexArrays(1, &model->m_vao);
        
        model->m_vbo = 0;
        model->m_ebo = 0;
        model->m_vao = 0;
    }


    float shape_vs[] = 
    {
        0.5f, -0.5f, -0.5f,
        0.5f, -0.5f, 0.5f,
        -0.5f, -0.5f, 0.5f,
        -0.5f, -0.5f, -0.5f,
        0.5f, 0.5f, -0.5f,
        0.5f, 0.5f, 0.5f,
        -0.5f, 0.5f, 0.5f,
        -0.5f, 0.5f, -0.5f
    };

    unsigned int shape_ind[] = 
    {
        1, 2, 3,
        4, 7, 6,
        4, 5, 1,
        1, 5, 6,
        6, 7, 3,
        4, 0, 3,
        0, 1, 3,
        5, 4, 6,
        0, 4, 1,
        2, 1, 6,
        2, 6, 3,
        7, 4, 3
    };

    void create_shape_buffer(unsigned int size, ShapeBuffer* sb) 
    {
        unsigned int vao, vbo, ebo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);
        
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(shape_vs), shape_vs, GL_STATIC_DRAW); 

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(shape_ind), shape_ind, GL_STATIC_DRAW); 
    
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); //position

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        sb->m_vao = vao;
        sb->m_vbo = vbo;
        sb->m_ebo = ebo;

        unsigned int ssbo;
        glGenBuffers(1, &ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, size * sizeof(ShapeEntry), NULL, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        sb->m_ssbo = ssbo;

        sb->m_shapes[0].m_offset = 0;
        sb->m_shapes[0].m_count = 36;
        sb->m_count = 0;
    }

    void push_shape_buffer(unsigned int size, Shape type, void* data, ShapeBuffer* sb) 
    {
        if (sb->m_count == Shape::SHAPE_COUNT && sb->m_partition[sb->m_count - 1].m_type != type) { return; }
        unsigned int offset = sb->m_count == 0 ? 0 : (sb->m_partition[sb->m_count - 1].m_offset + sb->m_partition[sb->m_count - 1].m_count);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, sb->m_ssbo);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, offset * sizeof(ShapeEntry), size * sizeof(ShapeEntry), data);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        if ( sb->m_count > 0 && sb->m_partition[sb->m_count - 1].m_type == type) { sb->m_partition[sb->m_count - 1].m_count += size; }
        else 
        {
            sb->m_partition[sb->m_count].m_offset = offset;
            sb->m_partition[sb->m_count].m_count = size;
            sb->m_partition[sb->m_count].m_type = type;
            sb->m_count++;
        }

    }

    void flush_shape_buffer(ShapeBuffer* sb) 
    {
        sb->m_count = 0;
    }

    void draw_shape_buffer(Program program, ShapeBuffer* sb) 
    {
        glBindVertexArray(sb->m_vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sb->m_ebo);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sb->m_ssbo);
        // printf("%d %d %d %d %d %d\n", sb->m_count, sb->m_partition[0].m_type, sb->m_partition[0].m_offset, sb->m_partition[0].m_count, sb->m_shapes[0].m_count, sb->m_shapes[0].m_offset);
        for (int i = 0; i < sb->m_count; i++) 
        {
            unsigned int s = sb->m_partition[i].m_type;
            set_uniform_int("offset", sb->m_partition[i].m_offset, program);
            glDrawElementsInstanced(GL_TRIANGLES, sb->m_shapes[s].m_count, GL_UNSIGNED_INT, (void*)(sizeof(unsigned int) * sb->m_shapes[s].m_offset), sb->m_partition[i].m_count);
        }
        // glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
    }

    void free_shape_buffer(ShapeBuffer* sb) 
    {
        glDeleteBuffers(1, &sb->m_ssbo);
        glDeleteVertexArrays(1, &sb->m_vao);
        glDeleteBuffers(1, &sb->m_ebo);
        glDeleteBuffers(1, &sb->m_vbo);
    }


    void bind_pipeline(Pipeline* pipe) 
    {
        glUseProgram(pipe->m_prog);
        glEnable(GL_CULL_FACE);

        switch (pipe->m_cull) 
        {
            case NONE:  glDisable(GL_CULL_FACE); break;
            case FRONT: glCullFace(GL_FRONT); break;
            case BACK:  glCullFace(GL_BACK); break;
        }

        switch (pipe->m_draw) 
        {
            case FILL: glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); break;
            case LINE: glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); break;
        }
    }

    float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
        // positions   // texCoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,

        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };

    void create_quad_buffer(QuadBuffer* qb) 
    {
        unsigned int vao, vbo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

        qb->m_vao = vao;
        qb->m_vbo = vbo;
    }

    void draw_quad_buffer(Program program, float* pos, float* size, float* color, QuadBuffer* qb) 
    {
        set_uniform_vec2("offset", pos, program);
        set_uniform_vec2("scale", size, program);
        set_uniform_vec4("color", color, program);

        glBindVertexArray(qb->m_vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }
    
    void free_quad_buffer(QuadBuffer* qb) 
    {
        glDeleteBuffers(1, &qb->m_vbo);
        glDeleteVertexArrays(1, &qb->m_vao);
        
    }

    float quad_data[] = 
    {
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 
        1.0f, 0.0f, 1.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        1.0f, 0.0f, 0.0f, 1.0f, 1.0f,
    };

    unsigned int quad_id[] = {0, 1, 2, 3};

    void create_voxel_buffer(unsigned int face_cap, unsigned int chunk_cap, unsigned int draw_cap, VoxelBuffer* vb) 
    {
        vb->face_cap = face_cap; vb->faces_used = 0;
        vb->chunk_cap = chunk_cap; vb->chunks_used = 0;
        vb->draw_call_cap = draw_cap; vb->draw_calls_used = 0;

        unsigned int vao, vbo, id_vbo, face_vbo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &face_vbo);
        glGenBuffers(1, &id_vbo);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, face_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(unsigned int) * face_cap, NULL, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, id_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quad_id), &quad_id, GL_STATIC_DRAW);
        
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quad_data), &quad_data, GL_STATIC_DRAW);
         
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, id_vbo); // this attribute comes from a different vertex buffer
        glVertexAttribIPointer(2, 1, GL_UNSIGNED_INT, sizeof(unsigned int), (void*)0);
        
        glEnableVertexAttribArray(3);
        glBindBuffer(GL_ARRAY_BUFFER, face_vbo); // this attribute comes from a different vertex buffer
        glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, sizeof(unsigned int), (void*)0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        
        glVertexAttribDivisor(3, 1); // tell OpenGL this is an instanced vertex attribute.

        vb->m_face_vbo = face_vbo;
        vb->m_id_vbo = id_vbo;
        vb->m_vbo = vbo;
        vb->m_vao = vao;

        unsigned int chunk_ssbo, draw_ssbo;
        glGenBuffers(1, &chunk_ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, chunk_ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, chunk_cap * sizeof(float) * 3 , NULL, GL_STATIC_DRAW);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        glGenBuffers(1, &draw_ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, draw_ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, draw_cap * sizeof(unsigned int), NULL, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        vb->m_chunk_ssbo = chunk_ssbo;
        vb->m_draw_ssbo = draw_ssbo;

        unsigned int draw_call_ib;
        glGenBuffers(1, &draw_call_ib);
        glBindBuffer(GL_DRAW_INDIRECT_BUFFER, chunk_ssbo);
        glBufferData(GL_DRAW_INDIRECT_BUFFER, draw_cap * sizeof(DrawArraysIndirectCommand), NULL, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);

        vb->m_draw_call_ib = draw_call_ib;
    }

    unsigned int push_faces_voxel_buffer(unsigned int* faces, unsigned int face_count, VoxelBuffer* vb) 
    {
        unsigned int start = vb->faces_used;

        glBindBuffer(GL_ARRAY_BUFFER, vb->m_face_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, start * sizeof(unsigned int), face_count * sizeof(unsigned int), faces);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        vb->faces_used += face_count;
        return start;
    }

    unsigned int push_chunk_voxel_buffer(float* pos, VoxelBuffer* vb)
    {
        unsigned int start = vb->chunks_used;
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, vb->m_chunk_ssbo);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, start * sizeof(unsigned int) * 3, 3 * sizeof(float), pos);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
        vb->chunks_used++;
        return start;
    }

    void draw_voxel_buffer(Program program, VoxelChunk** chunks, unsigned int* normals, unsigned int draw_count, VoxelBuffer* vb) 
    {
        unsigned int index_data[draw_count];
        DrawArraysIndirectCommand draw_data[draw_count]; 

        for (int i = 0; i < draw_count; i++) 
        {
            index_data[i] = (normals[i] & 0b111) | (chunks[i]->index << 3);
            draw_data[i].first = 0;
            draw_data[i].count = 4;
            draw_data[i].instanceCount = chunks[i]->m_normals[normals[i]].count;
            draw_data[i].baseInstance = chunks[i]->m_normals[normals[i]].start;
        }

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, vb->m_draw_ssbo);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, draw_count * sizeof(unsigned int), index_data);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

        // glBindBuffer(GL_DRAW_INDIRECT_BUFFER, vb->m_draw_call_ib);
        // glBufferSubData(GL_DRAW_INDIRECT_BUFFER, 0, draw_count * sizeof(DrawArraysIndirectCommand), draw_data);
        // glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);

        glBindVertexArray(vb->m_vao);

        // glBindBuffer(GL_DRAW_INDIRECT_BUFFER, vb->m_draw_call_ib);

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, vb->m_chunk_ssbo);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, vb->m_draw_ssbo);

        unsigned int n;
        for (n = 0; n < draw_count; n++) {
            const DrawArraysIndirectCommand *cmd;
            cmd = (const DrawArraysIndirectCommand  *)draw_data + n;
            // printf("%d %d %d %d\n", cmd->first, cmd->count, cmd->instanceCount, cmd->baseInstance);
            set_uniform_int("index_data0", n, program);
            glDrawArraysInstancedBaseInstance(GL_TRIANGLE_STRIP, cmd->first, cmd->count, cmd->instanceCount, cmd->baseInstance);
        }
        // set_uniform_int("index_data0", 0, program);
        // printf("%d %d\n", sizeof(DrawArraysIndirectCommand), sizeof(unsigned int) * 4);
        // glMultiDrawArraysIndirect(GL_TRIANGLE_STRIP, (GLvoid*)0, draw_count, 0);

        glBindVertexArray(0);
        // glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, 0);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, 0);
    }

    void free_voxel_buffer(VoxelBuffer* vb) 
    {
        glDeleteBuffers(1, &vb->m_vbo);
        glDeleteBuffers(1, &vb->m_id_vbo);
        glDeleteBuffers(1, &vb->m_face_vbo);
        glDeleteBuffers(1, &vb->m_chunk_ssbo);
        glDeleteBuffers(1, &vb->m_draw_ssbo);
        glDeleteBuffers(1, &vb->m_draw_call_ib);
        glDeleteVertexArrays(1, &vb->m_vao);
    }


    //Frame Buffers

    void create_framebuffer(unsigned int w, unsigned int h, FrameBuffer* fb) 
    {
        unsigned int fbo;
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo); 
        
        unsigned int color_texture;
        glGenTextures(1, &color_texture);
        glBindTexture(GL_TEXTURE_2D, color_texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture, 0);

        unsigned int rbo;
        glGenRenderbuffers(1, &rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h); // use a single renderbuffer object for both a depth AND stencil buffer.
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
            
        if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) 
        { 
            printf("Frame Buffer Failed\n");
            return;
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        fb->m_fbo = fbo;
        fb->m_color_texture = color_texture;
        fb->m_depth_rbo = rbo;
        fb->width = w;
        fb->hieght = h;
    }

    void bind_framebuffer(FrameBuffer* fb) 
    {
        glBindFramebuffer(GL_FRAMEBUFFER, fb->m_fbo);
    }

    void bind_default_framebuffer() 
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    
    void free_framebuffer(FrameBuffer* fb) 
    {
        glDeleteTextures(1, &fb->m_color_texture);
        glDeleteRenderbuffers(1, &fb->m_depth_rbo);
        glDeleteFramebuffers(1, &fb->m_fbo);  
    }



}