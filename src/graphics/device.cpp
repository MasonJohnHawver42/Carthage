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
            // std::cout << infoLog << std::endl;
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
            // std::cout << infoLog << std::endl;
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

    void set_uniform_mat4(const char* name, float* value, Program program) 
    {
        glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, value);
    }

    void set_uniform_vec3(const char* name, float* value, Program program)
    {
        glUniform3fv(glGetUniformLocation(program, name), 1, value); 
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
    
    void load_texture2d(int width, int height, int nc, unsigned char* data, Texture2D* tex) 
    {

        if (nc != 3 && nc != 4) { return; }

        bind_texture2d(tex);

        GLint pf = (nc == 3) ? GL_RGB : GL_RGBA;

        glTexImage2D(GL_TEXTURE_2D, 0, pf, width, height, 0, pf, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        tex->width = width;
        tex->height = height;
    }

    //util

    void clear() 
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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

    unsigned int add_material_model(Material* mat, Model* model) 
    {
        if (model->mat_count >= 64) { return 0; }
        Material* mat_dest = model->m_matpool + model->mat_count;
        *mat_dest = *mat;
        model->mat_count++;
        return model->mat_count - 1;
    }

    void add_mesh_model(Mesh* mesh, Model* model)
    {
        if (model->mesh_count >= 64) { return; }
        Mesh* mesh_dest = model->m_meshes + model->mesh_count;
        *mesh_dest = *mesh;
        model->mesh_count++;
    }

    void draw_model(Program program, Model* model) 
    {
        glBindVertexArray(model->m_vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model->m_ebo);

        // glDrawElements(GL_TRIANGLES, 786801, GL_UNSIGNED_INT, 0);
        // glDrawRangeElements(GL_TRIANGLES, 6, 9, 3, GL_UNSIGNED_INT, 0);

        for (unsigned int i = 0; i < model->mesh_count; i++) 
        {
            //load in material and textures
            set_uniform_vec3("Color", model->m_matpool[model->m_meshes[i].m_matindex].color, program);

            //draw mesh
            glDrawElements(GL_TRIANGLES, model->m_meshes[i].m_count, GL_UNSIGNED_INT, (void*)(sizeof(unsigned int) * model->m_meshes[i].m_offset));

            // std::cout <<  model->m_meshes[i].m_offset << std::endl;
            // glDrawRangeElements(GL_TRIANGLES, 
            //                     model->m_meshes[i].m_offset, 
            //                     model->m_meshes[i].m_offset + model->m_meshes[i].m_count, 
            //                     model->m_meshes[i].m_count, 
            //                     GL_UNSIGNED_INT, (void*)0);
        }

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

}