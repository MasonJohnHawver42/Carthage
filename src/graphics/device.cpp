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
        glUniform1f(glGetUniformLocation(program, name), (int)value);
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

    void add_mesh_model(Mesh* mesh, Model* model)
    {
        if (model->mesh_count >= 64) { return; }
        Mesh* mesh_dest = model->m_meshes + model->mesh_count;
        *mesh_dest = *mesh;
        model->mesh_count++;
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

}