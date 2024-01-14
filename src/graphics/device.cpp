#include <glad/glad.h>

#include "graphics/primitives.hpp"
#include "graphics/device.hpp"

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
            std::cout << infoLog << std::endl;
            return; //call error on future logging sys
        }

        glDeleteShader(vert);
        glDeleteShader(frag);

        *program = shaderProgram;
    }

    void bind_program(const Program& program) 
    {
        glUseProgram(program);
    }
    
    void set_uniform_int(const char* name, int value, const Program& program) 
    {
        glUniform1i(glGetUniformLocation(program, name), (int)value);
    }

    void set_uniform_mat4(const char* name, float* value, const Program& program) 
    {
        glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, value);
    }

    void free_program(Program* program) 
    {
        glDeleteProgram(*program);
        *program = 0;
    }

    //buffer

    void create_buffer(BufferTarget target, BufferUsage usage, const void* data, std::size_t size, Buffer* buffer) 
    {
        buffer->m_size = size;

        switch (usage) 
        {
            case DRAW_STATIC:
                buffer->m_usage = GL_STATIC_DRAW; break;
            case DRAW_DYNAMIC:
                buffer->m_usage = GL_DYNAMIC_DRAW; break;
            case DRAW_STREAM:
                buffer->m_usage = GL_STREAM_DRAW; break;
            default:
                return; //call error on future logging sys
        }

        switch (target) 
        {
            case VERTEX_DATA:
                buffer->m_target = GL_ARRAY_BUFFER; break;
            case INDEX_DATA:
                buffer->m_target = GL_ELEMENT_ARRAY_BUFFER; break;
            default:
                return; //call error on future logging sys
        }

        unsigned int buffer_id;
        glGenBuffers(1, &buffer_id);

        glBindBuffer(buffer->m_target, buffer_id);
        glBufferData(buffer->m_target, buffer->m_size, data, buffer->m_usage); 
        glBindBuffer(buffer->m_target, 0);

        buffer->buffer_id = buffer_id;
    }

    void bind_buffer(Buffer& buffer) 
    {
        glBindBuffer(buffer.m_target, buffer.buffer_id);
    }

    void unbind_buffer(Buffer& buffer) 
    {
        glBindBuffer(buffer.m_target, 0);
    }

    void free_buffer(Buffer* buffer) 
    {
        glDeleteBuffers(1, &buffer->buffer_id);
        buffer->buffer_id = 0;
    }

    //vao

    void create_vao(VertexAtributes* vao) 
    {
        unsigned int vao_id;
        glGenVertexArrays(1, &vao_id);

        *vao = vao_id;
    }

    void add_atributes(Atribute* atribs, unsigned int length, VertexAtributes& vao) 
    {

        glBindVertexArray(vao);

        for (unsigned int i = 0; i < length; i++) 
        {
            Atribute* atrib = atribs + i;
            bind_buffer(atrib->m_vb);
            switch(atrib->m_type) 
            {
                case FLOAT:
                    glVertexAttribPointer(atrib->m_index, atrib->m_size, GL_FLOAT, atrib->m_normalized ? GL_TRUE : GL_FALSE, atrib->m_stride, (void*)atrib->m_offset); break;
                case INT:
                    glVertexAttribIPointer(atrib->m_index, atrib->m_size, GL_INT, atrib->m_stride, (void*)atrib->m_offset); break;
                case DOUBLE:
                    glVertexAttribLPointer(atrib->m_index, atrib->m_size, GL_DOUBLE, atrib->m_stride, (void*)atrib->m_offset); break;
                default:
                    return; //call error on future logging sys
            }

            glEnableVertexAttribArray(atrib->m_index);
            unbind_buffer(atrib->m_vb);
        }

        glBindVertexArray(0);
    }

    void bind_vao(VertexAtributes& vao) 
    {
        glBindVertexArray(vao);
    }

    void unbind_vao() 
    {
        glBindVertexArray(0);
    }

    void free_vao(VertexAtributes* vao) 
    {
        glDeleteVertexArrays(1, vao);
        *vao = 0;
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
        glClear(GL_COLOR_BUFFER_BIT);
    }

    //mesh

    void create_mesh(void* vs, unsigned int* ind, unsigned int vc, unsigned int ic, Mesh* mesh) 
    {
        create_buffer(VERTEX_DATA, DRAW_STATIC, vs, sizeof(float) * 8 * vc, &mesh->m_vb);
        create_buffer(INDEX_DATA, DRAW_STATIC, (void*)ind, sizeof(unsigned int) * ic, &mesh->m_ebo);
        create_vao(&mesh->m_vao);

        {
            Atribute atribs[3] = {
                {mesh->m_vb, 0, 3, FLOAT, false, 8 * sizeof(float), 0},
                {mesh->m_vb, 1, 3, FLOAT, false, 8 * sizeof(float), 3 * sizeof(float)},
                {mesh->m_vb, 2, 2, FLOAT, false, 8 * sizeof(float), 6 * sizeof(float)}
            };
            
            add_atributes(atribs, 3, mesh->m_vao);
        }

        mesh->m_vc = vc;
        mesh->m_ic = ic;
    }

    void bind_mesh(Mesh& mesh) 
    {
        bind_vao(mesh.m_vao);
        bind_buffer(mesh.m_ebo);
    }

    void draw_mesh(Mesh& mesh) 
    {
        glDrawElements(GL_TRIANGLES, mesh.m_ic, GL_UNSIGNED_INT, 0);
    }

    void unbind_mesh(Mesh& mesh) 
    {
        unbind_buffer(mesh.m_ebo);
        unbind_vao();
    }

    void free_mesh(Mesh* mesh) 
    {
        free_buffer(&mesh->m_ebo);
        free_buffer(&mesh->m_vb);
        free_vao(&mesh->m_vao);
    }



}