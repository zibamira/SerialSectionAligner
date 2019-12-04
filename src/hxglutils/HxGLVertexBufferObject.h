#ifndef HX_GL_UTILS_GL_VERTEX_BUFFER_OBJECT_H
#define HX_GL_UTILS_GL_VERTEX_BUFFER_OBJECT_H

#include <mcgl/internal/mcgl.h>
#include <mclib/McDArray.h>
#include <mclib/internal/McString.h>

#include <hxglutils/api.h>
#include <hxglutils/HxContextCache.h>

class HXGLUTILS_API HxGLVertexBufferObject
{



    private:


        class GLVBO
        {

            public:

                 GLVBO();
                ~GLVBO();

            public:

                void updateResource(void* data);

            public:

                GLsizeiptr  mNumVerts;
                GLsizeiptr  mVertSize;
                GLuint      mVBO;
                GLenum      mTarget;
                bool        mEnable;
        };


        struct VBOData
        {
            void*       mData;
            GLsizeiptr  mNumVertices;
            GLenum      mTarget;
            GLsizeiptr  mVertexSize;
        };



    // constructors and destructor
    public:

                 /// default constructor
                 HxGLVertexBufferObject(void);


                /// default destructor
        virtual ~HxGLVertexBufferObject(void);



    // methods
    public:

        void disableAttribPointer(unsigned int contextID, int att, GLint size = 4, GLenum dataTyp = GL_FLOAT);

        void disableTexCoordPointer(unsigned int contextID, int tex, GLint size = 4, GLenum dataTyp = GL_FLOAT);

        void disableVertexAndTexCoordPointer(unsigned int contextID, GLint size = 4, GLenum dataTyp = GL_FLOAT);

        bool enableAttribPointer(unsigned int contextID, int att, GLint size = 4, GLenum dataTyp = GL_FLOAT);

        /*
         * see glVertexAttribPointer for description of parameters, stride is vertex size specified in setBuffer() method
         * NOT TESTED!
         * TODO test method
         */
        bool enableSingleAttribPointer(unsigned int contextID, int att, GLint size = 4, GLenum dataTyp = GL_FLOAT, const GLvoid* pointer = 0);

        bool enableTexCoordPointer(unsigned int contextID, int tex, GLint size = 4, GLenum dataTyp = GL_FLOAT);

        bool enableVertexAndTexCoordPointer(unsigned int contextID, GLint size = 4, GLenum dataTyp = GL_FLOAT);

        int  getNumVertices();

        void setBuffer(int numVertices, int vertexSize, void* data, GLenum target = GL_ARRAY_BUFFER);

    private:

        GLint getTypeSize(GLenum type);



    // attributes
    private:

        VBOData                   mData;
        HxContextCache < GLVBO >  mVBO;

};







#endif
