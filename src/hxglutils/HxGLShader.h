#ifndef HX_GL_UTILS_GL_SHADER_H
#define HX_GL_UTILS_GL_SHADER_H

#include <mcgl/internal/mcgl.h>
#include <mclib/internal/McFilename.h>

#include <hxglutils/api.h>
#include <hxglutils/HxContextCache.h>

/**
    Class HxGLShader

    This class allows simple shader handling. It is multi-thread
    stable and allows more than one graphic context. Another feature
    is the possibility of includes in shader code. One can use
    "#include" following by a filename with " to load sub-shader.
    The class will compose the codes and returnes the correct lines
    of the errors and their corresponding files.

    author: Norbert Lindow
*/
class HXGLUTILS_API HxGLShader
{
    // sub classes
    private:

        /**
            Class CodeIncludeTree

            Holds the start line and end line in the composed
            code of an included file. Furthermore it holds an
            array of children of includes. The tree will be
            constructed from outside, by the Code class. One
            can print the tree or computes the mapping of a line
            in the composed code to the original line in an
            include file.

            author: Norbert Lindow
        */
        class CodeIncludeTree
        {
            // TODO write destructor

            public:

                /**
                    Returns the original line and code file of an
                    input line of the composed code.
                */
                int getFileAndLine(int line, McString& file);

                /**
                    Prints an include tree in the console.
                */
                void print();

            private:

                /**
                    Recursive print function, that is used by the public
                    one.
                */
                void print(CodeIncludeTree* tree, McString& dummy);

            public:

                int                          mEnd;
                McFilename                   mFileName;
                int                          mStart;
                McDArray< CodeIncludeTree* > mIncludes;
        };


        /**
            Class CodeLine

            This class holds all words separated by non symbols of
            a line in code. The words will be saved as array of strings.
            The class provides some analysis methods to parse "#include"
            and the corresponding file in a code line.

            author: Norbert Lindow
        */
        class CodeLine
        {
            public:

                /**
                    Index operator of the words.
                */
                McString& operator[](int index);

                /**
                    Clears the array of words.
                */
                void clear();

                /**
                    Parse an include file. The include file has to be
                    the second word and it should starts with " and end
                    with ". If thats the case the method returns true, else
                    false.
                */
                bool parseIncludeFile(McString& includeFile);

                /**
                    Prints a code line in the console of amira.
                */
                void print();

                /**
                    Pushes a word to the end.
                */
                void push(const McString& word);

                /**
                    Proofs if a line start with the word given as parameter.
                */
                bool startWith(const McString& string);

            public:

                McDArray< McString > mWords;
        };


        /*
            Class Code

            The class provides loading shader code with includes. It holds
            the code as array of CodeLines and a CodeIncludeTree. One can
            get the composed code as openGL char. The method provides an
            code error parser, which allow to deliver the correct lines and
            files of the errors.

            author: Norbert Lindow
        */
        class Code
        {
            public:

                /**
                    Index operator on a code line.
                */
                CodeLine& operator[](int index);

                /**
                    Returns the code as c-string. The method reserves memory and
                    fills it with the code. The user has to care about freeing the
                    memory.
                */
                char* getCode(int& length);

                /**
                    Loads a shader with includes. The method creates the composed
                    code in an array of code lines an built up the include tree.
                */
                void load(const char* file);

                /**
                    Prints the code in the console of amira.
                */
                void print();

                /**
                    Parses the code errors and prints them with the correct line
                    in the original file.
                */
                void printErrorCode(char* code, int length);

            private:

                /**
                    Generates the code and include tree. called by load().
                */
                void generate(McDArray< CodeLine >& lines, const char* file, CodeIncludeTree* includeTree);

                /**
                    Parses the line of an error. The line is surrounded by "()".
                */
                int getErrorLine(McString& lineWord);

                /**
                    Loads a code file and returns the length and the code
                    as c-string. The method adds an new line at the end of
                    the code (this is better to scan the code lines).
                */
                char* loadCode(const char* file, int& length);

                /**
                    The method creates code "lines" of an input string "code".
                    The words of a line are divided by non-symbols likes tabulators
                    or spaces. These symbols will be removed. The parameter "length"
                    sets the length of the input string.
                */
                void scanCode(McDArray< CodeLine >& lines, char* code, int length);


            // attributes
            public:

                McDArray< CodeLine > mLines;
                CodeIncludeTree      mIncludeTree;


        };


        /**
            Structure VertexAttribute

            The struct hold the name and the position as index of a
            vertex attribute.

            author: Norbert Lindow
        */
        struct VertexAttribute
        {
            McString    mName;
            int         mPosition;
        };


        /**
            Class GLShader

            The class holds the shader program. It is used as template
            of HxContextCache. The class creates and loads/reloads a shader
            if necessary

            author: Norbert Lindow
        */
        class GLShader
        {

            // constructor destructor
            public:

                 GLShader();
                ~GLShader();


            // methods
            public:

                /**
                    Updates the shader. That means creating and loading.
                */
                void updateResource(void* data);


            private:

                /**
                    Clears the shader.
                */
                void clear();

                /**
                    Returns the errors of a shader object.
                */
                char* getInfoLog(GLhandleARB& object, int& size);

                /**
                    Loads a shader.
                */
                GLhandleARB loadShader(GLenum type);

                /**
                    Sets a vertex attribute.
                */
                void setVertexAttributes(McDArray < VertexAttribute >& attributes);


            // attributes
            public:

                bool        mEnable;
                McString    mName;
                GLhandleARB mShaderProgram;
        };


        /**
            Structure ShaderData

            The structure holds the name and properties of a shader.

            author: Norbert Lindow
        */
        struct ShaderData
        {
            McString                        mName;
            GLint                           mMaxVerticesOut;
            GLint                           mInType;
            GLint                           mOutType;
            McDArray < VertexAttribute >    mVertexAttributes;
        };



    // constructors and destructor
    public:

                 /// default constructor
                 HxGLShader(void);


                /// default destructor
        virtual ~HxGLShader(void);



    // methods
    public:

        /**
            Adds a vertex attribute to the shader.
        */
        void addVertexAttribute(int position, const char* name);

        /**
            Clears the array of vertex attributes.
        */
        void clearVertexAttributes();

        /**
            Disables a shader. Each enable call should have a
            disable call.
        */
        void disable();

        /**
            Enables a shader. All primitives rendered after that
            call will use the shader.
        */
        void enable(unsigned int contextID);

        /**
         * Enables a shader. All primitives rendered after that call will use the shader
         * Returns program object, returns 0 if program is not executable
         */
        GLuint enableAndReturnProgram(unsigned int contextID);

        /**
            Returns true if the shader is executable (without errors).
        */
        bool isExecutable(unsigned int contextID);

        /**
            Loads a normal shader program. The method expect a filename
            without extension and will load a vertex shader with the same
            name and extension "vert" and a fragment shader with extension
            "frag".
        */
        void load(const char* filename);

        /**
            Works similar to load for normal shader, but loads a geometry
            shader, too. This shader should have the extension "geom".
        */
        void load(const char* filename, GLint maxOut, GLint inType, GLint outType);




        /************** FUNTIONS MUST BE CALLED WHILE SHADER IS ENABLED **************/
        /**
            Sets a uniform shader parameter.
        */
        void setParameter1f(unsigned int contextID, const char* name, float value);
        void setParameter2f(unsigned int contextID, const char* name, const float value[2]);
        void setParameter3f(unsigned int contextID, const char* name, const float value[3]);
        void setParameter4f(unsigned int contextID, const char* name, const float value[4]);
        void setParameter1i(unsigned int contextID, const char* name, int value);
        void setParameterMatrix2f(unsigned int contextID, const char* name, const float* value);
        void setParameterMatrix3f(unsigned int contextID, const char* name, const float* value);
        void setParameterMatrix4f(unsigned int contextID, const char* name, const float* value);

       /*
        *  Sets a uniform texture unit
        *  textunit must be i in GL_TEXTUREi, DO NOT USE GL_TEXTUREi DIRECTLY
        */
        void setTextureUnit(unsigned int contextID, const char* name, int texunit) {
            setParameter1i(contextID, name, texunit);
        }
        /************** *********************************************** **************/




    // attributes
    private:

        ShaderData                   mData;
        HxContextCache < GLShader >  mShader;

};







#endif
