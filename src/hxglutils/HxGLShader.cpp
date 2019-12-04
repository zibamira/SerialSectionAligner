#include <cstdlib>
#include <fstream>

#include <hxcore/HxMessage.h>
#include <hxcore/HxResource.h>

#include "HxGLShader.h"


using namespace std;




HxGLShader::GLShader::GLShader()
    : mEnable(true)
    , mName("")
    , mShaderProgram(0)
{
}




HxGLShader::GLShader::~GLShader()
{
    clear();
}




void HxGLShader::GLShader::clear()
{
    if (mShaderProgram == 0) return;

    glDeleteObjectARB(mShaderProgram);
    mShaderProgram = 0;
}




char* HxGLShader::GLShader::getInfoLog(GLhandleARB& object, int& size)
{
    GLsizei logLength;
    glGetObjectParameterivARB(object, GL_OBJECT_INFO_LOG_LENGTH_ARB, &logLength);

    if (logLength > 1)
    {
        size = logLength + 2;

        char* logInfo = new char[size];
        int   length;

        glGetInfoLogARB(object, logLength, &length, logInfo);

        logInfo[size - 2] = (char) 10;
        logInfo[size - 1] = 0;
        //theMsg->printf("SHADER: %s\n %s\n", name.getString() , logInfo);
        //delete logInfo;

        return logInfo;
    }
    return 0;
}




GLhandleARB HxGLShader::GLShader::loadShader(GLenum type)
{
    McString        fileName = mName;
    GLhandleARB     shader   = 0;

    // shader dependent file and handle
    if (type == GL_VERTEX_SHADER_ARB)
    {
        fileName += ".vert";
        shader    = glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
    }
    else if (type == GL_FRAGMENT_SHADER_ARB)
    {
        fileName += ".frag";
        shader    = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
    }
    else if (type == GL_GEOMETRY_SHADER_EXT)
    {
        fileName += ".geom";
        shader    = glCreateShaderObjectARB(GL_GEOMETRY_SHADER_EXT);
    }

    // load and compile code

    Code code;

    code.load(fileName);

    GLint            codeSize  = 0;
    const GLcharARB* codeChars = code.getCode(codeSize);

    glShaderSourceARB(shader, 1, &codeChars, &codeSize);
    glCompileShaderARB(shader);

    // print errors and warnings

    int   errorLength = 0;
    char* errorChars  = getInfoLog(shader, errorLength);

    code.printErrorCode(errorChars, errorLength);

    delete[] errorChars;

    // proofs if code is executable

    GLint correct = 0;

    glGetObjectParameterivARB(shader, GL_COMPILE_STATUS, &correct);

    if (!correct)
    {
        glDeleteObjectARB(shader);
        shader = 0;
    }

    delete[] codeChars;

    return shader;
}




void HxGLShader::GLShader::setVertexAttributes(McDArray<VertexAttribute> &attributes)
{
    if (!mEnable) return;

    int numAttributes = attributes.size();

    for (int i = 0; i < numAttributes; ++i)
    {
        glBindAttribLocationARB(mShaderProgram, attributes[i].mPosition, attributes[i].mName.getString());

        unsigned int error = glGetError();

        if (error == GL_INVALID_VALUE)
        {
            int numAttribs = 0;
            glGetIntegerv(GL_MAX_VERTEX_ATTRIBS_ARB, &numAttribs);
            theMsg->printf("SHADER: discard %s, only indices between 0 and %d are allowed.\n", attributes[i].mName.getString(), numAttribs);
        }
        else if (error == GL_INVALID_OPERATION)
        {
            theMsg->printf("SHADER: discard %s, invalid name.\n", attributes[i].mName.getString());
        }
    }
}




void HxGLShader::GLShader::updateResource(void *data)
{
    if (!GLEW_ARB_vertex_shader || !GLEW_ARB_fragment_shader || !GLEW_EXT_geometry_shader4)
    {
        mEnable = false;
        return;
    }

    ShaderData* sData = (ShaderData*) data;

    // shader already loaded

    if (mName == sData->mName) return;

    mName = sData->mName;

    clear();

    mShaderProgram  = glCreateProgramObjectARB();

    if (sData->mMaxVerticesOut > 0)
    {
        glProgramParameteriEXT(mShaderProgram, GL_GEOMETRY_INPUT_TYPE_EXT, sData->mInType);
        glProgramParameteriEXT(mShaderProgram, GL_GEOMETRY_OUTPUT_TYPE_EXT, sData->mOutType);
        glProgramParameteriEXT(mShaderProgram, GL_GEOMETRY_VERTICES_OUT_EXT, sData->mMaxVerticesOut);
    }


    GLhandleARB vertexShader   = loadShader(GL_VERTEX_SHADER_ARB);
    GLhandleARB fragmentShader = loadShader(GL_FRAGMENT_SHADER_ARB);
    GLhandleARB geometryShader = 0;

    if (sData->mMaxVerticesOut > 0)
    {
        geometryShader = loadShader(GL_GEOMETRY_SHADER_EXT);
    }

    bool geometryValid = (sData->mMaxVerticesOut > 0 && geometryShader) || !geometryShader;

    // shader loading and / or compiling failed

    if (!vertexShader || !fragmentShader || !geometryValid)
    {
        if (vertexShader)   glDeleteObjectARB(vertexShader);
        if (fragmentShader) glDeleteObjectARB(fragmentShader);
        if (geometryShader) glDeleteObjectARB(geometryShader);
        clear();
        return;
    }

    glAttachObjectARB(mShaderProgram, vertexShader);
    glAttachObjectARB(mShaderProgram, fragmentShader);

    if (sData->mMaxVerticesOut > 0)
    {
        glAttachObjectARB(mShaderProgram, geometryShader);
    }

    setVertexAttributes(sData->mVertexAttributes);
    glLinkProgramARB(mShaderProgram);

    int   size;
    char* log = getInfoLog(mShaderProgram, size);

    if (log != 0) theMsg->printf("%s\n", log);

    // free shader objects

    if (vertexShader)   glDeleteObjectARB(vertexShader);
    if (fragmentShader) glDeleteObjectARB(fragmentShader);
    if (geometryShader) glDeleteObjectARB(geometryShader);


//    theMsg->printf("Loaded shader %s\n", sData->mName.getString());
}







HxGLShader::HxGLShader(void)
{
    mData.mVertexAttributes.resize(0);
}




HxGLShader::~HxGLShader(void)
{
}




void HxGLShader::addVertexAttribute(int position, const char* name)
{
    VertexAttribute vA;

    vA.mName        = name;
    vA.mPosition    = position;

    mData.mVertexAttributes.append(vA);

    mShader.touch();
}




void HxGLShader::clearVertexAttributes()
{
    mData.mVertexAttributes.resize(0);

    mShader.touch();
}




void HxGLShader::disable()
{
    glUseProgramObjectARB(0);
}




void HxGLShader::enable(unsigned int contextID)
{
    mShader.update(contextID, &mData);

    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    glUseProgramObjectARB(shader->mShaderProgram);
}




GLuint HxGLShader::enableAndReturnProgram(unsigned int contextID)
{

    GLuint ret = 0;

    mShader.update(contextID, &mData);

    GLShader* shader = mShader.getInstance(contextID);

    if (isExecutable(contextID)){
        glUseProgramObjectARB(shader->mShaderProgram);
        ret = shader->mShaderProgram;
    }

    return ret;

}




bool HxGLShader::isExecutable(unsigned int contextID)
{
    mShader.update(contextID, &mData);

    GLShader* shader = mShader.getInstance(contextID);

    return shader->mShaderProgram != 0;
}




void HxGLShader::load(const char *filename)
{
    mData.mName           = filename;
    mData.mMaxVerticesOut = 0;

    mShader.touch();

}




void HxGLShader::load(const char *filename, GLint maxOut, GLint inType, GLint outType)
{
    mData.mName             = filename;
    mData.mInType           = inType;
    mData.mMaxVerticesOut   = maxOut;
    mData.mOutType          = outType;

    mShader.touch();
}

void HxGLShader::setParameter1f(unsigned int contextID, const char* name, float value)
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniform1fARB(location, value);
}

void HxGLShader::setParameter2f(unsigned int contextID, const char* name, const float value[2])
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniform2fARB(location, value[0], value[1]);
}

void HxGLShader::setParameter3f(unsigned int contextID, const char* name, const float value[3])
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID))
        return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniform3fARB(location, value[0], value[1], value[2]);
}

void HxGLShader::setParameter4f(unsigned int contextID, const char* name, const float value[4])
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniform4fARB(location, value[0], value[1], value[2], value[3]);
}

void HxGLShader::setParameter1i(unsigned int contextID, const char* name, int value)
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniform1iARB(location, value);
}

void HxGLShader::setParameterMatrix2f(unsigned int contextID, const char* name, const float* value)
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniformMatrix2fvARB(location, 1, false, value);
}

void HxGLShader::setParameterMatrix3f(unsigned int contextID, const char* name, const float* value)
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniformMatrix3fvARB(location, 1, false, value);
}

void HxGLShader::setParameterMatrix4f(unsigned int contextID, const char* name, const float* value)
{
    GLShader* shader = mShader.getInstance(contextID);

    if (!isExecutable(contextID)) return;

    GLint location = glGetUniformLocationARB(shader->mShaderProgram, name);

    if (glGetError() == GL_INVALID_OPERATION)
    {
        theMsg->printf("SHADER: error, %s is not a parameter of %s\n", name, mData.mName.getString());
        return;
    }

    glUniformMatrix4fvARB(location, 1, false, value);
}


/************************************************************************/




int HxGLShader::CodeIncludeTree::getFileAndLine(int line, McString& file)
{
    int numIncludes = mIncludes.size();
    int lineRes     = line - mStart;

    for (int i = 0; i < numIncludes; ++i)
    {
        if (line >= mIncludes[i]->mStart &&
            line <= mIncludes[i]->mEnd)
        {
            return mIncludes[i]->getFileAndLine(line, file);
        }

        if (line > mIncludes[i]->mEnd)
        {
            lineRes -= (mIncludes[i]->mEnd - mIncludes[i]->mStart);
        }
    }

    file = (McString) mFileName;

    return lineRes;
}




void HxGLShader::CodeIncludeTree::print()
{
    McString dummy("");
    print(this, dummy);
}




void HxGLShader::CodeIncludeTree::print(HxGLShader::CodeIncludeTree* tree, McString& dummy)
{
    printf("%s%d - %d: %s\n", dummy.getString(), tree->mStart, tree->mEnd, tree->mFileName.getString());

    dummy += "\t";

    for (int i = 0; i < tree->mIncludes.size(); ++i)
    {
        print(tree->mIncludes[i], dummy);
    }

    dummy.removeLast();
}




/************************************************************************/




McString& HxGLShader::CodeLine::operator [](int index)
{
    return mWords[index];
}




void HxGLShader::CodeLine::clear()
{
    mWords.clear();
}




bool HxGLShader::CodeLine::parseIncludeFile(McString& includeFile)
{
    if (mWords.size() < 2) return false;

    char* nameText = mWords[1].getString();
    char* nameOut  = new char[mWords[1].size() - 2];

    if ((unsigned char) nameText[0] != 34 ||
        (unsigned char) nameText[mWords[1].size() - 2] != 34)
    {
        delete[] nameOut;
        return false;
    }

    for (int i = 0; i < mWords[1].size() - 3; ++i)
    {
        nameOut[i] = nameText[i + 1];
    }
    nameOut[mWords[1].size() - 3] = 0;

    includeFile = McString(nameOut);

    return true;
}




void HxGLShader::CodeLine::print()
{
    int numWords = mWords.size();

    McString word;

    for (int i = 0; i < numWords; ++i)
    {
        word += mWords[i];
        word += McString(" ");
    }
    theMsg->printf("%s", word.getString());
}




void HxGLShader::CodeLine::push(const McString& word)
{
    mWords.push_back(word);
}




bool HxGLShader::CodeLine::startWith(const McString& string)
{
    if (mWords.size() < 1) return false;

    return mWords[0] == string;
}




/************************************************************************/




HxGLShader::CodeLine& HxGLShader::Code::operator [](int index)
{
    return mLines[index];
}




void HxGLShader::Code::generate(McDArray< HxGLShader::CodeLine >& lines, const char* file,
                                HxGLShader::CodeIncludeTree* includeTree)
{
    McString path = McFilename::dirname(file);

    int   length;
    char* code = loadCode(file, length);

    scanCode(lines, code, length);


    int numLines = lines.size();

    for (int i = 0; i < numLines; ++i)
    {
        // tries to include a file
        if (lines[i].startWith(McString("#include")))
        {
            McString includeFile;
            if (lines[i].parseIncludeFile(includeFile))
            {
                McFilename include = path;
                include += includeFile;

                CodeIncludeTree* child = new CodeIncludeTree();
                includeTree->mIncludes.push_back(child);
                child->mStart       = includeTree->mStart + i;
                child->mFileName    = includeFile;

                McDArray< CodeLine > subLines;
                generate(subLines, include.getString(), child);

                child->mEnd = child->mStart + subLines.size() - 1;

                lines.remove(i);
                lines.insertArray(i, subLines);
                numLines = lines.size();

            }
            else
            {
                printf("bad include: ");
                lines[i].print();
            }
        }
    }
}




char* HxGLShader::Code::getCode(int& length)
{
    McString result;

    int numLines = mLines.size();

    for (int i = 0; i < numLines; ++i)
    {
        int numWords = mLines[i].mWords.size();

        for (int j = 0; j < numWords; ++j)
        {
            result += mLines[i][j];
            result += McString(" ");
        }
        result += McString("\n");
    }

    length = result.length();

    GLcharARB* chars = new GLcharARB[length];

    for (int i = 0; i < length; ++i) chars[i] = result[i];

    return chars;
}




int HxGLShader::Code::getErrorLine(McString& lineWord)
{
    int   length    = lineWord.length();
    char* lineChar  = new char[1024];
    bool  start     = false;
    int   pos       = 0;

    for (int i = 0; i < length; ++i)
    {
        if (lineWord[i] == ')') break;

        if (start)
        {
            lineChar[pos] = lineWord[i];
            pos++;
        }

        if (lineWord[i] == '(') start = true;
    }

    int line = -1;

    if (pos > 0)
    {
        lineChar[pos] = 0;
        line = atoi(lineChar);
    }

    delete[] lineChar;

    return line;
}




void HxGLShader::Code::load(const char* file)
{
    McFilename fileName = file;

    mIncludeTree.mStart = 0;
    mIncludeTree.mFileName = McString(fileName.getBasename());

    generate(mLines, file, &mIncludeTree);

    mIncludeTree.mEnd = mLines.size() - 1;
}




char* HxGLShader::Code::loadCode(const char* file, int& length)
{
    if (!file) return 0;

    McString fileName = McString(HxResource::getLocalDir()) + McString(file);

    ifstream is;
    is.open (fileName.getString(), ios::binary );

    if(!is.good())
    {
        fileName = McString(HxResource::getRootDir()) + McString(file);

        is.open(fileName.getString(), ios::binary);
        if (!is.good())
        {
            theMsg->printf("SHADER: can not open file %s \n", file);
            return 0;
        }
    }

    is.seekg (0, ios::end);
    length = is.tellg();
    length += 2;
    is.seekg (0, ios::beg);

    char* characters = new GLcharARB[length];

    is.read (characters, length - 2);
    is.close();

    characters[length - 2] = (char) 10;
    characters[length - 1] = (char)  0;

    return characters;
}




void HxGLShader::Code::print()
{
    int numLines = mLines.size();

    for (int i = 0; i < numLines; ++i)
    {
        mLines[i].print();
    }
}




void HxGLShader::Code::printErrorCode(char* code, int length)
{
    McDArray< CodeLine > lines;

    scanCode(lines, code, length);

    if (lines.size() == 0) return;

    theMsg->printf("\nShader log of main file: %s\n\n", mIncludeTree.mFileName.getString());

    for (int i = 0; i < lines.size(); ++i)
    {
        if (lines[i].mWords.size() == 0) continue;

        int lineNum  = getErrorLine(lines[i][0]);

        McString file;
        int lineReal = mIncludeTree.getFileAndLine(lineNum, file);

        lines[i].mWords.remove(0);
        lines[i].mWords.insert(0, McString(0, "file %s, line %d", file.getString(), lineReal));

        lines[i].print();
    }

    theMsg->printf("\n");
}




void HxGLShader::Code::scanCode(McDArray< CodeLine >& lines, char* code, int length)
{
    lines.clear();

    if (!code) return;

    char*       word    = new char[1024];
    int         wordPos = 0;
    CodeLine    line;

    for (int i = 0; i < length; ++i)
    {
        if ((unsigned char) code[i] <= 32  ||
            (unsigned char) code[i] >= 127)
        {
            // new word
            if (wordPos > 0)
            {
                word[wordPos] = 0;
                line.push(McString(word, 0, wordPos));
                wordPos = 0;
            }
            // new line
            if ((unsigned char) code[i] == 10)
            {
                lines.push(line);
                line.clear();
            }

            continue;
        }

        word[wordPos] = code[i];
        wordPos++;
    }

    delete[] word;
}

