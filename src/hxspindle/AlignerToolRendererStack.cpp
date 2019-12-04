#include <hxspindle/AlignerToolRendererStack.h>

#include <hxcore/HxMessage.h>
#include <hxcore/HxResource.h>
#include <hxcore/HxController.h>
#include <hxcore/HxViewer.h>

#include <hxfield/HxLattice3.h>
#include <hxfield/HxLocation3.h>
#include <hxfield/HxUniformCoord3.h>

#include <Inventor/nodes/SoText2.h>

#include <mclib/McVec3.h>







AlignerToolRendererStack::AlignerToolRendererStack(void)
    : mSectionChange(0)
    , mSectionHighlight(-1)
    , mSectionSelected(-1)
{
}




int AlignerToolRendererStack::getSectionChange() const
{
    return mSectionChange;
}




bool AlignerToolRendererStack::interaction(SbVec2s mousePos, bool mouseLeft, bool controlPress, HxSerialSectionStack* sectionStack)
{
    if (!sectionStack) return false;

    SbVec2f toolPosition;

    if (!windowToToolCoord(mousePos, toolPosition))
    {
        mSectionSelected  = -1;
        mSectionHighlight = -1;

        return false;
    }

    // start interaction computation

    int n = sectionStack->getNumSections();

    float g  =  1.25;
    float w  =  1.0;
    float h  =  1.8 / (float(n) * g - (g - 1.0));
    float sx = -0.5;
    float sy = -0.9;

    // mouse move

    int old = mSectionHighlight;

    mSectionHighlight = -1;

    for (int i = 0; i < n - 1; ++i)
    {
        if (toolPosition[0] > sx                    &&
            toolPosition[0] < sx + w                &&
            toolPosition[1] > sy + float(i) * g * h &&
            toolPosition[1] < sy + float(i) * g * h + h)
        {
            mSectionHighlight = i;
            break;
        }
    }


    // stop hit

    if (mSectionSelected >= 0 && !mouseLeft)
    {
        if (mSectionSelected == mSectionHighlight)
        {
            mSectionChange   = mSectionSelected;
            mSectionSelected = -1;

            return true;
        }
    }

    // start hit

    if (mSectionSelected < 0 && mouseLeft && mSectionHighlight >= 0)
    {
        mSectionSelected = mSectionHighlight;
    }

    if (old != mSectionHighlight) return true;

    return false;
}




void AlignerToolRendererStack::renderQuad(float x, float y, float w, float h, float z)
{
    glVertex3d(x,     y,     z);
    glVertex3d(x + w, y,     z);
    glVertex3d(x + w, y + h, z);
    glVertex3d(x,     y + h, z);
}




void AlignerToolRendererStack::renderStack(unsigned int contextID, HxSerialSectionStack* sectionStack, McDArray< McDArray < float > >& endPointDensities)
{
    if (!sectionStack) return;

    if (!updateFrameBuffer()) return;

    mFrameBuffer.bind(contextID);

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_BLEND);

    glDisable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_LIGHTING);

    // start rendering

    int n = sectionStack->getNumSections();

    float g  =  1.25;
    float w  =  1.0;
    float h  =  1.8 / (float(n) * g - (g - 1.0));
    float sx = -0.5;
    float sy = -0.9;

    // render content

    for (int i = 0; i < n; ++i)
    {
        if (sectionStack->getSection(i).mImageFileName != "")
        {
            if (mSectionHighlight == i) glColor3f(0.95f, 0.95f, 0.95f);
            else                        glColor3f(0.8f,  0.8f,  0.8f);

            glBegin(GL_QUADS);
                renderQuad(sx, sy + float(i) * g * h, w, h);
            glEnd();
        }
    }

    // highlight selected change

    if (mSectionChange < n)
    {
        McVec3f c = 1.5 * McVec3f(1.0, 165.0 / 255.0, 0.2);

        glColor4f(c.x, c.y, c.z, 0.9f);

        glBegin(GL_QUADS);
            renderQuad(sx, sy + float(mSectionChange) * g * h, w, h, 0.1);
        glEnd();
    }

    if (mSectionChange < n - 1)
    {
        McVec3f c = 1.5 * McVec3f(0.3, 165.0 / 255.0, 1.0);

        glColor4f(c.x, c.y, c.z, 0.8f);

        glBegin(GL_QUADS);
            renderQuad(sx, sy + float(mSectionChange + 1) * g * h, w, h, 0.1);
        glEnd();
    }

    // render frame

    for (int i = 0; i < n; ++i)
    {
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINE_LOOP);
            renderQuad(sx, sy + float(i) * g * h, w, h);
        glEnd();
    }

    // render end point distribution

    float maxDensity = 0.0;

    for (int i = 0; i < n - 1; ++i)
    {
        if (endPointDensities[i].size() < 4) continue;

        for (int j = 0; j < 4; ++j)
        {
            maxDensity = std::max(maxDensity, endPointDensities[i][endPointDensities[i].size() - j - 1]);
        }
    }

    for (int i = 0; i < n - 1; ++i)
    {
        if (endPointDensities[i].size() < 4) continue;

        for (int j = endPointDensities[i].size() - 1; j >= endPointDensities[i].size() - 4; --j)
        {
            float a = 0.1 * float((j) % 4);
            float ext = 0.0;

            if (j % 4 >= 2) ext = (g * h) - h;

            glBegin(GL_QUADS);
                glColor4f(1.0, 0.5 + a, a, 1.0);
                renderQuad(0.55, sy + (float(i + 1) * g * h + 0.25 * h) - (float(j % 4) * 0.25 * h + ext), 0.4 * (endPointDensities[i][j] / maxDensity), 0.25 * h);
            glEnd();
        }
    }

    // stop rendering

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glEnable(GL_DEPTH_TEST);

    glPopAttrib();

    mFrameBuffer.unbind(contextID);

    renderTexture(mFrameBuffer.getImageTexture(contextID));
}




void AlignerToolRendererStack::setSectionChange(int change)
{
    mSectionChange = change;
}


