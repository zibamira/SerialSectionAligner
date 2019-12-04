#include <hxspindle/AlignerToolRenderer.h>

#include <hxcore/HxMessage.h>
#include <hxcore/HxResource.h>
#include <hxcore/HxController.h>
#include <hxcore/HxViewer.h>

#include <hxfield/HxLattice3.h>
#include <hxfield/HxLocation3.h>
#include <hxfield/HxUniformCoord3.h>

#include <mclib/McVec3.h>




AlignerToolRenderer::AlignerToolRenderer(void)
    : mRenderPosition(-1.0, -1.0)
    , mRenderSize(2.0, 2.0)
{
}




GLuint AlignerToolRenderer::getNextPowerOfTwo(GLuint number)
{
    GLuint resNumber = 2;

    while (resNumber < number)
    {
        resNumber *= 2;
    }

    return resNumber;
}




void AlignerToolRenderer::renderTexture(GLuint textureID, float width /* = 1.0f */, float height /* = 1.0f */) const
{
    renderTexture(textureID, false, width, height);
}




void AlignerToolRenderer::renderTexture(GLuint textureID, bool transparent, float width /* = 1.0f */, float height /* = 1.0f */) const
{

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glEnable(GL_BLEND);

    if (transparent)
        glBlendFunc(GL_ONE, GL_ONE);
    else
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_LIGHTING);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(mRenderPosition[0], mRenderPosition[1], 0.0);
    glTexCoord2f(width, 0.0);
    glVertex3d(mRenderPosition[0] + mRenderSize[0], mRenderPosition[1], 0.0);
    glTexCoord2f(width, height);
    glVertex3d(mRenderPosition[0] + mRenderSize[0], mRenderPosition[1] + mRenderSize[1], 0.0);
    glTexCoord2f(0.0, height);
    glVertex3d(mRenderPosition[0], mRenderPosition[1] + mRenderSize[1], 0.0);
    glEnd();


    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib();
}




void AlignerToolRenderer::renderTextureDepth(GLuint textureID, GLuint depthID, float width /* = 1.0f */, float height /* = 1.0f */)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_LIGHTING);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depthID);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(mRenderPosition[0], mRenderPosition[1], 0.0);
    glTexCoord2f(width, 0.0);
    glVertex3d(mRenderPosition[0] + mRenderSize[0], mRenderPosition[1], 0.0);
    glTexCoord2f(width, height);
    glVertex3d(mRenderPosition[0] + mRenderSize[0], mRenderPosition[1] + mRenderSize[1], 0.0);
    glTexCoord2f(0.0, height);
    glVertex3d(mRenderPosition[0], mRenderPosition[1] + mRenderSize[1], 0.0);
    glEnd();

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib();
}




void AlignerToolRenderer::renderTexture(HxGLTexture2D& texture, unsigned int contextID)
{
    float texWidth  = float(mFrameBuffer.getWidth()) / float(texture.getWidth());
    float texHeight = float(mFrameBuffer.getHeight()) / float(texture.getHeight());

    renderTexture(texture.getTexture(contextID), texWidth, texHeight);
}




void AlignerToolRenderer::setPosition(float x, float y)
{
    mRenderPosition.setValue(x, y);
}




void AlignerToolRenderer::setSize(float width, float height)
{
    mRenderSize.setValue(width, height);
}




bool AlignerToolRenderer::updateFrameBuffer()
{
    HxViewer* viewer = theController->getCurrentViewer();

    if (!viewer) return false;

    // store model view and projection matrix for picking

    glGetDoublev(GL_MODELVIEW_MATRIX,  mMatrixModelView);
    glGetDoublev(GL_PROJECTION_MATRIX, mMatrixProjection);

    // take viewer size and compute tool size

    SbVec2s windowSize = viewer->getSize();

    windowSize[0] = (short)((float) windowSize[0] * (mRenderSize[0] / 2.0));
    windowSize[1] = (short)((float) windowSize[1] * (mRenderSize[1] / 2.0));

    // update frame buffer

    mFrameBuffer.resize(windowSize[0], windowSize[1]);
    mFrameBuffer.setImageFetchMode(GL_LINEAR);
    mFrameBuffer.setImageFormat(GL_RGBA32F, GL_RGBA);
    mFrameBuffer.setImageType(GL_FLOAT);

    return true;
}




bool AlignerToolRenderer::windowToToolCoord(const SbVec2s viewerPosition, SbVec2f& toolPosition) const
{
    // convert viewer pixel position in view coordinates [-1...1, -1...1]

    HxViewer* viewer = theController->getCurrentViewer();

    if (!viewer) return false;

    SbVec2s windowSize = viewer->getSize();
    SbVec2f windowPos(float(viewerPosition[0]) / float(windowSize[0]),
                      float(viewerPosition[1]) / float(windowSize[1]));

    windowPos = 2.0 * windowPos - SbVec2f(1.0, 1.0);

    // check slice renderer range

    if (windowPos[0] < mRenderPosition[0] || windowPos[0] >= mRenderPosition[0] + mRenderSize[0] ||
        windowPos[1] < mRenderPosition[1] || windowPos[1] >= mRenderPosition[1] + mRenderSize[1])
    {
        return false;
    }

    // compute coordinate in tool view space

    toolPosition[0] = (windowPos[0] - mRenderPosition[0]) / mRenderSize[0];
    toolPosition[1] = (windowPos[1] - mRenderPosition[1]) / mRenderSize[1];

    toolPosition = 2.0 * toolPosition - SbVec2f(1.0, 1.0);

    return true;
}
