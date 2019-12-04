#ifndef HXSPINDLE_ALIGNERTOOLRENDERERSTACK_H
#define HXSPINDLE_ALIGNERTOOLRENDERERSTACK_H

#include <hxspindle/api.h>
#include <hxspindle/AlignerToolRenderer.h>
#include <hxspindle/HxSerialSectionStack.h>

#include <mcgl/internal/mcgl.h>





/**
    Render a slice of the field data.
*/
class HXSPINDLE_API AlignerToolRendererStack : public AlignerToolRenderer
{
    public:


        AlignerToolRendererStack(void);


    public:


        int getSectionChange() const;




        bool interaction(SbVec2s mousePos, bool mouseLeft, bool controlPress, HxSerialSectionStack* sectionStack);




        void renderStack(unsigned int contextID, HxSerialSectionStack* sectionStack, McDArray< McDArray < float > >& endPointDensities);




        void setSectionChange(int change);


    private:


        void renderQuad(float x, float y, float w, float h, float z = 0.0);


    private:


        int mSectionChange;
        int mSectionHighlight;
        int mSectionSelected;
};




#endif

