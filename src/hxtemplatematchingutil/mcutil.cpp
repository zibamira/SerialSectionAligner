#include <math.h>

#include <mclib/McVec3i.h>
#include <mclib/McDArray.h>
#include <mclib/McBitfield.h>
#include <mclib/internal/McAssert.h>
#include <mclib/McMath.h>

#include "mcutil.h"
#include <hxcore/HxMessage.h>

static float mean(const McDArray<float>& values)
{
    float sum = 0.0f;
    for (int i = 0; i < values.size(); ++i) {
        sum += values[i];
    }
    return sum / float(values.size());
}

static float stddev(const McDArray<float>& values, const float meanValue)
{
    float sum2 = 0.0f;
    for (int i = 0; i < values.size(); ++i) {
        const float diff = (values[i] - meanValue);
        sum2 += (diff*diff);
    }
    return (sum2 != 0.0f) ? sqrt(sum2 / values.size()) : 1.0f;
}

void mcutil::normalizeValues(McDArray<float>* values)
{
    const float m = mean(*values);
    const float sd = stddev(*values, m);
    for (int i = 0; i < values->size(); ++i) {
        (*values)[i] = ((*values)[i] - m) / sd;
    }
}

McDArray<float> mcutil::GetSelection(
        const McDArray<float>& InArray,
        const McBitfield& selection)
{
    if(selection.nSetBits() == 0)
        return InArray;

    mcrequire(selection.nBits() == mculong(InArray.size()));

    McDArray<float> OutArray(selection.nSetBits());
    int index(0);
    for (int k = 0; k < InArray.size(); ++k) {
        if(selection[k]) {
            OutArray[index] = InArray[k];
            index++;
        }
    }

    return OutArray;
}

void mcutil::InsertSelection(
        const McDArray<float>& modiArray,
        const McBitfield& selection,
        McDArray<float>& InArray)
{
    if(selection.nSetBits() == 0)
        return;

    mcrequire(selection.nBits() == mculong(InArray.size()));

    int index(0);
    for (int k = 0; k < InArray.size(); ++k) {
        if(selection[k]) {
            InArray[k] = modiArray[index];
            index++;
        }
    }
}
