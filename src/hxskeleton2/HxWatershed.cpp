#include "HxWatershed.h"

#include <hxfield/HxUniformScalarField3.h>
#include <hxfield/HxUniformCoord3.h>
#include <hxcore/internal/HxWorkArea.h>
#include <hxcore/HxMessage.h>
#include <mclib/McException.h>
#include <hxskeleton2/Progress.h>
#include <mclib/McPrimTypeHelper.h>
#include <hxskeleton2/GridCell.h>
#include <mclib/McDArray.h>

HX_INIT_CLASS(HxWatershed,HxCompModule)

HxWatershed::HxWatershed()
    : HxCompModule (HxUniformScalarField3::getClassTypeId())
    , portLabels (this, "labels", tr("Labels"), HxUniformScalarField3::getClassTypeId ())
    , portObject (this, "object", tr("Object"), HxUniformScalarField3::getClassTypeId ())
    , portAction (this, "action", tr("Action")) {

}

HxWatershed::~HxWatershed() {
}

namespace {
    bool IsNonNegative (const int* dims, const short* scalarfield) {
        const mclong nnn = mclong(dims[0]) * dims[1] * dims[2];
        for (mclong i = 0; i < nnn; i++) {
            if (scalarfield[i] < 0) {
                return false;
            }
        }
        return true;
    }

    bool IsNonZeroEverywhere (const McDim3l& dims, const unsigned char* labels) {
        const mclong nnn = mclong(dims[0]) * dims[1] * dims[2];
        for (mclong i = 0; i < nnn; i++) {
            if (0 == labels[i]) {
                return false;
            }
        }
        return true;
    }

    short GetMax (const McDim3l& dims, const short* scalarfield) {
        const mclong nnn = mclong(dims[0]) * dims[1] * dims[2];

        short max = SHRT_MIN;
        for (mclong i = 0; i < nnn; i++) {
            if (scalarfield[i] > max) {
                max = scalarfield[i];
            }
        }
        return max;
    }

    short GetMin (const McDim3l& dims, const short* scalarfield) {
        const mclong nnn = mclong(dims[0]) * dims[1] * dims[2];

        short min = SHRT_MAX;
        for (mclong i = 0; i < nnn; i++) {
            if (scalarfield[i] < min) {
                min = scalarfield[i];
            }
        }
        return min;
    }

    template<class T1, class T2>
    void ComputeWatershed (const McDim3l& dims, T1* out, const T1* labels, const short* scalarfield, const T2* objdat, McProgressInterface *progress) {

        const mclong nnn = mclong(dims[0]) * dims[1] * dims[2];
        memset (out, 0, sizeof(T1)*nnn);

        const int lowQueue = GetMin (dims, scalarfield);
        const int nQueues = GetMax (dims, scalarfield) - lowQueue + 1;
        McDArray< McDArray<mclong> > queues (nQueues);

        progress->startWorking("propagating labels ...");
        int done = 0;

        // push labels on queue and copy to output
        {
            for (mclong i = 0; i < nnn; i++) {
                if (labels[i] && ( (!objdat) || objdat[i] ) ) {
                    queues[scalarfield[i] - lowQueue].append (i);
                    out[i] = labels[i];
                    done++;
                }
            }
        }

        const McVec3i offsets (1, dims[0], dims[0] * dims[1]);

        bool changed = true;
        while (changed) {
            changed = false;

            if ( progress->wasInterrupted() ) {
                progress->stopWorking();
                return;
            }

            progress->setProgressValue(float (done) / float(nnn));

            for (int q = 0; q < nQueues && !changed; q++) {
                const int nels  = queues[q].size();

                // propagate label
                for (int el = 0; el < nels; el++) {
                    const mclong idx = queues[q][el];
                    mcassert (idx >= 0 && idx < nnn);

                    const McVec3i ipos = IdxToPos (dims, idx);

                    // visit all 6-neighbors
                    for (int d = 0; d < 3; d++) {
                        for (int delta = -1; delta < 2; delta += 2) {
                            if ((ipos[d] + delta >= 0) && (ipos[d] + delta < dims[d])) { // ... only inside volume
                                const mclong neighboridx = idx + offsets[d] * delta;
                                if (0 == out[neighboridx] && ( (!objdat) || objdat[neighboridx] ) ) {
                                    out[neighboridx] = out[idx];
                                    done++;
                                    queues[scalarfield[neighboridx] - lowQueue].append (neighboridx);
                                    changed = true;
                                }
                            }
                        }
                    }
                }

                // delete propagated labels
                queues[q].remove (0, nels);
            }
        }
        //mcensure (IsNonZeroEverywhere (dims, out));
        progress->stopWorking();
    }

    template<class T>
    void callComputeWatershed (const McDim3l& dims, T* out, const T* labels, const short* scalarfield, HxLattice3* objLat, McProgressInterface *progress) {

        McPrimType type;

        if ( !objLat )
        {
            unsigned char* objdat = 0;
            ComputeWatershed (  dims, out, labels, scalarfield, objdat, progress);
        }
        else
        {
            switch(objLat->primType().getType()) {
                case McPrimType::MC_UINT8: {
                    const mcuint8* objdat =  static_cast<mcuint8*> (objLat->dataPtr());
                    ComputeWatershed (  dims, out, labels, scalarfield, objdat, progress);
                }
                break;
                case McPrimType::MC_UINT32: {
                    const mcuint32* objdat =  static_cast<mcuint32*> (objLat->dataPtr());
                    ComputeWatershed (  dims, out, labels, scalarfield, objdat, progress);
                }
                break;
                case McPrimType::MC_INT32: {
                    const mcint32* objdat = static_cast<mcint32*> (objLat->dataPtr());
                    ComputeWatershed (  dims, out, labels, scalarfield, objdat, progress);
                }
                break;
                case McPrimType::MC_UINT16: {
                    const mcuint16* objdat = static_cast<mcuint16*> (objLat->dataPtr());
                    ComputeWatershed (  dims, out, labels, scalarfield, objdat, progress);
                }
                break;
                case McPrimType::MC_INT16: {
                    const mcint16* objdat = static_cast<mcint16*> (objLat->dataPtr());
                    ComputeWatershed (  dims, out, labels, scalarfield, objdat, progress);
                }
                break;
                default:
                    mcthrow("Logic error: switch/case not handled");
            }
        }
    }
}


void HxWatershed::compute() {
    if (!portAction.wasHit()) {
        return;
    }

    HxUniformScalarField3* src = hxconnection_cast<HxUniformScalarField3>(portData);
    if (!src) {
        return;
    }

    HxLattice3* lat = mcinterface_cast<HxLattice3> (src);
    if (src->primType () != McPrimType::MC_INT16) {
        mcthrow("Scalar field input must be of type short.");
    }

    const McDim3l& dims = lat->getDims();

    HxData* labelsrc = hxconnection_cast<HxData> (portLabels);
    HxLattice3* labellat = hxconnection_cast<HxLattice3> (portLabels);
    if (!labellat || !labelsrc) {
        mcthrow("need labels");
    }

    if ( (labellat->primType () != McPrimType::MC_UINT8) &&
         (labellat->primType () != McPrimType::MC_UINT32) &&
         (labellat->primType () != McPrimType::MC_INT32) &&
         (labellat->primType () != McPrimType::MC_UINT16) &&
         (labellat->primType () != McPrimType::MC_INT16) ) {
        mcthrow("Label field input must be of type (unsigned) int, short or byte.");
    }


    HxUniformScalarField3* obj= hxconnection_cast<HxUniformScalarField3>(portObject);
    HxLattice3* objlat = 0;

    {
        const McDim3l& labeldims = labellat->getDims();
        if (! (
                    dims[0] == labeldims[0]
                 && dims[1] == labeldims[1]
                 && dims[2] == labeldims[2]
            )) {
            mcthrow("label input's and main input's dimensions do not match.");
        }
        if(obj){
            objlat = mcinterface_cast<HxLattice3> (obj);
            if ( (objlat->primType () != McPrimType::MC_UINT8) &&
                 (objlat->primType () != McPrimType::MC_UINT32) &&
                 (objlat->primType () != McPrimType::MC_INT32) &&
                 (objlat->primType () != McPrimType::MC_UINT16) &&
                 (objlat->primType () != McPrimType::MC_INT16) ) {
                mcthrow("Object input field input must be of type (unsigned) int, short or byte.");
            }
            const McDim3l& objdims = objlat->getDims();
            if (! (
                   dims[0] == objdims[0]
                   && dims[1] == objdims[1]
                   && dims[2] == objdims[2]
                )) {
                mcthrow("object input's and main input's dimensions do not match.");
            }
        }
    }

    const short* scalarfield = static_cast<short*> (lat->dataPtr());
    unsigned char* labeldat = static_cast<unsigned char*> (labellat->dataPtr());
    unsigned char* objdat = 0;
    if(obj){
        objdat = static_cast<unsigned char*> (objlat->dataPtr());
    }

    HxData* result = mcinterface_cast<HxData> (getResult());
    if ( !result ) result = labelsrc->duplicate();
    HxLattice3* nlat = mcinterface_cast<HxLattice3> (result);
    mcassert (nlat);

    switch (labellat->primType().getType()) {
        case McPrimType::MC_UINT8: {
            unsigned char* resultdat = static_cast<unsigned char*> (nlat->dataPtr());
            callComputeWatershed (  dims, resultdat,
                reinterpret_cast<unsigned char*> (labeldat), scalarfield, objlat, theWorkArea);
        }
        break;
        case McPrimType::MC_UINT32: {
            unsigned int* resultdat = static_cast<unsigned int*> (nlat->dataPtr());
            callComputeWatershed (  dims, resultdat,
                reinterpret_cast<unsigned int*> (labeldat), scalarfield, objlat, theWorkArea);
        }
        break;
        case McPrimType::MC_INT32: {
            int* resultdat = static_cast<int*> (nlat->dataPtr());
            callComputeWatershed (  dims, resultdat,
                reinterpret_cast<int*> (labeldat), scalarfield, objlat, theWorkArea);
        }
        break;
        case McPrimType::MC_UINT16: {
            unsigned short* resultdat = static_cast<unsigned short*> (nlat->dataPtr());
            callComputeWatershed (  dims, resultdat,
                reinterpret_cast<unsigned short*> (labeldat), scalarfield, objlat, theWorkArea);
        }
        break;
        case McPrimType::MC_INT16: {
            short* resultdat = static_cast<short*> (nlat->dataPtr());
            callComputeWatershed (  dims, resultdat,
                reinterpret_cast<short*> (labeldat), scalarfield, objlat, theWorkArea);
        }
        break;
        default:
            mcthrow("Logic error: switch/case not handled");
    }

    nlat->touchMinMax();
    result->composeLabel (src->getLabel(), "watershed");
    setResult (result);
}

