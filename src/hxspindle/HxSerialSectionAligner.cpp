#include <stdio.h>

#include <hxspindle/HxSerialSectionAligner.h>
#include <hxspindle/SpindleTools.h>

#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/elements/SoCacheElement.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/events/SoLocation2Event.h>
#include <Inventor/events/SoKeyboardEvent.h>

#include <hxalignmicrotubules/mtalign/cpd.h>
#include <hxalignmicrotubules/mtalign/matchingPGM.h>
#include <hxalignmicrotubules/MovingLeastSquares.h>

#include <hxcoremodules/internal/HxAnnotation.h>
#include <hxcore/internal/HxWorkArea.h>
#include <hxcore/HxObjectPool.h>
#include <hxcore/HxResource.h>
#include <hxcore/HxMessage.h>
#include <hxcore/HxController.h>
#include <hxcore/HxViewer.h>

#include <hxdistmap/internal/HxComputeDistanceMap.h>
#include <hxskeleton2/HxWatershed.h>

#include <mclib/internal/McWatch.h>
#include <amiramesh/AmiraMesh.h>


#include <algorithm>

#ifdef _OPENMP
    #include <omp.h>
#endif




HX_INIT_CLASS(HxSerialSectionAligner, HxModule)




namespace
{
    struct ProjectedEnd
    {
        McVec3f mPosition;
        McVec3f mDirection;
        float   mCurvature;
    };
};





/**
    Computes the points for CPD algorithm for one section given by its filament ends.
    Only the ends within the box range of the given box are considered. The range lies
    in [-1...1] and describes the percentage z part of the box. If the range is
    negative, the bottom part is used otherwise the top part. The method returns the
    id of the selected filament as well as the selected end of the edge, which is 0
    for the start and 1 for the end.
*/
static McDArray< McVec2i > computeCPDPoints(const HxSpatialGraph&             filaments,
                                            const McBox3f&                    box,
                                            const float                       boxRange,
                                                  mtalign::DirectionalPoints& points)
{
    McDArray< McVec2i > selectedFilaments;

    int numEdges = filaments.getNumEdges();

    points.positions.remax(numEdges * 2, 0);
    points.directions.remax(numEdges * 2, 0);

    float direction = boxRange < 0 ? -1.0f : 1.0f;
    float range     = boxRange < 0 ? box[4] : box[5];
            range    -= boxRange * (box[5] - box[4]);

    printf("d = %f min = %f, max = %f, r = %f\n", direction, box[4], box[5], range);

    float angleFilter = 0.01; // hard coded 0.01
    float maxAngle = (M_PI * 0.5 - M_PI * angleFilter);
    float maxDist = 2000.0; // hard coded distance of 2000 Angstrom

    // set range

    McVec3f zDir = direction * McVec3f(0.0f, 0.0f, 1.0f);

    // select end points

    for (int i = 0; i < numEdges; ++i)
    {
        int n = filaments.getNumEdgePoints(i);

        if (n < 2) continue;

        McVec3f p[2];
        McVec3f d[2];

        p[0] = filaments.getEdgePoint(i, 0);
        d[0] = p[0] - SpindleTools::computePointAtDist(filaments.getEdgePoints(i), maxDist, 1, 1);
        p[1] = filaments.getEdgePoint(i, n - 1);
        d[1] = p[1] - SpindleTools::computePointAtDist(filaments.getEdgePoints(i), maxDist, n - 2, -1);

        for (int j = 0; j < 2; ++j)
        {
            // check z range

            if ((direction < 0.0f && p[j][2] > range) ||
                (direction > 0.0f && p[j][2] < range))
            {
                continue;
            }

            // check direction

            if (direction * d[j][2] < 0.0)
            {
                continue;
            }

            // check angle

            if (zDir.angle(d[j]) > maxAngle)
            {
                continue;
            }

            d[j].normalize();

            // direction projection

            float z = 0.0;

            if (direction < 0.0) z = box[4];
            else                 z = box[5];

            float t = (z - p[j].z) / d[j].z;

            p[j] = p[j] + t * d[j];
            p[j].z = 0.0f;

            // orthogonal projection

            //if (direction > 0.0f) p[j][2] -= zMax;
            //else                  p[j][2] -= zMin;

            selectedFilaments.push(McVec2i(i, j));

            points.positions.push(p[j]);
            points.directions.push(-d[j]);
        }
    }

    // free empty space

    points.positions.resize(points.positions.size());
    points.directions.resize(points.directions.size());

    return selectedFilaments;
}




/**
    Computes the points for CPD. These points represent filament
    ends with directions of two neighboring sections. For section
    one all filament ends within the range of the top part of the
    section are considered and for section two the bottom part.
    The range is given in percentage in [0...1]. The method returns
    the selected filament ends for both sections.
*/
static McDArray< McDArray < McVec2i > > computeCPDPoints(const HxSerialSectionStack::Section& section1,
                                                         const HxSerialSectionStack::Section& section2,
                                                         const float                          range,
                                                               mtalign::FacingPointSets&      points)
{
    const HxSpatialGraph& filaments1 = *section1.mFilaments;
    const HxSpatialGraph& filaments2 = *section2.mFilaments;

    const McBox3f fieldBox1 = section1.mImageBoundingBox;
    const McBox3f fieldBox2 = section2.mImageBoundingBox;

    McDArray< McDArray < McVec2i > > selectedMTs(2);

    selectedMTs[0] = computeCPDPoints(filaments1, fieldBox1,  range, points.ref);
    selectedMTs[1] = computeCPDPoints(filaments2, fieldBox2, -range, points.trans);

    return selectedMTs;
}




/**
    Throws a set directional points into the object pool (for
    debugging).
*/
static HxSpatialGraph* debugPoints(const mtalign::FacingPointSets& points)
{
    HxSpatialGraph* graph = HxSpatialGraph::createInstance();

    EdgeVertexAttribute* vertAtt = graph->addVertexAttribute("sectionID", McPrimType::MC_INT32, 1);
    EdgeVertexAttribute* edgeAtt = graph->addEdgeAttribute("sectionID", McPrimType::MC_INT32, 1);
    PointAttribute*      pointAtt = graph->addPointAttribute("radius", McPrimType::MC_FLOAT, 1);

    for (int i = 0; i < points.ref.positions.size(); ++i)
    {
        McVec3f p1 = McVec3f(points.ref.positions[i].x, points.ref.positions[i].y, 0.0);
        McVec3f p2 = McVec3f(points.ref.positions[i].x + 1000.0f * points.ref.directions[i].x,
                             points.ref.positions[i].y + 1000.0f * points.ref.directions[i].y, 0.0);

        graph->addVertex(p1);
        graph->addVertex(p2);
        graph->addEdge(i * 2, i * 2 + 1);

        vertAtt->setIntDataAtIdx(i * 2, 0);
        vertAtt->setIntDataAtIdx(i * 2 + 1, 0);
        pointAtt->setFloatDataAtPoint(1.0f, i, 0);
        pointAtt->setFloatDataAtPoint(0.1f, i, 1);
        edgeAtt->setIntDataAtIdx(i, 0);

    }

    int vID = graph->getNumVertices();
    int eID = graph->getNumEdges();

    for (int i = 0; i < points.trans.positions.size(); ++i)
    {
        McVec3f p1 = McVec3f(points.trans.positions[i].x, points.trans.positions[i].y, 0.0);
        McVec3f p2 = McVec3f(points.trans.positions[i].x + 1000.0f * points.trans.directions[i].x,
                             points.trans.positions[i].y + 1000.0f * points.trans.directions[i].y, 0.0);

        graph->addVertex(p1);
        graph->addVertex(p2);
        graph->addEdge(vID + i * 2, vID + i * 2 + 1);

        vertAtt->setIntDataAtIdx(vID + i * 2, 1);
        vertAtt->setIntDataAtIdx(vID + i * 2 + 1, 1);
        pointAtt->setFloatDataAtPoint(1.0f, eID + i, 0);
        pointAtt->setFloatDataAtPoint(0.1f, eID + i, 1);
        edgeAtt->setIntDataAtIdx(eID + i, 1);
    }

    theObjectPool->addObject(graph);

    return graph;
}




/**
    Throws a set directional points into the object pool with additional
    matching information (for debugging).
*/
void debugPoints(const mtalign::FacingPointSets& points, const mtalign::MatchingPGM& matching)
{
    HxSpatialGraph* graph = debugPoints(points);

    EdgeVertexAttribute* edgeAtt = graph->findEdgeAttribute("sectionID");

    int eID = points.ref.positions.size();

    for (int i = 0; i < matching.matchedRefPointIds.size(); ++i)
    {
        int sourceID = matching.matchedRefPointIds[i];
        int targetID = matching.matchedTransPointIds[i];

        edgeAtt->setIntDataAtIdx(sourceID, i + 1);
        edgeAtt->setIntDataAtIdx(eID + targetID, i + 1);
    }

    theObjectPool->addObject(graph);
}




/**
    If two landmarks are too close in the top section, one of them
    is completely removed. The function is only applied to the landmarks
    of type temporary.
*/
static void filterLandmarks(HxSerialSectionStack::Landmarks& landmarks)
{
    float minDist = 200.0; // hard coded allowed minimal distance

    for (int i = 0; i < landmarks.getNumMarks(); ++i)
    {
        if (landmarks.getType(i) == HxSerialSectionStack::TEMPORARY) continue;

        for (int j = i + 1; j < landmarks.getNumMarks(); ++j)
        {
            if (landmarks.getType(j) != HxSerialSectionStack::TEMPORARY) continue;

            const McVec2f& p = landmarks.getMarks(1)[i];
            const McVec2f& q = landmarks.getMarks(1)[j];

            if ((p - q).length() < minDist)
            {
                landmarks.deleteMark(j);
                j--;
            }
        }
    }
}




/**
    Removes all temporary landmarks that lie too far away from
    the corresponding filament end.
*/
static void filterLandmarks(const HxSpatialGraph& graph, HxSerialSectionStack::Landmarks& landmarks)
{
    int   numEdges = graph.getNumEdges();
    float maxDist = 2000.0; // hard coded maximal allowed distance

    for (int i = 0; i < landmarks.getNumMarks(); ++i)
    {
        if (landmarks.getType(i) != HxSerialSectionStack::TEMPORARY) continue;

        bool keep = false;

        for (int j = 0; j < numEdges; ++j)
        {
            const McVec2f p1 = SpindleTools::vec2(graph.getEdgePoint(j, 0));
            const McVec2f p2 = SpindleTools::vec2(graph.getEdgePoint(j, graph.getNumEdgePoints(i) - 1));
            const McVec2f& q1 = landmarks.getMarks(0)[i];
            const McVec2f& q2 = landmarks.getMarks(0)[i];

            float d1 = (p1 - q1).length();
            float d2 = (p2 - q2).length();

            if (d1 < maxDist || d2 < maxDist)
            {
                keep = true;
                break;
            }
        }

        if (!keep)
        {
            landmarks.deleteMark(i);
            i--;
        }
    }
}




static bool projectEndPoint(const HxSpatialGraph& graph, int filamentID, int endID, float zRange, int section, const McBox3f& fieldBox, ProjectedEnd& point)
{
    float dir = section == 0 ? 1.0f : -1.0f;

    const McBox3f& box = graph.getBoundingBox();

    float zMin = box[4];
    float zMax = box[5];
    float angleFilter = 0.01; // hard coded 0.01
    float maxAngle = (M_PI * 0.5 - M_PI * angleFilter);
    float maxDist = 2000.0; // hard coded distance of 2000 Angstrom

    // set range

    if (dir > 0.0f) zMin = zMax - zRange * (zMax - zMin);
    else            zMax = zMin + zRange * (zMax - zMin);

    McVec3f zDir(0.0f, 0.0f, dir * 1.0f);

    // compute end point and direction

    int n = graph.getNumEdgePoints(filamentID);

    if (n < 2) return false;

    McVec3f p;
    McVec3f q;
    McVec3f d;

    float e;

    if (endID == 0)
    {
        p = graph.getEdgePoint(filamentID, 0);
        q = SpindleTools::computePointAtDist(graph.getEdgePoints(filamentID), maxDist, 1, 1, e);
        d = p - q;
    }
    else
    {
        p = graph.getEdgePoint(filamentID, n - 1);
        q = SpindleTools::computePointAtDist(graph.getEdgePoints(filamentID), maxDist, n - 2, -1, e);
        d = p - q;
    }

    // check z range

    if ((dir < 0.0f && p[2] > zMax) ||
        (dir > 0.0f && p[2] < zMin))
    {
        return false;
    }

    // check direction

    if (dir * d[2] < 0.0f)
    {
        return false;
    }

    // check angle

    if (zDir.angle(d) > maxAngle)
    {
        return false;
    }

    d.normalize();

    // direction projection

    float z = 0.0;

    if (dir < 0.0) z = fieldBox[4];
    else           z = fieldBox[5];

    float t = (z - p.z) / d.z;

    if (t < 0) return false;

    point.mPosition = p + t * d;
    point.mPosition.z = 0.0f;
    point.mDirection = d;
    point.mCurvature = e;

    // orthogonal projection

    //if (direction > 0.0f) p[j][2] -= zMax;
    //else                  p[j][2] -= zMin;

    return true;
}




/**
    Smooths the positions of the temporary landmarks using a
    Gaussian-like filter.
*/
static void smoothLandmarks(HxSerialSectionStack::Landmarks& landmarks)
{
    McWatch watch;

    watch.start();

    int n = landmarks.getNumMarks();

    if (n == 1) return;

    McDArray< float >   weights(n);
    McDArray< McVec2f > vectors(n);

    for (int i = 0; i < n; ++i)
    {
        if (landmarks.getType(i) == HxSerialSectionStack::MANUAL) continue;

        McVec2f p_i = McVec2f(landmarks.getMark(i, 0).x, landmarks.getMark(i, 0).y);
        float   w_i = 0.0f;

        for (int j = 0; j < n; ++j)
        {
            if (i == j) continue;

            McVec2f p_j = McVec2f(landmarks.getMark(j, 0).x, landmarks.getMark(j, 0).y);
            float   d_ij = (p_i - p_j).length();

            weights[j] = 1.0 / (d_ij * d_ij);

            w_i += weights[j];
        }

        for (int j = 0; j < n; ++j)
        {
            weights[j] = 0.5f * (weights[j] / w_i);
        }

        weights[i] = 0.5f;

        vectors[i].setValue(0.0f, 0.0f);

        for (int j = 0; j < n; ++j)
        {
            McVec2f p_j = McVec2f(landmarks.getMark(j, 0).x, landmarks.getMark(j, 0).y);
            McVec2f q_j = McVec2f(landmarks.getMark(j, 1).x, landmarks.getMark(j, 1).y);

            McVec2f v = q_j - p_j;

            vectors[i] += weights[j] * v;
        }
    }

    for (int i = 0; i < n; ++i)
    {
        if (landmarks.getType(i) == HxSerialSectionStack::MANUAL) continue;

        McVec2f p_i = landmarks.getMark(i, 0);
        McVec2f q_i = landmarks.getMark(i, 1);

        McVec2f m = 0.5 * (p_i + q_i);

        p_i = m - 0.5 * vectors[i];
        q_i = m + 0.5 * vectors[i];

        landmarks.setMark(p_i, i, 0);
        landmarks.setMark(q_i, i, 1);
    }

    float t = watch.stop();

    theMsg->printf("time: %f", t);
}




HxSerialSectionAligner::HxSerialSectionAligner(void)
    : HxModule(HxSerialSectionStack::getClassTypeId())
    , portMode(this, "mode", tr("Mode"), 2)
    , portSectionInterface(this, "sectionInterface", tr("Section Interface"))
    , portSliceTop(this, "slicePositionTop", tr("Slice Position Top"))
    , portSliceBottom(this, "slicePositionBottom", tr("Slice Position Bottom"))
    , portFilamentScale(this, "filamentScale", tr("Filament Scale"))
    , portSynchronizeSlices(this, "synchronizeSlices", tr("Synchronize Slices"))
    , portSliceQuality(this, "sliceQuality", tr("Slice Quality"))
    , portSliceQualityInteraction(this, "sliceQualityInteract", tr("Slice Quality Interaction"))
    , portWarpingQuality(this, "warpingQuality", tr("Warping Quality"))
    , portLandmarkScale(this, "landmarkScale", tr("Landmark Scale"))
    , portLandmarkShow(this, "landmarkShow", tr("Landmark Show"), 1)
    , portLandmarkDelete(this, "deleteLandmarks", tr("Landmarks Delete"), 3)
    , portMatching(this, "matching", tr("Matching"))
    , portMatchingRegion(this, "matchingRegion", tr("Matching Region"))
    , portMatchingCheck(this, "matchingCheck", "Matching Check")
    , port3DRadius(this, "cutRadius", "Cut Radius")
    , portSectionSelection(this, "selectSection", tr("Select Section"))
    , portComputeStack(this, "computeStack", tr("Compute Stack"))
    , portComputeStackResolution(this, "voxelSizeXY", tr("Voxel Size X-Y"))
    , mMouseLeft(false)
    , mMouseMiddle(false)
    , mCheckField(0)
{
    // ports

    portMode.setLabel(0, "alignment");
    portMode.setLabel(1, "matching");
    portMode.setValue(0);

    portLandmarkShow.setLabel(0, "grid");
    portLandmarkShow.setValue(0, 0);

    portSliceTop.setMinMax(0.0, 1.0);
    portSliceTop.setValue(0.2);

    portSliceBottom.setMinMax(0.0, 1.0);
    portSliceBottom.setValue(0.8);

    portLandmarkScale.setMinMax(0.1, 5.0);
    portLandmarkScale.setValue(1.0);

    portFilamentScale.setMinMax(0.1, 5.0);
    portFilamentScale.setValue(1.0);

    portSynchronizeSlices.setNum(1);
    portSynchronizeSlices.setLabel(0, "");
    portSynchronizeSlices.setValue(0, true);

    portSliceQuality.setNum(3);
    portSliceQuality.setLabel(0, "high");
    portSliceQuality.setLabel(1, "medium");
    portSliceQuality.setLabel(2, "low");
    portSliceQuality.setValue(1);

    portSliceQualityInteraction.setNum(3);
    portSliceQualityInteraction.setLabel(0, "high");
    portSliceQualityInteraction.setLabel(1, "medium");
    portSliceQualityInteraction.setLabel(2, "low");
    portSliceQualityInteraction.setValue(1);


    portWarpingQuality.setMinMax(3, 20);
    portWarpingQuality.setValue(10);

    portMatchingRegion.setMinMax(0.1, 1.0);
    portMatchingRegion.setValue(0.4);

    portLandmarkDelete.setNumButtons(2);
    portLandmarkDelete.setLabel(0, "manual");
    portLandmarkDelete.setLabel(1, "automatic");

    portMatching.insertPushButton(0, "compute");
    portMatching.insertPushButton(1, "clear");
    portMatching.insertCheckBox(2, "automatic landmarks");
    portMatching.setValue(2, 1);

    portMatchingCheck.insertPushButton(0, "<");
    portMatchingCheck.insertPushButton(1, ">");
    portMatchingCheck.insertPushButton(2, "confirm");
    portMatchingCheck.insertLabel(3, "...");
    portMatchingCheck.hide();

    portMatchingCheck.setItemToolTip(0, "previous");
    portMatchingCheck.setItemToolTip(1, "next (N)");
    portMatchingCheck.setItemToolTip(2, "confirm (C)");

    port3DRadius.setMinMax(1000.0f, 5000.0f);
    port3DRadius.setValue(2500.0f);
    port3DRadius.hide();

    portComputeStack.insertPushButton(0, "create");
    portComputeStack.insertCheckBox(1, "tomograms");
    portComputeStack.setValue(1, 1);

    portComputeStackResolution.insertFloatText(0, "%.1f", 10);
    portComputeStackResolution.setValue(0, 0.1f);
    portComputeStackResolution.insertLabel(1, "...");

    // oiv nodes

    McHandle< SoEventCallback > eventCB = new SoEventCallback;
    eventCB->addEventCallback(SoMouseButtonEvent::getClassTypeId(), mouseClickCB, this);
    eventCB->addEventCallback(SoLocation2Event::getClassTypeId(), mouseClickCB, this);
    eventCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), mouseClickCB, this);
                              

    mRootNode          = new SoSeparator();
    mAlignerTool       = new SoSerialSectionAligner();


    mRootNode->addChild(mAlignerTool);
    mRootNode->addChild(eventCB);

    // set camera

    HxViewer* viewer = theController->getCurrentViewer();

    if (viewer)
    {
        viewer->setCameraType(HxViewer::ORTHOGRAPHIC_CAMERA);
        mAlignerTool->touch();
    }
}




HxSerialSectionAligner::~HxSerialSectionAligner(void)
{
    if (mAnnotation)
    {
        theObjectPool->removeObject(mAnnotation);
    }
}




void HxSerialSectionAligner::checkNext(int direction)
{
    if (!mCheckNext) return;

    if (!mSerialSectionStack || mCheckCur >= mCheckMax)
    {
        portMatchingCheck.setSensitivity(0, 0);
        portMatchingCheck.setSensitivity(1, 0);
        portMatchingCheck.setSensitivity(2, 0);

        mCheckPos = -1;
        return;
    }

    int n = mCheckEnds.size();

    Matching& matching = mSerialSectionStack->getSectionInterface(portSectionInterface.getValue()).mMatching;

    for (int i = 0; i < n; ++i)
    {
        mCheckPos = (mCheckPos + n + direction) % n;

        int mID = mCheckEnds[mCheckPos] / 2;
        int eID = mCheckEnds[mCheckPos] % 2;

        if (matching.getState(mID, eID, 0) == HxSerialSectionStack::UNDEFINED ||
            matching.getState(mID, eID, 0) == HxSerialSectionStack::MATCHED_AUTOMATIC)
        {
            portMatchingCheck.setSensitivity(2, 1);
            return;
        }
    }

    portMatchingCheck.setSensitivity(2, 0);
    mCheckPos = -1;
}




void HxSerialSectionAligner::checkPos(const McVec2i& selection)
{
    if (selection.x < 0)
    {
        portMatchingCheck.setSensitivity(2, 0);
        mCheckNext = false;
        mCheckPos  = -1;
        return;
    }

    Matching& matching = mSerialSectionStack->getSectionInterface(portSectionInterface.getValue()).mMatching;

    int n = mCheckEnds.size();

    for (int i = 0; i < n; ++i)
    {
        mCheckPos = (mCheckPos + 1) % n;

        int mID = mCheckEnds[mCheckPos] / 2;
        int eID = mCheckEnds[mCheckPos] % 2;

        if (mID == selection.x &&
            eID == selection.y)
        {
            if (matching.getState(mID, eID, 0) == HxSerialSectionStack::UNDEFINED         ||
                matching.getState(mID, eID, 0) == HxSerialSectionStack::MATCHED_AUTOMATIC)
            {
                portMatchingCheck.setSensitivity(2, 1);
            }
            else
            {
                portMatchingCheck.setSensitivity(2, 0);
            }
            mCheckNext = false;
            return;
        }
    }

    portMatchingCheck.setSensitivity(2, 0);
    mCheckPos = -1;
}




void HxSerialSectionAligner::compute()
{
    // new stack

    if (portData.isNew(HxData::NEW_SOURCE))
    {
        mSerialSectionStack = hxconnection_cast< HxSerialSectionStack >(portData);

        portSectionInterface.setMinMax(0, mSerialSectionStack->getNumSections() - 2);
        portSectionInterface.setValue(0);

        portSectionSelection.setMinMax(0, mSerialSectionStack->getNumSections() - 1);
        portSectionSelection.setValue(mSerialSectionStack->getNumSections() / 2);

        updateResolutionLabel(true);

        if (mSerialSectionStack->getNumSections() > 1)
        {
            portSectionSelection.show();
        }
        else
        {
            portSectionSelection.hide();
        }

        if (mSerialSectionStack->getNumSections() > 2)
        {
            portSectionInterface.show();
        }
        else
        {
            portSectionInterface.hide();
        }

        mAlignerTool->setStack(mSerialSectionStack);
        mAlignerTool->setSectionChange(portSectionInterface.getValue());

        HxViewer* viewer = theController->getCurrentViewer();

        if (viewer)
        {
            viewer->viewAll();
            viewer->requestRedraw(true);
            mAlignerTool->touch();
        }

        showGeom(mRootNode);

        // annotation

        if (!mAnnotation)
        {
            mAnnotation = HxAnnotation::createInstance();

            theObjectPool->setHideNewModules(true);
            theObjectPool->addObject(mAnnotation);
            theObjectPool->setHideNewModules(false);

            mAnnotation->m_annotationTool.portText.setValue("");
            mAnnotation->m_annotationTool.portPosType.setValue(1);
            mAnnotation->m_annotationTool.portNdcPosition.setValue(0, 0.51);
            mAnnotation->m_annotationTool.portNdcPosition.setValue(1, 0.51);
            mAnnotation->fire();
        }

        // initialize endpoint densities
    }

    if (mAlignerTool == NULL) return;

    if (mSerialSectionStack == NULL) return;

    // sets working mode

    if (portMode.isNew())
    {
        mAlignerTool->setRenderMode(portMode.getValue());

        portLandmarkScale.hide();
        portLandmarkShow.hide();
        portLandmarkDelete.hide();

        portMatching.hide();
        portMatchingCheck.hide();
        portMatchingRegion.hide();
        port3DRadius.hide();

        // alignment mode
        if (portMode.getValue() == 0)
        {
            portLandmarkShow.show();
            portLandmarkScale.show();
            portLandmarkDelete.show();
        }
        // matching mode
        else
        {
            portMatching.show();
            portMatchingRegion.show();
            portMatchingCheck.show();
            port3DRadius.show();
        }

        updateAnnotation();
    }

    // sets a new section interface

    if (portSectionInterface.isNew())
    {
        mAlignerTool->setSectionChange(portSectionInterface.getValue());
        updateCheck();
        updateAnnotation();
    }

    // shows / hides

    if (portLandmarkShow.isNew())
    {
        if (portLandmarkShow.getValue(0)) mAlignerTool->showWarpingGrid();
        else                              mAlignerTool->hideWarpingGrid();
    }

    // changes the slice position of the top section

    if (portSliceTop.isNew())
    {
        mAlignerTool->setSlicePosition(portSliceTop.getValue(), 1);

        if (portSynchronizeSlices.getValue(0))
        {
            portSliceBottom.setValue(1.0 - portSliceTop.getValue());
            portSliceBottom.untouch();
            mAlignerTool->setSlicePosition(portSliceBottom.getValue(), 0);
        }
    }

    // changes the slice position of the bottom section

    if (portSliceBottom.isNew())
    {
        mAlignerTool->setSlicePosition(portSliceBottom.getValue(), 0);

        if (portSynchronizeSlices.getValue(0))
        {
            portSliceTop.setValue(1.0 - portSliceBottom.getValue());
            portSliceTop.untouch();
            mAlignerTool->setSlicePosition(portSliceTop.getValue(), 1);
        }
    }

    // changes the slice rendering quality

    if (portSliceQuality.isNew() || portSliceQualityInteraction.isNew())
    {
        mAlignerTool->setSliceQuality(portSliceQuality.getValue(), portSliceQualityInteraction.getValue());
    }

    // changes the warping quality

    if (portWarpingQuality.isNew())
    {
        mAlignerTool->setWarpingQuality(portWarpingQuality.getValue());
    }

    // changes the scale of the landmark rendering

    if (portLandmarkScale.isNew())
    {
        mAlignerTool->setLandmarkScale(portLandmarkScale.getValue());
    }

    // change filament scale

    if (portFilamentScale.isNew())
    {
        mAlignerTool->setMicrotubulesScale(portFilamentScale.getValue());
        mAlignerTool->touch();
    }

    // change cut radius for 3D view

    if (port3DRadius.isNew())
    {
        mAlignerTool->getSliceRenderer(4).set3DRadius(port3DRadius.getValue());
        mAlignerTool->touch();
    }

    // delete landmarks

    if (portLandmarkDelete.isNew())
    {
        SectionInterface& secInterface = mSerialSectionStack->getSectionInterface(portSectionInterface.getValue());
        Landmarks&        landmarks = secInterface.mLandmarks;

        if (portLandmarkDelete.wasHit(0))
        {
            landmarks.deleteMarks(HxSerialSectionStack::MANUAL);
        }
        else if (portLandmarkDelete.wasHit(1))
        {
            landmarks.deleteMarks(HxSerialSectionStack::AUTOMATIC);
            landmarks.deleteMarks(HxSerialSectionStack::TEMPORARY);
        }

        mAlignerTool->touch();

        mSerialSectionStack->touch();
    }

    // computes automatic matchings

    if (portMatching.isItemNew(0))
    {
        computeMatching();

        mAlignerTool->updateEndPointDensities();

        updateAnnotation();

        mSerialSectionStack->touch();
    }

    // clear automatic matchings

    if (portMatching.isItemNew(1))
    {
        SectionInterface& secInterface = mSerialSectionStack->getSectionInterface(portSectionInterface.getValue());

        secInterface.mMatching.remove(HxSerialSectionStack::TEMPORARY);

        if (portMatching.getValue(2))
        {
            secInterface.mLandmarks.deleteMarks(HxSerialSectionStack::TEMPORARY);

            computeMatchingLandmarks(HxSerialSectionStack::AUTOMATIC);
            computeMatchingLandmarks(HxSerialSectionStack::TEMPORARY);
        }

        mAlignerTool->updateEndPointDensities();

        updateAnnotation();

        mAlignerTool->setSectionChange(portSectionInterface.getValue());

        mSerialSectionStack->touch();
    }

    // update check when changing the region

    if (portMatchingRegion.isNew())
    {
        updateCheck();
        updateAnnotation();
    }

    // check matchings

    if (portMatchingCheck.isItemNew(0) ||
        portMatchingCheck.isItemNew(1) ||
        portMatchingCheck.isItemNew(2))
    {
        int             interfaceID = portSectionInterface.getValue();
        HxSpatialGraph* graph       = mSerialSectionStack->getSection(interfaceID).mFilaments;

        if (graph)
        {
            // confirm

            if (portMatchingCheck.isItemNew(2) && mCheckPos >= 0)
            {
                Matching& matching = mSerialSectionStack->getSectionInterface(interfaceID).mMatching;

                int  microtubleID = mCheckEnds[mCheckPos] / 2;
                int  endID        = mCheckEnds[mCheckPos] % 2;
                int  numMatches   = matching.getNumMatches();
                int  foundMatch   = -1;

                for (int i = 0; i < numMatches; ++i)
                {
                    if (matching.get(i).i == microtubleID &&
                        matching.get(i).j == endID)
                    {
                        foundMatch = i;
                        break;
                    }
                }

                if (matching.getState(microtubleID, endID, 0) == HxSerialSectionStack::UNDEFINED ||
                    matching.getState(microtubleID, endID, 0) == HxSerialSectionStack::MATCHED_AUTOMATIC)
                {
                    if (foundMatch >= 0)
                    {
                        matching.setState(HxSerialSectionStack::MATCHED, microtubleID, endID, 0);
                        matching.setType(HxSerialSectionStack::AUTOMATIC, foundMatch);
                    }
                    else
                    {
                        matching.setState(HxSerialSectionStack::UNMATCHED, microtubleID, endID, 0);
                    }

                    mCheckCur++;
                }

                if (!mCheckNext)
                {
                    portMatchingCheck.setSensitivity(2, 0);
                }

                updateCheckLabel();

                mAlignerTool->setSectionChange(portSectionInterface.getValue());

                mSerialSectionStack->touch();
            }

            // goto next

            int direction = 1;

            if (portMatchingCheck.isItemNew(0) ||
                portMatchingCheck.isItemNew(1))
            {
                mCheckNext = true;
            }

            if (portMatchingCheck.isItemNew(0))
            {
                direction = -1;
            }

            checkNext(direction);

            if (mCheckPos >= 0 && mCheckNext)
            {
                int  microtubleID = mCheckEnds[mCheckPos] / 2;
                int  endID        = mCheckEnds[mCheckPos] % 2;

                McBox3f box = mAlignerTool->getSection(0)->getBoundingBox();

                McVec3f v = graph->getEdgePoint(microtubleID, endID * (graph->getNumEdgePoints(microtubleID) - 1));
                McVec3f c = box.getCenter();
                float   z = (v.z - box.getMin().z) / (box.getMax().z - box.getMin().z);

                portSliceBottom.setValue(z);

                mAlignerTool->getTranslation().setValue(c.x - v.x, c.y - v.y);
                mAlignerTool->setSelection(McVec2i(microtubleID, endID));
                mAlignerTool->setSlicePosition(portSliceBottom.getValue(), 0);
                mAlignerTool->touch();
            }
        }
    }



    // create complete stack

    if (portComputeStack.isItemNew(0))
    {
        int oldInterfaceID = mAlignerTool->getSectionChange();

        mAlignerTool->setStack(0);

        HxSpatialGraph* graph = HxSpatialGraph::createInstance();
        HxUniformScalarField3* field = HxUniformScalarField3::createInstance();

        mSerialSectionStack->createStack(*graph, *field, portComputeStackResolution.getValue(0), portSectionSelection.getValue());

        theObjectPool->addObject(graph);
        theObjectPool->addObject(field);

        mAlignerTool->setStack(mSerialSectionStack, oldInterfaceID);
    }


    // show / hide tomogram resolution

    if (portComputeStack.isItemNew(1))
    {
        if (portComputeStack.getValue(1)) portComputeStackResolution.show();
        else                              portComputeStackResolution.hide();
    }

    // update MB label

    if (portComputeStackResolution.isItemNew(0))
    {
        updateResolutionLabel();
    }
}





void HxSerialSectionAligner::computeCheckField()
{
    if (!mSerialSectionStack) return;

    const HxUniformScalarField3* field = mAlignerTool->getSection(0);
    const HxSpatialGraph*        graph = mAlignerTool->getGraph(0);

    if (!field || !graph) return;

    const McBox3f& fieldBox = field->getBoundingBox();

    int     dims[3];
    float   box[6];

    dims[0] = field->lattice().getDims()[0];
    dims[1] = field->lattice().getDims()[1];
    dims[2] = 1;

    // restrict dimensions to 2048 (faster re-computation)

    if (dims[0] > 2048 || dims[1] > 2048)
    {
        if (dims[0] > dims[1])
        {
            dims[1] = (int)((2048.0 / (double) dims[0]) * (double) dims[1] + 0.1);
            dims[0] = 2048;
        }
        else
        {
            dims[0] = (int)((2048.0 / (double) dims[1]) * (double) dims[0] + 0.1);
            dims[1] = 2048;
        }
    }


    box[0] = fieldBox[0];
    box[1] = fieldBox[1];
    box[2] = fieldBox[2];
    box[3] = fieldBox[3];
    box[4] = 0.0f;
    box[5] = 1.0f;

    int numEnds = mCheckEnds.size();

    // create label field

    McHandle< HxUniformScalarField3 > labelField = new HxUniformScalarField3(dims, McPrimType::MC_UINT32, 0);

    labelField->lattice().setBoundingBox(box);

    McVec3f voxel = labelField->getVoxelSize();

    for (int i = 0; i < dims[0] * dims[1]; ++i)
    {
        ((unsigned int*)labelField->lattice().dataPtr())[i] = 0;
    }

    for (int i = 0; i < numEnds; ++i)
    {
        int mID = mCheckEnds[i] / 2;
        int eID = mCheckEnds[i] % 2;

        McVec3f p = graph->getEdgePoint(mID, 0);

        if (eID == 1)
        {
            p = graph->getEdgePoint(mID, graph->getNumEdgePoints(mID) - 1);
        }

        int x = (int)((p.x - box[0]) / voxel.x);
        int y = (int)((p.y - box[2]) / voxel.y);

        x = std::max(0, std::min(dims[0] - 1, x));
        y = std::max(0, std::min(dims[1] - 1, y));

        ((unsigned int*)labelField->lattice().dataPtr())[y * dims[0] + x] = i + 1;
    }

    // create distance map

    McHandle<HxComputeDistanceMap> distMapCmp = HxComputeDistanceMap::createInstance();

    distMapCmp->portData.connect(labelField);
    distMapCmp->portRegion.setValue(0, 1);
    distMapCmp->portAction.hit();
    distMapCmp->compute();

    McHandle< HxUniformScalarField3 > distMap = (HxUniformScalarField3*) distMapCmp->getResult(0);

    distMapCmp->portData.disconnect();
    distMap->portMaster.disconnect();

    // apply watershed

    McHandle< HxWatershed > watershedCmp = HxWatershed::createInstance();

    watershedCmp->portData.connect(distMap);
    watershedCmp->portLabels.connect(labelField);
    watershedCmp->portAction.hit();
    watershedCmp->compute();

    mCheckField = (HxUniformScalarField3*) watershedCmp->getResult(0);

    watershedCmp->portData.disconnect();
    watershedCmp->portLabels.disconnect();
    mCheckField->portMaster.disconnect();

    mAlignerTool->setCheck(mCheckField, mCheckEnds);
}




void HxSerialSectionAligner::computeMatching()
{
    int   interfaceID = portSectionInterface.getValue();
    float range       = portMatchingRegion.getValue();

    SectionInterface& secInterface = mSerialSectionStack->getSectionInterface(interfaceID);
    Landmarks&        landmarks    = secInterface.mLandmarks;
    Matching&         matching     = secInterface.mMatching;

    const Section& section1 = mSerialSectionStack->getSection(interfaceID);
    const Section& section2 = mSerialSectionStack->getSection(interfaceID + 1);

    // compute CPD, only if there are no automatic landmarks

    bool generatedTmp = false;

    if (landmarks.getNumMarks(HxSerialSectionStack::TEMPORARY) == 0 &&
        landmarks.getNumMarks(HxSerialSectionStack::AUTOMATIC) == 0)
    {
        mtalign::FacingPointSets points;

        computeCPDPoints(section1, section2, range, points);

        landmarks.transform(points);

        mtalign::CPDParams  cpdParams = mtalign::CPDParams::defaultsElastic();
        mtalign::WarpResult warping;
        mtalign::cpd(points, warping, cpdParams, 0);

        if (warping.alignInfo.sigmaSquare <= 0.001)
        {
            landmarks.transformInv(warping);

            for (int i = 0; i < warping.mlsParams.ps.size(); ++i)
            {
                landmarks.addMark(warping.mlsParams.qs[i], warping.mlsParams.ps[i], HxSerialSectionStack::TEMPORARY);
            }

            filterLandmarks(landmarks);

            generatedTmp = true;
        }
    }

    // compute matching

    mtalign::FacingPointSets matchingPoints;

    McDArray< McDArray < McVec2i > > selection = computeCPDPoints(section1, section2, range, matchingPoints);

    landmarks.transform(matchingPoints);

    mtalign::MatchingPGMParams pgmparams;

    pgmparams.evidence.clear();
    pgmparams = mtalign::MatchingPGMParams::defaultsWeber2014();

    const mtalign::MatchingPGM matches = matchingPGM(matchingPoints, pgmparams, 0);

    theMsg->printf("Matching: %d matched pairs, %d ambiguities, %d critical nodes.",
                    matches.matchedRefPointIds.size(),
                    matches.ambiguities.size(),
                    matches.criticalNodes.size());

    matching.set(selection, matches);

    if (generatedTmp)
    {
        landmarks.deleteMarks(HxSerialSectionStack::TEMPORARY);
    }

    // compute CPD (only for the matching MT)

    if (portMatching.getValue(2))
    {
        landmarks.deleteMarks(HxSerialSectionStack::AUTOMATIC);
    }

    computeMatchingLandmarks(HxSerialSectionStack::AUTOMATIC);
    computeMatchingLandmarks(HxSerialSectionStack::TEMPORARY);

    mAlignerTool->setSectionChange(portSectionInterface.getValue());
}




void HxSerialSectionAligner::computeMatchingLandmarks(HxSerialSectionStack::SettingType type)
{
    // landmarks will not be created based on matchings

    if (!portMatching.getValue(2))
    {
        return;
    }

    // creation of landmarks based on matchings using CPD

    int interfaceID = portSectionInterface.getValue();

    SectionInterface& secInterface = mSerialSectionStack->getSectionInterface(interfaceID);
    Landmarks&        landmarks    = secInterface.mLandmarks;
    Matching&         matching     = secInterface.mMatching;

    const HxSpatialGraph& graph1 = *mSerialSectionStack->getSection(interfaceID).mFilaments;
    const HxSpatialGraph& graph2 = *mSerialSectionStack->getSection(interfaceID + 1).mFilaments;

    const McBox3f& box1 = mSerialSectionStack->getSection(interfaceID).mImageBoundingBox;
    const McBox3f& box2 = mSerialSectionStack->getSection(interfaceID + 1).mImageBoundingBox;

    float zRange = portMatchingRegion.getValue();

    landmarks.deleteMarks(type);

    int numMatches = matching.getNumMatches();

    mtalign::FacingPointSets warpingPoints;

    warpingPoints.ref.directions.remax(numMatches,0);
    warpingPoints.ref.positions.remax(numMatches,0);
    warpingPoints.trans.directions.remax(numMatches,0);
    warpingPoints.trans.positions.remax(numMatches,0);

    for (int i = 0; i < numMatches; ++i)
    {
        if (type == HxSerialSectionStack::AUTOMATIC)
        {
            if (matching.getType(i) != HxSerialSectionStack::MANUAL)
            {
                continue;
            }
        }
        else if (type == HxSerialSectionStack::TEMPORARY)
        {
            if (matching.getType(i) != HxSerialSectionStack::TEMPORARY &&
                matching.getType(i) != HxSerialSectionStack::AUTOMATIC)
            {
                continue;
            }
        }

        McVec4i m = matching.get(i);

        if (matching.isLandmarkForbidden(m.i, m.j, 0) ||
            matching.isLandmarkForbidden(m.k, m.l, 1))
        {
            continue;
        }

        ProjectedEnd p1;
        ProjectedEnd p2;

        if (!projectEndPoint(graph1, m.i, m.j, zRange, 0, box1, p1) ||
            !projectEndPoint(graph2, m.k, m.l, zRange, 1, box2, p2))
        {
            continue;
        }

        warpingPoints.ref.directions.push(p1.mDirection);
        warpingPoints.ref.positions.push(p1.mPosition);
        warpingPoints.trans.directions.push(p2.mDirection);
        warpingPoints.trans.positions.push(p2.mPosition);
    }

    landmarks.transform(warpingPoints);

    mtalign::CPDParams  cpdParams = mtalign::CPDParams::defaultsElastic();
    mtalign::WarpResult warping;
    mtalign::cpd(warpingPoints, warping, cpdParams, 0);

    landmarks.transformInv(warping);

    for (int i = 0; i < warping.mlsParams.ps.size(); ++i)
    {
        landmarks.addMark(warping.mlsParams.qs[i], warping.mlsParams.ps[i], type);
    }
}




void HxSerialSectionAligner::computeMatchingLandmarksFast(HxSerialSectionStack::SettingType type)
{
    if (!mSerialSectionStack) return;

    // landmarks will not be created based on matchings

    if (!portMatching.getValue(2))
    {
        return;
    }

    // creation of landmarks based on matchings

    int interfaceID = portSectionInterface.getValue();

    SectionInterface& secInterface = mSerialSectionStack->getSectionInterface(interfaceID);
    Landmarks&        landmarks    = secInterface.mLandmarks;
    Matching&         matching     = secInterface.mMatching;

    const HxSpatialGraph& graph1 = *mSerialSectionStack->getSection(interfaceID).mFilaments;
    const HxSpatialGraph& graph2 = *mSerialSectionStack->getSection(interfaceID + 1).mFilaments;

    const McBox3f& box1 = mSerialSectionStack->getSection(interfaceID).mImageBoundingBox;
    const McBox3f& box2 = mSerialSectionStack->getSection(interfaceID + 1).mImageBoundingBox;

    float zRange = portMatchingRegion.getValue();

    landmarks.deleteMarks(HxSerialSectionStack::AUTOMATIC);
    landmarks.deleteMarks(HxSerialSectionStack::TEMPORARY);

    // thresholds

    float alpha = (15.0 / 180.0) * M_PI;

    // collect matchings

    int numMatchings = matching.getNumMatches();

    for (int i = 0; i < numMatchings; ++i)
    {
        if (matching.isLandmarkForbidden(i)) continue;

        McVec4i match = matching.get(i);

        ProjectedEnd point1;
        ProjectedEnd point2;

        if (!projectEndPoint(graph1, match.i, match.j, zRange, 0, box1, point1) ||
            !projectEndPoint(graph2, match.k, match.l, zRange, 1, box2, point2))
        {
            continue;
        }

        // curvature at the ends is too high (maybe distorted)

        if (point1.mCurvature > 0.20 || point2.mCurvature > 0.20)
        {
            continue;
        }

        // angular difference between directions is too large 

        if (point1.mDirection.angle(-point2.mDirection) > alpha)
        {
            continue;
        }

        if (matching.getType(i) == HxSerialSectionStack::MANUAL)
        {
            landmarks.addMark(SpindleTools::vec2(point1.mPosition), SpindleTools::vec2(point2.mPosition), HxSerialSectionStack::AUTOMATIC);
        }
        else
        {
            landmarks.addMark(SpindleTools::vec2(point1.mPosition), SpindleTools::vec2(point2.mPosition), HxSerialSectionStack::TEMPORARY);
        }
    }

    smoothLandmarks(landmarks);
}




void HxSerialSectionAligner::mouseClick(SoEventCallback* eventCB)
{
    if (!eventCB)             return;
    if (!eventCB->getEvent()) return;

    printf("interact\n");

    bool interact = false;

    if (eventCB->getEvent()->isOfType(SoMouseButtonEvent::getClassTypeId()))
    {
        const SoMouseButtonEvent* eventMouseButton = (SoMouseButtonEvent*) eventCB->getEvent();

        bool ctrl = (bool) eventMouseButton->wasCtrlDown();

        if (SoMouseButtonEvent::isButtonPressEvent(eventMouseButton, SoMouseButtonEvent::BUTTON1))
        {
            mMouseLeft = true;
        }

        if (SoMouseButtonEvent::isButtonReleaseEvent(eventMouseButton, SoMouseButtonEvent::BUTTON1))
        {
            mMouseLeft = false;
        }

        if (SoMouseButtonEvent::isButtonPressEvent(eventMouseButton, SoMouseButtonEvent::BUTTON2))
        {
            mMouseMiddle = true;
        }

        if (SoMouseButtonEvent::isButtonReleaseEvent(eventMouseButton, SoMouseButtonEvent::BUTTON2))
        {
            mMouseMiddle = false;
        }

        interact |= mAlignerTool->interaction(eventMouseButton->getPosition(), mMouseLeft, mMouseMiddle, ctrl);
    }


    if (eventCB->getEvent()->isOfType(SoLocation2Event::getClassTypeId()))
    {
        const SoLocation2Event* eventLocation = (SoLocation2Event*) eventCB->getEvent();

        bool ctrl = (bool) eventLocation->wasCtrlDown();

        interact |= mAlignerTool->interaction(eventLocation->getPosition(), mMouseLeft, mMouseMiddle, ctrl);
    }

    if (eventCB->getEvent()->isOfType(SoKeyboardEvent::getClassTypeId()))
    {
        const SoKeyboardEvent* keyboardEvent = (SoKeyboardEvent*) eventCB->getEvent();

        if (SoKeyboardEvent::isKeyPressEvent(keyboardEvent, SoKeyboardEvent::C))
        {
            portMatchingCheck.touchItem(2);
            this->fire();
        }
        else if (SoKeyboardEvent::isKeyPressEvent(keyboardEvent, SoKeyboardEvent::N))
        {
            portMatchingCheck.touchItem(1);
            this->fire();
        }
        else if (SoKeyboardEvent::isKeyPressEvent(keyboardEvent, SoKeyboardEvent::F))
        {
            McVec2i selectedEnd = mAlignerTool->getSliceRenderer(2).getSelectedMT();

            if (selectedEnd.x >= 0)
            {
                Matching& matching = mSerialSectionStack->getSectionInterface(portSectionInterface.getValue()).mMatching;

                matching.switchForbiddenState(selectedEnd.x, selectedEnd.y, 0);

                computeMatchingLandmarks(HxSerialSectionStack::AUTOMATIC);

                mAlignerTool->setSectionChange(mAlignerTool->getSectionChange());
            }
        }
    }

    // possible updates after interaction

    if (interact)
    {
        if (mAlignerTool->getSectionChange() != portSectionInterface.getValue())
        {
            portSectionInterface.setValue(mAlignerTool->getSectionChange());
        }

        if (portMode.getValue() == 1)
        {
            computeMatchingLandmarks(HxSerialSectionStack::AUTOMATIC);

            if (mAlignerTool->getSlicePosition(0) != portSliceBottom.getValue())
            {
                portSliceBottom.setValue(mAlignerTool->getSlicePosition(0));
                portSliceBottom.untouch();
            }

            checkPos(mAlignerTool->getSliceRenderer(2).getSelectedMT());

            updateCheckCurrent();
            updateAnnotation();
        }

        mAlignerTool->setSectionChange(mAlignerTool->getSectionChange());

        mSerialSectionStack->touch();
    }
}




int HxSerialSectionAligner::parse(Tcl_Interp* t, int argc, char **argv)
{
    const char* cmd = argv[1];

    // clear check

    if (CMD("clearCheck"))
    {
        int interfaceID = portSectionInterface.getValue();

        Matching& matching = mSerialSectionStack->getSectionInterface(interfaceID).mMatching;

        int numStates1 = matching.getNumStates(0);
        int numStates2 = matching.getNumStates(1);
        int numMatches = matching.getNumMatches();

        for (int i = 0; i < numStates1; ++i)
        {
            matching.setState(HxSerialSectionStack::UNDEFINED, i / 2, i % 2, 0);
        }

        for (int i = 0; i < numStates2; ++i)
        {
            matching.setState(HxSerialSectionStack::UNDEFINED, i / 2, i % 2, 1);
        }

        for (int i = 0; i < numMatches; ++i)
        {
            McVec4i match = matching.get(i);

            if (matching.getType(i) == HxSerialSectionStack::MANUAL)
            {
                matching.setState(HxSerialSectionStack::MATCHED, match.i, match.j, 0);
                matching.setState(HxSerialSectionStack::MATCHED, match.k, match.l, 1);
            }
            else if (matching.getType(i) == HxSerialSectionStack::AUTOMATIC)
            {
                matching.setType(HxSerialSectionStack::TEMPORARY, i);
                matching.setState(HxSerialSectionStack::MATCHED_AUTOMATIC, match.i, match.j, 0);
                matching.setState(HxSerialSectionStack::MATCHED_AUTOMATIC, match.k, match.l, 1);
            }
            else if (matching.getType(i) == HxSerialSectionStack::TEMPORARY)
            {
                matching.setState(HxSerialSectionStack::MATCHED_AUTOMATIC, match.i, match.j, 0);
                matching.setState(HxSerialSectionStack::MATCHED_AUTOMATIC, match.k, match.l, 1);
            };
        }

        updateCheck();

        mAlignerTool->touch();

        mSerialSectionStack->touch();
    }


    else if (CMD("clear"))
    {
        int interfaceID = portSectionInterface.getValue();

        Matching&  matching = mSerialSectionStack->getSectionInterface(interfaceID).mMatching;
        Landmarks& landmarks = mSerialSectionStack->getSectionInterface(interfaceID).mLandmarks;

        matching.remove(HxSerialSectionStack::MANUAL);
        matching.remove(HxSerialSectionStack::AUTOMATIC);
        matching.remove(HxSerialSectionStack::TEMPORARY);

        landmarks.deleteMarks(HxSerialSectionStack::MANUAL);
        landmarks.deleteMarks(HxSerialSectionStack::AUTOMATIC);
        landmarks.deleteMarks(HxSerialSectionStack::TEMPORARY);

        updateCheck();

        mAlignerTool->setSectionChange(interfaceID);

        mSerialSectionStack->touch();
    }

    else if (CMD("jump"))
    {
        ASSERTARG(4);

        int             interfaceID = portSectionInterface.getValue();
        HxSpatialGraph* graph       = mSerialSectionStack->getSection(interfaceID).mFilaments;

        if (graph)
        {
            int  microtubleID = atoi(argv[2]);
            int  endID        = atoi(argv[3]);

            McBox3f box = mAlignerTool->getSection(0)->getBoundingBox();

            McVec3f v = graph->getEdgePoint(microtubleID, endID * (graph->getNumEdgePoints(microtubleID) - 1));
            McVec3f c = box.getCenter();
            float   z = (v.z - box.getMin().z) / (box.getMax().z - box.getMin().z);

            portSliceBottom.setValue(z);

            mAlignerTool->getTranslation().setValue(c.x - v.x, c.y - v.y);
            mAlignerTool->setSelection(McVec2i(microtubleID, endID));
            mAlignerTool->setSlicePosition(portSliceBottom.getValue(), 0);
            mAlignerTool->touch();
        }
    }

    else
    {
        return HxModule::parse (t, argc, argv);
    }

    return TCL_OK;
}




void HxSerialSectionAligner::savePorts(FILE* fp)
{
    // save default ports

    HxModule::savePorts(fp);
}




void HxSerialSectionAligner::updateAnnotation()
{
    if (!mAnnotation) return;

    if (portMode.getValue() == 0)
    {
        mAnnotation->m_annotationTool.portText.setValue("");
        mAnnotation->fire();
        return;
    }

    Matching& matching = mSerialSectionStack->getSectionInterface(portSectionInterface.getValue()).mMatching;

    int numMatches = matching.getNumMatches();
    int numManual    = 0;
    int numAutomatic = 0;

    for (int i = 0; i < numMatches; ++i)
    {
        if (matching.getType(i) == HxSerialSectionStack::MANUAL) numManual++;
        else                                                     numAutomatic++;
    }

    QString numManualStr    = QString::number(numManual);
    QString numAutomaticStr = QString::number(numAutomatic);
    QString numEndsStr      = QString::number(mCheckEnds.size());

    QString label = QString("manual = ")      + numManualStr +
                    QString(", automatic = ") + numAutomaticStr +
                    QString(", ends = ")      + numEndsStr;

    mAnnotation->m_annotationTool.portText.setValue(label);
    mAnnotation->fire();
}




void HxSerialSectionAligner::updateCheck()
{
    mCheckEnds.clear();
    mCheckMax  = 0;
    mCheckCur  = 0;
    mCheckPos  = -1;
    mCheckNext = true;

    if (!mSerialSectionStack) return;

    int n = mSerialSectionStack->getNumSections();
    int c = portSectionInterface.getValue();

    const HxSpatialGraph* graph1 = mSerialSectionStack->getSection(c).mFilaments;
    const HxSpatialGraph* graph2 = mSerialSectionStack->getSection(c + 1).mFilaments;

    if (!graph1 || !graph2) return;

    mSerialSectionStack->getSectionInterface(c).mMatching.updateEndStates(graph1, graph2);

    const Matching& matching = mSerialSectionStack->getSectionInterface(c).mMatching;

    // detect end notes in user specified region
    const McBox3f& box = graph1->getBoundingBox();

    int numMT = graph1->getNumEdges();

    McDArray< bool > regionFilter(numMT * 2);

    float zMin = box[5] - portMatchingRegion.getValue() * (box[5] - box[4]);
    float zMax = box[5];

    for (int i = 0; i < numMT; ++i)
    {
        McVec3f p = graph1->getEdgePoint(i, 0);
        McVec3f q = graph1->getEdgePoint(i, graph1->getNumEdgePoints(i) - 1);

        if (p.z >= zMin && p.z <= zMax) regionFilter[i * 2]     = false;
        else                            regionFilter[i * 2]     = true;
        if (q.z >= zMin && q.z <= zMax) regionFilter[i * 2 + 1] = false;
        else                            regionFilter[i * 2 + 1] = true;
    }

    // prepare check array

    mCheckEnds.remax(numMT * 2, 0);

    for (int i = 0; i < numMT; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            if (regionFilter[i * 2 + j]) continue;

            mCheckMax++;
            mCheckEnds.push(i * 2 + j);

            if (matching.getState(i, j, 0) != HxSerialSectionStack::UNDEFINED &&
                matching.getState(i, j, 0) != HxSerialSectionStack::MATCHED_AUTOMATIC)
            {
                mCheckCur++;
            }
        }
    }

    if (mCheckCur < mCheckMax)
    {
        portMatchingCheck.setSensitivity(0, 1);
        portMatchingCheck.setSensitivity(1, 1);
    }
    else
    {
        portMatchingCheck.setSensitivity(0, 0);
        portMatchingCheck.setSensitivity(1, 0);
    }

    portMatchingCheck.setSensitivity(2, 0);

    computeCheckField();

    updateCheckLabel();
}




void HxSerialSectionAligner::updateCheckCurrent()
{
    if (!mSerialSectionStack) return;

    Matching& matching = mSerialSectionStack->getSectionInterface(portSectionInterface.getValue()).mMatching;

    mCheckCur = 0;

    for (int i = 0; i < mCheckEnds.size(); ++i)
    {
        int mID = mCheckEnds[i] / 2;
        int eID = mCheckEnds[i] % 2;

        if (matching.getState(mID, eID, 0) != HxSerialSectionStack::UNDEFINED &&
            matching.getState(mID, eID, 0) != HxSerialSectionStack::MATCHED_AUTOMATIC)
        {
            mCheckCur++;
        }
    }

    updateCheckLabel();
}



void HxSerialSectionAligner::updateCheckLabel()
{
    QString maxValue = QString::number(mCheckMax);
    QString curValue = QString::number(mCheckCur);

    QString label    = curValue + "/" + maxValue;

    portMatchingCheck.setValue(3, label);
}




void HxSerialSectionAligner::updateResolutionLabel(bool initialize)
{
    if (!mSerialSectionStack) return;

    if (initialize)
    {
        const McBox3f box  = mSerialSectionStack->getSection(0).mImageBoundingBox;
        const McDim3l dims = mSerialSectionStack->getSection(0).mImageDimensions;

        float v = (float) (box[1] - box[0]) / (dims[0] - 1);

        portComputeStackResolution.setValue(0, v);
    }

    // compute size

    mclong numBytes  = 0;
    int    numSec    = mSerialSectionStack->getNumSections();
    double voxelSize = (double) portComputeStackResolution.getValue(0);

    for (int i = 0; i < numSec; ++i)
    {
        const McBox3f box  = mSerialSectionStack->getSection(i).mImageBoundingBox;
        const McDim3l dims = mSerialSectionStack->getSection(i).mImageDimensions;

        numBytes += (mclong) ((1.1 * ((box[1] - box[0]) / voxelSize + 1.0)) *
                              (1.1 * ((box[3] - box[4]) / voxelSize + 1.0)) *
                              (double) dims[2] + 1.0);
    }

    numBytes /= 1048576;

    QString size  = " MB";

    if (numBytes > 10240)
    {
        numBytes /= 1024;
        size      = " GB";
    }

    QString numMB = QString::number(numBytes);
    QString label = numMB + size;

    portComputeStackResolution.setValue(1, label);
}



