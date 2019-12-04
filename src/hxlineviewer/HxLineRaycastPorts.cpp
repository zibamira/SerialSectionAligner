#include <hxlineviewer/HxLineRaycast.h>

#include <hxlines/internal/HxLineSetInterface.h>
#include <hxspatialgraph/internal/HxSpatialGraphInterface.h>

#include <sstream>
#include <iterator>


namespace
{
    bool
    identicalAttributes(const McDArray< SoLineRaycast::SpatialGraphAttribute >& att1,
                        const McDArray< SoLineRaycast::SpatialGraphAttribute >& att2)
    {
        if (att1.size() != att2.size())
            return false;

        for (int i = 0; i < att1.size(); ++i)
        {
            QString name1(att1[i].mName);
            QString name2(att2[i].mName);

            if (name1 != name2)
                return false;
        }

        return true;
    }
}

void
HxLineRaycast::deletePortContent()
{
    mPortLineRadiusData.deleteItem(0);
    mPortLineColorStyle.deleteItem(0);
    mPortLineColorStyle.deleteItem(1);

    mPortEndingRadiusData.deleteItem(0);
    mPortEndingRadiusData.deleteItem(1);
    mPortEndingColorStyle.deleteItem(0);
    mPortEndingColorStyle.deleteItem(1);
    mPortEndingColorStyle.deleteItem(2);

    mPortNodeRadiusData.deleteItem(0);
    mPortNodeColorStyle.deleteItem(0);
    mPortNodeColorStyle.deleteItem(1);
    mPortNodeColorStyle.deleteItem(2);
}

void
HxLineRaycast::initPorts()
{
    HxLineSetInterface*      lineSet = hxconnection_cast<HxLineSetInterface>(portData);
    HxSpatialGraphInterface* spatialGraph = hxconnection_cast<HxSpatialGraphInterface>(portData);

    // renaming ports depending on input (only when necessary)

    if (lineSet && !mLineSetInterface)
    {
        mPortDisplay.setLabel(0, "lines");
        mPortDisplay.setLabel(1, "line ending");
        mPortDisplay.setLabel(2, "line nodes");

        mPortLineRadiusData.setLabel("Line radius");
        mPortLineRadiusScale.setLabel("Line radius scale");
        mPortLineColorStyle.setLabel("Line color");
        mPortLineColorConstant.setLabel("Line color");
        mPortLineColorMap.setLabel("Line colormap");
        mPortLineOptions.setLabel("Line options");
        mPortLineBendAngle.setLabel("Line bend angle");

        mPortEndingRadiusData.setLabel("Ending radius");
        mPortEndingRadiusScale.setLabel("Ending radius scale");
        mPortEndingColorStyle.setLabel("Ending color");
        mPortEndingColorConstant.setLabel("Ending color");
        mPortEndingColorMap.setLabel("Ending colormap");

        mPortNodeRadiusData.setLabel("Node radius");
        mPortNodeRadiusScale.setLabel("Node radius scale");
        mPortNodeColorStyle.setLabel("Node color");
        mPortNodeColorConstant.setLabel("Node color");
        mPortNodeColorMap.setLabel("Node colormap");
    }
    else if (spatialGraph && !mSpatialGraphInterface)
    {
        mPortDisplay.setLabel(0, "edges");
        mPortDisplay.setLabel(1, "nodes");
        mPortDisplay.setLabel(2, "points");

        mPortLineBendAngle.setLabel("Edge bend angle");
        mPortLineColorMap.setLabel("Edge colormap");
        mPortLineColorStyle.setLabel("Edge color");
        mPortLineColorConstant.setLabel("Edge color");
        mPortLineOptions.setLabel("Edge options");
        mPortLineRadiusData.setLabel("Edge radius");
        mPortLineRadiusScale.setLabel("Edge radius scale");

        mPortNodeColorMap.setLabel("Point colormap");
        mPortNodeColorStyle.setLabel("Point color");
        mPortNodeColorConstant.setLabel("Point color");
        mPortNodeRadiusData.setLabel("Point radius");
        mPortNodeRadiusScale.setLabel("Point radius scale");

        mPortEndingColorMap.setLabel("Node colormap");
        mPortEndingColorConstant.setLabel("Node color");
        mPortEndingColorStyle.setLabel("Node color");
        mPortEndingRadiusData.setLabel("Node radius");
        mPortEndingRadiusScale.setLabel("Node radius scale");
    }

    // init special ports

    if (lineSet)
    {
        initForLineSetInterface(lineSet);
    }
    else if (spatialGraph)
    {
        initSegmentAndPointPortsForSpatialGraph(spatialGraph);
        initEndingPortsForSpatialGraph(spatialGraph);
    }
}

void
HxLineRaycast::initForLineSetInterface(HxLineSetInterface* newLineSet)
{
    int numDataValues = newLineSet->getNumDataValues();

    // previous input was a line set with the same number of
    // data values => GUI entries do not change

    if (mLineSetInterface && mLineSetDataValues == numDataValues)
    {
        return;
    }

    // store old adaption state

    int adaptEndingRad = 1;
    int adaptEndingCol = 1;
    int adaptNodeCol   = 1;

    if (mLineSetInterface)
    {
        adaptEndingRad = mPortEndingRadiusData.getValue(0);
        adaptEndingCol = mPortEndingColorStyle.getValue(0);
        adaptNodeCol   = mPortNodeColorStyle.getValue(0);
    }

    // remove old GUI and create new

    deletePortContent();

    QStringList labels;

    labels.append(QString("constant"));

    for (int i = 0; i < numDataValues; ++i)
    {
        labels.append(QString("data value %1").arg(i));
    }

    mPortLineRadiusData.insertComboBox(0, labels);
    mPortLineColorStyle.insertComboBox(0, labels);

    mPortEndingRadiusData.insertCheckBox(0, "adapt to lines");
    mPortEndingRadiusData.insertComboBox(1, labels);
    mPortEndingRadiusData.setValue(0, adaptEndingRad);
    mPortEndingColorStyle.insertCheckBox(0, "adapt to lines");
    mPortEndingColorStyle.insertComboBox(1, labels);
    mPortEndingColorStyle.setValue(0, adaptEndingCol);

    mPortNodeRadiusData.insertComboBox(0, labels);
    mPortNodeColorStyle.insertCheckBox(0, "adapt to lines");
    mPortNodeColorStyle.insertComboBox(1, labels);
    mPortNodeColorStyle.setValue(0, adaptNodeCol);

    mLineSetDataValues = numDataValues;

    mPortLineRadiusData.touch();
    mPortLineColorStyle.touch();
    mPortEndingRadiusData.touch();
    mPortEndingColorStyle.touch();
    mPortNodeRadiusData.touch();
    mPortNodeColorStyle.touch();
}

void
HxLineRaycast::initEndingPortsForSpatialGraph(HxSpatialGraphInterface* newSpatialGraph)
{
    int radiusAttId = mPortEndingRadiusData.getValue(1);
    int colorAttId = mPortEndingColorStyle.getValue(1);

    mEndingColorAtt.clear();
    mEndingRadiusAtt.clear();

    // collect all vertex attributes

    McDArray< SpatialGraphAttribute > colorAtt;
    McDArray< SpatialGraphAttribute > radiusAtt;

    int numVertexAtt = newSpatialGraph->getNumVertexAttributes();

    for (int i = 0; i < numVertexAtt; ++i)
    {
        const EdgeVertexAttribute* att = newSpatialGraph->getVertexAttribute(i);

        if (att->primType() != McPrimType::MC_FLOAT &&
            att->primType() != McPrimType::MC_INT32)
            continue;

        colorAtt.append(SpatialGraphAttribute((GraphAttribute*) att));

        if (newSpatialGraph->hasLabel(att))
            continue;

        radiusAtt.append(SpatialGraphAttribute((GraphAttribute*) att));
    }

    // test if GUI update is necessary

    if (mSpatialGraphInterface &&
        identicalAttributes(mEndingColorAtt, colorAtt) &&
        identicalAttributes(mEndingRadiusAtt, radiusAtt))
    {
        mEndingColorAtt = colorAtt;
        mEndingRadiusAtt = radiusAtt;

        return;
    }

    // store old state

    int endLabel = 1;

    if (mSpatialGraphInterface)
    {
        endLabel = mPortEndingColorStyle.getValue(2);
    }

    // remove old GUI elements

    mPortEndingRadiusData.deleteItem(0);
    mPortEndingRadiusData.deleteItem(1);
    mPortEndingColorStyle.deleteItem(0);
    mPortEndingColorStyle.deleteItem(1);
    mPortEndingColorStyle.deleteItem(2);

    // GUI needs to be updated

    mEndingColorAtt = colorAtt;
    mEndingRadiusAtt = radiusAtt;

    QStringList radLabels;
    QStringList colLabels;

    radLabels.append(QString("constant"));
    colLabels.append(QString("constant"));


    for (int i = 0; i < mEndingRadiusAtt.size(); ++i)
        radLabels.append(mEndingRadiusAtt[i].mName);

    for (int i = 0; i < mEndingColorAtt.size(); ++i)
        colLabels.append(mEndingColorAtt[i].mName);

    mPortEndingRadiusData.insertComboBox(1, radLabels);
    mPortEndingRadiusData.setValue(1, radiusAttId);
    mPortEndingColorStyle.insertComboBox(1, colLabels);
    mPortEndingColorStyle.insertCheckBox(2, "label");
    mPortEndingColorStyle.setValue(1, colorAttId);
    mPortEndingColorStyle.setValue(2, endLabel);
    mPortEndingColorStyle.setSensitivity(2, 0);

    mPortEndingRadiusData.touch();
    mPortEndingColorStyle.touch();
}

void
HxLineRaycast::initSegmentAndPointPortsForSpatialGraph(HxSpatialGraphInterface* newSpatialGraph)
{
    int numEdgeAtt = newSpatialGraph->getNumEdgeAttributes();
    int numPointAtt = newSpatialGraph->getNumEdgePointAttributes();

    // collect all necessary edge attributes

    McDArray< SpatialGraphAttribute > segmentColorAtt;
    McDArray< SpatialGraphAttribute > segmentRadiusAtt;

    for (int i = 0; i < numEdgeAtt; ++i)
    {
        const EdgeVertexAttribute* att = newSpatialGraph->getEdgeAttribute(i);

        if (att->primType() != McPrimType::MC_FLOAT &&
            att->primType() != McPrimType::MC_INT32)
            continue;

        segmentColorAtt.push_back(SpatialGraphAttribute((GraphAttribute*) att));

        if (newSpatialGraph->hasLabel(att))
            continue;

        segmentRadiusAtt.push_back(SpatialGraphAttribute((GraphAttribute*) att));
    }

    // collect all necessary point attributes

    McDArray< SpatialGraphAttribute > nodeColorAtt;
    McDArray< SpatialGraphAttribute > nodeRadiusAtt;

    for (int i = 0; i < numPointAtt; ++i)
    {
        const PointAttribute* att = newSpatialGraph->getEdgePointAttribute(i);

        if (att->primType() != McPrimType::MC_FLOAT &&
            att->primType() != McPrimType::MC_INT32)
            continue;

        nodeColorAtt.push_back(SpatialGraphAttribute((GraphAttribute*) att));

        if (newSpatialGraph->hasLabel(att))
            continue;

        nodeRadiusAtt.push_back(SpatialGraphAttribute((GraphAttribute*) att));
    }

    // test if GUI needs to be updated

    if (mSpatialGraphInterface &&
        identicalAttributes(mSegmentColorAtt, segmentColorAtt) &&
        identicalAttributes(mSegmentRadiusAtt, segmentRadiusAtt) &&
        identicalAttributes(mNodeColorAtt, nodeColorAtt) &&
        identicalAttributes(mNodeRadiusAtt, nodeRadiusAtt))
    {
        mSegmentColorAtt = segmentColorAtt;
        mSegmentRadiusAtt = segmentRadiusAtt;
        mNodeColorAtt = nodeColorAtt;
        mNodeRadiusAtt = nodeRadiusAtt;

        return;
    }

    theMsg->printf("seg and nodes update");

    mSegmentColorAtt = segmentColorAtt;
    mSegmentRadiusAtt = segmentRadiusAtt;
    mNodeColorAtt = nodeColorAtt;
    mNodeRadiusAtt = nodeRadiusAtt;

    // save old state

    int segLabel = 1;
    int nodeEdges = 1;
    int nodeLabel = 1;

    if (mSpatialGraphInterface)
    {
        segLabel = mPortLineColorStyle.getValue(1);
        nodeEdges = mPortNodeColorStyle.getValue(0);
        nodeLabel = mPortNodeColorStyle.getValue(2);
    }

    // remove old GUI

    mPortLineRadiusData.deleteItem(0);
    mPortLineColorStyle.deleteItem(0);
    mPortLineColorStyle.deleteItem(1);

    mPortNodeRadiusData.deleteItem(0);
    mPortNodeColorStyle.deleteItem(0);
    mPortNodeColorStyle.deleteItem(1);
    mPortNodeColorStyle.deleteItem(2);

    // create new GUI elements

    int attSegCol = mSegmentColorAtt.size();
    int attSegRad = mSegmentRadiusAtt.size();
    int attNoCol = mNodeColorAtt.size();
    int attNoRad = mNodeRadiusAtt.size();

    QStringList radLabels;
    QStringList colLabels;

    radLabels.append(QString("constant"));
    colLabels.append(QString("constant"));

    for (int i = 0; i < attSegRad; ++i)
    {
        radLabels.append(QString("edge: %1").arg(mSegmentRadiusAtt[i].mName));
    }
    for (int i = 0; i < attNoRad; ++i)
    {
        radLabels.append(QString("point: %1").arg(mNodeRadiusAtt[i].mName));
    }

    for (int i = 0; i < attSegCol; ++i)
    {
        colLabels.append(QString("edge: %1").arg(mSegmentColorAtt[i].mName));
    }
    for (int i = 0; i < attNoCol; ++i)
    {
        colLabels.append(QString("point: %1").arg(mNodeColorAtt[i].mName));
    }

    mPortLineRadiusData.insertComboBox(0, radLabels);
    mPortLineColorStyle.insertComboBox(0, colLabels);
    mPortLineColorStyle.insertCheckBox(1, "label");
    mPortLineColorStyle.setValue(1, segLabel);
    mPortLineColorStyle.setSensitivity(1, 0);

    mPortNodeRadiusData.insertComboBox(0, radLabels);
    mPortNodeColorStyle.insertComboBox(1, colLabels);
    mPortNodeColorStyle.insertCheckBox(0, "edges");
    mPortNodeColorStyle.insertCheckBox(2, "label");
    mPortNodeColorStyle.setValue(0, nodeEdges);
    mPortNodeColorStyle.setValue(2, nodeLabel);
    mPortNodeColorStyle.setSensitivity(2, 0);

    mPortLineRadiusData.touch();
    mPortLineColorStyle.touch();
    mPortNodeRadiusData.touch();
    mPortNodeColorStyle.touch();
}

void
HxLineRaycast::updateSelectionPorts()
{
    const McDArray<int>& edges = getSelectedEdges();
    std::stringstream buff;
    std::copy(edges.begin(), edges.end(), std::ostream_iterator<int>(buff, " "));
    mPortEdgeSelection.setValue(QString::fromLatin1(buff.str().c_str()));
}

void
HxLineRaycast::updateEndingColorPorts()
{
    if (mPortEndingColorStyle.isNew())
    {
        // handling adaption: only true for line sets
        if (mPortEndingColorStyle.getValue(0) && mAdaptLines)
        {
            mPortEndingColorStyle.setSensitivity(1, 0);
            mPortEndingColorConstant.hide();
            mPortEndingColorMap.hide();
        }
        // handling color show/hide
        else if (mPortDisplay.getValue(1))
        {
            int att = mPortEndingColorStyle.getValue(1);

            mPortEndingColorStyle.setSensitivity(1, 1);
            mPortEndingColorStyle.setSensitivity(2, 0);
            mPortEndingColorMap.hide();
            mPortEndingColorConstant.hide();

            // line set and att
            if (mLineSetInterface && att == 0)
                mPortEndingColorConstant.show();
            else if (mLineSetInterface && att > 0)
                mPortEndingColorMap.show();

            // spatial graph and att
            if (mSpatialGraphInterface && att == 0)
                mPortEndingColorConstant.show();
            else if (mSpatialGraphInterface && att > 0 && mSpatialGraphInterface->hasLabel(mEndingColorAtt[att - 1].mAttribute))
            {
                mPortEndingColorStyle.setSensitivity(2, 1);
                if (!mPortEndingColorStyle.getValue(2))
                    mPortEndingColorMap.show();
            }
            else if (att > 0)
                mPortEndingColorMap.show();
        }
    }
}

void
HxLineRaycast::updateLineColorPorts()
{
    if (mPortLineColorStyle.isNew())
    {
        int att = mPortLineColorStyle.getValue(0);
        int ns = mSegmentColorAtt.size();

        mPortLineColorStyle.setSensitivity(1, 0);
        mPortLineColorConstant.hide();
        mPortLineColorMap.hide();

        // line set and att
        if (mLineSetInterface)
        {
            if (att == 0)
                mPortLineColorConstant.show();
            else
                mPortLineColorMap.show();
        }

        // spatial graph and att
        if (mSpatialGraphInterface)
        {
            if (att == 0)
                mPortLineColorConstant.show();
            else if ((att - 1 < ns && att > 0 && mSpatialGraphInterface->hasLabel(mSegmentColorAtt[att - 1].mAttribute)) ||
                     (att - 1 >= ns && att > 0 && mSpatialGraphInterface->hasLabel(mNodeColorAtt[att - 1 - ns].mAttribute)))
            {
                mPortLineColorStyle.setSensitivity(1, 1);
                if (!mPortLineColorStyle.getValue(1))
                    mPortLineColorMap.show();
            }
            else if (att > 0)
                mPortLineColorMap.show();
        }
    }
}

void
HxLineRaycast::updateNodeColorPorts()
{
    // care about node color port
    if (mPortNodeColorStyle.isNew())
    {
        if (!mPortDisplay.getValue(2))
            return;

        bool adapt = mPortNodeColorStyle.getValue(0) && mPortNodeColorStyle.getSensitivity(0);
        int att = mPortNodeColorStyle.getValue(1);
        int ns = mSegmentColorAtt.size();

        mPortNodeColorStyle.setSensitivity(2, 0);
        mPortNodeColorStyle.setSensitivity(1, 0);
        mPortNodeColorConstant.hide();
        mPortNodeColorMap.hide();

        if (mLineSetInterface)
        {
            if (!adapt)
                mPortNodeColorStyle.setSensitivity(1, 1);
            if (att == 0 && !adapt)
                mPortNodeColorConstant.show();
            else if (att > 0 && !adapt)
                mPortNodeColorMap.show();
        }
        if (mSpatialGraphInterface)
        {
            if (!adapt)
                mPortNodeColorStyle.setSensitivity(1, 1);
            if (att == 0 && !adapt)
                mPortNodeColorConstant.show();
            else if (att > 0 && !adapt)
            {
                if ((att - 1 < ns && mSpatialGraphInterface->hasLabel(mSegmentColorAtt[att - 1].mAttribute)) ||
                    (att - 1 >= ns && mSpatialGraphInterface->hasLabel(mNodeColorAtt[att - 1 - ns].mAttribute)))
                {
                    mPortNodeColorStyle.setSensitivity(2, 1);
                    if (!mPortNodeColorStyle.getValue(2))
                        mPortNodeColorMap.show();
                }
                else
                    mPortNodeColorMap.show();
            }
        }
    }
}

void
HxLineRaycast::updateRadiusScalePort(HxPortFloatSlider& slider, float max)
{
    if ((bool)mPortSetting.getValue(0) && !mEmpty)
    {
        slider.setMinMax(0.0, max);
        slider.setValue(0.25 * max);
        slider.touch();
    }
}
