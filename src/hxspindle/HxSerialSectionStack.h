#pragma once


#include "api.h"
#include <hxcore/HxSpatialData.h>
#include <hxcore/HxPortInfo.h>
#include <hxcore/HxPortButtonList.h>
#include <hxcore/HxTightConnection.h>

#include <hxalignmicrotubules/mtalign.h>
#include <hxalignmicrotubules/mtalign/cpd.h>
#include <hxalignmicrotubules/MovingLeastSquares.h>

#include <hxspatialgraph/internal/HxSpatialGraph.h>
#include <amiramesh/AmiraMesh.h>
#include <mclib/McDim3l.h>



class HxUniformScalarField3;





/** This is a container for HxSpatialData that are stored on disk.
    With the add button you can select files on disk. They will be loaded
    and all HxSpatialData that are created are added to the HxMosaic.
    The Mosaic stores Filename, BoundingBox, Range, TypeId, Transform,
    PrimType, dims and NDataVar of the sections as parameters.
    They can be accessed via getBrickParameters ().
  */
class HXSPINDLE_API HxSerialSectionStack : public HxSpatialData {

    HX_HEADER (HxSerialSectionStack);


    // sub-classes

    public:


        enum SettingType
        {
            MANUAL      = 0,
            AUTOMATIC   = 1,
            TEMPORARY   = 2
        };




        enum FilamentEndState
        {
            UNDEFINED         = 0,
            MATCHED           = 1,
            MATCHED_AUTOMATIC = 11,
            UNMATCHED         = 2
        };




        /**
            Class Section

            Holds the Tomogram and the Filaments of the section.
            The Tomogram is only stored by the bounding box, the
            dimensions, and the filename. The image itself is
            loaded on demand.
        */
        class Section
        {
            public:


                /**
                    Default Constructor. Sets an empty section
                    (empty image name and empty filaments).
                */
                Section();


                /**
                    Loads and returns the image as handle. Note that the
                    image is loaded from hard disk.
                */
                McHandle< HxUniformScalarField3 > getImage() const;


            public:


                McHandle< HxSpatialGraph >  mFilaments;
                McBox3f                     mImageBoundingBox;
                McDim3l                     mImageDimensions;
                QString                     mImageFileName;
        };




        /**
            Class Landmarks.

            Describes a set of landmarks between two neighboring sections.
            A landmark is a pair of points (one in each section) that
            correspond to each other. The points are defined in the 2D
            x-y domain. For each landmark an additional flag is stored
            that holds the information if the landmark is manually,
            automatically or temporary placed.
        */
        class Landmarks
        {
            public:


                /**
                    Adds a new landmark by two points in the neighboring
                    sections and a setting type.
                */
                void addMark(const McVec2f& point1, const McVec2f& point2, const SettingType type);


                /**
                    Adds a new landmark by two points in the neighboring
                    sections and a setting type. Hint: The point coordinates
                    are casted to the internal float format.
                */
                void addMark(const McVec2d& point1, const McVec2d& point2, const SettingType type);


                /**
                    Removes all landmarks.
                */
                void clear();


                /**
                    Removes a single landmark (one pair of points).
                */
                void deleteMark(int markID);


                /**
                    Removes all landmarks of a certain setting type.
                */

                void deleteMarks(const SettingType type);


                /**
                    Returns one point of a certain landmark, where section
                    defines which point (0 bottom, 1 top)
                */
                const McVec2f& getMark(int markID, int section) const;


                /**
                    Returns all points of the landmarks, where section
                    defines which points (0 bottom, 1 top)
                */
                const McDArray< McVec2f >& getMarks(int section) const;


                /**
                    Returns the overall number of landmarks.
                */
                int getNumMarks() const;


                /**
                    Returns the number of landmarks of a certain setting type.
                */
                int getNumMarks(SettingType type) const;


                /**
                    Returns the setting type of a landmark
                */
                SettingType getType(int markID) const;


                /**
                    Changes the position of one point of a certain landmark,
                    where section defines which point (0 bottom, 1 top).
                */
                void setMark(const McVec2f& position, int markID, int section);


                /**
                    Sets the number of landmarks. The potential new landmarks
                    have undefined values.
                */
                void setNumMarks(const int numMarks);


                /**
                    Sets the type of a certain landmark
                */
                void setType(const int markID, const SettingType type);


                /**
                    Initializes the moving least squares with the landmarks as
                    control points for the warping.
                */
                void setupMLS(MovingLeastSquares& mls) const;


                /**
                    Transforms the points based on the current landmarks.
                */
                void transform(mtalign::FacingPointSets& points) const;


                /**
                    Transforms the points based on the current landmarks.
                */
                void transformInv(mtalign::WarpResult& warpResult) const;


            private:


                McDArray< McVec2f >     mMarks[2];
                McDArray< SettingType > mTypes;
        };




        /**
            Class Matching

            Stores a set of matches between filament ends of neighboring
            sections. A match is defined by a pair of 2D integer vectors.
            The first indicates the filament ID and the second, the end ID,
            where 0 means the start point of the filament and 1 the
            end point.
        */
        class Matching
        {
            public:


                /**
                    Adds a match to the data set. A filament end is
                    defined here by two integers. The first is the edge
                    ID in the spatial graph and the second is 0 or 1.
                    0 for the start of the edge and 1 for the end of the
                    edge. The type is set to MANUAL and the color is
                    computed randomly according to its type.
                */
                void add(const McVec2i& filamentEnd1, const McVec2i& filamentEnd2);


                /**
                    Adds a match to the data set. A filament end is
                    defined here by two integers. The first is the edge
                    ID in the spatial graph and the second is 0 or 1.
                    0 for the start of the edge and 1 for the end of the
                    edge.
                */
                void add(const McVec2i& filamentEnd1, const McVec2i& filamentEnd2, const McVec3f& color, const SettingType type);


                /**
                    Removes all matches.
                */
                void clear();


                /**
                    Returns a certain match.
                */
                const McVec4i& get(int matchID) const;


                /**
                    Returns the color for a certain match.
                */
                const McVec3f& getColor(int matchID) const;


                /**
                    Returns the number of matches in the matching.
                */
                int getNumMatches() const;

                
                /**
                    Returns the number of end states for the section.
                */
                int getNumStates(int section) const;


                /**
                    Returns the type of a match.
                */
                SettingType getType(int matchID) const;


                /**
                    Returns the state of a filament end.
                */
                FilamentEndState getState(int filamentID, int end, int section) const;


                /**
                    Returns true if an automatically setting of a landmark is
                    forbidden for this end.
                */
                bool isLandmarkForbidden(int filamentID, int end, int section) const;


                /**
                    Returns true if one of the ends is forbidden to set a landmark.
                */
                bool isLandmarkForbidden(int matchID) const;


                /**
                    Prints the matching in the console. Only for debugging.
                */
                void print();


                /**
                    Removes a match.
                */
                void remove(int matchID);


                /**
                    Removes all matches of a certain setting type.
                */
                void remove(SettingType type);


                /**
                    Sets the state of a filament end
                */
                void setState(FilamentEndState state, int filamentID, int end, int section);


                /**
                    Returns the type of a match.
                */
                void setType(SettingType type, int matchID);


                /**
                    Sets the matching of the filaments based on the PGM matching
                    of the end points. Selection contains two arrays with the filament
                    indices of the reference end points and the target end points.
                */
                void set(const McDArray< McDArray < McVec2i > >& selection,  const mtalign::MatchingPGM& matching);


                /**
                    Sets the number of end states
                */
                void setNumStates(int section, int numStates);


                /**
                    Changes the landmark forbidden state of a filament end.
                */
                void switchForbiddenState(int filamentID, int end, int section);


                /**
                    Updates the end states of the filaments for checking the matching.
                */
                void updateEndStates(const HxSpatialGraph* graph1, const HxSpatialGraph* graph2);


            private:


                /**
                    Computes a color for a match based on the setting type.
                */
                McVec3f computeColor(SettingType type) const;


            private:


                McDArray< McVec4i >          mMatches;
                McDArray< McVec3f >          mMatchColors;
                McDArray< SettingType >      mMatchTypes;

                McDArray< FilamentEndState > mFilamentEndState[2];
                McDArray< bool >             mFilamentEndForbidLandmarks[2];
        };




        /**
            Class SectionInterface

            Describes the interface between two neighboring sections. Each
            interface consists of a set of landmarks, that describe the
            deformation of the sections in x-y direction plus a matching
            which is the combinatorial connection between the filaments
            of the two sections.
        */
        class SectionInterface
        {
            public:


                Landmarks mLandmarks;
                Matching  mMatching;
        };


    // methods

    public:


        /**
            Does all interaction with the data object.
        */
        virtual void compute ();


        /**
            Creates the complete serial section stack into one spatial
            graph and one field. Both structures are aligned and matched
            based on the current interface. The voxel size in x-y can be
            changed to reduce the size of the overall field. The center
            section is not deformed and all others are aligned to this
            one.
        */
        void createStack(HxSpatialGraph& filaments, HxUniformScalarField3& field, float voxelSizeXY, int centerSectionID);



        /**
            Returns the overall bounding box.
        */
        virtual void getBoundingBox (float bbox[6]) const;


        /**
            Returns the number of bricks.
        */
        int getNumSections();


        /**
            Returns a section.
        */
        const Section& getSection(int sectionID) const;


        /**
            Returns the information for a section interface.
        */
        SectionInterface& getSectionInterface(int sectionInterface);


        /**
            Loads section data
        */
        McHandle<HxSpatialData> loadSectionData (const char* filename);


        /**
            Parse for TCL interpretation.
        */
        virtual int parse(Tcl_Interp* t, int argc, char **argv);


        /**
            Reads a serial section stack from file.
        */
        static int readAmiraMesh (AmiraMesh* m, const char* filename);


        /**
            Removes a section.
        */
        void removeSection(int sectionID);


        /**
            Saves a serial section stack in an AmiraMesh file.
        */
        int saveAmiraMesh(const char* filename);


        /**
            Updates the module.
        */
        virtual void update ();


    private:


        /**
            Adds an empty section to the stack and returns its ID.
        */
        int addSection();


        /**
            Adds or updates section data given by the
            section ID. If the section ID is -1 the
            data will be added to the end of the list.
        */
        void addSectionData(McHandle<HxSpatialData> data, const int sectionID);


        /**
            Adds or updates filaments given by the section ID.
            If the section ID is -1 the data will be added to
            the end of the list.
        */
        void setFilaments(McHandle<HxSpatialGraph> graph, const int sectionID);


        /**
            Adds or updates image data given by the section ID.
            If the section ID is -1 the data will be added to
            the end of the list.
        */
        void setImage(McHandle<HxUniformScalarField3> image, const int sectionID);


        /**
            Touches all port data of all modules connected to the stack.
        */
        void touchConnections();


        /**
            Updates the info port, which shows the tomogram data
        */
        void updateInfo();


    // ports

    public:


        HxPortButtonList portAction;
        HxPortInfo       portInfo;


    // attributes

    private:


        McDArray< Section >          mSections;
        McDArray< SectionInterface > mSectionInterfaces;
};

