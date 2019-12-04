#ifndef HXSPINDLE_FILAMENTGRID_H
#define HXSPINDLE_FILAMENTGRID_H


#include <hxspindle/api.h>
#include <hxcore/HxMessage.h>
#include <hxspatialgraph/internal/HxSpatialGraph.h>




class HXSPINDLE_API FilamentGrid
{

    // cell class

    public:


        class Cell
        {
            public:


                Cell();


            public:


                void add(int elementID, bool checkForDuplicates = false);




                void clear();




                void getElements(McDArray< int >& elements) const;


            private:


                McDArray< int > mElements;
        };


    // Construction / Deconstruction

    public:


        FilamentGrid(void);




        FilamentGrid(const HxSpatialGraph& graph, float cellSize);


    //Methods

    public:


        void add(const HxSpatialGraph& graph);




        void add(const HxSpatialGraph& graph, const McDArray< int >& selection);




        void add(const McVec3f& segmentStart, const McVec3f& segmentEnd, int segmentID);




        void clear();




        void getNeighbors(const McDArray< McVec3f >& filament, float radius, McDArray< int >& neighbors) const;




        void initialize(const HxSpatialGraph& graph, float cellSize);




        void initialize(const HxSpatialGraph& graph, const McDArray< int >& selection, float cellSize);


    private:


        int coordToCell(int x, int y, int z) const;




        int coordToCell(const McVec3f& position) const;




        void coordToCell(const McVec3f& position, int& x, int& y, int& z) const;




        void getNeighbors(const McVec3f& boxMin, const McVec3f& boxMax, McDArray< int >& neighbors) const;




        bool segCellIntersection(McVec3f& segmentStart, const McVec3f& segmentEnd, McVec3i& cell) const;




        void validateCell(int& x, int& y, int& z) const;




        bool validCell(int x, int y, int z) const;


    // Attributes

    private:


        McDArray< Cell > mCells;
        McVec3f          mCellSize;
        int              mDimensions[3];
        McVec3f          mGridMax;
        McVec3f          mGridMin;



};


#endif

