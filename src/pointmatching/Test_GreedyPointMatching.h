#ifndef TEST_GREEDY_POINT_MATCHING_H
#define TEST_GREEDY_POINT_MATCHING_H

#include <mclib/McDArray.h>
#include <mclib/McVec3.h>

class Test_GreedyPointMatching
{
public:
    static bool test1();
    static bool test2();
    static bool test3();
    static bool test4();
    static bool test5();

    static bool test(const McDArray< McVec3f > & coords1,
		     const McDArray< McVec3f > & coords2);
};

#endif
