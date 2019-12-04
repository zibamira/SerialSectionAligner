#include "Test_GreedyPointMatching.h"
#include "Test_ExactPointMatching.h"
#include "HxTest_PointMatching.h"

HX_INIT_CLASS(HxTest_PointMatching, HxCompModule);

HxTest_PointMatching::HxTest_PointMatching() :
    HxCompModule(HxData::getClassTypeId()),
    portAction(this, "action", tr("action"), 1)
{
    portAction.setLabel("Action");
    portAction.setLabel(0, "DoIt");
}

HxTest_PointMatching::~HxTest_PointMatching()
{}

void HxTest_PointMatching::compute()
{
    if ( portAction.wasHit() )
    {
        Test_GreedyPointMatching::test1();
        Test_ExactPointMatching::test1();

        Test_GreedyPointMatching::test2();
        Test_ExactPointMatching::test2();

        Test_GreedyPointMatching::test3();
        Test_ExactPointMatching::test3();

        Test_GreedyPointMatching::test4();
        Test_ExactPointMatching::test4();

        Test_GreedyPointMatching::test5();
        Test_ExactPointMatching::test5();
    }
}
