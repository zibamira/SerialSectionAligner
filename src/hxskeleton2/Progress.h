#ifndef DE_ZIB_MC_PROGRESS_H
#define DE_ZIB_MC_PROGRESS_H

#include <hxcore/internal/HxWorkArea.h>
#include <mclib/internal/McAssert.h>
#include <mclib/McException.h>

namespace de_zib_mc
{
    /** RAII/Exception interface to McProgressBar.

    Your code must be exception safe. SetProgress() may throw CanceledException.

    Example
\code
void DoSomeWork () {
    ProgressReporter progress ("doing some work ...");

    // ...
    for (...) {
        progress.SetProgress (float (i) / float (nnn));  // may throw CanceledException
        // ...
    }
}
\endcode
     
     \todo TODO: should be independent of amira
         implementation ideas
              - Thread Local storage + ProgressInterface?
              - global thread safe reporting facility?
         But we don't care for now, because everything's inline, 
         Nonetheless, if you're using ProgressReporter you must link with Amira
 */
    class ProgressReporter
    {
    public:
        ProgressReporter()
        {
            theWorkArea->startWorking("working ...");
        }

        explicit ProgressReporter(const char* task)
        {
            theWorkArea->startWorking(QString::fromLatin1(task));
        }

        /** \throw de_zib_mc::CanceledException
          */
        void
        SetProgress(float p)
        {
            theWorkArea->setProgressValue(p);
            if (theWorkArea->wasInterrupted())
            {
                mcthrow("user interrupted operation");
            }
        }

        ~ProgressReporter()
        {
            theWorkArea->stopWorking();
        }
    };

    class ProgressSubTasks
    {
    public:
        explicit ProgressSubTasks(int n)
        {
            theWorkArea->subdivide(n);
        }

        ~ProgressSubTasks()
        {
            theWorkArea->undivide();
        }
    };

} // END of namespace

#ifndef NO_DE_ZIB_MCLIB2_NS_ALIAS
namespace mc = de_zib_mc;
#endif

#endif
