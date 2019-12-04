#ifndef HX_GL_UTILS_CONTEXT_CACHE_H
#define HX_GL_UTILS_CONTEXT_CACHE_H

#include <boost/shared_ptr.hpp>

#include <mcgl/internal/mcgl.h>
#include <mclib/McDArray.h>
#include <mclibthreaded/internal/McScopedLock.h>

#undef Status
#undef Bool
#undef CursorShape
#include <hxcore/HxController.h>

template < class GPUDataT >
class HxContextCache
{

    // subclasses
    private:

        /**
            Deletes the GPU items.
        */
        class DeleterItem : public SxGLHandleDeleter
        {
            public:

                DeleterItem(GPUDataT* obj)
                    : mObj(obj)
                {
                }

                ~DeleterItem()
                {
                    delete mObj;
                }

                GPUDataT* mObj;
        };


        /**
            Holdes and updates GPU data.
        */
        class ContextItem
        {

            public:

                /// Constructor. Creates GPU data.
                ContextItem(unsigned int id)
                    : mTouchTime(0)
                    , mGPUData( new GPUDataT )
                    , mContextID(id)
                {
                }

                /// Updates GPU data if necessary.
                void update(unsigned int touchTime, void* data)
                {
                    if (touchTime != mTouchTime)
                    {
                        McScopedLock lock(mMutex);
                        mGPUData->updateResource(data);
                        mTouchTime = touchTime;
                    }
                }

                /// Creates a DeleterItem and adds it to the controller.
                ~ContextItem()
                {
                    theController->glDeferredDelete(mContextID, new DeleterItem(mGPUData));
                }


                // attributes
                unsigned int mTouchTime;
                GPUDataT*    mGPUData;
                unsigned int mContextID;
                McMutex      mMutex;

        };

        typedef boost::shared_ptr< ContextItem > PContextItem;


    // methods
    public:

        HxContextCache()
            : mContextItems(0)
            , mTouchTime(0)
        {
        }


        /// Returns the current GPU data instance.
        GPUDataT* getInstance(unsigned int contextID)
        {
            ContextItem* item = getContextItem(contextID);

            return item->mGPUData;
        }


        /// Force an update call next time on rendering.
        void touch()
        {
            mTouchTime++;
        }


        /// Updates
        void update(unsigned int contextId, void* data)
        {
            ContextItem* item = getContextItem(contextId);
            item ->update(mTouchTime, data);
        }


    private:

        /// Returns or creates a context item.
        ContextItem* getContextItem(unsigned int id)
        {
            McScopedLock lock(mMutex);

            int numItems = (int) mContextItems.size();

            for (int i = 0; i < numItems; ++i)
            {
                if (mContextItems[i]->mContextID == id)
                {
                    return mContextItems[i].get();
                }
            }

            mContextItems.push_back(PContextItem(new ContextItem(id)));

            return mContextItems.back().get();
        }


    // attributes
    private:

         McDArray< PContextItem > mContextItems;
         McMutex                  mMutex;
         unsigned int             mTouchTime;
};






#endif
