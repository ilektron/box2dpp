/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_BLOCK_ALLOCATOR_H
#define B2_BLOCK_ALLOCATOR_H

#include <Box2D/Common/b2Settings.h>

namespace box2d
{
const int32_t b2_chunkSize = 16 * 1024;
const int32_t b2_maxBlockSize = 640;
const int32_t b2_blockSizes = 14;
const int32_t b2_chunkArrayIncrement = 128;

struct b2Block;
struct b2Chunk;

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
class b2BlockAllocator
{
public:
    b2BlockAllocator();
    ~b2BlockAllocator();

    /// Allocate memory. This will use b2Alloc if the size is larger than b2_maxBlockSize.
    void* Allocate(int32_t size);

    /// Free memory. This will use b2Free if the size is larger than b2_maxBlockSize.
    void Free(void* p, int32_t size);

    void Clear();

private:
    b2Chunk* m_chunks;
    int32_t m_chunkCount;
    int32_t m_chunkSpace;

    b2Block* m_freeLists[b2_blockSizes];

    static int32_t s_blockSizes[b2_blockSizes];
    static uint8_t s_blockSizeLookup[b2_maxBlockSize + 1];
    static bool s_blockSizeLookupInitialized;
};
}

#endif
