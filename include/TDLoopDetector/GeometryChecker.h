/**************************************************************************
 * Copyright (c) 2019 Chimney Xu. All Rights Reserve.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************/
/* *************************************************************************
   * File Name     : GeometryChecker.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-12 14:17:10
   * Last Modified : smallchimney
   * Modified Time : 2019-12-15 20:01:53
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_GEOMETRY_CHECKER_H__
#define __ROCKAUTO_TDLD_GEOMETRY_CHECKER_H__

#include <TDBoW/TemplatedDescriptor.hpp>

#include "KeyPoint.h"
#include "MatchPair.h"
#include "Transform.h"

namespace TDLoopDetector {

    template <size_t Dim = 3>
    class GeometryChecker {
    public:
        // Key point types define
        typedef TemplatedKeyPoint<Dim>      KeyPoint;
        typedef TemplatedKeyPoints<Dim>     KeyPoints;
        typedef TemplatedKeyPointArray<Dim> KeyPointArray;
        typedef std::function<bool(const MatchPairArray&, Transform&,
                const KeyPointArray&, const KeyPointArray&)> PointPairsGeomChecker;

        /**
         * @brief  Geometry check interface, only a reference but never force to use.
         *         This interface should be promised equals with `PointPairsGeomChecker`
         *
         *         The `TemplatedLoopDetector` is designed to use `std::function` to
         *         call for the geometry checking, so it's OK to use something like
         *         std::bind and lambda or others.
         *
         *         The reason to design like this is that you can consider the two
         *         objects (detector and checker) are detached, so you can use it both.
         *         But actually, they are combined by functional call.
         * @author smallchimney
         * @param  _Matches          The given points pairs.
         * @param  _Transform (out)  Only if the there are geometric consistency
         *                           between the two scenes, s.t. return {@code true}
         *                           this field will be filled the transform between
         *                           the two scenes. (//todo: from where to where)
         * @param  _HistoryKp        The key points in history scene
         * @param  _QueryKp          The key points in current scene
         * @return                   Whether there are geometric consistency between
         *                           the two scenes
         */
        virtual bool doGeometryCheck(
                const MatchPairArray& _Matches, Transform& _Transform,
                const KeyPointArray& _HistoryKp, const KeyPointArray& _QueryKp) = 0;

        /**
         * @brief  Current ISO C++ standard don't supported implement this in base class
         * @author smallchimney
         * @return The suitable `PointPairsGeomChecker` callback attached with
         *         current instance
         */
         virtual PointPairsGeomChecker checker() noexcept = 0;
    };

}

#endif //__ROCKAUTO_TDLD_GEOMETRY_CHECKER_H__
