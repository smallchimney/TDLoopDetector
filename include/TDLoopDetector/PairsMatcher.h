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
   * File Name     : PairsMatcher.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-14 15:30:59
   * Last Modified : smallchimney
   * Modified Time : 2019-12-18 13:45:43
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PAIRS_MATCHER_H__
#define __ROCKAUTO_TDLD_PAIRS_MATCHER_H__

#include "KeyPoint.h"
#include "MatchPair.h"
#include <TDBoW/TemplatedDescriptor.hpp>

namespace TDLoopDetector {

    template <typename TScalar, size_t L, size_t Dim = 3>
    class PairsMatcher {
    public:
        // Descriptor types define
        typedef TDBoW::TemplatedDescriptorUtil<TScalar, L> DescriptorUtil;
        TDBOW_DESCRIPTOR_DEF(DescriptorUtil)
        // Key point types define
        typedef TemplatedKeyPoint<Dim>      KeyPoint;
        typedef TemplatedKeyPoints<Dim>     KeyPoints;
        typedef TemplatedKeyPointArray<Dim> KeyPointArray;
        typedef std::function<MatchPairArray(
                const DescriptorArray&, const KeyPointArray&,
                const DescriptorArray&, const KeyPointArray&)> KeyPointsMatcher;

        /**
         * @brief  Key points pairs matcher interface, only a recommend reference
         *         but never force to use.
         *         This interface should be promised equals with `KeyPointsMatcher`
         *
         *         The `TemplatedLoopDetector` is designed to use `std::function` to
         *         call for the alternative key points pair matching, so it's OK to
         *         use something like std::bind and lambda or others.
         *
         *         The reason to design like this is that you can consider the two
         *         objects (detector and checker) are detached, so you can use it both.
         *         But actually, they are combined by functional call.
         * @author smallchimney
         * @param  _HistoryDesc  The history cloud's descriptors in order.
         * @param  _HistoryKp    The point position for the history descriptors, in
         *                       history cloud's coordinate system.
         * @param  _QueryDesc    The (current) query cloud's descriptors in order.
         * @param  _QueryKp      The point position for the history descriptors, in
         *                       (current) query cloud's coordinate system.
         * @return               Matched key points pairs between two cloud.
         */
        virtual MatchPairArray doKeyPointsMatch(
                const DescriptorArray& _HistoryDesc,
                const KeyPointArray& _HistoryKp,
                const DescriptorArray& _QueryDesc,
                const KeyPointArray& _QueryKp) = 0;

        /**
         * @brief  Current ISO C++ standard don't supported implement this in base class
         * @author smallchimney
         * @return The suitable `KeyPointsMatcher` callback attached with current instance
         */
        virtual KeyPointsMatcher matcher() noexcept = 0;
    };

}


#endif //__ROCKAUTO_TDLD_PAIRS_MATCHER_H__
