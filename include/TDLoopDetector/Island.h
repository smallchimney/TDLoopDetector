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
   * File Name     : Island.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-07 21:56:40
   * Last Modified : smallchimney
   * Modified Time : 2019-12-10 15:01:03
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_ISLAND_H__
#define __ROCKAUTO_TDLD_ISLAND_H__

#include <TDBoW/QueryResults.h>
#include <cassert>

namespace TDLoopDetector {

    typedef struct sIsland {
        typedef TDBoW::traits::basic_traits<sIsland>::Ptr Ptr;
        typedef TDBoW::traits::basic_traits<sIsland>::ConstPtr ConstPtr;
        typedef TDBoW::EntryId EntryId;

        // entry span in ID: [begin, end)
        std::pair<EntryId, EntryId> span;

        /// Island score
        double score{}; // score of island

        /// Entry of the island with the highest score
        EntryId best_entry{}; // id and score of the entry with the highest score
        /// Highest single score in the island
        double best_score{};  // in the island

        /**
         * @brief Creates an empty island
         */
        sIsland() = default;

        /**
         * @brief Creates an island
         * @param _Beg  first entry
         * @param _End  last entry
         */
        sIsland(EntryId _Beg, EntryId _End);

        /**
         * @brief Creates an island
         * @param _Beg    first entry
         * @param _End    last entry
         * @param _Score  value of the island
         */
        sIsland(EntryId _Beg, EntryId _End, double _Score);

        /**
         * @brief Creates an island with a group of result
         * @param _Results  grouped query result
         */
        sIsland(TDBoW::QueryResults _Results);

        /**
         * @brief  Calculate the size of the island
         * @author smallchimney
         * @return The island range, not actual match query number
         */
        size_t size() const noexcept {
            return span.second - span.first;
        }

        /**
         * @return Whether the island is empty.
         */
        bool empty() const noexcept {
            return size() == 0;
        }

        /**
         * @brief  Check whether two island is crossing.
         * @author smallchimney
         * @param  _Another  Another island to be checked.
         * @return           Whether two islands are crossing.
         */
        bool cross(const sIsland& _Another) {
            if(empty() || _Another.empty())return false;
            return (span.first <= _Another.span.first &&
                    span.second > _Another.span.first) ||
                    (_Another.span.first <= span.first &&
                    _Another.span.second > span.first);
        }

        /**
         * @brief  Says whether this score is less than the score of another island
         * @param  _Another
         * @return  whether this score < _Another.score
         */
        bool operator < (const sIsland& _Another) const {
            return score < _Another.score;
        }

        /**
         * @brief  Says whether this score is greater than the score of another island
         * @param  _Another
         * @return whether this score < _Another.score
         */
        bool operator > (const sIsland& _Another) const {
            return score > _Another.score;
        }

        /**
         * Returns a printable version of the island
         * @return printable island
         */
        std::string toString() const;

        /**
         * @brief  Output the printable version of the island
         * @author smallchimney
         * @param  _Out  Output stream reference
         * @return       Outputted stream reference
         */
        std::ostream& operator <<(std::ostream& _Out) {
            return _Out << toString();
        }

    } Island;
    typedef Island::Ptr      IslandPtr;
    typedef Island::ConstPtr IslandConstPtr;

}

#endif //__ROCKAUTO_TDLD_ISLAND_H__
