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
   * File Name     : TemporalWindow.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-08 20:26:52
   * Last Modified : smallchimney
   * Modified Time : 2019-12-14 21:25:47
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_TEMPORAL_WINDOW_H__
#define __ROCKAUTO_TDLD_TEMPORAL_WINDOW_H__

#include "Island.h"

namespace TDLoopDetector {

/// Temporal consistency window
typedef struct sTemporalWindow {
    /// Island matched in the last query
    Island last_matched_island;
    /// Last query id
    Island::EntryId last_query_id{};
    /// Number of consistent entries in the window
    size_t entries_num{};

    /**
     * @brief Creates an empty temporal window
     */
    sTemporalWindow() = default;

    void add(const Island& _Island, const Island::EntryId& _ID) {
        last_query_id = _ID;
        last_matched_island = _Island;
        entries_num++;
    }

    /**
     * @brief  Clear the window todo: check usage
     * @author smallchimney
     */
    void clear() {
        entries_num = 0;
        last_query_id = 0;
        last_matched_island = Island();
    }

    /**
     * @brief  Get the size of the window
     * @author smallchimney
     * @return Window size
     */
    size_t size() { return entries_num; }

    /**
     * @brief  Check whether the window is empty
     * @author smallchimney
     * @return {@code true} if empty
     */
    bool empty() { return entries_num == 0; }

} TemporalWindow;

}   // namespace TDLoopDetector

#endif //__ROCKAUTO_TDLD_TEMPORAL_WINDOW_H__
