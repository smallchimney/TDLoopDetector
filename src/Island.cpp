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
   * File Name     : Island.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-07 22:16:04
   * Last Modified : smallchimney
   * Modified Time : 2019-12-09 20:52:56
************************************************************************* */
#include <TDLoopDetector/Island.h>
#include <sstream>
#include <algorithm>

namespace TDLoopDetector {

sIsland::sIsland(EntryId _Beg, EntryId _End) {
    assert(_Beg <= _End);   // equal mean empty island
    span = std::make_pair(_Beg, _End);
}

sIsland::sIsland(EntryId _Beg, EntryId _End, double _Score) : score(_Score) {
    assert(_Beg <= _End);   // equal mean empty island
    span = std::make_pair(_Beg, _End);
}

sIsland::sIsland(TDBoW::QueryResults _Results) {
    if(_Results.empty()) {
        span = std::make_pair(0, 0);
        return;
    }
    std::sort(_Results.begin(), _Results.end(),
            [](const TDBoW::Result& _A, const TDBoW::Result& _B) {
        return _A.Id < _B.Id;
    });
    span = std::make_pair(_Results.begin() -> Id, _Results.rbegin() -> Id + 1);
    score = 0; best_score = 0;
    for(const auto& result : _Results) {
        if(result.Score > best_score) {
            best_entry = result.Id;
            best_score = result.Score;
        }
        score += result.Score;
    }
}

std::string Island::toString() const {
    static std::stringstream ss;
    ss.clear(); ss.str("");
    ss << "<[" << span.first << ", " << span.second << "): " << score << " | best: <"
       << best_entry << ": " << best_score << ">>";
    return ss.str();
}

}   // namespace TDLoopDetector
