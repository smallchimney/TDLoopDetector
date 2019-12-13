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
   * File Name     : MatchPair.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-11 22:08:31
   * Last Modified : smallchimney
   * Modified Time : 2019-12-16 16:32:26
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_MATCH_PAIR_H__
#define __ROCKAUTO_TDLD_MATCH_PAIR_H__

#include <tuple>
#include <vector>

namespace TDLoopDetector {

// <history index, query index, distance>
typedef std::tuple<size_t, size_t, float> MatchPair;
typedef std::vector<MatchPair> MatchPairArray;

}

#endif //__ROCKAUTO_TDLD_MATCH_PAIR_H__
