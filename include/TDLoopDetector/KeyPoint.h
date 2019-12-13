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
   * File Name     : KeyPoint.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-11 22:01:31
   * Last Modified : smallchimney
   * Modified Time : 2019-12-11 22:04:09
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_KEY_POINT_H__
#define __ROCKAUTO_TDLD_KEY_POINT_H__

#include <eigen3/Eigen/Core>
#include <vector>

namespace TDLoopDetector {

template <size_t Dim = 3>
using TemplatedKeyPoint = Eigen::Matrix<float, Dim, 1, Eigen::DontAlign>;

template <size_t Dim = 3>
using TemplatedKeyPoints = Eigen::Matrix<float, Dim, Eigen::Dynamic, Eigen::DontAlign>;

template <size_t Dim = 3>
using TemplatedKeyPointArray = std::vector<TemplatedKeyPoint<Dim>,
        Eigen::aligned_allocator<TemplatedKeyPoint<Dim>>>;

}

#endif //__ROCKAUTO_TDLD_KEY_POINT_H__
