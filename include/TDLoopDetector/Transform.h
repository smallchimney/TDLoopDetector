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
   * File Name     : Transform.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-12 13:40:01
   * Last Modified : smallchimney
   * Modified Time : 2019-12-12 13:40:58
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_TRANSFORM_H__
#define __ROCKAUTO_TDLD_TRANSFORM_H__

#include <eigen3/Eigen/Core>
#include <vector>

namespace TDLoopDetector {

typedef Eigen::Matrix4f Transform;
typedef std::vector<Transform, Eigen::aligned_allocator<Transform>> Transforms;

}

#endif //__ROCKAUTO_TDLD_TRANSFORM_H__
