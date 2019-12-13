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
   * File Name     : traits.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-12 13:14:08
   * Last Modified : smallchimney
   * Modified Time : 2019-12-12 13:16:40
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PC_TRAITS_H__
#define __ROCKAUTO_TDLD_PC_TRAITS_H__

#include <pcl/point_types.h>

namespace TDLoopDetector {
namespace pc{
namespace traits {

    template <typename PointT>
    class pcl_type {
    protected:
        static constexpr size_t dim() {
            if(std::is_same<PointT, pcl::PointXYZ>()) {
                return 3;
            } else if(std::is_same<PointT, pcl::PointXY>()) {
                return 2;
            } else if(std::is_same<PointT, pcl::PointXYZI>()) {
                return 4;
            } else {
                return 0;
            }
        }
    public:
        static constexpr size_t DIM = dim();
    };

}}}

#endif //__ROCKAUTO_TDLD_PC_TRAITS_H__
