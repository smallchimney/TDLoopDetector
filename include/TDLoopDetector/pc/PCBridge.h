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

/*! \mainpage TDLoopDetector Library
 *
 * TDLoopDetector library for C++:
 * Loop detector for pointcloud sequences based on bags of words
 *
 * Written by Yichong Xu,
 * Lanzhou University, China
 *
 * Forked from Dorian Galvez-Lopez,
 * University of Zaragoza
 *
 * \section requirements Requirements
 * The TDBoW and it's dependency libraries are required.
 * PCL is recommend so we add support for it.
 * Btw, OpenCV is still supported for CV users.
 *
 * \section citation Citation
 * If you use this software in academic works, please cite:
 <pre>
   @@ARTICLE{
   todo: Still work-in-progress
   }
  }
 </pre>
 *
 */

/* *************************************************************************
   * File Name     : PCBridge.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-08 18:01:36
   * Last Modified : smallchimney
   * Modified Time : 2019-12-12 14:12:01
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PC_BRIDGE_H__
#define __ROCKAUTO_TDLD_PC_BRIDGE_H__

#include "GroupClusterMatcher.hpp"
#include "PCLNdtGeometryChecker.hpp"
#include <TDLoopDetector/TDLoopDetector.h>
#include <TDBoW/PCBridge.h>

namespace TDLoopDetector {

/** @brief Loop detector based on FPFH-33 descriptor */
typedef TemplatedLoopDetector<TDBoW::FPFH33Database> FPFH33LoopDetector;

/** @brief Point pairs matcher based on PCL implemented KD-Tree on FPFH-33 descriptor */
typedef pc::PCLKdPairsMatcher<float, 33, 3> FPFH33KdPairsMatcher;

/** @brief Point pairs matcher based on PCL implemented KD-Tree on FPFH-33 descriptor */
typedef pc::GroupClusterMatcher<float, 33, pcl::PointXYZ> FPFH33GroupClusterMatcher;

}

#endif //__ROCKAUTO_TDLD_PC_BRIDGE_H__
