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
   * File Name     : Exception.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-27 18:10:47
   * Last Modified : smallchimney
   * Modified Time : 2019-12-29 17:08:11
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_EXCEPTION_H__
#define __ROCKAUTO_TDLD_EXCEPTION_H__

#include <TDBoW/Exception.h>

namespace TDLoopDetector {

// Continue use the TDBoW's Exceptions
#define __DEF_EX(Ex) typedef ::TDBoW:: Ex Ex;

__DEF_EX(Exception)
// Program design exceptions
__DEF_EX(LogicException)
__DEF_EX(NotInitailizedException)
__DEF_EX(ParametersException)
__DEF_EX(OutOfRangeException)
__DEF_EX(MethodNotMatchException)
// IO Exceptions
__DEF_EX(IOException)
__DEF_EX(FileNotExistException)
__DEF_EX(FileNotOpenException)
// Data Exceptions
__DEF_EX(DataException)
__DEF_EX(FormatException)
__DEF_EX(EmptyDataException)
__DEF_EX(NanDataException)

#undef __DEF_EX

} // namespace TDLoopDetector

#endif //__ROCKAUTO_TDBOW_EXCEPTION_H__
