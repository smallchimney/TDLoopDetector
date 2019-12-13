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
   * File Name     : Params.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-08 18:09:44
   * Last Modified : smallchimney
   * Modified Time : 2019-12-09 21:50:41
************************************************************************* */
#include <TDLoopDetector/Params.h>

namespace TDLoopDetector {

sParameters::sParameters(float _Frequency, bool _UseNss,
        float _Alpha, unsigned _K, GeometricalCheck _GeomCheck, unsigned _DiLevels)
        : use_nss(_UseNss), alpha(_Alpha), k(_K), geom_check(_GeomCheck), di_levels(_DiLevels) {
    min_nss_factor = 0.005;
    min_Fpoints = 12;
    max_ransac_iterations = 500;
    ransac_probability = 0.99;
    max_reprojection_error = 2.0;
    max_neighbor_ratio = 0.6;
    setFreq(_Frequency);
}

void Parameters::setFreq(float _Frequency) {
    // parameters corresponding with frequency
    excepted_frequency           = _Frequency;
    dead_zone                    = static_cast<unsigned>(20 * _Frequency);
    max_db_results               = static_cast<unsigned>(50 * _Frequency);
    min_matches_per_group        = static_cast<unsigned>(_Frequency);
    max_intragroup_range         = static_cast<unsigned>(10 * _Frequency);
    max_distance_between_groups  = static_cast<unsigned>(3 * _Frequency);
    max_distance_between_queries = static_cast<unsigned>(2 * _Frequency);
}

}   // namespace TDLoopDetector
