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
   * File Name     : Params.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-08 18:10:01
   * Last Modified : smallchimney
   * Modified Time : 2019-12-08 19:02:44
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PARAMS_H__
#define __ROCKAUTO_TDLD_PARAMS_H__

namespace TDLoopDetector {

    /// Geometrical checking methods
    enum GeometricalCheck {
        /// Do the geometry check (DI in default or override when construct)
        GEOM_ON,
        /// Do not perform geometrical checking
        GEOM_OFF
    };

    /// Parameters to create a loop detector
    typedef struct sParameters {
        // Main loop detector parameters
        /// Use normalized similarity score?
        bool use_nss;
        /// Alpha threshold
        float alpha;
        /// Min consistent matches to pass the temporal check
        unsigned k;
        /// Geometrical check
        GeometricalCheck geom_check;
        /// If using direct index for geometrical checking, direct index levels
        unsigned di_levels;

        // These are less deciding parameters of the system

        /// Distance between entries to be consider a match
        unsigned dead_zone;
        /// Max number of results from db queries to consider
        unsigned max_db_results;
        /// Min raw score between current entry and previous one to consider a match
        float min_nss_factor;
        /// Min number of close matches to consider some of them
        unsigned min_matches_per_group;
        /// Max separation between matches to consider them of the same group
        unsigned max_intragroup_range;
        /// Max separation between groups of matches to consider them consistent
        unsigned max_distance_between_groups;
        /// Max separation between two queries to consider them consistent
        unsigned max_distance_between_queries;

        // These are for the RANSAC to compute the F

        /// Min number of inliers when computing a fundamental matrix
        int min_Fpoints;
        /// Max number of iterations of RANSAC
        int max_ransac_iterations;
        /// Success probability of RANSAC
        double ransac_probability;
        /// Max reprojection error of fundamental matrices
        double max_reprojection_error;

        // This is to compute correspondences

        /// Max value of the neighbour-ratio of accepted correspondences
        double max_neighbor_ratio;

        /** @brief excepted work frequency in Hz */
        float excepted_frequency;

        /**
         * @brief Creates parameters by default
         * @param _Frequency  The data excepted feeding rate in Hz.
         * @param _UseNss     use normalized similarity score
         * @param _Alpha      alpha parameter
         * @param _K          k parameter (number of temporary consistent matches)
         * @param _GeomCheck  type of geometrical check
         * @param _DiLevels   direct index levels when geom == GEOM_DI
         */
        explicit sParameters(float _Frequency = 1.f, bool _UseNss = true,
                             float _Alpha = 0.3, unsigned _K = 3,
                             GeometricalCheck _GeomCheck = GEOM_ON,
                             unsigned _DiLevels = 0);

        /**
         * @brief Sets some parameters according to the frequency.
         *         When the detector's process speed is not excepted,
         *         this method should be recalled.
         * @param frequency The data feeding rate in Hz.
         */
        void setFreq(float frequency);

    } Parameters;

}   // namespace TDLoopDetector

#endif //__ROCKAUTO_TDLD_PARAMS_H__
