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
   * File Name     : DetectionResult.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-08 19:04:32
   * Last Modified : smallchimney
   * Modified Time : 2019-12-08 19:08:16
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_DETECTION_RESULT_H__
#define __ROCKAUTO_TDLD_DETECTION_RESULT_H__

#include <TDBoW/QueryResults.h>
#include <TDLoopDetector/Island.h>

namespace TDLoopDetector {

    /// Reasons for dismissing loops
    enum DetectionStatus {
        UNKOWN = 0,
        /// All the matches are very recent
        CLOSE_MATCHES_ONLY,
        /// No matches against the database
        NO_DB_RESULTS,
        /// Score of current image against previous one too low
        LOW_NSS_FACTOR,
        /// Scores (or NS Scores) were below the alpha threshold
        LOW_SCORES,
        /// Not enough matches to create groups
        NO_GROUPS,
        /// Low scores in a long range
        LOW_GROUPS_SCORE,
        /// Not enough temporary consistent matches (k)
        NO_TEMPORAL_CONSISTENCY,
        /// The geometrical consistency failed
        NO_GEOMETRICAL_CONSISTENCY = 99,
        /// Loop correctly detected
        LOOP_DETECTED = 100
    };

    /// Candidate of a detection
    typedef struct sCandidate{
        typedef TDBoW::traits::basic_traits<sCandidate>::Ptr Ptr;
        typedef TDBoW::traits::basic_traits<sCandidate>::ConstPtr ConstPtr;
        typedef TDBoW::EntryId EntryId;
        typedef TDBoW::Result  Result;

        sCandidate() = default;

        sCandidate(const Result& _queryResult) {
            id = _queryResult.Id;
            score = _queryResult.Score;
        }

        sCandidate(const sIsland& _isResult) {
            id = _isResult.best_entry;
            score = _isResult.best_score;
        }

        /// Matched id
        EntryId id;
        /// The score of the candidate
        double  score;
    } Candidate;
    typedef Candidate::Ptr CandidatePtr;
    typedef Candidate::ConstPtr CandidateConstPtr;

    /// Result of a detection
    typedef struct sDetectionResult {
        typedef TDBoW::traits::basic_traits<sDetectionResult>::Ptr Ptr;
        typedef TDBoW::traits::basic_traits<sDetectionResult>::ConstPtr ConstPtr;
        typedef TDBoW::EntryId EntryId;

        sDetectionResult() = default;

        /// Detection status. LOOP_DETECTED if loop detected
        DetectionStatus status = UNKOWN;
        /// Query id
        EntryId query{};
        /// Matched id if loop detected, otherwise, best candidate
        /// Best Candidate
        CandidateConstPtr bestCandidate = nullptr;
        /// All Candidates
        std::vector<Candidate> allCandidates;
        /// Selected Candidate
        std::vector<Candidate> selectCandidates;

        operator bool() const noexcept {
            return detected();
        }

        /**
        * @brief  Checks if the loop was detected
        * @return true if a loop was detected
        */
        bool detected() const noexcept {
            return status == LOOP_DETECTED;
        }
    } DetectionResult;
    typedef DetectionResult::Ptr DetectionResultPtr;
    typedef DetectionResult::ConstPtr DetectionResultConstPtr;

}

#endif //__ROCKAUTO_TDLD_DETECTION_RESULT_H__
