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
/**
 * File: TemplatedLoopDetector
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: templated loop detector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __ROCKAUTO_TDLD_TEMPLATED_LOOP_DETECTOR_HPP__
#define __ROCKAUTO_TDLD_TEMPLATED_LOOP_DETECTOR_HPP__

#include "Params.h"
#include "KeyPoint.h"
#include "MatchPair.h"
#include "Transform.h"
#include "TemporalWindow.h"
#include "DetectionResult.h"
#include "GeometryChecker.h"
#include "PairsMatcher.h"

#include <TDBoW/TDBoW.h>

namespace TDLoopDetector {

template<class TDatabase, size_t Dim = 3>
/// Generic Loop detector
class TemplatedLoopDetector {
public:
    // BoW database
    typedef TDatabase DatabaseType;
    typedef std::unique_ptr<TDatabase> DatabasePtr;
    // Vocabulary
    typedef typename TDatabase::VocabularyType Vocabulary;
    typedef typename TDatabase::VocabularyPtr  VocabularyPtr;
    typedef typename Vocabulary::ScalarType    ScalarType;
    TDBOW_DESCRIPTOR_DEF(Vocabulary::util)
    // Key points database
    typedef TemplatedKeyPoint<Dim>             KeyPoint;
    typedef TemplatedKeyPoints<Dim>            KeyPoints;
    typedef TemplatedKeyPointArray<Dim>        KeyPointArray;
    typedef std::vector<KeyPointArray>         KeyPointsDatabase;
    typedef std::unique_ptr<KeyPointsDatabase> KeyPointsDatabasePtr;
    // Descriptors database
    typedef DescriptorsSet DescriptorsDatabase;
    typedef std::unique_ptr<DescriptorsDatabase> DescriptorsDatabasePtr;
    // Runtime context parameters
    typedef Parameters params;
    // Extendable function interface
    typedef std::function<void(MatchPairArray&,
            const KeyPointArray&, const KeyPointArray&)> PointPairsFilter;
    typedef typename PairsMatcher<ScalarType,
            Vocabulary::util::DescL, Dim>::KeyPointsMatcher KeyPointsMatcher;
    typedef typename GeometryChecker<Dim>::PointPairsGeomChecker PointPairsGeomChecker;

public:

    /**
     * @brief  Initialize a loop detector based on a vocabulary
     * @author smallchimney
     * @param  _GeomChecker Geometry check method based on matched points
     *                      pair, keep {@code nullptr} when setting "GEOM_OFF"
     * @param  _Vocabulary  The vocabulary to be used when query, initialized
     *                      required, or will throw error when detection
     * @param  _Param       Loop detector runtime parameters, see details in
     *                      `Params.h`
     * @param  _Filter      Match points pairs filter, aim to filter false-
     *                      positive match, keep {@code nullptr} for no filter.
     *                      Normal use in case "GEOM_ON" and not override `_Matcher`
     * @param  _Matcher     Points pair matcher based on descriptors and keypoints.
     *                      Keep {@code nullptr} when "GEOM_OFF" or using default
     *                      points match method based on DI and descriptor distance.
     *                      More details can be founded in `KeyPointMatcherDI()`
     */
    explicit TemplatedLoopDetector(const PointPairsGeomChecker& _GeomChecker,
            Vocabulary&& _Vocabulary, const params& _Param = params(),
            const PointPairsFilter& _Filter = nullptr,
            const KeyPointsMatcher& _Matcher = nullptr);

    /**
     * // todo: complete
     * @brief  Setup detector in default parameters, load the database & vocabulary
     *         (or only vocabulary) for initialization.
     * @author smallchimney
     * @param  _Param  loop detector parameters
     */
    explicit TemplatedLoopDetector(const PointPairsGeomChecker& _GeomChecker,
            TDatabase&& _Database, const params& _Param = params(),
            const PointPairsFilter& _Filter = nullptr,
            const KeyPointsMatcher& _Matcher = nullptr);

//    /**
//     * // todo: complete
//     * @brief  Setup detector in default parameters, load the database & vocabulary
//     *         (or only vocabulary) for initialization.
//     * @author smallchimney
//     * @param  _Param  loop detector parameters
//     */
//    explicit TemplatedLoopDetector(const PointPairsGeomChecker& _GeomChecker,
//            const std::string& _Filename, const params& _Param = params(),
//            const PointPairsFilter& _Filter = nullptr,
//            const KeyPointsMatcher& _Matcher = nullptr);

    // todo: implement the constructors

    /**
     * Destructor
     */
    virtual ~TemplatedLoopDetector();

    // Functional methods
    /**
     * @brief query one image from the databases. In case ("DYNAMIC_UPDATE")
     *        the database will be updated after query.
     * @param _KeyPoints    Key points of the image
     * @param _Descriptors  Descriptors associated to the given key points
     * @param _Match (out)  Match or failing information
     * @return              If a loop is detected, the method will return
     *                      the transform matrix from (todo: complete) to (todo: complete)
     *                      If not, a zero matrix will be returned.
     */
    Transform detectLoop(const KeyPointArray& _KeyPoints,
                    const DescriptorArray& _Descriptors,
                    DetectionResult& _Match);

    /**
     * @breif  Allocates some memory for the following entries.
     * @author smallchimney
     * @param  _ExceptedEntries  Expected number of entries
     * @param  _KeyNumPerImage   Expected number of key points per image
     */
    void allocate(size_t _ExceptedEntries, size_t _KeyNumPerImage = 0);

    /**
     * @brief Empty the current window, in normal case ("DYNAMIC_UPDATE")
     *        will clear the history descriptors, key points and the database.
     *        Such that the next entry will be 0 again.
     * @author smallchimney
     */
    void clear();

    /**
     * @brief  Whether the database is ready. Especially in stationary mode,
     *         the database should not be empty.
     * @author smallchimney
     * @return {@code true} after the database is ready for query.
     */
    bool ready() const noexcept {
        return m_pDatabase && m_pDatabase -> ready() &&
                (m_bStationaryDatabase ? !m_pDatabase -> empty() : true);
    }

    /**
     * @breif  Returns whether the database is empty.
     * @return {@code true} before any valid query.
     */
    bool empty() const noexcept {
        return !m_pDatabase || m_pDatabase -> empty();
    }

    /**
     * @breif  Returns the number of entries in the database
     * @return number of entries
     */
    size_t size() const noexcept {
        return m_pDatabase ? m_pDatabase -> size() : 0;
    }

    // Setter methods

    /**
     * @brief  Set the given database to use, will empty the current window
     * @author smallchimney
     * @param  _DB             Database to be used
     * @param  _DescriptorsDB  Descriptors of database's entries.
     * @param  _KeyPointsDB    Key points of database's entries.
     */
    TemplatedLoopDetector<TDatabase, Dim>& setDatabase(TDatabase&& _DB,
                     DescriptorsDatabasePtr&& _DescriptorsDB = nullptr,
                     KeyPointsDatabasePtr&& _KeyPointsDB = nullptr) noexcept(false);

    /**
     * @brief  Set the given database to use, will empty the current window
     * @author smallchimney
     * @param  _DB             Database to be used
     * @param  _DescriptorsDB  Descriptors of database's entries.
     * @param  _KeyPointsDB    Key points of database's entries.
     */
    TemplatedLoopDetector<TDatabase, Dim>& setDatabase(DatabasePtr&& _DB,
                     DescriptorsDatabasePtr&& _DescriptorsDB = nullptr,
                     KeyPointsDatabasePtr&& _KeyPointsDB = nullptr) noexcept(false);
  
    /**
     * @brief  move the given vocabulary to a new built database, and set database.
     * @author smallchimney
     * @param  _Vocab  the vocabulary to use in query
     */
    TemplatedLoopDetector<TDatabase, Dim>& setVocabulary(Vocabulary&& _Vocab) noexcept(false);

    /**
     * @brief  move the given vocabulary to a new built database, and set database.
     * @author smallchimney
     * @param  _Vocab  the vocabulary to use in query
     */
    TemplatedLoopDetector<TDatabase, Dim>& setVocabulary(VocabularyPtr&& _Vocab) noexcept(false);

    /**
     * @brief  Set the database stationary, so it won't be updated after feat.
     * @author smallchimney
     * @param  _Stationary  Whether to set the database stationary.
     */
    TemplatedLoopDetector<TDatabase, Dim>& setStationary(bool _Stationary = true) noexcept {
        if(empty() && _Stationary) {
            std::cerr << TDBOW_LOG("Warning: The detector is working "
                         "in stationary mode on empty database.");
        }
        m_bStationaryDatabase = _Stationary;
        return *this;
    }

    // Getter methods

    /**
     * @breif  Retrieves a reference to the database used by the loop detector
     * @return const reference to database
     */
    const TDatabase& getDatabase() const noexcept(false) {
        if(!m_pDatabase) {
            throw std::runtime_error(TDBOW_LOG("database is not ready yet."));
        }
        return *m_pDatabase;
    }

    /**
     * @breif  Retrieves a reference to the vocabulary used by the loop detector
     * @return const reference to vocabulary
     */
    const Vocabulary& getVocabulary() const noexcept(false) {
        if(!m_pDatabase || !m_pDatabase -> ready()) {
            throw std::runtime_error(TDBOW_LOG("vocabulary is not ready yet."));
        }
        return m_pDatabase -> getVocabulary();
    }

protected:

    /**
     * @brief  Update the KeyPointsDatabase and DescriptorsDatabase.
     *         The RI & DI database should be updated in sync.
     *         Put the RI & DI database updating outside for avoiding
     *         redundant transform request.
     * @author smallchimney
     * @param  _KeyPoints    Key points array of the image, should be
     *                       same order with {@code _Descriptors}
     * @param  _Descriptors  Descriptor array of each key points the image,
     *                       should be same order with {@code _KeyPoints}
     */
    void updateDatabase(const DetectionResult::EntryId& _ID,
                        const KeyPointArray& _KeyPoints,
                        const DescriptorArray& _Descriptors) noexcept(false);

    /**
     * @brief  Removes the low score results from ordered results
     * @param  _Results    Results from query
     * @param  _Threshold  Min value of the resting results
     *                     (that should be alpha * ns_factor)
     */
    void removeLowScores(TDBoW::QueryResults& _Results, double _Threshold) const noexcept;

    /**
     * @brief Returns the islands of the given matches in ascending
     *        order of entries' indices.
     * @param _Results       Results from query after filter
     * @param _Islands (out) computed islands
     */
    void computeIslands(const TDBoW::QueryResults& _Results,
                        std::vector<Island>& _Islands) const noexcept;

    /**
     * @brief Updates the temporal window by adding the given match
     *        <island, id>, such that the window will contain only those
     *        islands which are consistent
     * @param _Island   the best matched island
     */
    void updateTemporalWindow(const Island& _Island);

    /**
     * @brief  Key points matcher based on DI and descriptor distance implement.
     *         Will only be called when `GEOM_ON` and `m_cKeyPointsMatcher` is
     *         {@code nullptr}
     * @author smallchimney
     * @param  _HistoryEntryId  The history image ID.
     * @param  _HistoryImgDesc  The history image descriptors.
     * @param  _FeatureVec      The feature vector of query image.
     * @param  _QueryImgDesc    The query image descriptors.
     * @return                  Whether the two image is geometry like.
     */
    MatchPairArray KeyPointMatcherDI(
            const DetectionResult::EntryId& _HistoryEntryId,
            const DescriptorArray& _HistoryImgDesc,
            const TDBoW::FeatureVectorConstPtr& _FeatureVec,
            const DescriptorArray& _QueryImgDesc);

    /**
     * @brief  Calculate the matched descriptors pairs by distance
     * @author smallchimney
     * @param  _A         The complete descriptors list of image A
     * @param  _IdxA      The indices of selected descriptors to be matched
     *                    in image A
     * @param  _B         The complete descriptors list of image A
     * @param  _IdxB      The indices of selected descriptors to be matched
     *                    in image B
     * @param  _Callback  Distance calculate method, default implement can
     *                    be founded in {@code TDBoW::TemplatedDescriptor.hpp}
     * @return            The matched descriptors pairs' indices in format
     *                    <indexInA, indexInB>, count in range
     *                    {@code [ 0, min(_Idx.size(), _IdxB.size()) )}
     */
    MatchPairArray findMatchesByDistance(
            const DescriptorArray& _A, const std::vector<size_t>& _IdxA,
            const DescriptorArray& _B, const std::vector<size_t>& _IdxB,
            const typename Vocabulary::util::DistanceCallback&
                    _Callback = Vocabulary::util::distance) const noexcept(false);

protected:
  
    /** @brief Temporal consistency window */
    TemporalWindow m_sWindow;

    /**
     * @brief Whether the database will be updated after each query.
     */
    bool m_bStationaryDatabase = false;

    /**
     * @brief Since we will do the geometry checking between the images,
     *        the history images' should be saved.
     *
     *        This implement use several geometry checking methods, e.g.
     *        `geometryCheckDI()`, `geometryCheckFLANN()` and
     *        `geometryCheckExhaustive()`. All these methods require only
     *        descriptors and key points' position to calculate the
     *        geometrically consistent.
     *
     *        So this implement will only save the key points for history
     *        image. If the pipeline will do the geometry checking
     *        outside the detector, or even not do the checking, use the
     *        parameter "GEOM_NONE" to avoid the redundant memory wasting.
     *
     *        The current implement will always keep all the data in memory,
     *        so in case the database will be updated, the memory cost will
     *        grow. Maybe a good save/load method for the data will be very
     *        helpful.
     */
    KeyPointsDatabasePtr m_pKeyPointsDb;

    /**
     * @brief Since we will do the geometry checking between the images,
     *        the history images' should be saved.
     *
     *        This implement use several geometry checking methods, e.g.
     *        `geometryCheckDI()`, `geometryCheckFLANN()` and
     *        `geometryCheckExhaustive()`. All these methods require only
     *        descriptors and key points' position to calculate the
     *        geometrically consistent.
     *
     *        So this implement will only save the descriptors for history
     *        image. If the pipeline will do the geometry checking
     *        outside the detector, or even not do the checking, use the
     *        parameter "GEOM_NONE" to avoid the redundant memory wasting.
     *
     *        The current implement will always keep all the data in memory,
     *        so in case the database will be updated, the memory cost will
     *        grow. Maybe a good save/load method for the data will be very
     *        helpful.
     */
    DescriptorsDatabasePtr m_pDescriptorsDb;

    /** @brief last bag-of-words vector used to calculate normal similarity score */
    TDBoW::BowVector m_mLastBow;

    /** @brief The history entries to be queried */
    DatabasePtr m_pDatabase;

    /** @brief The current/next ID for query */
    DetectionResult::EntryId m_uEntryId;

    /** @brief Parameters of the loop detector */
    params m_sParams;

    /**
     * @brief Key points match method, using the default method (using DI and
     *        descriptor distance) when {@code nullptr}
     */
    KeyPointsMatcher m_cKeyPointsMatcher;

    /**
     * @brief Key points match pair filter, aim to filter false positive match.
     *        remain {@code nullptr} for no filter.
     */
    PointPairsFilter m_cPairsFilter;

    /**
     * @breif Main geometry checking method, must be defined when construct
     */
    PointPairsGeomChecker m_cGeomChecker;

};

/* ********************************************************************************
 *                        CONSTRUCTION && INITIALIZATION                          *
 ******************************************************************************** */

template<class TDatabase, size_t Dim>
TemplatedLoopDetector<TDatabase, Dim>::TemplatedLoopDetector(
        const PointPairsGeomChecker& _GeomChecker,
        Vocabulary&& _Vocabulary, const params& _Param,
        const PointPairsFilter& _Filter, const KeyPointsMatcher& _Matcher)
        : m_bStationaryDatabase(false), m_pDatabase(nullptr), m_sParams(_Param),
        m_cKeyPointsMatcher(_Matcher), m_cPairsFilter(_Filter), m_cGeomChecker(_GeomChecker) {
    if(m_cGeomChecker == nullptr) {
        throw std::runtime_error(TDBOW_LOG("The geometry check method is required!"));
    }
    setVocabulary(std::forward<Vocabulary>(_Vocabulary));
}

template<class TDatabase, size_t Dim>
TemplatedLoopDetector<TDatabase, Dim>::TemplatedLoopDetector(
        const PointPairsGeomChecker& _GeomChecker,
        TDatabase&& _Database, const params& _Param,
        const PointPairsFilter& _Filter, const KeyPointsMatcher& _Matcher)
        : m_bStationaryDatabase(false), m_pDatabase(nullptr), m_sParams(_Param),
        m_cKeyPointsMatcher(_Matcher), m_cPairsFilter(_Filter), m_cGeomChecker(_GeomChecker) {
    if(m_cGeomChecker == nullptr) {
        throw std::runtime_error(TDBOW_LOG("The geometry check method is required!"));
    }
    _Database.clear();
    setDatabase(std::forward<TDatabase>(_Database));
}

template<class TDatabase, size_t Dim>
TemplatedLoopDetector<TDatabase, Dim>::~TemplatedLoopDetector() {
    m_pDatabase.reset();
    m_pKeyPointsDb.reset();
    m_pDescriptorsDb.reset();
}

/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

template<class TDatabase, size_t Dim>
Transform TemplatedLoopDetector<TDatabase, Dim>::detectLoop(const KeyPointArray& _KeyPoints,
        const DescriptorArray& _Descriptors, DetectionResult& _Match) {
    assert(_Descriptors.size() == _KeyPoints.size());
    if(!ready()) {
        throw std::runtime_error(TDBOW_LOG(
                "Cannot found vocabulary, query require a valid vocabulary."));
    }
    Transform transform = Transform::Zero();
    _Match.query = m_uEntryId;

    TDBoW::FeatureVectorPtr featureVec = nullptr;
    if(m_sParams.geom_check == GEOM_ON && m_cKeyPointsMatcher == nullptr) {
        featureVec.reset(new TDBoW::FeatureVector);
    }
    TDBoW::BowVector bowVector;
    getVocabulary().transform(_Descriptors, bowVector, featureVec, m_sParams.di_levels);

    if(m_uEntryId <= m_sParams.dead_zone) {
        if(m_bStationaryDatabase) {
            throw std::runtime_error(TDBOW_LOG(
                    "Too few entries for a stationary database."));
        }
        m_pDatabase -> add(bowVector, featureVec);
        updateDatabase(m_uEntryId, _KeyPoints, _Descriptors);
        m_uEntryId++;
        if(m_sParams.use_nss && m_uEntryId == m_sParams.dead_zone) {
            m_mLastBow = bowVector;
        }
        _Match.status = CLOSE_MATCHES_ONLY;
        return transform;
    }
    auto result = getDatabase().query(bowVector,
            m_sParams.max_db_results, 0, m_uEntryId - m_sParams.dead_zone);
    do {
        // declare loop only for break
        if(result.empty()) {
            _Match.status = NO_DB_RESULTS; break;
        }
        // save all candidates
        _Match.allCandidates.reserve(result.size());
        for(const auto& ret: result) {
             _Match.allCandidates.emplace_back(ret);
        }

        // factor to compute normalized similarity score, if necessary
        auto factor = m_sParams.use_nss ?
                      getVocabulary().score(bowVector, m_mLastBow) : 1.0;
        // The factor is scaled in [0, 1], so the {@code min_nss_factor}
        // should not greater than 1, so no need to check the {@code use_nss}.
        if(factor < m_sParams.min_nss_factor) {
            _Match.status = LOW_NSS_FACTOR; break;
        }
        // scores in {@code result} must be divided by {@code factor}
        // to obtain the normalized similarity score, but we can
        // speed this up by moving {@code factor} to alpha's

        // remove those scores whose nss is lower than alpha
        // (ret is sorted in descending score order now)
        removeLowScores(result, m_sParams.alpha * factor);
        if(result.empty()) {
            _Match.status = LOW_SCORES; break;
        }
        // compute islands
        std::vector<Island> islands;
        computeIslands(result, islands);
        if(islands.empty()) {
            // the best candidate is the one with highest score by now
            _Match.bestCandidate.reset(new Candidate(result[0]));
            _Match.status = NO_GROUPS; break;
        }

        // save selected candidates
        for(const auto& island: islands) {
            _Match.selectCandidates.emplace_back(Candidate(island));
        }
        Island island;
        while(true) {
            if(islands.empty()) {
                // the best candidate is the one with highest score by now
                _Match.bestCandidate.reset(new Candidate(result[0]));
                _Match.status = LOW_GROUPS_SCORE; break;
            }
            const auto iter = std::max_element(islands.begin(), islands.end());
            const auto limit = iter -> size() * m_sParams.alpha * factor; // todo: maybe parameter beta?
            if(iter -> score > limit) {
                island = *iter;
                break;
            }
            islands.erase(iter);
        }
        if(_Match.status == LOW_GROUPS_SCORE)break;
        // get the best candidate (maybe match)
        _Match.bestCandidate.reset(new Candidate(island));
        // check temporal consistency of this island
        updateTemporalWindow(island);
        if(m_sWindow.size() <= m_sParams.k) {
            _Match.status = NO_TEMPORAL_CONSISTENCY; break;
        }
        // check geometry
        bool detected = false;
        switch (m_sParams.geom_check) {
        case GeometricalCheck::GEOM_OFF: // direct accept the match
            detected = true; break;

        case GeometricalCheck::GEOM_ON:
            const auto& historyDesc = (*m_pDescriptorsDb)[island.best_entry];
            const auto& historyKeys = (*m_pKeyPointsDb)[island.best_entry];
            assert(historyDesc.size() == historyKeys.size());
            MatchPairArray matches;
            if(m_cKeyPointsMatcher == nullptr) {
                matches = KeyPointMatcherDI(
                        island.best_entry, historyDesc, featureVec, _Descriptors);
            } else {
                matches = m_cKeyPointsMatcher(
                        historyDesc, historyKeys, _Descriptors, _KeyPoints);
            }
            if(m_cPairsFilter != nullptr) {
                m_cPairsFilter(matches, historyKeys, _KeyPoints);
            }
            assert(m_cGeomChecker != nullptr);
            detected = m_cGeomChecker(matches, transform, historyKeys, _KeyPoints);
            break;
        }
        _Match.status = detected ? LOOP_DETECTED : NO_GEOMETRICAL_CONSISTENCY;
    } while(false);
    // update record
    if(!m_bStationaryDatabase) {
        m_pDatabase -> add(bowVector, featureVec);
        if(m_sParams.geom_check == GeometricalCheck::GEOM_ON) {
            updateDatabase(m_uEntryId, _KeyPoints, _Descriptors);
        }
    }
    m_uEntryId++;
    // store this bow vector if we are going to use it in next iteration
    if(m_sParams.use_nss) {
        m_mLastBow = bowVector;
    }
    return transform;
}

template<class TDatabase, size_t Dim>
void TemplatedLoopDetector<TDatabase, Dim>::allocate(
        size_t _ExceptedEntries, size_t _KeyNumPerImage) {
    bool isEmpty = empty();
    const auto capacity = m_pDescriptorsDb -> capacity();
    // Normally the size is equal with capacity, but do this for secure
    const auto size = m_pDescriptorsDb -> size();
    assert(size == m_pKeyPointsDb -> size());
    // resize so the unused vectors are initialized, and we can then
    // reserve the memory for them.
    m_pDescriptorsDb -> resize(capacity + _ExceptedEntries);
    m_pKeyPointsDb -> resize(capacity + _ExceptedEntries);
    // The RI & DI database use std::list and std::map to store the features,
    // so it's hard to pre-allocate the memory for them, unless we implement
    // ourselves allocate memory pool.
    // So the `size()` may different from KeyPointsDB and RI & DI database
    // When output the database, should erase the unused space in
    // KeyPointsDB and DescriptorsDB
    // m_pDatabase -> reserve(capacity + _ExceptedEntries);

    size_t keyNum;
    if(!isEmpty) {
        size_t idx = size - 1, sum = 0, i = 0;
        while(i < 10) {
            sum += (*m_pDescriptorsDb)[idx].size();
            i++;
            if(idx-- == 0)break;
        }
        keyNum = sum / i;
    } else {
        keyNum = _KeyNumPerImage;
    }
    if(keyNum) {
        for(size_t i = size; i < capacity + _ExceptedEntries; i++) {
            (*m_pDescriptorsDb)[i].reserve(keyNum);
            (*m_pKeyPointsDb)[i].reserve(keyNum);
        }
    }
}

template<class TDatabase, size_t Dim>
void TemplatedLoopDetector<TDatabase, Dim>::clear() {
    if(!m_bStationaryDatabase) {
        m_pDatabase -> clear();
        m_pDescriptorsDb = DescriptorsDatabasePtr(new DescriptorsDatabase);
        m_pKeyPointsDb = KeyPointsDatabasePtr(new KeyPointsDatabase);
        auto time = std::chrono::minutes::period::num * 10;
        allocate(static_cast<size_t>(time * m_sParams.excepted_frequency), 200);
    }
    m_uEntryId = m_pDatabase -> size();
    m_sWindow.clear();
}

/* ********************************************************************************
 *                               GET/SET METHODS                                  *
 ******************************************************************************** */

template<class TDatabase, size_t Dim>
TemplatedLoopDetector<TDatabase, Dim>&
TemplatedLoopDetector<TDatabase, Dim>::setDatabase(TDatabase&& _DB,
        DescriptorsDatabasePtr&& _DescriptorsDB, KeyPointsDatabasePtr&& _KeyPointsDB) noexcept(false) {
    return setDatabase(
            DatabasePtr(new TDatabase(std::forward<TDatabase>(_DB))),
            std::forward<DescriptorsDatabasePtr>(_DescriptorsDB),
            std::forward<KeyPointsDatabasePtr>(_KeyPointsDB));
}

template<class TDatabase, size_t Dim>
TemplatedLoopDetector<TDatabase, Dim>&
TemplatedLoopDetector<TDatabase, Dim>::setDatabase(DatabasePtr&& _DB,
        DescriptorsDatabasePtr&& _DescriptorsDB, KeyPointsDatabasePtr&& _KeyPointsDB) noexcept(false) {
    m_pDatabase = std::forward<DatabasePtr>(_DB);
    if(m_pDatabase && !m_pDatabase -> empty()) {
        if(!_DescriptorsDB || !_KeyPointsDB
                || (_DescriptorsDB -> size() != m_pDatabase -> size())
                || (_KeyPointsDB -> size() != m_pDatabase -> size())) {
            throw std::runtime_error(TDBOW_LOG("Database not matched."));
        }
        for(size_t i = 0; i < _DescriptorsDB -> size(); i++) {
            if((*_DescriptorsDB)[i].size() != (*_KeyPointsDB)[i].size()) {
                throw std::runtime_error(TDBOW_LOG("Database not matched."));
            }
        }
        m_pDescriptorsDb = std::forward<DescriptorsDatabasePtr>(_DescriptorsDB);
        m_pKeyPointsDb = std::forward<KeyPointsDatabasePtr>(_KeyPointsDB);
    }
    // If you want to use the detector in stationary mode, should call `setStationary` before
    // e.g., `detector.setStationary().setDatabase(db, descDb, keysDb);`
    clear();
    return *this;
}

template<class TDatabase, size_t Dim>
TemplatedLoopDetector<TDatabase, Dim>&
TemplatedLoopDetector<TDatabase, Dim>::setVocabulary(Vocabulary&& _Vocab) noexcept(false) {
    return setVocabulary(VocabularyPtr(new Vocabulary(std::forward<Vocabulary>(_Vocab))));
}

template<class TDatabase, size_t Dim>
TemplatedLoopDetector<TDatabase, Dim>&
TemplatedLoopDetector<TDatabase, Dim>::setVocabulary(VocabularyPtr&& _Vocab) noexcept(false) {
    return setStationary(false).setDatabase(DatabasePtr(new TDatabase(
            m_sParams.geom_check == GEOM_ON && m_cKeyPointsMatcher == nullptr,
            m_sParams.di_levels, std::forward<VocabularyPtr>(_Vocab))));
}

/* ********************************************************************************
 *                                INNER METHODS                                   *
 ******************************************************************************** */

template<class TDatabase, size_t Dim>
void TemplatedLoopDetector<TDatabase, Dim>::updateDatabase(
        const DetectionResult::EntryId& _ID, const KeyPointArray& _KeyPoints,
        const DescriptorArray& _Descriptors) noexcept(false) {
    assert(!m_bStationaryDatabase && _Descriptors.size() == _KeyPoints.size());
    // m_image_keys and m_image_descriptors have the same length
    if(m_pDescriptorsDb -> size() == _ID) {
        m_pKeyPointsDb -> emplace_back(_KeyPoints);
        m_pDescriptorsDb -> emplace_back(_Descriptors);
    } else {
        auto& keys = (*m_pKeyPointsDb)[_ID];
        keys.clear();
        keys.insert(keys.end(), _KeyPoints.begin(), _KeyPoints.end());
        auto& descriptor = (*m_pDescriptorsDb)[_ID];
        descriptor.clear();
        descriptor.insert(descriptor.end(), _Descriptors.begin(), _Descriptors.end());
    }
}

template<class TDatabase, size_t Dim>
void TemplatedLoopDetector<TDatabase, Dim>::removeLowScores(
        TDBoW::QueryResults& _Results, double _Threshold) const noexcept {
    // _Results is already sorted in score, so we can do binary search to speed up
    TDBoW::Result aim(0, _Threshold);
    auto iter = std::lower_bound(
            _Results.begin(), _Results.end(), aim, std::greater<>());
    _Results.resize(static_cast<size_t>(iter - _Results.begin()));
    _Results.shrink_to_fit();
}

template<class TDatabase, size_t Dim>
void TemplatedLoopDetector<TDatabase, Dim>::computeIslands(
        const TDBoW::QueryResults& _Results, std::vector<Island>& _Islands) const noexcept {
    _Islands.clear();
    _Islands.shrink_to_fit();
    if(_Results.empty())return;
    if(_Results.size() == 1) {
        _Islands.emplace_back(_Results);
        return;
    }
    auto leftResults = _Results;
    // each iteration try to select one island
    while(!leftResults.empty()) {
        TDBoW::QueryResults group;
        // Add the first (best) entry into group
        auto entryId = leftResults.begin() -> Id;
        group.emplace_back(leftResults[0]);
        leftResults.erase(leftResults.begin());
        // Find nearby entries
        auto iter = leftResults.begin();
        while(iter != leftResults.end()) {
            auto offset = iter -> Id > entryId ?
                    iter -> Id - entryId : entryId - iter -> Id;
            if(offset <= m_sParams.max_intragroup_range) {
                group.emplace_back(*iter);
                iter = leftResults.erase(iter);
            } else {
                iter++;
            }
        }
        // Add island with enough entries
        if(group.size() >= m_sParams.min_matches_per_group) {
            _Islands.emplace_back(group);
        }
    }
}

template<class TDatabase, size_t Dim>
void TemplatedLoopDetector<TDatabase, Dim>::updateTemporalWindow(const Island& _Island) {
    // The index in ascending order should be granted.
    if(m_uEntryId - m_sWindow.last_query_id > m_sParams.max_distance_between_queries) {
        m_sWindow.clear();
    }
    if(m_sWindow.empty()) {
        m_sWindow.add(_Island, m_uEntryId);
        return;
    }
    if(!m_sWindow.last_matched_island.cross(_Island)) {
        auto gap = _Island.span.first >= m_sWindow.last_matched_island.span.second
                ? _Island.span.first - m_sWindow.last_matched_island.span.second + 1
                : m_sWindow.last_matched_island.span.first - _Island.span.second + 1;
        if(gap > m_sParams.max_distance_between_groups) {
            m_sWindow.clear();
        }
    }
    m_sWindow.add(_Island, m_uEntryId);
}

template<class TDatabase, size_t Dim>
MatchPairArray TemplatedLoopDetector<TDatabase, Dim>::KeyPointMatcherDI(
        const DetectionResult::EntryId& _HistoryEntryId,
        const DescriptorArray& _HistoryImgDesc,
        const TDBoW::FeatureVectorConstPtr& _FeatureVec,
        const DescriptorArray& _QueryImgDesc) {
    assert(_FeatureVec != nullptr);
    // Match descriptors between two images by distance (reduce calculation by DI)
    const auto& matchedFeature = m_pDatabase -> retrieveFeatures(_HistoryEntryId);
    MatchPairArray matches;
    auto matchIt = matchedFeature.begin();
    auto queryIt = _FeatureVec -> begin();
    while(matchIt != matchedFeature.end() && queryIt != _FeatureVec -> end()) {
        const auto& matchNode = matchIt -> first;
        const auto& queryNode = queryIt -> first;
        if(matchNode == queryNode) {
            // <indexInMatch, indexInQuery>
            const auto& pairs = findMatchesByDistance(
                    _HistoryImgDesc, matchIt -> second,
                    _QueryImgDesc, queryIt -> second);
            matches.insert(matches.end(), pairs.begin(), pairs.end());
            matchIt++; queryIt++;
        } else if(matchNode < queryNode) {
            matchIt = matchedFeature.lower_bound(queryNode);
            // matchIt = (first element >= queryNode)
        } else {
            queryIt = _FeatureVec -> lower_bound(matchNode);
            // queryIt = (first element >= matchNode)
        }
    }
    return matches;
}

template<class TDatabase, size_t Dim>
MatchPairArray TemplatedLoopDetector<TDatabase, Dim>::findMatchesByDistance(
        const DescriptorArray& _A, const std::vector<size_t>& _IdxA,
        const DescriptorArray& _B, const std::vector<size_t>& _IdxB,
        const typename Vocabulary::util::DistanceCallback& _Callback) const noexcept(false) {
    MatchPairArray results;
    results.reserve(std::min(_IdxA.size(), _IdxB.size()));
    if(_IdxA.empty() || _IdxB.empty())return results;
    for(const auto& idxA : _IdxA) {
        typedef typename Vocabulary::util::distance_type distance_type;
        size_t bestIdx = 0;
        distance_type bestScore[2] = {1e9};
        for(const auto& idxB : _IdxB) {
            auto score = _Callback(_A[idxA], _B[idxB]);
            if(score < bestScore[0]) {
                bestIdx = idxB;
                bestScore[1] = bestScore[0];
                bestScore[0] = score;
            } else if(score < bestScore[1]) {
                bestScore[1] = score;
            }
        }
        if(bestScore[0] / bestScore[1] <= m_sParams.max_neighbor_ratio) {
            bool alreadyMatched = false;
            for(auto& pair : results) {
                auto& idx = std::get<0>(pair);
                if(std::get<1>(pair) == bestIdx) {
                    if(_Callback(_A[idx], _B[bestIdx]) > bestScore[0]) {
                        idx = idxA;
                    }
                    alreadyMatched = true;
                    break;
                }
            }
            if(!alreadyMatched) {
                results.emplace_back(idxA, bestIdx, bestScore[0]);
            }
        }
    }
    return results;
}

} // namespace TDLoopDetector

#endif  // __ROCKAUTO_TDLD_TEMPLATED_LOOP_DETECTOR_HPP__
