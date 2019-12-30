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
   * File Name     : GroupClusterMatcher.hpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-25 21:22:02
   * Last Modified : smallchimney
   * Modified Time : 2019-12-27 17:40:03
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PC_GROUP_CLUSTER_MATCHER_HPP__
#define __ROCKAUTO_TDLD_PC_GROUP_CLUSTER_MATCHER_HPP__

#include "PCLKdPairsMatcher.hpp"
#include "traits.h"

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/dynamic_bitset.hpp>

namespace TDLoopDetector {
namespace pc {

template <typename TScalar, size_t L, typename PointT = pcl::PointXYZ>
class GroupClusterMatcher : public PCLKdPairsMatcher<TScalar, L, traits::pcl_type<PointT>::DIM> {
public:
    // The dim of the point type
    static constexpr auto DIM = traits::pcl_type<PointT>::DIM;
    typedef PCLKdPairsMatcher<TScalar, L, DIM> KdMatcher;
    // Descriptor types define
    typedef typename KdMatcher::DescriptorUtil DescriptorUtil;
    TDBOW_DESCRIPTOR_DEF(DescriptorUtil)
    // Key point types define
    typedef typename KdMatcher::KeyPoint      KeyPoint;
    typedef typename KdMatcher::KeyPoints     KeyPoints;
    typedef typename KdMatcher::KeyPointArray KeyPointArray;
    typedef typename KdMatcher::KeyPointsMatcher KeyPointsMatcher;
    // PCL types define
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::search::KdTree<PointT> KdTree;

    explicit GroupClusterMatcher(double _NeighborRange = 2.,
            float _KdFilter = .3f, double _NeighborFilter = .3)
            : KdMatcher(true, _KdFilter), NEIGHBOR_RANGE(_NeighborRange),
            CLUSTER_THRESHOLD(_NeighborFilter), m_pSceneCloud(nullptr) {
    }

    /** @brief Destructor */
    virtual ~GroupClusterMatcher() = default;

    /**
     * @brief  Current ISO C++ standard don't supported implement this in base class
     * @author smallchimney
     * @return The suitable `KeyPointsMatcher` callback attached with current instance
     */
    KeyPointsMatcher matcher() noexcept override {
        using namespace std::placeholders;
        return std::bind(&GroupClusterMatcher<TScalar, L, PointT>::doKeyPointsMatch,
                         this, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3, std::placeholders::_4);
    }

    /**
     * @brief  Set the scene cloud, should be called before every match.
     * @author smallchimney
     * @param  _Ptr  Descriptors for each point in the scene.
     */
    GroupClusterMatcher& setSceneCloud(const PointCloudPtr& _Ptr) noexcept {
        m_pSceneCloud = _Ptr;
        return *this;
    }

    /**
     * @brief  Set the scene cloud, should be called before every match.
     * @author smallchimney
     * @param  _Descriptors  Descriptors for each point in the scene.
     */
    GroupClusterMatcher& setSceneCloud(const KeyPointArray& _Points) noexcept {
        m_pSceneCloud = _transform(_Points);
        return *this;
    }

    /**
     * @brief  Key points pairs match method based on grouped cluster.
     * @author smallchimney
     * @param  _HistoryDesc  The history cloud's descriptors in order.
     * @param  _HistoryKp    The point position for the history descriptors, in
     *                       history cloud's coordinate system.
     * @param  _QueryDesc    The (current) query cloud's descriptors in order.
     * @param  _QueryKp      The point position for the history descriptors, in
     *                       (current) query cloud's coordinate system.
     * @return               Matched key points pairs between two cloud.
     */
    MatchPairArray doKeyPointsMatch(
            const DescriptorArray& _HistoryDesc,
            const KeyPointArray& _HistoryKp,
            const DescriptorArray& _QueryDesc,
            const KeyPointArray& _QueryKp) override;

protected:

    /** @brief Range threshold for find the neighborhood */
    const double NEIGHBOR_RANGE;

    /** @brief Filter limit when clustering */
    const double CLUSTER_THRESHOLD;

    /** @brief Whole scene pointcloud, required before each match */
    PointCloudPtr m_pSceneCloud;

    /**
     * @brief  transform points array to PCL pointcloud type
     * @author smallchimney
     * @param  _Points  the original points array
     * @return          the pointer to transformed PCL structure pointcloud
     */
    static PointCloudPtr _transform(const KeyPointArray& _Points);

    typedef std::map<size_t, MatchPairArray> SplitMatches;

    /**
     * @brief  split raw matches according to model's index.
     * @author smallchimney
     * @param  _Matches  The original matches calculated from KdMatcher,
     *                   corresponding the shortest point in model for
     *                   each point in scene. (filtered the farthest points
     *                   based on {@code KdMatcher::m_fFilterFactor})
     * @return           The split matches based on model's index.
     */
    static inline SplitMatches _splitMatches(const MatchPairArray& _Matches);

    /**
     * @brief  preform non maximum suppression on split matches,
     *         reduce noise and down sample the matches.
     * @author smallchimney
     * @param  _In        The split matches
     * @param  _Out (out) The output matches
     * @return            size of output matches
     */
    size_t _nonMaximumSuppression(
            const SplitMatches& _In, SplitMatches& _Out) const noexcept;

    /**
     * @brief  Group filter for all key points in the model,
     *         indeed contact all filtered matches for each key point.
     * @author smallchimney
     * @param _Matches    Whole matches of the model.
     * @param _Size       The size of the split matches.
     * @param _HistoryKp  The position of the key points, used for find
     *                    neighbors for the specific key point
     * @return            The contacted matches.
     */
    MatchPairArray _groupFilter(const SplitMatches& _Matches,
            size_t _Size, const PointCloudConstPtr& _HistoryKp) const noexcept(false);

    /**
     * @brief  Group filter for specific key point in the model,
     *         find the matches voted by key point's neighbors.
     * @author smallchimney
     * @param  _Matches  Whole matches of the model, should be selected
     *                   according to the {@code _Indices}
     * @param  _Clouds   The split scene clouds based on model key points
     * @param  _Indices  Key points cluster's indices, the first one will
     *                   be regarded as the specific key point.
     * @return           All filtered matches corresponding with the
     *                   specific key point.
     */
    void _groupFilter(const SplitMatches& _Matches,
            const std::vector<PointCloudPtr>& _Clouds,
            const std::vector<int>& _Indices,
            MatchPairList& _Clusters) const noexcept;

};

/* ********************************************************************************
 *                        CONSTRUCTION && INITIALIZATION                          *
 ******************************************************************************** */

/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

template <typename TScalar, size_t L, typename PointT>
MatchPairArray GroupClusterMatcher<TScalar, L, PointT>::doKeyPointsMatch(
        const DescriptorArray& _HistoryDesc, const KeyPointArray& _HistoryKp,
        const DescriptorArray& _QueryDesc, const KeyPointArray& _QueryKp) {
    assert(_HistoryDesc.size() == _HistoryKp.size() && _QueryDesc.size() == _QueryKp.size());
    if(m_pSceneCloud == nullptr) {
        throw std::runtime_error(TDBOW_LOG("The scene cloud is required, "
                                 "please call `setSceneCloud()` before `detectLoop()`"));
    }
    auto prevSetting = KdMatcher::getSelectedNum();
    KdMatcher::setSelectedNum(1);
    auto rawMatches = KdMatcher::doKeyPointsMatch(
            _HistoryDesc, _HistoryKp, _QueryDesc, _QueryKp);
    KdMatcher::setSelectedNum(prevSetting);
    SplitMatches splitMatches;
    auto size = _nonMaximumSuppression(_splitMatches(rawMatches), splitMatches);
    return _groupFilter(splitMatches, size, _transform(_HistoryKp));
}

/* ********************************************************************************
 *                               INTERNAL METHODS                                 *
 ******************************************************************************** */

template <typename TScalar, size_t L, typename PointT>
typename GroupClusterMatcher<TScalar, L, PointT>::PointCloudPtr
GroupClusterMatcher<TScalar, L, PointT>::_transform(const KeyPointArray& _Points) {
    auto cloud = boost::make_shared<PointCloud>();
    if(_Points.empty())return cloud;
    cloud -> resize(_Points.size());
    if(sizeof(cloud -> at(0)) == sizeof(_Points[0])) {
        memcpy(cloud -> at(0).data, _Points[0].data(),
               sizeof(_Points[0]) * _Points.size());
    } else {
        for(size_t i = 0; i < _Points.size(); i++) {
            memcpy(cloud -> at(i).data, _Points[i].data(), sizeof(float) * DIM);
        }
    }
}

template <typename TScalar, size_t L, typename PointT>
typename GroupClusterMatcher<TScalar, L, PointT>::SplitMatches
GroupClusterMatcher<TScalar, L, PointT>::_splitMatches(const MatchPairArray& _Matches) {
    assert(!_Matches.empty());
    SplitMatches splitMatches{};
    auto prevIdx = std::get<0>(_Matches[0]);
    auto prevIter = _Matches.begin();
    for(auto curtIter = prevIter; curtIter != _Matches.end(); curtIter++) {
        const auto& curtIdx = std::get<0>(*curtIter);
        if(curtIdx != prevIdx) {
            MatchPairArray copy(static_cast<size_t>(curtIter - prevIter));
            std::copy(prevIter, curtIter, copy.begin());
            splitMatches.insert(splitMatches.end(), std::make_pair(prevIdx, std::move(copy)));
            prevIdx = curtIdx; prevIter = curtIter;
        }
    }
    MatchPairArray copy(static_cast<size_t>(_Matches.end() - prevIter));
    std::copy(prevIter, _Matches.end(), copy.begin());
    splitMatches.insert(splitMatches.end(), std::make_pair(prevIdx, std::move(copy)));
    return splitMatches;
}

template <typename TScalar, size_t L, typename PointT>
size_t GroupClusterMatcher<TScalar, L, PointT>::_nonMaximumSuppression(
        const SplitMatches& _In, SplitMatches& _Out) const noexcept {
    assert(&_In != &_Out);
    _Out.clear();
    const double NMS_THRESHOLD = NEIGHBOR_RANGE / 3;
    size_t count = 0;
    for(const auto& pair : _In) {
        auto selectedMatches = pair.second;
        // Generate scene cloud for the selected model point
        auto sceneCloud = boost::make_shared<PointCloud>();
        sceneCloud -> points.reserve(selectedMatches.size());
        for(const auto& match : selectedMatches) {
            sceneCloud -> points.emplace_back(m_pSceneCloud -> at(std::get<1>(match)));
        }
        sceneCloud -> resize(sceneCloud -> points.size());

        KdTree radiusSearcher;
        std::vector<int> nearbyIndices;
        std::vector<float> neighborSquaredDis;
        radiusSearcher.setInputCloud(sceneCloud);
        boost::dynamic_bitset<> filteredPoints(selectedMatches.size());
        // Since the selected matches are already ordered by distance
        for(size_t i = 0; i < selectedMatches.size(); i++) {
            if(filteredPoints[i])continue;
            radiusSearcher.radiusSearch(
                    m_pSceneCloud -> at(std::get<1>(selectedMatches[i])),
                    NMS_THRESHOLD, nearbyIndices, neighborSquaredDis);
            // Skip the searched point itself
            for(size_t j = 1; j < nearbyIndices.size(); j++) {
                filteredPoints.set(static_cast<size_t>(nearbyIndices[j]));
            }
        }
        // Remove from the selected matches
        for(auto i = static_cast<int>(selectedMatches.size() - 1); i >= 0; i--) {
            if(filteredPoints[i]) {
                selectedMatches.erase(selectedMatches.begin() + i);
            }
        }
        count += selectedMatches.size();
        _Out.insert(_Out.end(), std::make_pair(
                pair.first, std::move(selectedMatches)));
    }
    return count;
}

template <typename TScalar, size_t L, typename PointT>
MatchPairArray GroupClusterMatcher<TScalar, L, PointT>::_groupFilter(
        const SplitMatches& _Matches, const size_t _Size,
        const PointCloudConstPtr& _HistoryKp) const noexcept(false) {
    // Split scene according to match
    std::vector<PointCloudPtr> clouds(_HistoryKp -> size());
    for(size_t i = 0; i < clouds.size(); i++) {
        auto& cloud = clouds[i];
        auto iter = _Matches.find(i);
        if(iter == _Matches.end())continue;
        const auto& matches = iter -> second;
        cloud.reset(new PointCloud);
        cloud -> points.reserve(matches.size());
        for(const auto& match : matches) {
            cloud -> points.emplace_back(
                    m_pSceneCloud -> at(std::get<1>(match)));
        }
        cloud -> resize(cloud -> points.size());
    }
    // Filter based on neighbors
    KdTree modelSearcher;
    modelSearcher.setInputCloud(_HistoryKp);
    MatchPairArray results;
    results.reserve(_Size);
    for(const auto& pair : _Matches) {
        const auto& selectedIdx = pair.first;
        const auto& matches = pair.second;
        assert(selectedIdx < _HistoryKp -> size());
        if(matches.empty())continue;
        // Find neighbors
        const auto selectedP = _HistoryKp -> at(selectedIdx);
        std::vector<int> neighborsIndices;
        std::vector<float> squaredDis;
        modelSearcher.radiusSearch(selectedP, NEIGHBOR_RANGE, neighborsIndices, squaredDis);
        assert(!neighborsIndices.empty());
        if(neighborsIndices[0] != selectedIdx) {
            throw FormatException(TDBOW_LOG("Duplicated key points are forbade."));
        }
        MatchPairList clusters;
        _groupFilter(_Matches, clouds, neighborsIndices, clusters);
        auto iter = results.end();
        results.resize(results.size() + clusters.size());
        std::copy(clusters.begin(), clusters.end(), iter);
    }
    return results;
}

template <typename TScalar, size_t L, typename PointT>
void GroupClusterMatcher<TScalar, L, PointT>::_groupFilter(
        const SplitMatches& _Matches, const std::vector<PointCloudPtr>& _Clouds,
        const std::vector<int>& _Indices, MatchPairList& _Clusters) const noexcept {
    assert(!_Indices.empty());
    _Clusters.clear();
    if(_Indices.size() == 1)return;
    const auto& matches = _Matches.find(static_cast<size_t>(_Indices[0])) -> second;
    // Filter the useless neighbors
    auto indices = _Indices;
    auto iter = indices.begin() + 1;
    for(size_t i = 1; i < _Indices.size(); i++) {
        const auto& cloud = _Clouds[_Indices[i]];
        if(!cloud || cloud -> empty()) {
            iter = indices.erase(iter);
        } else {
            iter++;
        }
    }
    // Remove the non-grouping matches
    const auto limit = static_cast<size_t>(
            (indices.size() - 1) * CLUSTER_THRESHOLD);
    for(const auto& match : matches) {
        size_t count = 0;
        const auto& queryP = m_pSceneCloud -> at(std::get<1>(match));
        for(size_t i = 1; i < indices.size(); i++) {
            const auto& cloud = _Clouds[indices[i]];
            KdTree radiusSearcher;
            std::vector<int> nearbyIndices;
            std::vector<float> distances;
            radiusSearcher.setInputCloud(cloud);
            if(radiusSearcher.radiusSearch(queryP,
                    NEIGHBOR_RANGE, nearbyIndices, distances)) {
                count++;
            }
        }
        if(count >= limit) {
            _Clusters.emplace_back(match);
        }
    }
}

}}  // namespace TDLoopDetector::pc

#endif //__ROCKAUTO_TDLD_PC_GROUP_CLUSTER_MATCHER_HPP__
