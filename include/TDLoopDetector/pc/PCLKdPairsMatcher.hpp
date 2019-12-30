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
   * File Name     : PCLKdPairsMatcher.hpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-14 15:28:39
   * Last Modified : smallchimney
   * Modified Time : 2019-12-27 14:24:26
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PC_PCL_KD_PAIRS_MATCHER_HPP__
#define __ROCKAUTO_TDLD_PC_PCL_KD_PAIRS_MATCHER_HPP__

#include <TDLoopDetector/Exception.h>
#include <TDLoopDetector/PairsMatcher.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <boost/make_shared.hpp>

namespace TDLoopDetector {
namespace pc {

template <typename TScalar, size_t L, size_t Dim = 3>
class PCLKdPairsMatcher : public PairsMatcher<TScalar, L, Dim> {
public:
    // Descriptor types define
    typedef typename PairsMatcher<TScalar, L, Dim>::DescriptorUtil DescriptorUtil;
    TDBOW_DESCRIPTOR_DEF(DescriptorUtil)
    // PCL types define
    typedef pcl::PointCloud<Descriptor> DescPointCloud;
    typedef typename DescPointCloud::Ptr DescPointCloudPtr;
    typedef pcl::KdTreeFLANN<Descriptor> DescKDTree;
    // Key point types define
    typedef typename PairsMatcher<TScalar, L, Dim>::KeyPoint      KeyPoint;
    typedef typename PairsMatcher<TScalar, L, Dim>::KeyPoints     KeyPoints;
    typedef typename PairsMatcher<TScalar, L, Dim>::KeyPointArray KeyPointArray;
    typedef typename PairsMatcher<TScalar, L, Dim>::KeyPointsMatcher KeyPointsMatcher;

    explicit PCLKdPairsMatcher(bool _SceneModel = true, float _Threshold = 0.2f)
            : m_bSceneModelMode(_SceneModel), m_pSceneDesc(nullptr), m_ulK(1) {
        m_fFilterFactor = std::max(1e-5f, std::min(1.f, _Threshold));
    }

    /** @brief Destructor */
    virtual ~PCLKdPairsMatcher() = default;

    /**
     * @brief  Current ISO C++ standard don't supported implement this in base class
     * @author smallchimney
     * @return The suitable `KeyPointsMatcher` callback attached with current instance
     */
    KeyPointsMatcher matcher() noexcept override {
        return std::bind(&PCLKdPairsMatcher<TScalar, L, Dim>::doKeyPointsMatch,
                         this, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3, std::placeholders::_4);
    }

    /**
     * @brief  Key points pairs match method based on PCL implement, using
     *         KD-tree indeed.
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

    /**
     * @brief  Set the scene descriptors, only needed in scene-model mode.
     * @author smallchimney
     * @param  _Ptr  Descriptors for each point in the scene.
     * @return       The instance itself
     */
    PCLKdPairsMatcher& setSceneDesc(const DescPointCloudPtr& _Ptr) noexcept {
        m_pSceneDesc.reset(_Ptr);
        return *this;
    }

    /**
     * @brief  Set the scene descriptors, only needed in scene-model mode.
     * @author smallchimney
     * @param  _Descriptors  Descriptors for each point in the scene.
     * @return     The instance itself
     */
    PCLKdPairsMatcher& setSceneDesc(const DescriptorArray& _Descriptors) noexcept {
        m_pSceneDesc.reset(new DescPointCloud);
        m_pSceneDesc -> points = _Descriptors;
        m_pSceneDesc -> resize(m_pSceneDesc -> points.size());
        return *this;
    }

    /**
     * @brief  Set the how many points in model will be selected for
     *         each point in scene.
     * @author smallchimney
     * @param  _K  The high limit of the selected points' number of each
     *             scene point.
     * @return     The instance itself
     */
    PCLKdPairsMatcher& setSelectedNum(const size_t _K) {
        m_ulK = _K;
        return *this;
    }

    /**
     * @brief  Get the current setting selected number.
     * @author smallchimney
     * @return The current setting selected number.
     */
    size_t getSelectedNum() const noexcept {
        return m_ulK;
    }

protected:

    /**
     * @brief Mode designed for RANSAC, will using scene-model check and generate
     *        a lot of false-positive matches. In this mode, each point in the scene
     *        will calculate the shortest distance on descriptor to model's key point.
     */
    const bool m_bSceneModelMode;

    /** @brief Whole scene descriptors for each points, required when scene-model mode */
    DescPointCloudPtr m_pSceneDesc;

    /**
     * @brief  Match pairs based on descriptor distance, using KD-Tree find closest
     *         point in the model points for each scene points. The number of scene
     *         points should be much greater than model.
     *         This method will generate lots of false-positive matches, recommend
     *         to use RANSAC on the result matches.
     *
     *         The the TDLoopDetector will not use full descriptors, so
     *         `setSceneDesc()` is required, in this case, the history cloud's full
     *         descriptors are dropped, so we use current query cloud as scene,
     *         and use history descriptors as model.
     * @author smallchimney
     * @param  _FilterFactor  Dynamic distance filter threshold in percent, value
     *                        in [1e-5, 1]. s.t., will keep how many percent matches
     *                        will be kept.
     * @param  _K             Number of closest pairs will be selected
     * @param  _ModelDesc     Descriptors for key points in the model.
     * @param  _SceneDesc     Full descriptors for each point in the scene.
     * @return
     */
    static MatchPairArray _sceneModelMatcher(
            float _FilterFactor, size_t _K,
            const DescPointCloudPtr& _ModelDesc,
            const DescPointCloudPtr& _SceneDesc);

private:

    /**
     * @brief  Dynamic distance filter threshold in percent, value in [1e-5, 1]
     *         s.t., will keep how many percent matches will be kept.
     * @author smallchimney
     */
    float m_fFilterFactor;

    /**
     * @brief  Find closest K point from query cloud for each point in history cloud
     */
    size_t m_ulK;

};

/* ********************************************************************************
 *                        CONSTRUCTION && INITIALIZATION                          *
 ******************************************************************************** */

/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

template <typename TScalar, size_t L, size_t Dim>
MatchPairArray PCLKdPairsMatcher<TScalar, L, Dim>::doKeyPointsMatch(
        const DescriptorArray& _HistoryDesc, const KeyPointArray& _HistoryKp,
        const DescriptorArray& _QueryDesc, const KeyPointArray& _QueryKp) {
    assert(_HistoryDesc.size() == _HistoryKp.size() && _QueryDesc.size() == _QueryKp.size());
    if(!m_bSceneModelMode) {
        throw LogicException(TDBOW_LOG("The model-model mode hadn't been complete yet."));
    } else {
        if(m_pSceneDesc == nullptr) {
            throw NotInitailizedException(TDBOW_LOG("The scene descriptors is required in "
                    "scene-model mode, please call `setSceneDesc()` before `detectLoop()`"));
        }
        auto modelDesc = boost::make_shared<DescPointCloud>();
        modelDesc -> points = _HistoryDesc;
        modelDesc -> resize(modelDesc -> points.size());
        auto ret = _sceneModelMatcher(m_fFilterFactor, m_ulK, modelDesc, m_pSceneDesc);
        m_pSceneDesc.reset();
        return ret;
    }
}

/* ********************************************************************************
 *                                 INNER METHODS                                  *
 ******************************************************************************** */

template <typename TScalar, size_t L, size_t Dim>
MatchPairArray PCLKdPairsMatcher<TScalar, L, Dim>::_sceneModelMatcher(
        const float _FilterFactor, const size_t _K,
        const DescPointCloudPtr& _ModelDesc, const DescPointCloudPtr& _SceneDesc) {
    MatchPairArray result;
    result.reserve(_SceneDesc -> size());
    // For each point in scene, search in model.
    DescKDTree searchTree;
    searchTree.setInputCloud(_ModelDesc);
    // Iterate the scene points
    std::map<size_t, MatchPairArray> matches;
    for(int i = 0; i < _SceneDesc -> size(); ++i) {
        std::vector<int> indices(_K);
        std::vector<float> squaredDis(_K);
        auto ret = searchTree.nearestKSearch(_SceneDesc -> at(i), _K, indices, squaredDis);
        if(ret) {
            for(size_t j = 0; j < ret; j++) {
                auto iter = matches.find(indices[j]);
                if(iter == matches.end()) {
                    MatchPairArray arr = {std::make_tuple(indices[j], i, squaredDis[j])};
                    matches.insert(std::make_pair(indices[j], arr));
                } else {
                    iter -> second.emplace_back(indices[j], i, squaredDis[j]);
                }
            }
        }
    }
    for(auto& pair : matches) {
        auto& arr = pair.second;
        std::sort(arr.begin(), arr.end(), [](const auto& _A, const auto& _B) {
            return std::get<2>(_A) < std::get<2>(_B);
        });
        auto dynamicLimit = static_cast<size_t>(arr.size() * _FilterFactor);
        if(dynamicLimit == 0)dynamicLimit++;    // arr is granted not empty
        assert(dynamicLimit >= 0 && dynamicLimit <= arr.size());
        const auto value = std::get<2>(arr[dynamicLimit]);
        auto iter = arr.begin();
        while(iter != arr.end() && std::get<2>(*iter) <= value) {
            iter++;
        }
        result.insert(result.end(), arr.begin(), iter);
    }
    return result;
}

}}  // namespace TDLoopDetector::pc

#endif //__ROCKAUTO_TDLD_PC_PCL_KD_PAIRS_MATCHER_HPP__
