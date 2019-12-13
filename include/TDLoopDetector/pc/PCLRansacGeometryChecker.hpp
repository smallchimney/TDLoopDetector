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
   * File Name     : PCLRansacGeometryChecker.hpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-12 13:18:53
   * Last Modified : smallchimney
   * Modified Time : 2019-12-15 20:03:16
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PC_PCL_RANSAC_GEOMETRY_CHECKER_HPP__
#define __ROCKAUTO_TDLD_PC_PCL_RANSAC_GEOMETRY_CHECKER_HPP__

#include "traits.h"
#include <TDLoopDetector/GeometryChecker.h>
#include <TDBoW/TemplatedDescriptor.hpp>

#include <pcl/recognition/cg/geometric_consistency.h>
#include <boost/make_shared.hpp>

namespace TDLoopDetector {
namespace pc {

template <typename PointT = pcl::PointXYZ>
class PCLRansacGeometryChecker : public GeometryChecker<traits::pcl_type<PointT>::DIM> {
public:
    // The dim of the point type
    static constexpr auto DIM = traits::pcl_type<PointT>::DIM;
    // PCL types define
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef pcl::GeometricConsistencyGrouping<PointT, PointT> Checker;
    typedef std::vector<pcl::Correspondences> CorrespondencesArray;
    // Key point types define
    typedef typename GeometryChecker<DIM>::KeyPoint      KeyPoint;
    typedef typename GeometryChecker<DIM>::KeyPoints     KeyPoints;
    typedef typename GeometryChecker<DIM>::KeyPointArray KeyPointArray;
    typedef typename GeometryChecker<DIM>::PointPairsGeomChecker PointPairsGeomChecker;

    /**
     * // todo: complete
     * @param _Resolution
     * @param _Threshold
     */
    PCLRansacGeometryChecker(double _Resolution, int _Threshold, bool _SceneModel = false);

    /**
     * @brief  Current ISO C++ standard don't supported implement this in base class
     * @author smallchimney
     * @return The suitable `PointPairsGeomChecker` callback attached with
     *         current instance
     */
    PointPairsGeomChecker checker() noexcept override {
        using namespace std::placeholders;
        return std::bind(&PCLRansacGeometryChecker<PointT>::doGeometryCheck,
                this, _1, _2, _3, _4);
    }

    /**
     * @brief  Geometry checking method based on PCL implement, using RANSAC
     *         indeed.
     * @author smallchimney
     * @param  _Matches          The given points pairs.
     * @param  _Transform (out)  Only if the there are geometric consistency
     *                           between the two scenes, s.t. return {@code true}
     *                           this field will be filled the transform between
     *                           the two scenes. (//todo: from where to where)
     * @param  _HistoryKp        The key points in history scene
     * @param  _QueryKp          The key points in current scene
     * @return                   Whether there are geometric consistency between
     *                           the two scenes
     */
    bool doGeometryCheck(const MatchPairArray& _Matches,
                         Transform& _Transform,
                         const KeyPointArray& _HistoryKp,
                         const KeyPointArray& _QueryKp) override;

    /**
     * @brief  Clear the previous match data
     * @author smallchimney
     */
    void clear() {
        m_aFoundedTf.clear();
        m_aFoundedTf.shrink_to_fit();
        m_aCorrespondences.clear();
        m_aCorrespondences.shrink_to_fit();
    }

    /**
     * @brief  Get the size of last check's result.
     * @author smallchimney
     * @return The number of possible instance (query image) founded in the
     *         scene (history image)
     */
    size_t size() const noexcept { return m_aFoundedTf.size(); }

    /**
     * @brief  Whether there are no any matched instance in the scene.
     * @author smallchimney
     * @return No any match in the two clouds, s.t. no geometric consistency.
     */
    bool empty() const noexcept { return size() == 0; }

    /**
     * @brief  Set the  //todo: ???
     * @author smallchimney
     * @param  _Resolution  //todo: ???
     * @return              The instance itself
     */
    PCLRansacGeometryChecker& setResolution(const double _Resolution) {
        m_cChecker.setGCSize(_Resolution);
        return *this;
    }

    /**
     * @brief  Set the  //todo: ???
     * @author smallchimney
     * @param  _Resolution  //todo: ???
     * @return              The instance itself
     */
    PCLRansacGeometryChecker& setThreshold(unsigned long _Threshold) {
        m_cChecker.setGCThreshold(_Threshold);
        return *this;
    }

    /**
     * @brief  Set the scene points, only needed in scene-model mode.
     * @author smallchimney
     * @param  _Cloud  Descriptors for each point in the scene.
     * @return         The instance itself
     */
    PCLRansacGeometryChecker& setSceneCloud(const PointCloudPtr& _Cloud) noexcept {
        m_pSceneCloud.reset(_Cloud);
        return *this;
    }

    /**
     * @brief  Set the scene points, only needed in scene-model mode.
     * @author smallchimney
     * @param  _Cloud  Descriptors for each point in the scene.
     * @return         The instance itself
     */
    PCLRansacGeometryChecker& setSceneCloud(const KeyPointArray& _Cloud) noexcept {
        m_pSceneCloud = _transform(_Cloud);
        return *this;
    }

    /**
     * @brief  Try to get the last check's transforms' matrix.
     * @author smallchimney
     * @return Return a constant reference of last check's transforms' matrix
     */
    const Transforms& getTransforms() const noexcept { return m_aFoundedTf; }

    /**
     * @brief  Try to get the last check's transforms' correspondences.
     * @author smallchimney
     * @return Return a constant reference of last check's transforms' correspondences
     */
    const CorrespondencesArray& getCorrespondences() const noexcept {
        return m_aCorrespondences;
    }

protected:
    /** @brief Store the last geometric consistency transform matrix */
    Transforms m_aFoundedTf;

    /** @brief Store the last geometric consistency correspondences relation */
    CorrespondencesArray m_aCorrespondences;

    /**
     * @brief Mode designed for RANSAC, will using scene-model check and generate
     *        a lot of false-positive matches. In this mode, each point in the scene
     *        will calculate the shortest distance on descriptor to model's key point.
     */
    const bool m_bSceneModelMode;

    /** @brief Full scene clouds, required when scene-model mode */
    PointCloudPtr m_pSceneCloud;

    /**
     * @brief  Transform key points into cloud, this method will not check the
     *         infinite position, because the indices will get error after filter,
     *         so please make sure the input key points are finite.
     * @author smallchimney
     * @param  _Keys  Inputted key points position (must be finite)
     * @return        PCL format point cloud pointer
     */
    PointCloudPtr _transform(const KeyPointArray& _Keys) const noexcept;

private:
    /** @brief PCL geometric consistency checker based on RANSAC */
    Checker m_cChecker;

};

/* ********************************************************************************
 *                        CONSTRUCTION && INITIALIZATION                          *
 ******************************************************************************** */

template <typename PointT>
PCLRansacGeometryChecker<PointT>::PCLRansacGeometryChecker(
        const double _Resolution, const int _Threshold, const bool _SceneModel)
        : m_bSceneModelMode(_SceneModel), m_cChecker() {
    setResolution(_Resolution).setThreshold(_Threshold);
}

/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

template <typename PointT>
bool PCLRansacGeometryChecker<PointT>::doGeometryCheck(
        const MatchPairArray& _Matches, Transform& _Transform,
        const KeyPointArray& _HistoryKp, const KeyPointArray& _QueryKp) {
    PointCloudPtr queryClouds;
    if(m_bSceneModelMode) {
        if(m_pSceneCloud == nullptr) {
            throw std::runtime_error(TDBOW_LOG("The scene cloud is required in "
                    "scene-model mode, please call `setScene()` before `detectLoop()`"));
        }
        queryClouds = m_pSceneCloud;
    } else {
        queryClouds = _transform(_QueryKp);
    }
    auto correspondences = boost::make_shared<pcl::Correspondences>();
    // Transform two clouds
    auto historyPoints = _transform(_HistoryKp);
    for(const auto& pair : _Matches) {
        // model, scene, distance
        correspondences -> emplace_back(
                std::get<0>(pair), std::get<1>(pair), std::get<2>(pair));
    }
    m_cChecker.setInputCloud(historyPoints);
    m_cChecker.setSceneCloud(queryClouds);
    m_cChecker.setModelSceneCorrespondences(correspondences);
    clear();
    if(!m_cChecker.recognize(m_aFoundedTf, m_aCorrespondences))return false;
    m_aCorrespondences.emplace_back(*correspondences);
    m_pSceneCloud.reset();
    if(!empty()) {
        _Transform = m_aFoundedTf[0];
        return true;
    }
    return false;
}

/* ********************************************************************************
 *                                 INNER METHODS                                  *
 ******************************************************************************** */

template <typename PointT>
typename PCLRansacGeometryChecker<PointT>::PointCloudPtr
PCLRansacGeometryChecker<PointT>::_transform(const KeyPointArray& _Keys) const noexcept {
    auto cloud = boost::make_shared<PointCloud>();
    cloud -> resize(_Keys.size());
    for(size_t i = 0; i < _Keys.size(); i++) {
        memcpy(cloud -> points[i].data, _Keys[i].data(), sizeof(float) * DIM);
    }
    return cloud;
}

}}  // namespace TDLoopDetector::pc

#endif //__ROCKAUTO_TDLD_PC_PCL_RANSAC_GEOMETRY_CHECKER_HPP__
