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
   * File Name     : PCLNdtGeometryChecker.hpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-27 11:50:39
   * Last Modified : smallchimney
   * Modified Time : 2019-12-27 17:21:05
************************************************************************* */
#ifndef __ROCKAUTO_TDLD_PC_PCL_NDT_GEOMETRY_CHECKER_HPP__
#define __ROCKAUTO_TDLD_PC_PCL_NDT_GEOMETRY_CHECKER_HPP__

#include "PCLRansacGeometryChecker.hpp"

#include <boost/make_shared.hpp>
#include <pcl/registration/ndt.h>

namespace TDLoopDetector {
namespace pc {

template <typename PointT = pcl::PointXYZ>
class PCLNdtGeometryChecker : public PCLRansacGeometryChecker<PointT> {
public:
    // Typedef for the PCLRansacGeometryChecker
    typedef PCLRansacGeometryChecker<PointT> RansacChecker;
    using RansacChecker::m_pSceneCloud;
    using RansacChecker::DIM;
    using RansacChecker::_transform;
    // PCL types define
    typedef pcl::NormalDistributionsTransform<PointT, PointT> Ndt;
    typedef typename RansacChecker::PointCloud PointCloud;
    typedef typename RansacChecker::PointCloudPtr PointCloudPtr;
    // Key point types define
    typedef typename RansacChecker::KeyPointArray KeyPointArray;
    typedef typename RansacChecker::PointPairsGeomChecker PointPairsGeomChecker;

    /**
     * @brief  The maximum number of NDT iterations
     * @author smallchimney
     * @param  _MaxNdtIter  Maximum limit of NDT iterations number
     */
    explicit PCLNdtGeometryChecker(unsigned _MaxNdtIter = 20)
            : RansacChecker(1., 10, true) {
        setNdtMaximumIterations(_MaxNdtIter)
                .setNdtTransformationEpsilon(1e-6)
                .setNdtResolution(1.0)
                .setNdtStepSize(.1);
    }

    /**
     * @brief  Current ISO C++ standard don't supported implement this in base class
     * @author smallchimney
     * @return The suitable `PointPairsGeomChecker` callback attached with
     *         current instance
     */
    PointPairsGeomChecker checker() noexcept override {
        return std::bind(&PCLNdtGeometryChecker<PointT>::doGeometryCheck,
                         this, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3, std::placeholders::_4);
    }

    /**
     * @brief  Geometry checking method based on PCL implement, using RANSAC
     *         indeed.
     * @author smallchimney
     * @param  _Matches          The given points pairs.
     * @param  _Transform (out)  Only if the there are geometric consistency
     *                           between the two scenes, s.t. return {@code true}
     *                           this field will be filled the transform between
     *                           the two scenes. (from model to scene)
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
     * @brief Set the maximum distance threshold between two correspondent
     *        points in source <-> target. If the distance is larger than
     *        this threshold, the points will be ignored in the alignment
     *        process. (default value is no limit)
     * @author smallchimney
     * @param  _CorrDistance  The maximum distance threshold between a point
     *                        and its nearest neighbor correspondent in order
     *                        to be considered in the alignment process.
     * @return               This instance
     */
    PCLNdtGeometryChecker& setMaxCorrespondenceDistance(const double _CorrDistance) {
        m_cNdt.setMaxCorrespondenceDistance(_CorrDistance);
        return *this;
    }

    /**
     * @brief Set the maximum number of iterations the internal optimization
     *        should run for.
     * @author smallchimney
     * @param _MaxIterations  The maximum number of iterations the internal
     *                        optimization of NDT should run for.
     * @return                This instance
     */
    PCLNdtGeometryChecker& setNdtMaximumIterations(const unsigned _MaxIterations) {
        m_cNdt.setMaximumIterations(_MaxIterations);
        return *this;
    }

    /**
     * @brief  Set/change the resolution of NDT's voxel grid.
     * @author smallchimney
     * @param  _Resolution  Side length of voxels.
     * @return              This instance
     */
    PCLNdtGeometryChecker& setNdtResolution(const float _Resolution) {
        m_cNdt.setResolution(_Resolution);
        return *this;
    }

    /**
     * @brief  Set the stop epsilon for the optimization iteration.
     * @param  _Epsilon  If the step offset is less than epsilon,
     *                   the optimization will be stopped.
     * @return           This instance
     */
    PCLNdtGeometryChecker& setNdtTransformationEpsilon(const double _Epsilon) {
        m_cNdt.setTransformationEpsilon(_Epsilon);
        return *this;
    }

    /**
     * @brief  Set/change the newton line search maximum step length.
     * @param  _Size Maximum step length
     * @return       This instance
     */
    PCLNdtGeometryChecker& setNdtStepSize(const double _Size) {
        m_cNdt.setStepSize(_Size);
        return *this;
    }

    /**
     * @brief  Get the final iteration number
     * @return The actual iteration times of last check.
     */
    unsigned getNdtFinalNumIteration() const noexcept {
        return static_cast<unsigned>(m_cNdt.getFinalNumIteration());
    }

protected:


private:

    /** @brief the PCL implemented NDT handle */
    Ndt m_cNdt;

};  // class PCLNdtGeometryChecker


/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

template <typename PointT>
bool PCLNdtGeometryChecker<PointT>::doGeometryCheck(
        const MatchPairArray& _Matches, Transform& _Transform,
        const KeyPointArray& _HistoryKp, const KeyPointArray& _QueryKp) {
    // RansacChecker will do the checking and cleanup the scene, so make it backup.
    auto sceneCloud = m_pSceneCloud;
    bool detected = RansacChecker::doGeometryCheck(
            _Matches, _Transform, _HistoryKp, _QueryKp);
    if(!detected)return false;
    m_cNdt.setInputTarget(sceneCloud);
    m_cNdt.setInputSource(_transform(_HistoryKp));
    static PointCloud output;
    m_cNdt.align(output, _Transform);
    detected = m_cNdt.hasConverged() && m_cNdt.getFitnessScore(20) < 1;
    _Transform = m_cNdt.getFinalTransformation();
    return detected;
}

/* ********************************************************************************
 *                               INTERNAL METHODS                                 *
 ******************************************************************************** */




}}  // namespace TDLoopDetector::pc

#endif //__ROCKAUTO_TDLD_PC_PCL_NDT_GEOMETRY_CHECKER_HPP__
