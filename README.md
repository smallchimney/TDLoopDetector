TDLoopDetector
=============

## Overview

TDLoopDetector is an open source C++ library to detect loops in a
sequence of pointclouds (or images) collected by a mobile robot.
It implements the algorithm presented in (todo: Still work-in-progress),
based on a bag-of-words database created from local descriptors,
and temporal and geometrical constraints.
The current implementation only support pointcloud (Still work-in-progress).
TDLoopDetector is based on the TDBoW library, so that it can work
with any other type of descriptor with little effort.

TDLoopDetector requires at least TDBoW and it's dependency libraries,
(Boost, Eigen and yaml-cpp).
PCL is recommend so we add support for it (Still work-in-progress).
By the way, OpenCV is still supported for CV users (Still work-in-progress).

(Still work-in-progress).TDLoopDetector has been tested on several
real datasets, yielding an execution time of ~9 ms to detect a loop
a in a sequence with more than 19000 images (without considering the feature extraction).
When BRIEF descriptors were used, the feature extraction and the loop
detection were performed in 16 ms on average.

## Citing

If you use this software in an academic work, please cite:

    @ARTICLE{
        Still work-in-progress
    }
