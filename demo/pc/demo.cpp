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
   * File Name     : demo.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2019-12-08 17:35:35
   * Last Modified : smallchimney
   * Modified Time : 2019-12-14 22:26:24
************************************************************************* */
#include <TDLoopDetector/pc/PCBridge.h>

// PCL feature extractor
#include <pcl/io/pcd_io.h>
#pragma GCC diagnostic ignored "-Wpedantic"
#include <pcl/keypoints/harris_3d.h>
#ifdef FOUND_OPENMP
#include <pcl/features/fpfh_omp.h>
#else
#include <pcl/features/fpfh.h>
#endif

#include <chrono>

using std::cout; using std::endl;
using TDBoW::FPFH33Vocabulary;
using TDBoW::FPFH33Database;
// TDBoW type define
typedef FPFH33Vocabulary::Descriptor     Descriptor;
typedef FPFH33Vocabulary::ConstDataSet   ConstDataSet;
typedef FPFH33Vocabulary::DescriptorsSet DescriptorsSet;
// TDLoopDetector type define
typedef TDLoopDetector::pc::PCLRansacGeometryChecker<> Checker;
typedef typename Checker::KeyPointArray KeyPointArray;
typedef std::vector<KeyPointArray> KeyPointArraySet;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

std::vector<std::string> loadFeatures(DescriptorsSet&, KeyPointArraySet&);
void buildVocab(const ConstDataSet&, const DescriptorsSet&);
void testLoopDetect(const DescriptorsSet&,
        const KeyPointArraySet&, const std::vector<std::string>&);

// ----------------------------------------------------------------------------

int main(int argc, const char* argv[]) {
    // Load files, calculate FPFH-33 descriptors and do transform
    // DescriptorArray and Descriptor are recommended types when query
    DescriptorsSet features;
    KeyPointArraySet keypoints;
    auto files = loadFeatures(features, keypoints);

    // In this simple case, we had not prepare too many data, so we
    // use the same data for both create and query.
    // `make_shared`(inner method) will drop the original data, so make copy
    auto copy = features;
    // DataSet type is only used in vocabulary create.
    auto dataset = FPFH33Vocabulary::util::make_const(copy);
    // Vocabulary testing
    buildVocab(dataset, features);

    // Loop detector testing
    testLoopDetect(features, keypoints, files);

    return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------

std::vector<std::string> loadFeatures(DescriptorsSet& _Features, KeyPointArraySet& _KeyPoints) {
    using namespace boost::filesystem;
    const auto resourceDir = path(PKG_DIR)/"demo/pc/pointclouds";
    // Automatically find all `.pcd` files
    std::vector<path> files;
    for(const auto& file : recursive_directory_iterator(resourceDir)) {
        if(!is_regular_file(file))continue;
        const auto& filePath = file.path();
        auto extension = filePath.extension().native();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        if(extension != ".pcd")continue;
        files.emplace_back(filePath);
    }
    // Reserve the features space
    _Features.clear();
    _Features.shrink_to_fit();
    _Features.resize(files.size());
    // Reserve the keypoints space
    _KeyPoints.clear();
    _KeyPoints.shrink_to_fit();
    _KeyPoints.resize(files.size());
    // Iterator each file
    typedef pcl::PointCloud<pcl::PointXYZ>    PointCloudXYZ;
    typedef pcl::PointCloud<pcl::PointXYZI>   PointCloudXYZI;
    typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
#ifdef FOUND_OPENMP
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(std::thread::hardware_concurrency());
#else
    pcl::FPFHEstimation<PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
#endif
    std::vector<std::string> names(0);
    names.reserve(files.size());
    for(size_t i = 0; i < files.size(); i++) {
        // Load pointcloud
        auto input = boost::make_shared<PointCloudXYZ>();
        if(pcl::io::loadPCDFile(files[i].native(), *input) == -1)continue;
        auto output = boost::make_shared<PointCloudXYZI>();
        // Calculate keypoints using 3D harris
        harris.setInputCloud(input);
        harris.setNonMaxSupression(true);
        harris.setRadius(0.5f);
        harris.setThreshold(0.01f);
        harris.compute(*output);
        auto indices = harris.getKeypointsIndices();
        // Compute normal for each points
        auto normals = boost::make_shared<PointCloudNormal>();
        auto kdTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
        ne.setInputCloud(input);
        ne.setSearchMethod(kdTree);
        ne.setKSearch(10);
        ne.compute(*normals);
        // Compute descriptors for each points
        auto descriptors = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
        fpfh.setInputCloud(input);
        fpfh.setInputNormals(normals);
        fpfh.setSearchMethod(kdTree);
        fpfh.setRadiusSearch(0.5f);
        fpfh.compute(*descriptors);
        // Collect the descriptors
        typedef FPFH33Vocabulary::Descriptor Descriptor;
        auto& feature = _Features[i];
        feature.resize(indices -> indices.size());
        auto& keypoints = _KeyPoints[i];
        keypoints.resize(indices -> indices.size());
        for(size_t j = 0; j < indices -> indices.size(); j++) {
            const auto& index = static_cast<size_t>(indices -> indices[j]);
            const auto& descriptor = descriptors -> at(index);
            const auto& point = output -> points[j];
            feature[j] = Descriptor::Map(
                    descriptor.histogram, 1, pcl::FPFHSignature33::descriptorSize());
            keypoints[j] = Checker::KeyPoint::Map(point.data, Checker::DIM, 1);
        }
        names.emplace_back(files[i].filename().native());
    }
    return names;
}

// ----------------------------------------------------------------------------

void buildVocab(const ConstDataSet& _DataSet, const DescriptorsSet& _Features) {
    // branching factor and depth levels
    using namespace TDBoW;
    const int k = 6;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType score = L1_NORM;
    FPFH33Vocabulary vocab(k, L, weight, score);

    size_t count = 0;
    for(const auto& image : _DataSet) {
        count += image.size();
    }
    cout << "Features size: " << count << endl;

    cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
    using namespace std::chrono;
    auto start = system_clock::now();
    vocab.create(_DataSet);
    auto end = system_clock::now();
    std::cout << "Spent time: " << duration_cast<milliseconds>(end - start).count() << " ms." << endl;
    cout << "... done!" << endl;

    vocab.stopWords(0.5);
    cout << "Vocabulary stop words by weight: 0.5" << endl;
    cout << "Vocabulary information: " << endl << vocab << endl;

    // save the vocabulary to disk
    cout << endl << "Saving vocabulary..." << endl;
    vocab.save("small_voc.bin.qp");
    cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void testLoopDetect(const DescriptorsSet& _Features,
        const KeyPointArraySet& _KeyPoints, const std::vector<std::string>& _Names) {
    using namespace TDLoopDetector;
    typedef TemplatedLoopDetector<FPFH33Database> Detector;
    cout << "Creating a loop detector based on vocabulary..." << endl;
    // load the vocabulary from disk
    Checker checker(20, 2);  // todo: alter the parameters
    Detector::params params;
    // in this case, the sequence is not consecutive, so make some uncommon trick
    params.dead_zone = 0;   // Forbid the dead zone check
    params.k = 0;           // Drop the temporal consistency checking
    Detector detector(checker.checker(), FPFH33Vocabulary("small_voc.bin.qp"), params);

    // query the features and then add images to the database
    cout << "Querying the loop detector: " << endl;
    cout << "===================================="
            "====================================" << endl;
    assert(_Features.size() == _KeyPoints.size());
    for(size_t i = 0; i < _Features.size(); i++) {
        cout << "Try to detect image " << _Names[i] << "..." << endl;
        DetectionResult result;
        auto transform = detector.detectLoop(_KeyPoints[i], _Features[i], result);
        auto matchedFile = result.bestCandidate ? _Names[result.bestCandidate -> id] : "";
        if(result.detected()) {
            cout << "- Loop found with image " << matchedFile << "!" << endl;
            cout << "transform matrix:" << endl << transform << endl;
        } else {
            cout << "- No loop: ";
            switch(result.status) {
                case CLOSE_MATCHES_ONLY:
                    cout << "All the images in the database are very recent" << endl;
                    break;

                case NO_DB_RESULTS:
                    cout << "There are no matches against the database (few features in"
                            " the image?)" << endl;
                    break;

                case LOW_NSS_FACTOR:
                    cout << "Little overlap between this image and the previous one"
                         << endl;
                    break;

                case LOW_SCORES:
                    cout << "No match reaches the score threshold (alpha: " <<
                         params.alpha << ")" << endl;
                    break;

                case LOW_GROUPS_SCORE:
                    cout << "All match in low score for a long range (alpha: " <<
                         params.alpha << ")" << endl;
                    break;

                case NO_GROUPS:
                    cout << "Not enough close matches to create groups. "
                         << "Best candidate: " << matchedFile << endl;
                    break;

                case NO_TEMPORAL_CONSISTENCY:
                    cout << "No temporal consistency (k: " << params.k << "). "
                         << "Best candidate: " << matchedFile << endl;
                    break;

                case NO_GEOMETRICAL_CONSISTENCY:
                    cout << "No geometrical consistency. Best candidate: "
                         << matchedFile << endl;
                    break;

                default:
                    break;
            }
        }
        cout << "===================================="
                "====================================" << endl;
    }
}

// ----------------------------------------------------------------------------
