#ifndef SEEKF_TYPES_H
#define SEEKF_TYPES_H

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> stateT;
typedef Eigen::Matrix<double, 6, 6> covT;
typedef Eigen::Matrix<double, 2, 1> inputT;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> msgT;
typedef Eigen::Matrix<double, 1, 1> singleT;
typedef Eigen::Matrix<double, 2, 1> doubleT;
typedef Eigen::Matrix<double, 3, 1> tripleT;
typedef Eigen::Matrix<double, 1, 1> singleCovT;
typedef Eigen::Matrix<double, 2, 2> doubleCovT;
typedef Eigen::Matrix<double, 3, 3> tripleCovT;
typedef Eigen::Matrix<double, 1, 6> singleObsMatrix;
typedef Eigen::Matrix<double, 2, 6> doubleObsMatrix;
typedef Eigen::Matrix<double, 3, 6> tripleObsMatrix;
typedef std::pair<stateT, covT> estimationT;

#endif