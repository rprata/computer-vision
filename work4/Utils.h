#ifndef __UTILS__
#define __UTILS__

#include <iostream>
#include <string>
#include <utility> 
#include <vector>
#include <cmath>

#include <QVector>

#include <eigen3/Eigen/Dense>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace std;
using namespace Eigen;
using namespace cv;

#define USING_QT

#define MIN_HESSIAN 250

#define FOCAL_DISTANCE 114.873
#define PIXEL_SZ_X 0.0130887
#define PIXEL_SZ_Y 0.0130887
#define CENTER_PX_X 1936
#define CENTER_PX_Y 1296

typedef vector< pair<Vector3d, Vector3d> > vectorPairPoints;

class Utils
{

private:
	Utils() {};	
    vectorPairPoints m_vectorPairPoints;

    Matrix3d K1;
    Matrix3d K2;
    MatrixXd P1;
    MatrixXd P2;
    Matrix3d R1;
    Matrix3d R2;
    MatrixXd Rt1;
    MatrixXd Rt2;
    Vector3d T1;
    Vector3d T2;
	
public:
	static Utils * getInstance()
    {
    	static Utils * instance = new Utils();
        return instance;
    }

    Utils(Utils const&);
    void operator=(Utils const&);

    void generateCameraMatrix(void);
    Matrix3d generateK(double focalDistance, double pixelSizeX, double pixelSizeY, double centerPxX, double centerPxY);
    vectorPairPoints generateMatchingPoints(const string& argv1, const string& argv2);

    //RANSAC 
    vectorPairPoints generateRandPairs(int numberOfCorrespondences, int size);
    MatrixXd generateMatrixH2(vectorPairPoints vec);
    QVector<int> getRansacInliers(Matrix3d H, float threshold);
    float squaredEuclideanDistance(Vector3d a, Vector3d b);
    Matrix3d gaussNewton(Matrix3d H, QVector<Vector3d> pointsFirstImage, QVector<Vector3d> pointsSecondImage);
    Matrix3d ransac(double N, double threshold, int randomSize);


};


#endif