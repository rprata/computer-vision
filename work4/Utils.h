#ifndef __UTILS__
#define __UTILS__

#include <iostream>
#include <string>
#include <utility> 
#include <vector>
#include <cmath>

#include <iostream>
#include <fstream>

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

    Matrix3d Ti;
    Matrix3d Tii;

    Matrix3d F;
    Matrix3d E;
    vector<MatrixXd> Pls;

    vector<Vector3d> points3D;
    QVector<Vector3d> points2D_l1;
    QVector<Vector3d> points2D_l2;
    ofstream file;

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
    void generateTMatrix(void); //according 4.4 - page 107

    //RANSAC 
    vectorPairPoints generateRandPairs(int numberOfCorrespondences, int size);
    MatrixXd generateMatrixH2(vectorPairPoints vec);
    QVector<int> getRansacInliers(Matrix3d H, double threshold);
    double squaredEuclideanDistance(Vector3d a, Vector3d b);
    Matrix3d gaussNewton(Matrix3d H, QVector<Vector3d> pointsFirstImage, QVector<Vector3d> pointsSecondImage);
    Matrix3d ransac(double N, double threshold, int randomSize);
    
    Matrix3d generateF(vectorPairPoints vec);
    void generateE(void);
    void generateP(void);
    bool read3DPointsFromObj(const string& filename, int max_point_count);
    void obtain2DPointsCorrespondenceFrom3DPoints(void);

    QVector<VectorXd> get3DPointsByTriangulation(MatrixXd P, MatrixXd Pl);

    void saveInObj(const string filename, QVector<VectorXd> points3D);
    
    vector<MatrixXd> getPls(void) 
    {
        return Pls;
    }

    Matrix3d getK1(void) 
    {
        return K1;
    }

};


#endif