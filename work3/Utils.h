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

typedef vector< pair<Vector3d, Vector3d> > vectorPairPoints;

class Utils
{

private:
	Utils() {};	
	vectorPairPoints m_vectorPairPoints;
    Matrix3d Hn;
    Matrix3d H;
    Matrix3d Ti;
    Matrix3d Tii;

public:
	static Utils * getInstance()
    {
    	static Utils * instance = new Utils();
        return instance;
    }

    Utils(Utils const&);
    void operator=(Utils const&);


	vectorPairPoints generateMatchingPoints(const string& argv1, const string& argv2); //using opencv - sift
	void printVectorPairPoints(void);
    void calculateDLT(void); //according table 4.1 - page 91
    void printMatrixHn(void);
    void printMatrixH(void);
    MatrixXd getMatrixH(void);
    void generateTMatrix(void); //according 4.4 - page 107

    //RANSAC 
    vectorPairPoints generateRandPairs(int numberOfCorrespondences, int size);
    MatrixXd generateMatrixH2(vectorPairPoints vec);

    //I understand, but it's difficult to implement. Based Flavio's homework (sorry :´()
    QVector<int> getRansacInliers(Matrix3d H, float threshold);
    float squaredEuclideanDistance(Vector3d a, Vector3d b);
    Matrix3d gaussNewton(Matrix3d H, QVector<Vector3d> pointsFirstImage, QVector<Vector3d> pointsSecondImage);
    Matrix3d ransac(double N, double threshold, int randomSize);



};


#endif