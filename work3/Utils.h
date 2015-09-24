#ifndef __UTILS__
#define __UTILS__

#include <iostream>
#include <string>
#include <utility> 
#include <vector>
#include <cmath>

#include <eigen3/Eigen/Dense>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace std;
using namespace Eigen;
using namespace cv;


#define MIN_HESSIAN 400

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
    void generateTMatrix(void); //according 4.4 - page 107

};


#endif