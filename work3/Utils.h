#ifndef __UTILS__
#define __UTILS__

#include <iostream>
#include <string>
#include <utility> 
#include <vector>

#include <eigen3/Eigen/Dense>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace std;
using Eigen::Vector3d;
using namespace cv;


#define MIN_HESSIAN 400

typedef vector< pair<Vector3d, Vector3d> > vectorPairPoints;

class Utils
{

private:
	Utils() {};	
	vectorPairPoints m_vectorPairPoints;

public:
	static Utils * getInstance()
    {
    	static Utils * instance = new Utils();
        return instance;
    }

    Utils(Utils const&);
    void operator=(Utils const&);


	vectorPairPoints generateMatchingPoints(const string& argv1, const string& argv2);
	void printVectorPairPoints(void);
};


#endif