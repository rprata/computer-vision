#include "Utils.h"

vectorPairPoints Utils::generateMatchingPoints(const string& argv1, const string& argv2)
{

	m_vectorPairPoints.clear();

	Mat img_1 = imread( argv1, CV_LOAD_IMAGE_GRAYSCALE );
 	Mat img_2 = imread( argv2, CV_LOAD_IMAGE_GRAYSCALE );

 	if( !img_1.data || !img_2.data )
  	{ 
  		cout<< "Erro ao ler as imagens!" << endl; 
  		return vectorPairPoints();
  	}

  	//-- Step 1: Detect the keypoints using SURF Detector
	SurfFeatureDetector detector( MIN_HESSIAN );

	std::vector<KeyPoint> keypoints_1, keypoints_2;

	detector.detect( img_1, keypoints_1 );
	detector.detect( img_2, keypoints_2 );

	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_1, descriptors_2;

	extractor.compute( img_1, keypoints_1, descriptors_1 );
	extractor.compute( img_2, keypoints_2, descriptors_2 );

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors_1.rows; i++ )
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	// cout << "-- Max dist : " << max_dist << endl;
	// cout << "-- Min dist : " << min_dist << endl;

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector< DMatch > good_matches;

	for( int i = 0; i < descriptors_1.rows; i++ )
	{ 
		if( matches[i].distance <= max(2*min_dist, 0.02) )
		{ 
			good_matches.push_back( matches[i]); 
		}	
	}

	Vector3d aux_1, aux_2;
 	
 	for( int i = 0; i < (int)good_matches.size(); i++ )
  	{ 
		// printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
		float x = keypoints_1[good_matches[i].queryIdx].pt.x;
    	float y = keypoints_1[good_matches[i].queryIdx].pt.y;
    	
    	aux_1 << x, y, 1;
    	x = keypoints_2[good_matches[i].trainIdx].pt.x;
    	y = keypoints_2[good_matches[i].trainIdx].pt.y;
    	aux_2 << x, y, 1;

    	m_vectorPairPoints.push_back(make_pair(aux_1, aux_2));
	}


	return m_vectorPairPoints;
}

void Utils::printVectorPairPoints(void)
{
	for(unsigned i = 0; i < m_vectorPairPoints.size(); i++)
	{

		cout << "vector 1: " << m_vectorPairPoints.at(i).first << endl;
		cout << "vector 2: " << m_vectorPairPoints.at(i).second << endl;

	}
}

void Utils::calculateDLT(void)
{
	MatrixXd A(m_vectorPairPoints.size()*2, 9);

    for(unsigned i = 0; i < m_vectorPairPoints.size(); i++) 
    {
    	pair <Vector3d, Vector3d> point = m_vectorPairPoints.at(i);

        A.row(i * 2) << 0, 0, 0, -point.first(0), -point.first(1), -1, point.second(1)*point.first(0), point.second(1)*point.first(1), point.second(1);
        A.row(i * 2 + 1) << point.first(0), point.first(1), 1, 0, 0, 0, -point.second(0)*point.first(0), -point.second(0)*point.first(1), -point.second(0);
    }

    JacobiSVD<MatrixXd> SVD(A, Eigen::ComputeThinV);
    VectorXd h = SVD.matrixV().col(SVD.matrixV().cols() - 1);
    Hn <<    h(0), h(1), h(2),
            h(3), h(4), h(5),
            h(6), h(7), h(8);
    
    generateTMatrix();

    H = Ti.inverse().eval() * Hn * Tii;
}

void Utils::printMatrixHn(void)
{
	cout << Hn << endl;
}

void Utils::printMatrixH(void)
{
	cout << H << endl;
}

MatrixXd Utils::getMatrixH(void)
{
	return H;
}

void Utils::generateTMatrix(void)
{
    double u_avg = 0;
    double v_avg = 0;
    int u_sum = 0;
    int v_sum = 0;

    for(unsigned i = 0; i < m_vectorPairPoints.size(); i++) 
    {
        pair<Vector3d, Vector3d> point = m_vectorPairPoints.at(i);
        u_sum += point.first(0);
        v_sum += point.first(1);
    }

    u_avg = u_sum / (double)m_vectorPairPoints.size();
    v_avg = v_sum / (double)m_vectorPairPoints.size();


    double sum = 0;
    double s = 0;

    for(unsigned i = 0; i < m_vectorPairPoints.size(); i++) 
    {
        pair<Vector3d, Vector3d> point;
        sum += sqrt(pow(((double)point.first(0) - u_avg), 2) + pow(((double)point.first(1) - v_avg), 2));
    }

    s = (sqrt(2) * (double)m_vectorPairPoints.size()) / sum;

    Ti << s, 0, -s*u_avg,
         0, s, -s*v_avg,
         0, 0, 1;

    u_avg = 0;
    v_avg = 0;
    u_sum = 0;
    v_sum = 0;

    for(unsigned i = 0; i < m_vectorPairPoints.size(); i++) 
    {
        pair<Vector3d, Vector3d> point = m_vectorPairPoints.at(i);
        u_sum += point.second(0);
        v_sum += point.second(1);
    }

    u_avg = u_sum / (double)m_vectorPairPoints.size();
    v_avg = v_sum / (double)m_vectorPairPoints.size();


    sum = 0;
    s = 0;

    for(unsigned i = 0; i < m_vectorPairPoints.size(); i++) 
    {
        pair<Vector3d, Vector3d> point;
        sum += sqrt(pow(((double)point.second(0) - u_avg), 2) + pow(((double)point.second(1) - v_avg), 2));
    }

    s = (sqrt(2) * (double)m_vectorPairPoints.size()) / sum;

    Tii << s, 0, -s*u_avg,
         0, s, -s*v_avg,
         0, 0, 1;
}