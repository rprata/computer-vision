#include "Utils.h"

void Utils::generateCameraMatrix(void) 
{

    K1 = generateK(FOCAL_DISTANCE, PIXEL_SZ_X, PIXEL_SZ_Y, CENTER_PX_X, CENTER_PX_Y);
    K2 = K1;

    R1 <<   0.980106, -0.0199563, 0.197469,
            0.0558328, 0.982476, -0.177828,
            -0.190459, 0.185315, 0.964045;

    R2 << 0.914099, -0.0148061, -0.40522,
        -0.0540653, 0.98596, -0.157987,
        0.401869, 0.166324, 0.900465;

    T1 << 79.3959, -114.356, -499.541;

    T2 << -227.173, -103.559, -460.851;

    Rt1 = MatrixXd(3,4);
    Rt2 = MatrixXd(3,4);
    Rt1.block(0, 0, 3, 3) = R1;
    Rt2.block(0, 0, 3, 3) = R2;
    Rt1.col(3) = -R1*T1;
    Rt2.col(3) = -R2*T2;

    P1 = MatrixXd(3,4);
    P2 = MatrixXd(3,4);
    P1 = Rt1;
    P2 = Rt2;
}

Matrix3d Utils::generateK(double focalDistance, double pixelSizeX, double pixelSizeY, double centerPxX, double centerPxY)
{
	Matrix3d K;

    double ax = focalDistance / pixelSizeX;
    double ay = focalDistance / pixelSizeY;

    double x0 = centerPxX;
    double y0 = centerPxY;

    K << ax, 0, x0,
         0, ay, y0,
         0,  0,  1;

    return K;
}

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
		double x = keypoints_1[good_matches[i].queryIdx].pt.x;
    	double y = keypoints_1[good_matches[i].queryIdx].pt.y;
    	
    	aux_1 << x, y, 1;
    	x = keypoints_2[good_matches[i].trainIdx].pt.x;
    	y = keypoints_2[good_matches[i].trainIdx].pt.y;
    	aux_2 << x, y, 1;

    	m_vectorPairPoints.push_back(make_pair(aux_1, aux_2));
	}

	return m_vectorPairPoints;
}

Matrix3d Utils::ransac(double N, double threshold, int randomSize)
{
    int pairsQuantity = m_vectorPairPoints.size();
    int ransacCounter = 0;
    QVector<int> inliers, maxInliers;
    Matrix3d Ffinal;
    double e = 0.5;
    double p = 0.99;
    QVector<Vector3d> bestInliersA, bestInliersB;

    generateTMatrix();
    while(ransacCounter++ < N)
    {
        vectorPairPoints randomPairsIndexes = generateRandPairs(randomSize, pairsQuantity);

        Matrix3d Ftemp = generateF(randomPairsIndexes);
        cout << Ftemp << endl;
        cout << "==============" << endl;

        inliers = getRansacInliers(Ftemp, threshold);
        if(inliers.size() > maxInliers.size()){
            maxInliers = inliers;
            Ffinal = Ftemp;
        
            e = 1 - ((double)inliers.size()/(double)pairsQuantity);
            N = (double)log(1-p)/(double)log(1 - pow(1 - e, randomSize));
            if (N > 10000)
                N = 10000; //evita ficar preso no loop em alguns casos
        }
        cout << "ransacCounter " << ransacCounter << endl;
        cout << "N " << N << endl;

    }
    for(int i =0 ; i < maxInliers.size(); i++ )
    {
        bestInliersA.push_back(m_vectorPairPoints[maxInliers.at(i)].first);
        bestInliersB.push_back(m_vectorPairPoints[maxInliers.at(i)].second);
    }
    Matrix3d FfinalA = Ffinal;
    VectorXd A(Map<VectorXd>(Ffinal.data(), Ffinal.cols()*Ffinal.rows()));
    Ffinal = gaussNewton(Ffinal, bestInliersA, bestInliersB);
    Matrix3d FfinalB = Ffinal;
    VectorXd B(Map<VectorXd>(Ffinal.data(), Ffinal.cols()*Ffinal.rows()));

    if (A.squaredNorm() < B.squaredNorm())
    {
        cout << "Erro de +" << (( B.squaredNorm() - A.squaredNorm() ) / A.squaredNorm())*100 << "%" << endl;
    }
    else
    {
        cout << "Erro de -" << (( A.squaredNorm() - B.squaredNorm() ) / A.squaredNorm())*100 << "%" << endl;
    }

    if (FfinalA.norm() < FfinalB.norm())
    {
        cout << "Erro de +" << (( FfinalB.norm() - FfinalA.norm() ) / FfinalA.norm())*100 << "%" << endl;
    }
    else
    {
        cout << "Erro de -" << (( FfinalA.norm() - FfinalB.norm() ) / FfinalA.norm())*100 << "%" << endl;
    }

    return Ffinal;
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

vectorPairPoints Utils::generateRandPairs(int numberOfCorrespondences, int size)
{
    vectorPairPoints l_vectorPairPoints;
    cout << "size : " << size << endl;
    for (int j = 0; j < numberOfCorrespondences; j++)
    {
        int correspondence = rand() % size;
        cout << correspondence << endl;
        l_vectorPairPoints.push_back(m_vectorPairPoints.at(correspondence));
    }
    return l_vectorPairPoints;
}

MatrixXd Utils::generateMatrixH2(vectorPairPoints vec)
{
    //CREATE MATRIX
    MatrixXd m(8, 8);
    for(unsigned i = 0; i< vec.size(); i++)
    {
        MatrixXd lineA(1,8);
        lineA << vec[i].first(0), vec[i].first(1), 1, 0, 0, 0, -(vec[i].first(0)*vec[i].second(0)), -(vec[i].first(1)*vec[i].second(0));
        
        MatrixXd lineB(1,8);
        lineB << 0, 0, 0, vec[i].first(0), vec[i].first(1), 1, -(vec[i].first(0)*vec[i].second(1)), -(vec[i].first(1)*vec[i].second(1));
        
        m.row(i*2) << lineA;
        m.row(i*2 + 1) << lineB;
    }

    MatrixXd A  = m.inverse();
    MatrixXd B(8,1);
    
    for(unsigned i = 0; i< vec.size(); i++)
    {
        B.row(i*2) << vec[i].second(0);
        B.row(i*2 +1) << vec[i].second(1);
    }
    
    MatrixXd v = A*B;
    Matrix3d h;

    h << v(0), v(1), v(2), v(3), v(4), v(5), v(6), v(7), 1;

    return h;
}

QVector<int> Utils::getRansacInliers(Matrix3d H, double threshold)
{
    QVector<int> inliers;
    for(unsigned i = 0; i < m_vectorPairPoints.size(); i++)
    {
        Vector3d v;
        v = H * m_vectorPairPoints.at(i).first;
        v /= v(2);
        double distance = squaredEuclideanDistance(v, m_vectorPairPoints.at(i).second);
        if(distance < threshold){
            inliers.push_back(i);
        }
    }
    return inliers;
}

double Utils::squaredEuclideanDistance(Vector3d a, Vector3d b)
{
    double distance = 0;
    for (int i = 0; i < a.size(); i++)
         distance += pow( (a(i) - b(i)), 2);
    return distance;
}

Matrix3d Utils::gaussNewton(Matrix3d H, QVector<Vector3d> pointsFirstImage, QVector<Vector3d> pointsSecondImage)
{
    MatrixXd Ji(2, 9);
    MatrixXd J(pointsFirstImage.size()*2, 9);
    VectorXd fh(pointsFirstImage.size()*2);
    VectorXd X(pointsFirstImage.size()*2);
    VectorXd fFtemp(pointsFirstImage.size()*2);
    VectorXd XTemp(pointsFirstImage.size()*2);
    Matrix3d Ftemp;
    int counter = 0;
    double lambda = 1;
    double squaredNorm, squaredNormTemp;
    squaredNorm = squaredNormTemp = 0;
    H(0,2) = H(0,2) - 10;
    H(1,2) = H(1,2) - 10;
    do
    {
        counter++;
        for (int i = 0; i < (int)pointsFirstImage.size(); i++)
        {
            Vector3d xi = pointsFirstImage.at(i);
            Vector3d xilinha = pointsSecondImage.at(i);
            Ji << xi(0)/xilinha(2), xi(1)/xilinha(2), xi(2)/xilinha(2), 0, 0, 0, -xi(0)*xilinha(0)/xilinha(2), xi(1)*xilinha(0)/xilinha(2), xi(2)*xilinha(0)/xilinha(2),
                  0, 0, 0, xi(0)/xilinha(2), xi(1)/xilinha(2), xi(2)/xilinha(2), -xi(0)*xilinha(1)/xilinha(2), xi(1)*xilinha(1)/xilinha(2), xi(2)*xilinha(1)/xilinha(2);
            J.row(i*2) << Ji.row(0);
            J.row(i*2 + 1) << Ji.row(1);

            Vector3d Hxi = H*xi;
            Hxi /= Hxi(2);
            fh(i*2) = Hxi(0);
            fh(i*2 + 1) = Hxi(1);
            X(i*2) = xilinha(0);
            X(i*2 + 1) = xilinha(1);
         }

        MatrixXd JT(9, pointsFirstImage.size()*2);
        JT = J.transpose().eval();

        VectorXd deltaX(9);
        deltaX = (JT*J).inverse().eval()*(-JT)*fh;

        cout << "Erro mais externo: " << fabs(squaredNormTemp - squaredNorm) << endl;
        cout << "Erro mais externo: " << 100000000*fabs(squaredNormTemp - squaredNorm) << endl;

        if ( counter >= 0 && fabs(squaredNormTemp - squaredNorm)*100000000 < 8 )
            break;

        VectorXd diff = X - fh;
        squaredNorm = diff.squaredNorm();

        while(true)
        {
            Ftemp(0,0) = H(0,0) + lambda*deltaX(0);
            Ftemp(0,1) = H(0,1) + lambda*deltaX(1);
            Ftemp(0,2) = H(0,2) + lambda*deltaX(2);
            Ftemp(1,0) = H(1,0) + lambda*deltaX(3);
            Ftemp(1,1) = H(1,1) + lambda*deltaX(4);
            Ftemp(1,2) = H(1,2) + lambda*deltaX(5);
            Ftemp(2,0) = H(2,0) + lambda*deltaX(6);
            Ftemp(2,1) = H(2,1) + lambda*deltaX(7);
            Ftemp(2,2) = H(2,2) + lambda*deltaX(8);

            for (int i = 0; i < (int)pointsFirstImage.size(); i++)
            {
                Vector3d xi = pointsFirstImage.at(i);
                Vector3d xilinha = pointsSecondImage.at(i);
                Vector3d Hxi = Ftemp*xi;
                Hxi /= Hxi(2);
                fFtemp(i*2) = Hxi(0);
                fFtemp(i*2 + 1) = Hxi(1);
                XTemp(i*2) = xilinha(0);
                XTemp(i*2 + 1) = xilinha(1);
             }

             VectorXd diffTemp = XTemp - fFtemp;
             squaredNormTemp = diffTemp.squaredNorm();
             /*cout << endl;
             cout << "counter" << endl;
             cout << counter << endl;
             cout << "lambda" << endl;
             cout << lambda << endl;
             cout << "squaredNormTemp" << endl;
             cout << squaredNormTemp << endl;
             cout << "squaredNorm" << endl;
             cout << squaredNorm << endl;
             cout << endl;*/
             if (squaredNormTemp <= squaredNorm)
                 break;
             lambda /= 2;
        }

        H = Ftemp;
        lambda = std::max(2*lambda, 1.0);
     } while (counter <= 1000);
     return H;
}


Matrix3d Utils::generateF(vectorPairPoints vec)
{   
    int pairsQuantity = vec.size();
    vectorPairPoints randomPairsIndexes = generateRandPairs(8, pairsQuantity);

    QVector<Vector3d> l1, l2;
    for (unsigned i = 0; i < randomPairsIndexes.size(); i++)
    {
        l1.push_back(randomPairsIndexes.at(i).first);
        l2.push_back(randomPairsIndexes.at(i).second);
    }

    MatrixXd A(8,9);

    Matrix3d FTemp, FRestricted;
    for (int i = 0; i < 8; i++)
    {
        Vector3d first = l1.at(i);
        Vector3d second = l2.at(i);

        double x = (Ti * first)(0);
        double y = (Ti* first)(1);
        double xlinha = (Tii * second)(0);
        double ylinha = (Tii * second)(1);

        A.row(i) << xlinha*x, xlinha*y, xlinha, ylinha*x, ylinha*y, ylinha, x, y, 1;
    }

    JacobiSVD<MatrixXd> SVD(A, ComputeFullV);
    VectorXd x = SVD.matrixV().col(SVD.matrixV().cols() - 1);
    FTemp << x(0), x(1), x(2),
             x(3), x(4), x(5),
             x(6), x(7), x(8);

    JacobiSVD<MatrixXd> SVD2(FTemp, ComputeFullV | ComputeFullU);
    VectorXd singularValues;
    singularValues = SVD2.singularValues();
    DiagonalMatrix<double, 3,3> D(singularValues(0), singularValues(1), 0);
    Matrix3d DMat = D.toDenseMatrix();
    FRestricted = SVD2.matrixU() * DMat * SVD2.matrixV().transpose();

    return Tii.transpose().eval()*FRestricted*Ti;
}

void Utils::generateE(void)
{
    E = K2.transpose().eval() * F * K1;

    Eigen::JacobiSVD<MatrixXd> SVD(E, ComputeFullV | ComputeFullU);

    Eigen::DiagonalMatrix<double, 3,3> D(1, 1, 0);
    Eigen::Matrix3d DMat = D.toDenseMatrix();
    E = SVD.matrixU() * DMat * SVD.matrixV().transpose();
}

void Utils::generateP(void)
{
    Matrix3d W;
    W <<   0,  -1,  0,
           1,   0,  0,
           0,   0,  1;
    Matrix3d Z;
    Z <<   0,  1,  0,
          -1,  0,  0,
           0,  0,  0;
    Matrix3d U, V;
    JacobiSVD<MatrixXd> SVD(E, ComputeFullU | ComputeFullV);
    U = SVD.matrixU();
    V = SVD.matrixV();

   if((U * V.transpose()).determinant() < 0)
   {
       V.col(2) *= -1;
   }

   Vector3d u3;
   u3 = U.col(2);
   MatrixXd P1(3,4);
   MatrixXd P2(3,4);
   MatrixXd P3(3,4);
   MatrixXd P4(3,4);

   P1.topLeftCorner(3,3) = U * W * V.transpose();
   P1.col(3) = u3;

   P2.topLeftCorner(3,3) = U * W * V.transpose();
   P2.col(3) = -u3;

   P3.topLeftCorner(3,3) = U * W.transpose() * V.transpose();
   P3.col(3) = u3;

   P4.topLeftCorner(3,3) = U * W.transpose() * V.transpose();
   P4.col(3) = -u3;

   Pls.push_back(P1);
   Pls.push_back(P2);
   Pls.push_back(P3);
   Pls.push_back(P4);

}

