#include <iostream>
#include "Utils.h"

int main(int argc, char ** argv)
{
  	if (argc != 1)
  	{
  		std::cout << "usage: "<<  argv[0] << std::endl;
  		return -1;
  	}
    Utils::getInstance()->generateCameraMatrix();
    Utils::getInstance()->read3DPointsFromObj("../imgs/work4/thai-lion/thai-lion.obj", 50000);
    Utils::getInstance()->obtain2DPointsCorrespondenceFrom3DPoints();
  	Utils::getInstance()->generateMatchingPoints("../imgs/work4/thai-lion/DSC_0176.JPG", "../imgs/work4/thai-lion/DSC_0179.JPG");
    Utils::getInstance()->ransac(1000, 0.05, 8);

    Utils::getInstance()->generateE();
    Utils::getInstance()->generateP();

    QVector<VectorXd> points;
    MatrixXd P(3,4);
    P = MatrixXd::Identity(3, 4);
    P.block(0, 0, 3, 3) = Utils::getInstance()->getK1();
    Matrix3d K2 = Utils::getInstance()->getK1();
    vector<MatrixXd> Pls = Utils::getInstance()->getPls();

    string outputPath;
    for (unsigned i = 0; i < Pls.size(); i++)
    {
        stringstream str;
        str << "output" << i << ".obj";
        outputPath = str.str();
        Pls.at(i) = K2 * Pls.at(i);
        points = Utils::getInstance()->get3DPointsByTriangulation(P, Pls.at(i));
        Utils::getInstance()->saveInObj(outputPath, points);
    }

  	return 0;
}