#pragma once
#include <Eigen/Dense>  
#include <Eigen/SVD>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include<opencv2/opencv.hpp> 
typedef pcl::PointXYZ Point3d;

static void point3dPlus(Point3d& p1,Point3d& p2)
{
	for (int i = 0; i < 3; ++i)
	{
		p1.x = p1.x + p2.x;
		p1.y = p1.y + p2.y;
		p1.z = p1.z + p2.z;
	}
};

static void point3dMinus(Point3d& p1,Point3d& p2)
{
	for (int i = 0; i < 3; ++i)
	{
		p1.x = p1.x - p2.x;
		p1.y = p1.y - p2.y;
		p1.z = p1.z - p2.z;
	}
};

static Point3d point3dMinus2(Point3d& p1, Point3d& p2)
{
	Point3d result;
	for (int i = 0; i < 3; ++i)
	{
		result.x = p1.x - p2.x;
		result.y = p1.y - p2.y;
		result.z = p1.z - p2.z;
	}
	return result;
};

static void point3dDivide(Point3d& p1,Point3d& p2)
{
	for (int i = 0; i < 3; ++i)
	{
		p1.x = p1.x / p2.x;
		p1.y = p1.y / p2.y;
		p1.z = p1.z / p2.z;
	}
};

static void point3dDivide(Point3d& p1, int N)
{
	for (int i = 0; i < 3; ++i)
	{
		p1.x = p1.x / N;
		p1.y = p1.y / N;
		p1.z = p1.z / N;
	}
};
//计算旋转矩阵R和平移矩阵T
static void myICP_calculate_RT(std::vector<Point3d>& pts1,
	std::vector<Point3d>& pts2,
	Eigen::Matrix3d& R, Eigen::Matrix3d& T)
{
	// center of mass
	Point3d p1, p2;
	int N = pts1.size();
	for (int i = 0; i < N; ++i)
	{
		point3dPlus(p1, pts1[i]);
		point3dPlus(p2, pts2[i]);
	}
	//求均值
	point3dDivide(p1,N);
	point3dDivide(p2,N);

	// subtract COM(center of mass)
	std::vector<Point3d> q1(N), q2(N);
	for (int i = 0; i < N; ++i)
	{
		q1[i] = point3dMinus2(pts1[i], p1);
		q2[i] = point3dMinus2(pts2[i], p2);
	}

	//compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; ++i)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * 
			Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	std::cout << "W= " << W << std::endl;

	//SVD on W 
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	std::cout << "U=" << U << std::endl;
	std::cout << "V=" << V << std::endl;

	Eigen::Matrix3d R_ = U * (V.transpose());
	Eigen::Vector3d T_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

	//convert to cv::Mat
	R << R_(0, 0), R_(0, 1), R_(0, 2),
		 R_(1, 0), R_(1, 1), R_(1, 2),
		 R_(2, 0), R_(2, 1), R_(2, 2);

	T << T_(0, 0), T_(1, 0), T_(2, 0);
}
/**
* @brief 求实对称矩阵的特征值及特征向量的雅克比法
* 利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量
* @param pMatrix				长度为n*n的数组，存放实对称矩阵
* @param nDim					矩阵的阶数
* @param pdblVects				长度为n*n的数组，返回特征向量(按列存储)
* @param dbEps					精度要求
* @param nJt					整型变量，控制最大迭代次数
* @param pdbEigenValues			特征值数组
* @return
*/
static bool JacobiCor(double* pMatrix,int nDim,double* pdblVects,
	double* pdbEigenValues,double dbEps,int nJt)
{
	//构造单位矩阵
	for (int i = 0; i < nDim; i++)
	{
		pdblVects[i*nDim + i] = 1.0f;
		for (int j = 0; j < nDim; j++)
		{
			if (i != j)
				pdblVects[i*nDim + j] = 0.0f;
		}
	}

	int nCount = 0;//迭代次数
	while (1)
	{
		//在pMatrix的非对角线上找到最大元素
		double dbMax = pMatrix[1];
		int nRow = 0;
		int nCol = 1;
		for (int i = 0; i < nDim; ++i)
		{
			for (int j = 0; j < nDim; ++j)
			{
				double d = fabs(pMatrix[i*nDim + j]);
				if ((i != j) && (d > dbMax)) //不是对角线且d值较大
				{
					//记录当前d值，以及它的位置
					dbMax = d;
					nRow = i;
					nCol = j;
				}
			}
		}
		if (dbMax < dbEps) //精度符合要求
			break;
		if (nCount > nJt) //迭代次数超过限制
			break;

		nCount++;

		double dbApp = pMatrix[nRow*nDim + nRow];
		double dbApq = pMatrix[nRow*nDim + nCol];
		double dbAqq = pMatrix[nCol*nDim + nCol];

		//计算旋转角度
		double dbAngle = 0.5*atan2(-2 * dbApq, dbAqq - dbApp);
		double dbSinTheta = sin(dbAngle);
		double dbCosTheta = cos(dbAngle);
		double dbSin2Theta = sin(2 * dbAngle);
		double dbCos2Theta = cos(2 * dbAngle);

		pMatrix[nRow*nDim + nRow] = dbApp * dbCosTheta*dbCosTheta +
			dbAqq * dbSinTheta*dbSinTheta + 2 * dbApq*dbCosTheta*dbSinTheta;
		pMatrix[nCol*nDim + nCol] = dbApp * dbSinTheta*dbSinTheta +
			dbAqq * dbCosTheta*dbCosTheta - 2 * dbApq*dbCosTheta*dbSinTheta;
		pMatrix[nRow*nDim + nCol] = 0.5*(dbAqq - dbApp)*dbSin2Theta + dbApq * dbCos2Theta;
		pMatrix[nCol*nDim + nRow] = pMatrix[nRow*nDim + nCol];

		for (int i = 0; i < nDim; i++)
		{
			if ((i != nCol) && (i != nRow))
			{
				int u = i * nDim + nRow;	//p  
				int w = i * nDim + nCol;	//q
				dbMax = pMatrix[u];
				pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax * dbCosTheta;
				pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax * dbSinTheta;
			}
		}

		for (int j = 0; j < nDim; j++)
		{
			if ((j != nCol) && (j != nRow))
			{
				int u = nRow * nDim + j;	//p
				int w = nCol * nDim + j;	//q
				dbMax = pMatrix[u];
				pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax * dbCosTheta;
				pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax * dbSinTheta;
			}
		}

		//计算特征向量
		for (int i = 0; i < nDim; i++)
		{
			int u = i * nDim + nRow;		//p   
			int w = i * nDim + nCol;		//q
			dbMax = pdblVects[u];
			pdblVects[u] = pdblVects[w] * dbSinTheta + dbMax * dbCosTheta;
			pdblVects[w] = pdblVects[w] * dbCosTheta - dbMax * dbSinTheta;
		}
	}
	//对特征值进行排序以及重新排列特征向量,特征值即pMatrix主对角线上的元素
	std::map<double, int> mapEigen;
	for (int i = 0; i < nDim; ++i)
	{
		pdbEigenValues[i] = pMatrix[i*nDim + i];
		mapEigen.insert(std::make_pair(pdbEigenValues[i], i));
	}

	double *pdbTmpVec = new double[nDim*nDim];
	std::map<double, int>::reverse_iterator iter = mapEigen.rbegin();
	for (int j = 0; iter != mapEigen.rend(), j < nDim; ++iter, ++j)
	{
		for (int i = 0; i < nDim; ++i)
		{
			pdbTmpVec[i*nDim + j] = pdblVects[i*nDim + iter->second];
		}
		//特征值重新排列
		pdbEigenValues[j] = iter->first;
	}

	//设定正负号
	for (int i = 0; i < nDim; i++)
	{
		double dSumVec = 0;
		for (int j = 0; j < nDim; j++)
			dSumVec += pdbTmpVec[j * nDim + i];
		if (dSumVec < 0)
		{
			for (int j = 0; j < nDim; j++)
				pdbTmpVec[j * nDim + i] *= -1;
		}
	}

	memcpy(pdblVects, pdbTmpVec, sizeof(double)*nDim*nDim);
	delete[] pdbTmpVec;

	return 1;
}