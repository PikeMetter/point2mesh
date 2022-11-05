#pragma once
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
typedef pcl::PointXYZ PointT;

bool meshRec()
{// Load input file
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile("cad.pcd", *cloud) == -1)
	{
		cout << "�������ݶ�ȡʧ�ܣ�" << endl;
	}

	std::cout << "Orginal points number: " << cloud->points.size() << std::endl;

	// ----------------------��ʼ��Ĵ���--------------------------//
	// ��ο�֮ǰ�����е����²������˲���ƽ�������ݣ��Լ�PCL����ʵ�����¹��ܡ����벻�ѡ�

	// �²���
	pcl::VoxelGrid<PointT> downSampled;  //�����˲�����
	downSampled.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	downSampled.setLeafSize(10.0f, 10.0f, 10.0f);  //�����˲�ʱ�������������Ϊ1cm��������
	downSampled.filter(*cloud_downSampled);           //ִ���˲������洢���
	//pcl::io::savePCDFile("downsampledPC.pcd", *cloud_downSampled);

	// ͳ���˲�
	pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;       //�����˲�������
	statisOutlierRemoval.setInputCloud(cloud_downSampled);            //���ô��˲��ĵ���
	statisOutlierRemoval.setMeanK(50);                                //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	statisOutlierRemoval.setStddevMulThresh(3.0);                     //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ:��ֵ+1.0*��׼��
	statisOutlierRemoval.filter(*cloud_filtered);                     //�˲�����洢��cloud_filtered
	//pcl::io::savePCDFile("filteredPC.pcd", *cloud_filtered);

	// �Ե����ز���  
	pcl::search::KdTree<PointT>::Ptr treeSampling(new pcl::search::KdTree<PointT>);// �������������������KD-Tree
	pcl::PointCloud<PointT> mls_point;    //���MLS
	pcl::MovingLeastSquares<PointT, PointT> mls; // ������С����ʵ�ֵĶ���mls
	mls.setComputeNormals(false);  //��������С���˼������Ƿ���Ҫ�洢����ķ���
	mls.setInputCloud(cloud_filtered);         //���ô��������
	mls.setPolynomialOrder(2);            // ���2�׶���ʽ���
	mls.setPolynomialFit(false);     // ����Ϊfalse���� ���� smooth
	mls.setSearchMethod(treeSampling);         // ����KD-Tree��Ϊ��������
	mls.setSearchRadius(0.05);           // ��λm.����������ϵ�K���ڰ뾶
	mls.process(mls_point);                 //���
	// ����ز������
	cloud_smoothed = mls_point.makeShared();
	std::cout << "cloud_smoothed: " << cloud_smoothed->size() << std::endl;
	//save cloud_smoothed
	//pcl::io::savePCDFileASCII("cloud_smoothed.pcd", *cloud_smoothed);

	// ���߹���
	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;             //�������߹��ƵĶ���
	normalEstimation.setInputCloud(cloud_smoothed);                         //�������
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);// �������������������KD-Tree
	normalEstimation.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // ��������ĵ��Ʒ���
	// K����ȷ��������ʹ��k������㣬����ȷ��һ����rΪ�뾶��Բ�ڵĵ㼯��ȷ�������ԣ�����ѡ1����
	normalEstimation.setKSearch(10);// ʹ�õ�ǰ����Χ�����10����
	//normalEstimation.setRadiusSearch(0.03);            //����ÿһ���㶼�ð뾶Ϊ3cm�Ľ���������ʽ
	normalEstimation.compute(*normals);               //���㷨��
	// �������
	std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
	//pcl::io::savePCDFileASCII("normals.pcd", *normals);

	// ������λ�ˡ���ɫ��������Ϣ���ӵ�һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
	//pcl::io::savePCDFileASCII("cloud_with_normals.pcd", *cloud_with_normals);

	// ̰��ͶӰ���ǻ�
	//��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// ���ǻ�
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // �������ǻ�����
	pcl::PolygonMesh triangles; //�洢�������ǻ�������ģ��

	// �������ǻ�����
	gp3.setSearchRadius(0.1);  //��������ʱ�İ뾶��Ҳ����KNN����뾶
	gp3.setMu(2.5);  //������������������ڵ����Զ����Ϊ2.5��������ֵ2.5-3��������ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);    //���������������������������������ֵ��50-100

	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10��
	gp3.setMaximumAngle(2 * M_PI / 3); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120��

	gp3.setMaximumSurfaceAngle(M_PI / 4); // ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45�㣬�������������ʱ�����Ǹõ�
	gp3.setNormalConsistency(false);  //���øò���Ϊtrue��֤���߳���һ�£�����Ϊfalse�Ļ�������з���һ���Լ��

	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2);   //����������ʽ
	gp3.reconstruct(triangles);  //�ؽ���ȡ���ǻ�

	// ��ʾ���񻯽��
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);  //
	viewer->addPolygonMesh(triangles, "wangge");  //
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return true;
}