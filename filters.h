#pragma once
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>  //ֱͨ�˲���ͷ�ļ�
#include<pcl/filters/voxel_grid.h>  //�����˲���ͷ�ļ�
#include<pcl/filters/statistical_outlier_removal.h>   //ͳ���˲���ͷ�ļ�
#include <pcl/filters/conditional_removal.h>    //�����˲���ͷ�ļ�
#include <pcl/filters/radius_outlier_removal.h>   //�뾶�˲���ͷ�ļ�


bool filters(const int i,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after)
{

	std::cout << "ԭʼ�������ݵ�����" << cloud->points.size() << std::endl << std::endl;

	if (i == 1) {
		///****************************************************
		/*����һ��ֱͨ�˲����Ե��ƽ��д���*/

		pcl::PassThrough<pcl::PointXYZRGB> passthrough;
		passthrough.setInputCloud(cloud);//�������
		passthrough.setFilterFieldName("z");//��z����в���
		passthrough.setFilterLimits(0.0, 400.0);//����ֱͨ�˲���������Χ
		//passthrough.setFilterLimitsNegative(true);//true��ʾ������Χ�ڣ�false��ʾ������Χ��
		passthrough.filter(*cloud_after);//ִ���˲������˽�������� cloud_after_PassThrough

		std::cout << "ֱͨ�˲���������ݵ�����" << cloud_after->points.size() << std::endl;
		return true;
	}
	else if (i == 2) {
		///****************************************************
		/*�������������˲���ʵ���²���*/

		pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
		voxelgrid.setInputCloud(cloud);//�����������
		voxelgrid.setLeafSize(2.0f, 2.0f, 2.0f);//AABB�����
		voxelgrid.filter(*cloud_after);

		std::cout << "���ػ����񷽷���������ݵ�����" << cloud_after->points.size() << std::endl;
		///****************************************************
		return true;
	}
	else if (i == 3) {
		///****************************************************
		/*��������ͳ���˲����˲�*/

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Statistical;
		Statistical.setInputCloud(cloud);
		Statistical.setMeanK(20);//ȡƽ��ֵ���ٽ�����
		Statistical.setStddevMulThresh(5);//�ٽ�������Ŀ���ڶ���ʱ�ᱻ����
		Statistical.filter(*cloud_after);

		std::cout << "ͳ�Ʒ����˲���������ݵ�����" << cloud_after->points.size() << std::endl;
		///****************************************************
		return true;
	}
	else if (i == 4) {
		///****************************************************
		/*�����ģ������˲���*/

		pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZRGB>());
		range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0)));  //GT��ʾ���ڵ���
		range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 0.8)));  //LT��ʾС�ڵ���

		pcl::ConditionalRemoval<pcl::PointXYZRGB> condition;
		condition.setCondition(range_condition);
		condition.setInputCloud(cloud);                   //�������
		condition.setKeepOrganized(true);

		condition.filter(*cloud_after);
		std::cout << "�����˲���������ݵ�����" << cloud_after->points.size() << std::endl;
		///****************************************************
		return true;
	}
	else if (i == 5)
	{
		///****************************************************
		/*�����壺�뾶�˲���*/

		pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radiusoutlier;  //�����˲���

		radiusoutlier.setInputCloud(cloud);    //�����������
		radiusoutlier.setRadiusSearch(100);     //���ð뾶Ϊ100�ķ�Χ�����ٽ���
		radiusoutlier.setMinNeighborsInRadius(2); //���ò�ѯ�������㼯��С��2��ɾ��

		radiusoutlier.filter(*cloud_after);
		std::cout << "�뾶�˲���������ݵ�����" << cloud_after->points.size() << std::endl;
		return true;
	}

	return false;
}