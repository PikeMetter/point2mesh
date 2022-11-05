#pragma once
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件


bool filters(const int i,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after)
{

	std::cout << "原始点云数据点数：" << cloud->points.size() << std::endl << std::endl;

	if (i == 1) {
		///****************************************************
		/*方法一：直通滤波器对点云进行处理。*/

		pcl::PassThrough<pcl::PointXYZRGB> passthrough;
		passthrough.setInputCloud(cloud);//输入点云
		passthrough.setFilterFieldName("z");//对z轴进行操作
		passthrough.setFilterLimits(0.0, 400.0);//设置直通滤波器操作范围
		//passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
		passthrough.filter(*cloud_after);//执行滤波，过滤结果保存在 cloud_after_PassThrough

		std::cout << "直通滤波后点云数据点数：" << cloud_after->points.size() << std::endl;
		return true;
	}
	else if (i == 2) {
		///****************************************************
		/*方法二：体素滤波器实现下采样*/

		pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
		voxelgrid.setInputCloud(cloud);//输入点云数据
		voxelgrid.setLeafSize(2.0f, 2.0f, 2.0f);//AABB长宽高
		voxelgrid.filter(*cloud_after);

		std::cout << "体素化网格方法后点云数据点数：" << cloud_after->points.size() << std::endl;
		///****************************************************
		return true;
	}
	else if (i == 3) {
		///****************************************************
		/*方法三：统计滤波器滤波*/

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Statistical;
		Statistical.setInputCloud(cloud);
		Statistical.setMeanK(20);//取平均值的临近点数
		Statistical.setStddevMulThresh(5);//临近点数数目少于多少时会被舍弃
		Statistical.filter(*cloud_after);

		std::cout << "统计分析滤波后点云数据点数：" << cloud_after->points.size() << std::endl;
		///****************************************************
		return true;
	}
	else if (i == 4) {
		///****************************************************
		/*方法四：条件滤波器*/

		pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZRGB>());
		range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0)));  //GT表示大于等于
		range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 0.8)));  //LT表示小于等于

		pcl::ConditionalRemoval<pcl::PointXYZRGB> condition;
		condition.setCondition(range_condition);
		condition.setInputCloud(cloud);                   //输入点云
		condition.setKeepOrganized(true);

		condition.filter(*cloud_after);
		std::cout << "条件滤波后点云数据点数：" << cloud_after->points.size() << std::endl;
		///****************************************************
		return true;
	}
	else if (i == 5)
	{
		///****************************************************
		/*方法五：半径滤波器*/

		pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radiusoutlier;  //创建滤波器

		radiusoutlier.setInputCloud(cloud);    //设置输入点云
		radiusoutlier.setRadiusSearch(100);     //设置半径为100的范围内找临近点
		radiusoutlier.setMinNeighborsInRadius(2); //设置查询点的邻域点集数小于2的删除

		radiusoutlier.filter(*cloud_after);
		std::cout << "半径滤波后点云数据点数：" << cloud_after->points.size() << std::endl;
		return true;
	}

	return false;
}