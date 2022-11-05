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

void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(*object_cloud, *cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.filter(*filtered);
	cout << "passthrough filter complete" << endl;

	cout << "begin normal estimation" << endl;
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;//计算点云法向
	ne.setNumberOfThreads(8);//设定临近点
	ne.setInputCloud(filtered);
	ne.setRadiusSearch(0.01);//设定搜索半径
	Eigen::Vector4f centroid;
	compute3DCentroid(*filtered, centroid);//计算点云中心
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);//将向量计算原点置于点云中心

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	ne.compute(*cloud_normals);
	cout << "normal estimation complete" << endl;
	cout << "reverse normals' direction" << endl;

	//将法向量反向
	for (size_t i = 0; i < cloud_normals->size(); ++i)
	{
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	//融合RGB点云和法向
	cout << "combine points and normals" << endl;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

	//泊松重建
	cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	//poisson.setDegree(2);
	poisson.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	poisson.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	poisson.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	poisson.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	poisson.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	poisson.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	poisson.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	poisson.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	poisson.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度

	poisson.setConfidence(false);
	poisson.setManifold(false);
	poisson.setOutputPolygons(false);

	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	cout << "finish poisson reconstruction" << endl;

	//给mesh染色
	pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh);

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	// K nearest neighbor search
	int K = 5;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	for (int i = 0; i < cloud_color_mesh.points.size(); ++i)
	{
		uint8_t r = 0;
		uint8_t g = 0;
		uint8_t b = 0;
		float dist = 0.0;
		int red = 0;
		int green = 0;
		int blue = 0;
		uint32_t rgb;

		if (kdtree.nearestKSearch(cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
			{

				r = cloud->points[pointIdxNKNSearch[j]].r;
				g = cloud->points[pointIdxNKNSearch[j]].g;
				b = cloud->points[pointIdxNKNSearch[j]].b;

				red += int(r);
				green += int(g);
				blue += int(b);
				dist += 1.0 / pointNKNSquaredDistance[j];

				std::cout << "red: " << int(r) << std::endl;
				std::cout << "green: " << int(g) << std::endl;
				std::cout << "blue: " << int(b) << std::endl;
				cout << "dis:" << dist << endl;
			}
		}

		cloud_color_mesh.points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].g = int(green / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh.points[i].b = int(blue / pointIdxNKNSearch.size() + 0.5);


	}
	toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
	pcl::io::savePLYFile("object_mesh.ply", mesh);
}