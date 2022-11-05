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
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;//������Ʒ���
	ne.setNumberOfThreads(8);//�趨�ٽ���
	ne.setInputCloud(filtered);
	ne.setRadiusSearch(0.01);//�趨�����뾶
	Eigen::Vector4f centroid;
	compute3DCentroid(*filtered, centroid);//�����������
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);//����������ԭ�����ڵ�������

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	ne.compute(*cloud_normals);
	cout << "normal estimation complete" << endl;
	cout << "reverse normals' direction" << endl;

	//������������
	for (size_t i = 0; i < cloud_normals->size(); ++i)
	{
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	//�ں�RGB���ƺͷ���
	cout << "combine points and normals" << endl;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

	//�����ؽ�
	cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	//poisson.setDegree(2);
	poisson.setConfidence(false); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	poisson.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
	poisson.setDepth(8); //���������ȣ����2^d x 2^d x 2^d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
	poisson.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
	poisson.setManifold(false); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	poisson.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	poisson.setSamplesPerNode(3.0); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
	poisson.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	poisson.setSolverDivide(8); //����������Է������Gauss-Seidel�������������

	poisson.setConfidence(false);
	poisson.setManifold(false);
	poisson.setOutputPolygons(false);

	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	cout << "finish poisson reconstruction" << endl;

	//��meshȾɫ
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