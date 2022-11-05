
#include "poissonRec.h"
#include "meshRec.h"
#include "vtkRec.h"
#include "filters.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

int maina(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile("cad.pcd", *object_cloud) == -1)
	{
		cout << "点云数据读取失败！" << endl;
	}

	std::cout << "Orginal points number: " << object_cloud->points.size() << std::endl;

	
	//滤波
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after(new pcl::PointCloud<pcl::PointXYZRGB>);
	filters(2,object_cloud, cloud_after);

	//1
	//meshRec();
	//2
	//poisson_reconstruction(cloud_after);
	//3
	addCloudPoints(*cloud_after);
	//STLshow();
	//STL2STEP();
	//stl2ply();
	return 0;
}

