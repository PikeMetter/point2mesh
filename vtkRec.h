#pragma once
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkDelaunay3D.h>
#include <vtkDelaunay2D.h>
#include <vtkDataSetMapper.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkPLYWriter.h>
#include <vtkRenderer.h>
#include <vtkNamedColors.h>
#include <vtkSTLReader.h>
#include <vtkRenderWindow.h>
#include <vtkSTLWriter.h>
#include <vtkPointPicker.h>
#include <vtkVoxelContoursToSurfaceFilter.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#include <TopoDS_Shape.hxx>
#include <AIS_Shape.hxx>
#include <StlAPI_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>


void saveSTL(vtkPolyData* StlPolyData)
{
	vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
	stlWriter->SetFileName("db-out.STL");
	stlWriter->SetInputData((vtkDataObject *)StlPolyData);
	stlWriter->SetFileTypeToBinary();
	stlWriter->Update();
	stlWriter->Write();
	std::cout << "write STL";
}

void stl2ply()
{
	std::string filename = "db-out.STL";
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(filename.c_str());
	reader->Update();

	vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName("db-out.ply");
	plyWriter->SetInputConnection(reader->GetOutputPort());
	plyWriter->SetFileTypeToASCII();
	plyWriter->SetColorModeToOff();

	plyWriter->Update();
	plyWriter->Write();

}

/* brief 添加点云（无色）
* param[in] cloud 添加PCL点云指针
*/
void addCloudPoints(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	vtkSmartPointer<vtkPoints>points = vtkSmartPointer<vtkPoints>::New();//点数据
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();//单元数据 顶点类型
 
	for (int i = 0; i < cloud.points.size();i++)
	{
		//定义存储顶点索引的中间变量，类似Int、long类型
		vtkIdType pointId[1];
		//将每个点的坐标加入vtkPoints，InsertNextPoint()返回加入点的索引号
		pointId[0] = points->InsertNextPoint(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
		//为每个坐标点分别创建一个顶点，顶点是单元类型里面的一种
		vertices->InsertNextCell(1,pointId);
	}
	//创建vtkPolyData对象
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	//指定数据的几何结构（由points指定）和拓扑结构（由vertices指定）
	polyData->SetPoints(points);
	polyData->SetVerts(vertices);
	
	std::cout << "start delaunay3D" << std::endl;
	vtkSmartPointer<vtkDelaunay2D> delaunay3D = vtkSmartPointer<vtkDelaunay2D>::New();
	delaunay3D->SetInputData(polyData);
	delaunay3D->SetAlpha(10);
	//delaunay3D->SetTolerance(10);
	delaunay3D->Update();
	std::cout << "end delaunay3D" << std::endl;

	
	//vtkSmartPointer<vtkDataSetMapper> delaunayMapper = vtkSmartPointer<vtkDataSetMapper>::New();
	//delaunayMapper->SetInputConnection(delaunay3D->GetOutputPort());

	vtkSmartPointer<vtkNamedColors> color = vtkSmartPointer<vtkNamedColors>::New();
	//vtkSmartPointer<vtkActor> delaunayActor = vtkSmartPointer<vtkActor>::New();
	//delaunayActor->SetMapper(delaunayMapper);
	//delaunayActor->GetProperty()->SetColor(color->GetColor3d("banana").GetData());
	//delaunayActor->GetProperty()->EdgeVisibilityOn();

	vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
		vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
	smoothFilter->SetInputConnection(delaunay3D->GetOutputPort());
	smoothFilter->SetNumberOfIterations(200);
	smoothFilter->Update();

	vtkSmartPointer<vtkVoxelContoursToSurfaceFilter> voxelFilter =
		vtkSmartPointer<vtkVoxelContoursToSurfaceFilter>::New();
	voxelFilter->SetInputConnection(delaunay3D->GetOutputPort());
	voxelFilter->Update();

	//vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	//glyphFilter->SetInputData(delaunay3D->GetOutput());

	vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	pointMapper->SetInputConnection(smoothFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> pointActor = vtkSmartPointer<vtkActor>::New();
	pointActor->GetProperty()->SetColor(color->GetColor3d("Tomato").GetData());
	pointActor->GetProperty()->SetPointSize(5);
	pointActor->SetMapper(pointMapper);

	vtkSmartPointer<vtkRenderer> delaunayRenderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(900, 900);

	renderWindow->AddRenderer(delaunayRenderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	//delaunayRenderer->AddActor(delaunayActor);
	delaunayRenderer->AddActor(pointActor);
	delaunayRenderer->SetBackground(color->GetColor3d("Mint").GetData());

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	renderWindowInteractor->SetInteractorStyle(style);

	// Render and interact
	renderWindow->SetWindowName("Delaunay3D");
	renderWindow->Render();
	renderWindowInteractor->Start();

	//保存PLY、STL
	saveSTL(delaunay3D->GetOutput());
	stl2ply();
}

void STLshow()
{
	// Read a stl file.
	vtkSmartPointer<vtkPolyData> input1 = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkSTLReader> reader1 = vtkSmartPointer<vtkSTLReader>::New();
	reader1->SetFileName("db-out.STL");
	reader1->Update();
	input1->DeepCopy(reader1->GetOutput());

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader1->GetOutputPort());
	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetOpacity(0.7);

	// Create a renderer, render window, and interactor
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->Render();
	renderWindow->SetWindowName("PointPicker");
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkPointPicker> pointPicker = vtkSmartPointer<vtkPointPicker>::New();

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetPicker(pointPicker);
	renderWindowInteractor->SetRenderWindow(renderWindow);

	// Add the actor to the scene
	renderer->AddActor(actor);
	renderer->SetBackground(0, 0, 0);

	// Render and interact
	renderWindow->Render();
	renderWindowInteractor->Start();
}

bool STL2STEP()
{
	StlAPI_Reader aReader_Stl;
	TopoDS_Shape shape_Stl;
	aReader_Stl.Read(shape_Stl, "db-out.STL");

	try {
		STEPControl_Writer aWriter;
		IFSelect_ReturnStatus status;
		status = aWriter.Transfer(shape_Stl, STEPControl_AsIs);
		if (status == IFSelect_RetDone)
			status = aWriter.Write("db.step");
			return true;
	}
	catch (Standard_Failure) {
		Standard_Failure::Raise("Could not export in STEP format");
	}
	return false;
}