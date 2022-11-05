#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkLandmarkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkMatrix4x4.h>
#include <vtkInteractorStyleTrackballCamera.h>
//研究一下坐标系显示位置
#include <vtkOrientationMarkerWidget.h>
#include <vtkIterativeClosestPointTransform.h>
int main()
{
	vtkSmartPointer<vtkPoints> sourcePoints =
		vtkSmartPointer<vtkPoints>::New();
	double sourcePoint1[3] = { 0.5, 0, 0 };
	sourcePoints->InsertNextPoint(sourcePoint1);
	double sourcePoint2[3] = { 0, 0.5, 0 };
	sourcePoints->InsertNextPoint(sourcePoint2);
	double sourcePoint3[3] = { 0, 0, 0.5 };
	sourcePoints->InsertNextPoint(sourcePoint3);

	vtkSmartPointer<vtkPoints> targetPoints =
		vtkSmartPointer<vtkPoints>::New();
	double targetPoint1[3] = { 0.0, 0.0, 0.55 };
	targetPoints->InsertNextPoint(targetPoint1);
	double targetPoint2[3] = { 0.0, 0.55, 0.0 };
	targetPoints->InsertNextPoint(targetPoint2);
	double targetPoint3[3] = { -0.55, 0.0, 0.0 };
	targetPoints->InsertNextPoint(targetPoint3);

	//利用Landmark算法求变换矩阵
	vtkSmartPointer<vtkLandmarkTransform> landmarkTransform =
		vtkSmartPointer<vtkLandmarkTransform>::New();
	landmarkTransform->SetSourceLandmarks(sourcePoints);
	landmarkTransform->SetTargetLandmarks(targetPoints);
	landmarkTransform->SetModeToRigidBody(); //执行刚体配准
	landmarkTransform->Update();
	//构造PolyData类型 进行图形可视化
		vtkSmartPointer<vtkPolyData> source =
		vtkSmartPointer<vtkPolyData>::New();
	source->SetPoints(sourcePoints);
	vtkSmartPointer<vtkPolyData> target =
		vtkSmartPointer<vtkPolyData>::New();
	target->SetPoints(targetPoints);
	
		vtkSmartPointer<vtkVertexGlyphFilter> sourceGlyphFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	sourceGlyphFilter->SetInputData(source);
	sourceGlyphFilter->Update();

	vtkSmartPointer<vtkVertexGlyphFilter> targetGlyphFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	targetGlyphFilter->SetInputData(target);
	targetGlyphFilter->Update();
	//源数据施加配准变换矩阵
		vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(sourceGlyphFilter->GetOutput());
	transformFilter->SetTransform(landmarkTransform);
	transformFilter->Update();

	//进行ICP配准求变换矩阵
	vtkSmartPointer<vtkIterativeClosestPointTransform> icptrans =
		vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
	icptrans->SetSource(transformFilter->GetOutput());
	icptrans->SetTarget(targetGlyphFilter->GetOutput());
	icptrans->GetLandmarkTransform()->SetModeToRigidBody();
	icptrans->SetMaximumNumberOfIterations(50);
	icptrans->StartByMatchingCentroidsOn();
	icptrans->Modified();
	icptrans->Update();

	vtkSmartPointer<vtkMatrix4x4> landmarkMat = landmarkTransform->GetMatrix();
	vtkSmartPointer<vtkMatrix4x4> icpMat = icptrans->GetMatrix();
	vtkSmartPointer<vtkMatrix4x4> mat = vtkMatrix4x4::New();
	vtkMatrix4x4::Multiply4x4(landmarkMat, icpMat, mat);
	std::cout << "Matrix: " << std::endl;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			std::cout << mat->GetElement(i, j) << " ";
		}
		std::cout << std::endl;
	}
	vtkSmartPointer<vtkTransformPolyDataFilter> icpFilter =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	icpFilter->SetInputData(transformFilter->GetOutput());
	icpFilter->SetTransform(icptrans);
	icpFilter->Update();


		vtkSmartPointer<vtkPolyDataMapper> sourceMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	sourceMapper->SetInputConnection(sourceGlyphFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> sourceActor =
		vtkSmartPointer<vtkActor>::New();
	sourceActor->SetMapper(sourceMapper);
	sourceActor->GetProperty()->SetColor(1, 1, 0);
	sourceActor->GetProperty()->SetPointSize(10);

	vtkSmartPointer<vtkPolyDataMapper> targetMapper = 
		vtkSmartPointer<vtkPolyDataMapper>::New();
	targetMapper->SetInputConnection(targetGlyphFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> targetActor =
		vtkSmartPointer<vtkActor>::New();
	targetActor->SetMapper(targetMapper);
	targetActor->GetProperty()->SetColor(0, 1, 0);
	targetActor->GetProperty()->SetPointSize(10);

	vtkSmartPointer<vtkPolyDataMapper> solutionMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	solutionMapper->SetInputConnection(transformFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> solutionActor =
		vtkSmartPointer<vtkActor>::New();
	solutionActor->SetMapper(solutionMapper);
	solutionActor->GetProperty()->SetColor(1, 0, 0);
	solutionActor->GetProperty()->SetPointSize(10);

	vtkSmartPointer<vtkPolyDataMapper> icpMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	icpMapper->SetInputConnection(icpFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> icpActor =
		vtkSmartPointer<vtkActor>::New();
	icpActor->SetMapper(icpMapper);
	icpActor->GetProperty()->SetColor(0, 1, 1);
	icpActor->GetProperty()->SetPointSize(10);

	vtkSmartPointer<vtkRenderer> render =
		vtkSmartPointer<vtkRenderer>::New();
	render->AddActor(sourceActor);
	render->AddActor(targetActor);
	render->AddActor(solutionActor);
	render->AddActor(icpActor);
	render->SetBackground(0, 0, 0);

 //-------------------------------------------------------------

	vtkSmartPointer<vtkRenderWindow> rw =
		vtkSmartPointer<vtkRenderWindow>::New();
	rw->AddRenderer(render);
	rw->SetSize(480, 480);
	rw->SetWindowName("Regisration by Landmark");
	//设置坐标系显示功能
	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetScale(10);
	render->AddActor(axes);

	vtkSmartPointer<vtkRenderWindowInteractor> rwi =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	rwi->SetRenderWindow(rw);

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	rwi->SetInteractorStyle(style);
	/************************************************************/
	vtkSmartPointer<vtkOrientationMarkerWidget> widget =
		vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	widget->SetOutlineColor(0.9300, 0.5700, 0.1300);
	widget->SetOrientationMarker(axes);
	widget->SetInteractor(rwi); //加入鼠标交互
	widget->SetViewport(0.0, 0.0, 0.3, 0.3);  //设置显示位置
	widget->SetEnabled(1);
	widget->InteractiveOn();//开启鼠标交互
	/************************************************************/
	render->ResetCamera();
	rw->Render();
	rwi->Initialize();
	rwi->Start();

	return 0;
}