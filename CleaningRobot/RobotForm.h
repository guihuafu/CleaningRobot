#pragma once
#include <stdlib.h>
#include <stdio.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <Windows.h>
#include "MotionCombine.h"
#using <System.Windows.Forms.DataVisualization.dll> 

namespace CleaningRobot {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Windows::Forms::DataVisualization::Charting;

	public ref class RobotForm : public System::Windows::Forms::Form
	{
	public:
		// 构造函数代码
		RobotForm(void)
		{
			this->mAllAxisShow = false;
			this->mCurrentTime = 0;
			this->mJointMoveNum = 0;
			this->mJointPos = new double[MaxAxisNum];
			this->mSpacePos = new double[MaxAxisNum];
			this->mJointVel = new double[MaxAxisNum];
			this->mJointPos[0] = 0.0; this->mJointPos[1] = -90.0; this->mJointPos[2] = 180.0;
			this->mJointPos[3] = 0.0; this->mJointPos[4] = 90.0; this->mJointPos[5] = 0.0;
			this->mJointPos[6] = 0.0; this->mJointPos[7] = 0.0; this->mJointPos[8] = 0.0;
			memset(this->mSpacePos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mJointVel, 0.0, sizeof(double)*MaxAxisNum);

			CreateConsole();		// 控制台窗口初始化
			InitializeComponent();  // 空间窗口初始化

			// 设置绘制曲线信息
			this->JointChart->Series->Clear();

			// 添加关节1位置曲线
			Series^ JointSeries1 = gcnew Series("关节1");
			JointSeries1->ChartType = SeriesChartType::Line;  // 折线图
			JointSeries1->Color = Color::Blue;
			this->JointChart->Series->Add(JointSeries1);
			// 添加关节2位置曲线
			Series^ JointSeries2 = gcnew Series("关节2");
			JointSeries2->ChartType = SeriesChartType::Line;
			JointSeries2->Color = Color::Red;
			this->JointChart->Series->Add(JointSeries2);
			// 添加关节3位置曲线
			Series^ JointSeries3 = gcnew Series("关节3");
			JointSeries3->ChartType = SeriesChartType::Line;
			JointSeries3->Color = Color::Green;
			this->JointChart->Series->Add(JointSeries3);
			// 添加关节4位置曲线
			Series^ JointSeries4 = gcnew Series("关节4");
			JointSeries4->ChartType = SeriesChartType::Line;
			JointSeries4->Color = Color::Orange;
			this->JointChart->Series->Add(JointSeries4);

			// 添加关节速度1位置曲线
			Series^ JointVelSeries1 = gcnew Series("关节速度1");
			JointVelSeries1->ChartType = SeriesChartType::Line;  // 折线图
			JointVelSeries1->Color = Color::Blue;
			this->JointVelChart->Series->Add(JointVelSeries1);
			// 添加关节速度2位置曲线
			Series^ JointVelSeries2 = gcnew Series("关节速度2");
			JointVelSeries2->ChartType = SeriesChartType::Line;
			JointVelSeries2->Color = Color::Red;
			this->JointVelChart->Series->Add(JointVelSeries2);
			// 添加关节速度3位置曲线
			Series^ JointVelSeries3 = gcnew Series("关节速度3");
			JointVelSeries3->ChartType = SeriesChartType::Line;
			JointVelSeries3->Color = Color::Green;
			this->JointVelChart->Series->Add(JointVelSeries3);
			// 添加关节速度4位置曲线
			Series^ JointVelSeries4 = gcnew Series("关节速度4");
			JointVelSeries4->ChartType = SeriesChartType::Line;
			JointVelSeries4->Color = Color::Orange;
			this->JointVelChart->Series->Add(JointVelSeries4);

			// 添加空间X轴位置曲线
			Series^ SpaceSeries1 = gcnew Series("空间X轴");
			SpaceSeries1->ChartType = SeriesChartType::Line;  // 折线图
			SpaceSeries1->Color = Color::Blue;
			this->SpaceChart->Series->Add(SpaceSeries1);
			// 添加空间Y轴位置曲线
			Series^ SpaceSeries2 = gcnew Series("空间Y轴");
			SpaceSeries2->ChartType = SeriesChartType::Line;
			SpaceSeries2->Color = Color::Red;
			this->SpaceChart->Series->Add(SpaceSeries2);
			// 添加空间Z轴位置曲线
			Series^ SpaceSeries3 = gcnew Series("空间Z轴");
			SpaceSeries3->ChartType = SeriesChartType::Line;
			SpaceSeries3->Color = Color::Green;
			this->SpaceChart->Series->Add(SpaceSeries3);

			// 设置关节坐标轴 
			this->JointChart->ChartAreas[0]->AxisX->Title = "时间 (s)";
			this->JointChart->ChartAreas[0]->AxisY->Title = "位置 (度)";
			this->JointChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			// 设置关节速度坐标轴 
			this->JointVelChart->ChartAreas[0]->AxisX->Title = "时间 (s)";
			this->JointVelChart->ChartAreas[0]->AxisY->Title = "毫米/秒 (mm/s)";
			this->JointVelChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			// 设置空间坐标轴 
			this->SpaceChart->ChartAreas[0]->AxisX->Title = "时间 (s)";
			this->SpaceChart->ChartAreas[0]->AxisY->Title = "毫米 (mm)";
			this->SpaceChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			motioncombine = new hsc3::algo::MotionCombine();
			
		}

	protected:
		~RobotForm()
		{
			if (components)
			{
				delete []mJointPos;
				delete []mJointVel;
				delete []mSpacePos;
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  MoveStartButton;
	protected: 

	private: System::Windows::Forms::DataVisualization::Charting::Chart^  JointChart;
	private: System::ComponentModel::IContainer^  components;
	protected: 

	private:
		// 私有变量定义

	private: System::Windows::Forms::DataVisualization::Charting::Chart^  SpaceChart;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  JointVelChart;
	private: System::Windows::Forms::Button^  J1PosButton;
	private: System::Windows::Forms::Timer^  JointMoveTimer;
	private: System::Windows::Forms::Button^  J1NegButton;
	private: System::Windows::Forms::Button^  J2PosButton;
	private: System::Windows::Forms::Button^  J2NegButton;
	private: System::Windows::Forms::Button^  J3NegButton;
	private: System::Windows::Forms::Button^  J3PosButton;
	private: System::Windows::Forms::Button^  J4NegButton;
	private: System::Windows::Forms::Button^  J4PosButton;
	private: System::Windows::Forms::Button^  ClearChart;
	private: System::Windows::Forms::CheckBox^  AllAxisShow;
	hsc3::algo::MotionCombine *motioncombine;
	bool mAllAxisShow;
	int mCurrentTime;
	int mJointMoveNum;
	double *mJointPos;
	double *mSpacePos;
	double *mJointVel;


		 

#pragma region Windows Form Designer generated code

		void CreateConsole()
		{
			AllocConsole();  // 创建控制台
			freopen_s((FILE**)stdout, "CONOUT$", "w", stdout);  // 重定向stdout 
		}

		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea2 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea3 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			this->MoveStartButton = (gcnew System::Windows::Forms::Button());
			this->JointChart = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->SpaceChart = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->JointVelChart = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->J1PosButton = (gcnew System::Windows::Forms::Button());
			this->JointMoveTimer = (gcnew System::Windows::Forms::Timer(this->components));
			this->J1NegButton = (gcnew System::Windows::Forms::Button());
			this->J2PosButton = (gcnew System::Windows::Forms::Button());
			this->J2NegButton = (gcnew System::Windows::Forms::Button());
			this->J3NegButton = (gcnew System::Windows::Forms::Button());
			this->J3PosButton = (gcnew System::Windows::Forms::Button());
			this->J4NegButton = (gcnew System::Windows::Forms::Button());
			this->J4PosButton = (gcnew System::Windows::Forms::Button());
			this->ClearChart = (gcnew System::Windows::Forms::Button());
			this->AllAxisShow = (gcnew System::Windows::Forms::CheckBox());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointChart))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->SpaceChart))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointVelChart))->BeginInit();
			this->SuspendLayout();
			// 
			// MoveStartButton
			// 
			this->MoveStartButton->Location = System::Drawing::Point(23, 34);
			this->MoveStartButton->Name = L"MoveStartButton";
			this->MoveStartButton->Size = System::Drawing::Size(85, 45);
			this->MoveStartButton->TabIndex = 0;
			this->MoveStartButton->Text = L"开始运动";
			this->MoveStartButton->UseVisualStyleBackColor = true;
			this->MoveStartButton->Click += gcnew System::EventHandler(this, &RobotForm::MoveToStart_Click);
			// 
			// JointChart
			// 
			chartArea1->Name = L"ChartArea1";
			this->JointChart->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->JointChart->Legends->Add(legend1);
			this->JointChart->Location = System::Drawing::Point(12, 108);
			this->JointChart->Name = L"JointChart";
			this->JointChart->Size = System::Drawing::Size(649, 289);
			this->JointChart->TabIndex = 1;
			this->JointChart->Text = L"关节位置";
			// 
			// SpaceChart
			// 
			chartArea2->Name = L"ChartArea1";
			this->SpaceChart->ChartAreas->Add(chartArea2);
			legend2->Name = L"Legend1";
			this->SpaceChart->Legends->Add(legend2);
			this->SpaceChart->Location = System::Drawing::Point(12, 403);
			this->SpaceChart->Name = L"SpaceChart";
			this->SpaceChart->Size = System::Drawing::Size(649, 289);
			this->SpaceChart->TabIndex = 2;
			this->SpaceChart->Text = L"空间位置";
			// 
			// JointVelChart
			// 
			chartArea3->Name = L"ChartArea1";
			this->JointVelChart->ChartAreas->Add(chartArea3);
			legend3->Name = L"Legend1";
			this->JointVelChart->Legends->Add(legend3);
			this->JointVelChart->Location = System::Drawing::Point(681, 108);
			this->JointVelChart->Name = L"JointVelChart";
			this->JointVelChart->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->JointVelChart->Size = System::Drawing::Size(649, 289);
			this->JointVelChart->TabIndex = 3;
			this->JointVelChart->Text = L"关节速度";
			// 
			// J1PosButton
			// 
			this->J1PosButton->Location = System::Drawing::Point(220, 13);
			this->J1PosButton->Name = L"J1PosButton";
			this->J1PosButton->Size = System::Drawing::Size(77, 39);
			this->J1PosButton->TabIndex = 4;
			this->J1PosButton->Text = L"J1+";
			this->J1PosButton->UseVisualStyleBackColor = true;
			this->J1PosButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J1PosButton_MouseDown);
			this->J1PosButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J1PosButton_MouseUp);
			// 
			// JointMoveTimer
			// 
			this->JointMoveTimer->Interval = 10;
			this->JointMoveTimer->Tick += gcnew System::EventHandler(this, &RobotForm::JointMoveTimer_Tick);
			// 
			// J1NegButton
			// 
			this->J1NegButton->Location = System::Drawing::Point(314, 13);
			this->J1NegButton->Name = L"J1NegButton";
			this->J1NegButton->Size = System::Drawing::Size(77, 39);
			this->J1NegButton->TabIndex = 5;
			this->J1NegButton->Text = L"J1-";
			this->J1NegButton->UseVisualStyleBackColor = true;
			this->J1NegButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J1NegButton_MouseDown);
			this->J1NegButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J1NegButton_MouseUp);
			// 
			// J2PosButton
			// 
			this->J2PosButton->Location = System::Drawing::Point(220, 58);
			this->J2PosButton->Name = L"J2PosButton";
			this->J2PosButton->Size = System::Drawing::Size(77, 39);
			this->J2PosButton->TabIndex = 6;
			this->J2PosButton->Text = L"J2+";
			this->J2PosButton->UseVisualStyleBackColor = true;
			this->J2PosButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J2PosButton_MouseDown);
			this->J2PosButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J2PosButton_MouseUp);
			// 
			// J2NegButton
			// 
			this->J2NegButton->Location = System::Drawing::Point(314, 58);
			this->J2NegButton->Name = L"J2NegButton";
			this->J2NegButton->Size = System::Drawing::Size(77, 39);
			this->J2NegButton->TabIndex = 7;
			this->J2NegButton->Text = L"J2-";
			this->J2NegButton->UseVisualStyleBackColor = true;
			this->J2NegButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J2NegButton_MouseDown);
			this->J2NegButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J2NegButton_MouseUp);
			// 
			// J3NegButton
			// 
			this->J3NegButton->Location = System::Drawing::Point(520, 13);
			this->J3NegButton->Name = L"J3NegButton";
			this->J3NegButton->Size = System::Drawing::Size(77, 39);
			this->J3NegButton->TabIndex = 9;
			this->J3NegButton->Text = L"J3-";
			this->J3NegButton->UseVisualStyleBackColor = true;
			this->J3NegButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J3NegButton_MouseDown);
			this->J3NegButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J3NegButton_MouseUp);
			// 
			// J3PosButton
			// 
			this->J3PosButton->Location = System::Drawing::Point(426, 13);
			this->J3PosButton->Name = L"J3PosButton";
			this->J3PosButton->Size = System::Drawing::Size(77, 39);
			this->J3PosButton->TabIndex = 8;
			this->J3PosButton->Text = L"J3+";
			this->J3PosButton->UseVisualStyleBackColor = true;
			this->J3PosButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J3PosButton_MouseDown);
			this->J3PosButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J3PosButton_MouseUp);
			// 
			// J4NegButton
			// 
			this->J4NegButton->Location = System::Drawing::Point(520, 58);
			this->J4NegButton->Name = L"J4NegButton";
			this->J4NegButton->Size = System::Drawing::Size(77, 39);
			this->J4NegButton->TabIndex = 11;
			this->J4NegButton->Text = L"J4-";
			this->J4NegButton->UseVisualStyleBackColor = true;
			this->J4NegButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J4NegButton_MouseDown);
			this->J4NegButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J4NegButton_MouseUp);
			// 
			// J4PosButton
			// 
			this->J4PosButton->Location = System::Drawing::Point(426, 58);
			this->J4PosButton->Name = L"J4PosButton";
			this->J4PosButton->Size = System::Drawing::Size(77, 39);
			this->J4PosButton->TabIndex = 10;
			this->J4PosButton->Text = L"J4+";
			this->J4PosButton->UseVisualStyleBackColor = true;
			this->J4PosButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J4PosButton_MouseDown);
			this->J4PosButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J4PosButton_MouseUp);
			// 
			// ClearChart
			// 
			this->ClearChart->Location = System::Drawing::Point(630, 34);
			this->ClearChart->Name = L"ClearChart";
			this->ClearChart->Size = System::Drawing::Size(85, 45);
			this->ClearChart->TabIndex = 12;
			this->ClearChart->Text = L"清除";
			this->ClearChart->UseVisualStyleBackColor = true;
			this->ClearChart->Click += gcnew System::EventHandler(this, &RobotForm::ClearChart_Click);
			// 
			// AllAxisShow
			// 
			this->AllAxisShow->AutoSize = true;
			this->AllAxisShow->Location = System::Drawing::Point(779, 34);
			this->AllAxisShow->Name = L"AllAxisShow";
			this->AllAxisShow->Size = System::Drawing::Size(59, 19);
			this->AllAxisShow->TabIndex = 13;
			this->AllAxisShow->Text = L"全显";
			this->AllAxisShow->UseVisualStyleBackColor = true;
			this->AllAxisShow->CheckedChanged += gcnew System::EventHandler(this, &RobotForm::AllAxisShow_CheckedChanged);
			// 
			// RobotForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1361, 754);
			this->Controls->Add(this->AllAxisShow);
			this->Controls->Add(this->ClearChart);
			this->Controls->Add(this->J4NegButton);
			this->Controls->Add(this->J4PosButton);
			this->Controls->Add(this->J3NegButton);
			this->Controls->Add(this->J3PosButton);
			this->Controls->Add(this->J2NegButton);
			this->Controls->Add(this->J2PosButton);
			this->Controls->Add(this->J1NegButton);
			this->Controls->Add(this->J1PosButton);
			this->Controls->Add(this->JointVelChart);
			this->Controls->Add(this->SpaceChart);
			this->Controls->Add(this->JointChart);
			this->Controls->Add(this->MoveStartButton);
			this->Name = L"RobotForm";
			this->Text = L"CleaningRobot";
			this->Load += gcnew System::EventHandler(this, &RobotForm::RobotForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointChart))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->SpaceChart))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointVelChart))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}

#pragma endregion
	private: System::Void setChart(int axixnum, int count, double *jointpos, double *jointvel, double *spacepos, bool allshow)
			{
				std::stringstream ss;
				ss << axixnum;
				std::string strAxisNum = "关节1";
				if(!allshow)
				{
					this->JointChart->Series["关节1"]->Points->AddXY(count, jointpos[axixnum]);			// 添加关节1数据点
					this->JointVelChart->Series["关节速度1"]->Points->AddXY(count, jointvel[axixnum]);	// 添加关节速度1数据点
					this->SpaceChart->Series["空间X轴"]->Points->AddXY(count, spacepos[axixnum]);			// 添加空间X轴数据点
				}
				else
				{
					this->JointChart->Series["关节1"]->Points->AddXY(count, jointpos[0]);			// 添加关节1数据点
					this->JointChart->Series["关节2"]->Points->AddXY(count, jointpos[1]);			// 添加关节2数据点
					this->JointChart->Series["关节3"]->Points->AddXY(count, jointpos[2]);			// 添加关节3数据点
					this->JointChart->Series["关节4"]->Points->AddXY(count, jointpos[3]);			// 添加关节4数据点

					this->JointVelChart->Series["关节速度1"]->Points->AddXY(count, jointvel[0]);	// 添加关节速度1数据点
					this->JointVelChart->Series["关节速度2"]->Points->AddXY(count, jointvel[1]);	// 添加关节速度2数据点
					this->JointVelChart->Series["关节速度3"]->Points->AddXY(count, jointvel[2]);	// 添加关节速度3数据点
					this->JointVelChart->Series["关节速度4"]->Points->AddXY(count, jointvel[3]);	// 添加关节速度4数据点

					this->SpaceChart->Series["空间X轴"]->Points->AddXY(count, spacepos[0]);			// 添加空间X轴数据点
					this->SpaceChart->Series["空间Y轴"]->Points->AddXY(count, spacepos[1]);			// 添加空间Y轴数据点
					this->SpaceChart->Series["空间Z轴"]->Points->AddXY(count, spacepos[2]);			// 添加空间Z轴数据点
				}
			 }

	private: System::Void planJoint(int axisnum, bool dir, double *nowpos)
			 {
				 this->mJointMoveNum = axisnum;
				 this->motioncombine->planManual(axisnum, dir, nowpos);
				 this->JointMoveTimer->Start();
			 }

	private: System::Void stayJoint(int axisnum)
			 {
				 this->mCurrentTime = this->mCurrentTime + 1;
				 this->motioncombine->execManualIntMove(this->mJointPos, this->mJointVel, this->mSpacePos);
				 this->setChart(axisnum, this->mCurrentTime, this->mJointPos, this->mJointVel, this->mSpacePos, this->mAllAxisShow);
				 printf("stayJoint: %f %f %f %f %f %f \n",mJointPos[0],mJointPos[1],mJointPos[2],mJointPos[3],mJointPos[4],mJointPos[5]);
			 }

	private: System::Void stopJoint(int axisnum)
			 {
				 hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
				 this->mJointMoveNum = axisnum;
				 this->JointMoveTimer->Stop();
				 this->motioncombine->stopPlanManual();
				 while(status != hsc3::algo::M_Done)
				 {
					 this->mCurrentTime = this->mCurrentTime + 1;
					 status = this->motioncombine->execManualIntMove(this->mJointPos, this->mJointVel, this->mSpacePos);
					 this->setChart(axisnum, this->mCurrentTime, this->mJointPos, this->mJointVel, this->mSpacePos, this->mAllAxisShow);
					 printf("stopJoint: %f %f %f %f %f %f \n",mJointPos[0],mJointPos[1],mJointPos[2],mJointPos[3],mJointPos[4],mJointPos[5]);
				 }
			 }

	private: System::Void RobotForm_Load(System::Object^  sender, System::EventArgs^  e) {}

	private: System::Void MoveToStart_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				static int iCurrentTime = 0;
				double dJointPos[MaxAxisNum] = {0.0};
				double dJointVel[MaxAxisNum] = {0.0};
				double dSpacePos[MaxAxisNum] = {0.0};
				hsc3::algo::IntData intdata;
				hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;

				//motioncombine->planSpace();
				//while(status != hsc3::algo::M_Done)
				//{
				//	status = motioncombine->execSpaceIntMove(dJointPos, dJointVel, dSpacePos);
				//	this->setChart(iCurrentTime, dJointPos, dJointVel, dSpacePos);
				//	//printf("status=%d, outJPos: %f %f %f %f %f %f \n", status,dJointPos[0],dJointPos[1],dJointPos[2],dJointPos[3],dJointPos[4],dJointPos[5]);
				//	//printf("status=%d, outSPos: %f %f %f %f %f %f \n", status,dSpacePos[0],dSpacePos[1],dSpacePos[2],dSpacePos[3],dSpacePos[4],dSpacePos[5]);
				//	printf("status=%d, outVelPos: %f %f %f %f %f %f \n", status,dJointVel[0],dJointVel[1],dJointVel[2],dJointVel[3],dJointVel[4],dJointVel[5]);
				//	iCurrentTime = iCurrentTime + 1;
				//}

				motioncombine->planJoint();
				while(status != hsc3::algo::M_Done)
				{
					status = motioncombine->execJointIntMove(dJointPos, dJointVel, dSpacePos);
					this->setChart(0, iCurrentTime, dJointPos, dJointVel, dSpacePos, true);
					//printf("status=%d, outPos: %f %f %f %f %f %f \n", status, dJointPos[0],dJointPos[1],dJointPos[2],dJointPos[3],dJointPos[4],dJointPos[5]);
					printf("status=%d, outVelPos: %f %f %f %f %f %f \n", status,dJointVel[0],dJointVel[1],dJointVel[2],dJointVel[3],dJointVel[4],dJointVel[5]);
					iCurrentTime = iCurrentTime + 1;
				}
			 }

	private: System::Void JointMoveTimer_Tick(System::Object^  sender, System::EventArgs^  e) {
				this->stayJoint(this->mJointMoveNum);
			 }

	private: System::Void J1PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(0, true, mJointPos);
			 }

	private: System::Void J1PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(0);
			 }

	private: System::Void J1NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(0, false, mJointPos);
			 }

	private: System::Void J1NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(0);
			 }

	private: System::Void J2PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(1, true, mJointPos);
			 }

	private: System::Void J2PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(1);
			 }

	private: System::Void J2NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(1, false, mJointPos);
			 }

	private: System::Void J2NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(1);
			 }

	private: System::Void J3PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(2, true, mJointPos);
			 }

	private: System::Void J3PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(2);
			 }

	private: System::Void J3NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(2, false, mJointPos);
			 }

	private: System::Void J3NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(2);
			 }

	private: System::Void J4PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(3, true, mJointPos);
			 }

	private: System::Void J4PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(3);
			 }

	private: System::Void J4NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(3, false, mJointPos);
			 }

	private: System::Void J4NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(3);
			 }

	private: System::Void ClearChart_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				for each (Series^ series in this->JointChart->Series)
					series->Points->Clear();
				for each (Series^ series in this->JointVelChart->Series)
					series->Points->Clear();
				for each (Series^ series in this->SpaceChart->Series)
					series->Points->Clear();
			 }
	private: System::Void AllAxisShow_CheckedChanged(System::Object^  sender, System::EventArgs^  e) 
			 {
				this->mAllAxisShow = AllAxisShow->Checked;
				printf("AllAxisShow_CheckedChanged %d \n", mAllAxisShow);
			 }
};

}

