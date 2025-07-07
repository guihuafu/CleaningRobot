#pragma once
#include <stdlib.h>
#include <stdio.h> 
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
		// ���캯������
		RobotForm(void)
		{
			this->mCurrentTime = 0;
			this->mJointPos = new double[MaxAxisNum];
			this->mJointPos[0] = 0.0; this->mJointPos[1] = -90.0; this->mJointPos[2] = 180.0;
			this->mJointPos[3] = 0.0; this->mJointPos[4] = 90.0; this->mJointPos[5] = 0.0;
			this->mJointPos[6] = 0.0; this->mJointPos[7] = 0.0; this->mJointPos[8] = 0.0;

			CreateConsole();		// ����̨���ڳ�ʼ��
			InitializeComponent();  // �ռ䴰�ڳ�ʼ��

			// ���û���������Ϣ
			this->JointChart->Series->Clear();

			// ��ӹؽ�1λ������
			Series^ JointSeries1 = gcnew Series("�ؽ�1");
			JointSeries1->ChartType = SeriesChartType::Line;  // ����ͼ
			JointSeries1->Color = Color::Blue;
			this->JointChart->Series->Add(JointSeries1);
			// ��ӹؽ�2λ������
			Series^ JointSeries2 = gcnew Series("�ؽ�2");
			JointSeries2->ChartType = SeriesChartType::Line;
			JointSeries2->Color = Color::Red;
			this->JointChart->Series->Add(JointSeries2);
			// ��ӹؽ�3λ������
			Series^ JointSeries3 = gcnew Series("�ؽ�3");
			JointSeries3->ChartType = SeriesChartType::Line;
			JointSeries3->Color = Color::Green;
			this->JointChart->Series->Add(JointSeries3);
			// ��ӹؽ�4λ������
			Series^ JointSeries4 = gcnew Series("�ؽ�4");
			JointSeries4->ChartType = SeriesChartType::Line;
			JointSeries4->Color = Color::Orange;
			this->JointChart->Series->Add(JointSeries4);

			// ��ӹؽ��ٶ�1λ������
			Series^ JointVelSeries1 = gcnew Series("�ؽ��ٶ�1");
			JointVelSeries1->ChartType = SeriesChartType::Line;  // ����ͼ
			JointVelSeries1->Color = Color::Blue;
			this->JointVelChart->Series->Add(JointVelSeries1);
			// ��ӹؽ��ٶ�2λ������
			Series^ JointVelSeries2 = gcnew Series("�ؽ��ٶ�2");
			JointVelSeries2->ChartType = SeriesChartType::Line;
			JointVelSeries2->Color = Color::Red;
			this->JointVelChart->Series->Add(JointVelSeries2);
			// ��ӹؽ��ٶ�3λ������
			Series^ JointVelSeries3 = gcnew Series("�ؽ��ٶ�3");
			JointVelSeries3->ChartType = SeriesChartType::Line;
			JointVelSeries3->Color = Color::Green;
			this->JointVelChart->Series->Add(JointVelSeries3);
			// ��ӹؽ��ٶ�4λ������
			Series^ JointVelSeries4 = gcnew Series("�ؽ��ٶ�4");
			JointVelSeries4->ChartType = SeriesChartType::Line;
			JointVelSeries4->Color = Color::Orange;
			this->JointVelChart->Series->Add(JointVelSeries4);

			// ��ӿռ�X��λ������
			Series^ SpaceSeries1 = gcnew Series("�ռ�X��");
			SpaceSeries1->ChartType = SeriesChartType::Line;  // ����ͼ
			SpaceSeries1->Color = Color::Blue;
			this->SpaceChart->Series->Add(SpaceSeries1);
			// ��ӿռ�Y��λ������
			Series^ SpaceSeries2 = gcnew Series("�ռ�Y��");
			SpaceSeries2->ChartType = SeriesChartType::Line;
			SpaceSeries2->Color = Color::Red;
			this->SpaceChart->Series->Add(SpaceSeries2);
			// ��ӿռ�Z��λ������
			Series^ SpaceSeries3 = gcnew Series("�ռ�Z��");
			SpaceSeries3->ChartType = SeriesChartType::Line;
			SpaceSeries3->Color = Color::Green;
			this->SpaceChart->Series->Add(SpaceSeries3);

			// ���ùؽ������� 
			this->JointChart->ChartAreas[0]->AxisX->Title = "ʱ�� (s)";
			this->JointChart->ChartAreas[0]->AxisY->Title = "λ�� (��)";
			this->JointChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			// ���ùؽ��ٶ������� 
			this->JointVelChart->ChartAreas[0]->AxisX->Title = "ʱ�� (s)";
			this->JointVelChart->ChartAreas[0]->AxisY->Title = "����/�� (mm/s)";
			this->JointVelChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			// ���ÿռ������� 
			this->SpaceChart->ChartAreas[0]->AxisX->Title = "ʱ�� (s)";
			this->SpaceChart->ChartAreas[0]->AxisY->Title = "���� (mm)";
			this->SpaceChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			motioncombine = new hsc3::algo::MotionCombine();
			
		}

	protected:
		~RobotForm()
		{
			if (components)
			{
				delete []mJointPos;
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  MoveStartButton;
	protected: 

	private: System::Windows::Forms::DataVisualization::Charting::Chart^  JointChart;
	private: System::ComponentModel::IContainer^  components;
	protected: 

	private:
		// ˽�б�������

	private: System::Windows::Forms::DataVisualization::Charting::Chart^  SpaceChart;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  JointVelChart;
private: System::Windows::Forms::Button^  J1PosButton;
private: System::Windows::Forms::Timer^  J1MoveTimer;
		 hsc3::algo::MotionCombine *motioncombine;
		 int mCurrentTime;
		 double *mJointPos;

#pragma region Windows Form Designer generated code

		void CreateConsole()
		{
			AllocConsole();  // ��������̨
			freopen_s((FILE**)stdout, "CONOUT$", "w", stdout);  // �ض���stdout 
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
			this->J1MoveTimer = (gcnew System::Windows::Forms::Timer(this->components));
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointChart))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->SpaceChart))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointVelChart))->BeginInit();
			this->SuspendLayout();
			// 
			// MoveStartButton
			// 
			this->MoveStartButton->Location = System::Drawing::Point(12, 12);
			this->MoveStartButton->Name = L"MoveStartButton";
			this->MoveStartButton->Size = System::Drawing::Size(85, 45);
			this->MoveStartButton->TabIndex = 0;
			this->MoveStartButton->Text = L"��ʼ�˶�";
			this->MoveStartButton->UseVisualStyleBackColor = true;
			this->MoveStartButton->Click += gcnew System::EventHandler(this, &RobotForm::MoveToStart_Click);
			// 
			// JointChart
			// 
			chartArea1->Name = L"ChartArea1";
			this->JointChart->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->JointChart->Legends->Add(legend1);
			this->JointChart->Location = System::Drawing::Point(12, 76);
			this->JointChart->Name = L"JointChart";
			this->JointChart->Size = System::Drawing::Size(649, 289);
			this->JointChart->TabIndex = 1;
			this->JointChart->Text = L"�ؽ�λ��";
			// 
			// SpaceChart
			// 
			chartArea2->Name = L"ChartArea1";
			this->SpaceChart->ChartAreas->Add(chartArea2);
			legend2->Name = L"Legend1";
			this->SpaceChart->Legends->Add(legend2);
			this->SpaceChart->Location = System::Drawing::Point(12, 371);
			this->SpaceChart->Name = L"SpaceChart";
			this->SpaceChart->Size = System::Drawing::Size(649, 289);
			this->SpaceChart->TabIndex = 2;
			this->SpaceChart->Text = L"�ռ�λ��";
			// 
			// JointVelChart
			// 
			chartArea3->Name = L"ChartArea1";
			this->JointVelChart->ChartAreas->Add(chartArea3);
			legend3->Name = L"Legend1";
			this->JointVelChart->Legends->Add(legend3);
			this->JointVelChart->Location = System::Drawing::Point(681, 76);
			this->JointVelChart->Name = L"JointVelChart";
			this->JointVelChart->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->JointVelChart->Size = System::Drawing::Size(649, 289);
			this->JointVelChart->TabIndex = 3;
			this->JointVelChart->Text = L"�ؽ��ٶ�";
			// 
			// J1PosButton
			// 
			this->J1PosButton->Location = System::Drawing::Point(213, 12);
			this->J1PosButton->Name = L"J1PosButton";
			this->J1PosButton->Size = System::Drawing::Size(72, 32);
			this->J1PosButton->TabIndex = 4;
			this->J1PosButton->Text = L"J1+";
			this->J1PosButton->UseVisualStyleBackColor = true;
			this->J1PosButton->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J1PosButton_MouseDown);
			this->J1PosButton->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &RobotForm::J1PosButton_MouseUp);
			// 
			// J1MoveTimer
			// 
			this->J1MoveTimer->Interval = 10;
			this->J1MoveTimer->Tick += gcnew System::EventHandler(this, &RobotForm::J1MoveTimer_Tick);
			// 
			// RobotForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1361, 673);
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

		}

#pragma endregion
	private: System::Void setChart(int count, double *jointpos, double *jointvel, double *spacepos)
			 {
				this->JointChart->Series["�ؽ�1"]->Points->AddXY(count, jointpos[0]); // ��ӹؽ�1���ݵ�
				this->JointChart->Series["�ؽ�2"]->Points->AddXY(count, jointpos[1]); // ��ӹؽ�2���ݵ�
				this->JointChart->Series["�ؽ�3"]->Points->AddXY(count, jointpos[2]); // ��ӹؽ�3���ݵ�
				this->JointChart->Series["�ؽ�4"]->Points->AddXY(count, jointpos[3]); // ��ӹؽ�4���ݵ�

				this->JointVelChart->Series["�ؽ��ٶ�1"]->Points->AddXY(count, jointvel[0]); // ��ӹؽ��ٶ�1���ݵ�
				this->JointVelChart->Series["�ؽ��ٶ�2"]->Points->AddXY(count, jointvel[1]); // ��ӹؽ��ٶ�2���ݵ�
				this->JointVelChart->Series["�ؽ��ٶ�3"]->Points->AddXY(count, jointvel[2]); // ��ӹؽ��ٶ�3���ݵ�
				this->JointVelChart->Series["�ؽ��ٶ�4"]->Points->AddXY(count, jointvel[3]); // ��ӹؽ��ٶ�4���ݵ�

				this->SpaceChart->Series["�ռ�X��"]->Points->AddXY(count, spacepos[0]); // ��ӿռ�X�����ݵ�
				this->SpaceChart->Series["�ռ�Y��"]->Points->AddXY(count, spacepos[1]); // ��ӿռ�Y�����ݵ�
				this->SpaceChart->Series["�ռ�Z��"]->Points->AddXY(count, spacepos[2]); // ��ӿռ�Z�����ݵ�
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
					this->setChart(iCurrentTime, dJointPos, dJointVel, dSpacePos);
					//printf("status=%d, outPos: %f %f %f %f %f %f \n", status, dJointPos[0],dJointPos[1],dJointPos[2],dJointPos[3],dJointPos[4],dJointPos[5]);
					printf("status=%d, outVelPos: %f %f %f %f %f %f \n", status,dJointVel[0],dJointVel[1],dJointVel[2],dJointVel[3],dJointVel[4],dJointVel[5]);
					iCurrentTime = iCurrentTime + 1;
				}

				//motioncombine->planManual();
				//for(int i=0; i<200; i++)
				//{
				//	status = motioncombine->execManualIntMove(dJointPos);
				//	printf("status:%d,outPos: %f %f %f %f %f %f \n",status,dJointPos[0],dJointPos[1],dJointPos[2],dJointPos[3],dJointPos[4],dJointPos[5]);
				//}
				//motioncombine->stopPlanManual();
				//for(int i=0; i<100; i++)
				//{
				//	status = motioncombine->execManualIntMove(dJointPos);
				//	printf("status=%d,outStopPos: %f %f %f %f %f %f \n",status,dJointPos[0],dJointPos[1],dJointPos[2],dJointPos[3],dJointPos[4],dJointPos[5]);
				//}
			 }

	private: System::Void J1PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
				 hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;
				 this->J1MoveTimer->Stop();
				 this->motioncombine->stopPlanManual();
				 while(status != hsc3::algo::M_Done)
				 {
					 this->mCurrentTime = this->mCurrentTime + 1;
					 status = this->motioncombine->execManualIntMove(mJointPos);
					 printf("outStopPos: %f %f %f %f %f %f \n",mJointPos[0],mJointPos[1],mJointPos[2],mJointPos[3],mJointPos[4],mJointPos[5]);
				 }
			 }

	private: System::Void J1PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) 
			 {
				 this->motioncombine->planManual(mJointPos);
				 this->J1MoveTimer->Start();
			 }
	private: System::Void J1MoveTimer_Tick(System::Object^  sender, System::EventArgs^  e) 
			 {
				 this->mCurrentTime = this->mCurrentTime + 1;
				 this->motioncombine->execManualIntMove(mJointPos);
				 this->setChart(this->mCurrentTime, mJointPos, mJointPos, mJointPos);
				 printf("outJointPos: %f %f %f %f %f %f \n",mJointPos[0],mJointPos[1],mJointPos[2],mJointPos[3],mJointPos[4],mJointPos[5]);
			 }
};

}

