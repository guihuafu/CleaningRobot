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
		// ���캯������
		RobotForm(void)
		{
			this->mIsJointMove = true;
			this->mAllAxisShow = false;
			this->mCurrentTime = 0;
			this->mJointMoveNum = 0;
			this->mJointPos = new double[MaxAxisNum];
			this->mSpacePos = new double[MaxAxisNum];
			this->mJointVel = new double[MaxAxisNum];
			this->mJointAcc = new double[MaxAxisNum];
			this->mJointPos[0] = 0.0; this->mJointPos[1] = -90.0; this->mJointPos[2] = 180.0;
			this->mJointPos[3] = 0.0; this->mJointPos[4] = 90.0; this->mJointPos[5] = 0.0;
			this->mJointPos[6] = 0.0; this->mJointPos[7] = 0.0; this->mJointPos[8] = 0.0;
			memset(this->mSpacePos, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mJointVel, 0.0, sizeof(double)*MaxAxisNum);
			memset(this->mJointAcc, 0.0, sizeof(double)*MaxAxisNum);

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

			// ��ӹؽڼ��ٶ�1λ������
			Series^ JointAccSeries1 = gcnew Series("�ؽڼ��ٶ�1");
			JointAccSeries1->ChartType = SeriesChartType::Line;  // ����ͼ
			JointAccSeries1->Color = Color::Blue;
			this->JointAccChart->Series->Add(JointAccSeries1);
			// ��ӹؽڼ��ٶ�2λ������
			Series^ JointAccSeries2 = gcnew Series("�ؽڼ��ٶ�2");
			JointAccSeries2->ChartType = SeriesChartType::Line;
			JointAccSeries2->Color = Color::Red;
			this->JointAccChart->Series->Add(JointAccSeries2);
			// ��ӹؽڼ��ٶ�3λ������
			Series^ JointAccSeries3 = gcnew Series("�ؽڼ��ٶ�3");
			JointAccSeries3->ChartType = SeriesChartType::Line;
			JointAccSeries3->Color = Color::Green;
			this->JointAccChart->Series->Add(JointAccSeries3);
			// ��ӹؽڼ��ٶ�4λ������
			Series^ JointAccSeries4 = gcnew Series("�ؽڼ��ٶ�4");
			JointAccSeries4->ChartType = SeriesChartType::Line;
			JointAccSeries4->Color = Color::Orange;
			this->JointAccChart->Series->Add(JointAccSeries4);

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
			// ��ӿռ�A��λ������
			Series^ SpaceSeries4 = gcnew Series("�ռ�A��");
			SpaceSeries4->ChartType = SeriesChartType::Line;
			SpaceSeries4->Color = Color::Orange;
			this->SpaceChart->Series->Add(SpaceSeries4);

			// ���ùؽ������� 
			this->JointChart->ChartAreas[0]->AxisX->Title = "ʱ�� (s)";
			this->JointChart->ChartAreas[0]->AxisY->Title = "λ�� (��)";
			this->JointChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			// ���ùؽ��ٶ������� 
			this->JointVelChart->ChartAreas[0]->AxisX->Title = "ʱ�� (s)";
			this->JointVelChart->ChartAreas[0]->AxisY->Title = "����/�� (mm/s)";
			this->JointVelChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			// ���ùؽڼ��ٶ������� 
			this->JointAccChart->ChartAreas[0]->AxisX->Title = "ʱ�� (s)";
			this->JointAccChart->ChartAreas[0]->AxisY->Title = "����^2/�� (mm^2/s)";
			this->JointAccChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			// ���ÿռ������� 
			this->SpaceChart->ChartAreas[0]->AxisX->Title = "ʱ�� (s)";
			this->SpaceChart->ChartAreas[0]->AxisY->Title = "���� (mm)";
			this->SpaceChart->ChartAreas[0]->AxisX->IntervalAutoMode = IntervalAutoMode::VariableCount;

			this->Coordinate->SelectedIndex = 1;	// ��ʾ�б��еĵ�һ��
			motioncombine = new hsc3::algo::MotionCombine();
		}

	protected:
		~RobotForm()
		{
			if (components)
			{
				delete []mJointPos;
				delete []mJointVel;
				delete []mJointAcc;
				delete []mSpacePos;
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  MoveStartButton;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  JointChart;
	private: System::ComponentModel::IContainer^  components;
	protected: 

	private:
		// ˽�б�������

	private: System::Windows::Forms::DataVisualization::Charting::Chart^  SpaceChart;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  JointVelChart;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  JointAccChart;
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
	private: System::Windows::Forms::TextBox^  JointAngle1;
	private: System::Windows::Forms::TextBox^  JointAngle2;
	private: System::Windows::Forms::TextBox^  JointAngle3;
	private: System::Windows::Forms::TextBox^  JointAngle4;
	private: System::Windows::Forms::TextBox^  SpaceA;
	private: System::Windows::Forms::TextBox^  SpaceZ;
	private: System::Windows::Forms::TextBox^  SpaceY;
	private: System::Windows::Forms::TextBox^  SpaceX;
	private: System::Windows::Forms::Label^  J1;
	private: System::Windows::Forms::Label^  J2;
	private: System::Windows::Forms::Label^  J3;
	private: System::Windows::Forms::Label^  J4;
	private: System::Windows::Forms::Label^  X;
	private: System::Windows::Forms::Label^  Y;
	private: System::Windows::Forms::Label^  Z;
	private: System::Windows::Forms::Label^  A;
	private: System::Windows::Forms::ComboBox^  Coordinate;
	private: System::Windows::Forms::TextBox^  MoveToPos1;
	private: System::Windows::Forms::TextBox^  MoveToPos2;
	private: System::Windows::Forms::TextBox^  MoveToPos3;
	private: System::Windows::Forms::TextBox^  MoveToPos4;
	private: System::Windows::Forms::TextBox^  MessageBox;
	private: System::Windows::Forms::Label^  PosLabel;
	private: System::Windows::Forms::Button^  HomePosButton;
	private: System::Windows::Forms::Button^  TestPosButton;
	private: System::Windows::Forms::TextBox^  RatioBox;
	private: System::Windows::Forms::Label^  RatioLabel;
	hsc3::algo::MotionCombine *motioncombine;
	bool mIsJointMove;
	bool mAllAxisShow;
	int mCurrentTime;
	int mJointMoveNum;
	double *mJointPos;
	double *mSpacePos;
	double *mJointVel;
	double *mJointAcc;

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
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea4 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend4 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
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
			this->JointAccChart = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->JointAngle1 = (gcnew System::Windows::Forms::TextBox());
			this->JointAngle2 = (gcnew System::Windows::Forms::TextBox());
			this->JointAngle3 = (gcnew System::Windows::Forms::TextBox());
			this->JointAngle4 = (gcnew System::Windows::Forms::TextBox());
			this->SpaceA = (gcnew System::Windows::Forms::TextBox());
			this->SpaceZ = (gcnew System::Windows::Forms::TextBox());
			this->SpaceY = (gcnew System::Windows::Forms::TextBox());
			this->SpaceX = (gcnew System::Windows::Forms::TextBox());
			this->J1 = (gcnew System::Windows::Forms::Label());
			this->J2 = (gcnew System::Windows::Forms::Label());
			this->J3 = (gcnew System::Windows::Forms::Label());
			this->J4 = (gcnew System::Windows::Forms::Label());
			this->X = (gcnew System::Windows::Forms::Label());
			this->Y = (gcnew System::Windows::Forms::Label());
			this->Z = (gcnew System::Windows::Forms::Label());
			this->A = (gcnew System::Windows::Forms::Label());
			this->Coordinate = (gcnew System::Windows::Forms::ComboBox());
			this->MoveToPos1 = (gcnew System::Windows::Forms::TextBox());
			this->MoveToPos2 = (gcnew System::Windows::Forms::TextBox());
			this->MoveToPos3 = (gcnew System::Windows::Forms::TextBox());
			this->MoveToPos4 = (gcnew System::Windows::Forms::TextBox());
			this->MessageBox = (gcnew System::Windows::Forms::TextBox());
			this->PosLabel = (gcnew System::Windows::Forms::Label());
			this->HomePosButton = (gcnew System::Windows::Forms::Button());
			this->TestPosButton = (gcnew System::Windows::Forms::Button());
			this->RatioBox = (gcnew System::Windows::Forms::TextBox());
			this->RatioLabel = (gcnew System::Windows::Forms::Label());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointChart))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->SpaceChart))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointVelChart))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointAccChart))->BeginInit();
			this->SuspendLayout();
			// 
			// MoveStartButton
			// 
			this->MoveStartButton->Location = System::Drawing::Point(1188, 7);
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
			this->JointChart->Location = System::Drawing::Point(158, 108);
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
			this->SpaceChart->Location = System::Drawing::Point(158, 403);
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
			this->JointVelChart->Location = System::Drawing::Point(817, 108);
			this->JointVelChart->Name = L"JointVelChart";
			this->JointVelChart->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->JointVelChart->Size = System::Drawing::Size(649, 289);
			this->JointVelChart->TabIndex = 3;
			this->JointVelChart->Text = L"�ؽ��ٶ�";
			// 
			// J1PosButton
			// 
			this->J1PosButton->Location = System::Drawing::Point(156, 10);
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
			this->J1NegButton->Location = System::Drawing::Point(250, 10);
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
			this->J2PosButton->Location = System::Drawing::Point(156, 55);
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
			this->J2NegButton->Location = System::Drawing::Point(250, 55);
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
			this->J3NegButton->Location = System::Drawing::Point(456, 10);
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
			this->J3PosButton->Location = System::Drawing::Point(362, 10);
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
			this->J4NegButton->Location = System::Drawing::Point(456, 55);
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
			this->J4PosButton->Location = System::Drawing::Point(362, 55);
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
			this->ClearChart->Location = System::Drawing::Point(1188, 57);
			this->ClearChart->Name = L"ClearChart";
			this->ClearChart->Size = System::Drawing::Size(85, 45);
			this->ClearChart->TabIndex = 12;
			this->ClearChart->Text = L"���";
			this->ClearChart->UseVisualStyleBackColor = true;
			this->ClearChart->Click += gcnew System::EventHandler(this, &RobotForm::ClearChart_Click);
			// 
			// AllAxisShow
			// 
			this->AllAxisShow->AutoSize = true;
			this->AllAxisShow->Location = System::Drawing::Point(669, 23);
			this->AllAxisShow->Name = L"AllAxisShow";
			this->AllAxisShow->Size = System::Drawing::Size(59, 19);
			this->AllAxisShow->TabIndex = 13;
			this->AllAxisShow->Text = L"ȫ��";
			this->AllAxisShow->UseVisualStyleBackColor = true;
			this->AllAxisShow->CheckedChanged += gcnew System::EventHandler(this, &RobotForm::AllAxisShow_CheckedChanged);
			// 
			// JointAccChart
			// 
			chartArea4->Name = L"ChartArea1";
			this->JointAccChart->ChartAreas->Add(chartArea4);
			legend4->Name = L"Legend1";
			this->JointAccChart->Legends->Add(legend4);
			this->JointAccChart->Location = System::Drawing::Point(817, 403);
			this->JointAccChart->Name = L"JointAccChart";
			this->JointAccChart->Size = System::Drawing::Size(649, 289);
			this->JointAccChart->TabIndex = 14;
			this->JointAccChart->Text = L"�ؽڼ��ٶ�";
			// 
			// JointAngle1
			// 
			this->JointAngle1->Location = System::Drawing::Point(47, 108);
			this->JointAngle1->Name = L"JointAngle1";
			this->JointAngle1->Size = System::Drawing::Size(96, 25);
			this->JointAngle1->TabIndex = 15;
			// 
			// JointAngle2
			// 
			this->JointAngle2->Location = System::Drawing::Point(47, 149);
			this->JointAngle2->Name = L"JointAngle2";
			this->JointAngle2->Size = System::Drawing::Size(96, 25);
			this->JointAngle2->TabIndex = 16;
			// 
			// JointAngle3
			// 
			this->JointAngle3->Location = System::Drawing::Point(47, 190);
			this->JointAngle3->Name = L"JointAngle3";
			this->JointAngle3->Size = System::Drawing::Size(96, 25);
			this->JointAngle3->TabIndex = 17;
			// 
			// JointAngle4
			// 
			this->JointAngle4->Location = System::Drawing::Point(47, 231);
			this->JointAngle4->Name = L"JointAngle4";
			this->JointAngle4->Size = System::Drawing::Size(96, 25);
			this->JointAngle4->TabIndex = 18;
			// 
			// SpaceA
			// 
			this->SpaceA->Location = System::Drawing::Point(47, 526);
			this->SpaceA->Name = L"SpaceA";
			this->SpaceA->Size = System::Drawing::Size(96, 25);
			this->SpaceA->TabIndex = 22;
			// 
			// SpaceZ
			// 
			this->SpaceZ->Location = System::Drawing::Point(47, 485);
			this->SpaceZ->Name = L"SpaceZ";
			this->SpaceZ->Size = System::Drawing::Size(96, 25);
			this->SpaceZ->TabIndex = 21;
			// 
			// SpaceY
			// 
			this->SpaceY->Location = System::Drawing::Point(47, 444);
			this->SpaceY->Name = L"SpaceY";
			this->SpaceY->Size = System::Drawing::Size(96, 25);
			this->SpaceY->TabIndex = 20;
			// 
			// SpaceX
			// 
			this->SpaceX->Location = System::Drawing::Point(47, 403);
			this->SpaceX->Name = L"SpaceX";
			this->SpaceX->Size = System::Drawing::Size(96, 25);
			this->SpaceX->TabIndex = 19;
			// 
			// J1
			// 
			this->J1->AutoSize = true;
			this->J1->Location = System::Drawing::Point(19, 111);
			this->J1->Name = L"J1";
			this->J1->Size = System::Drawing::Size(23, 15);
			this->J1->TabIndex = 23;
			this->J1->Text = L"J1";
			// 
			// J2
			// 
			this->J2->AutoSize = true;
			this->J2->Location = System::Drawing::Point(19, 154);
			this->J2->Name = L"J2";
			this->J2->Size = System::Drawing::Size(23, 15);
			this->J2->TabIndex = 24;
			this->J2->Text = L"J2";
			// 
			// J3
			// 
			this->J3->AutoSize = true;
			this->J3->Location = System::Drawing::Point(19, 194);
			this->J3->Name = L"J3";
			this->J3->Size = System::Drawing::Size(23, 15);
			this->J3->TabIndex = 25;
			this->J3->Text = L"J3";
			// 
			// J4
			// 
			this->J4->AutoSize = true;
			this->J4->Location = System::Drawing::Point(19, 237);
			this->J4->Name = L"J4";
			this->J4->Size = System::Drawing::Size(23, 15);
			this->J4->TabIndex = 26;
			this->J4->Text = L"J4";
			// 
			// X
			// 
			this->X->AutoSize = true;
			this->X->Location = System::Drawing::Point(26, 409);
			this->X->Name = L"X";
			this->X->Size = System::Drawing::Size(15, 15);
			this->X->TabIndex = 27;
			this->X->Text = L"X";
			// 
			// Y
			// 
			this->Y->AutoSize = true;
			this->Y->Location = System::Drawing::Point(26, 450);
			this->Y->Name = L"Y";
			this->Y->Size = System::Drawing::Size(15, 15);
			this->Y->TabIndex = 28;
			this->Y->Text = L"Y";
			// 
			// Z
			// 
			this->Z->AutoSize = true;
			this->Z->Location = System::Drawing::Point(26, 488);
			this->Z->Name = L"Z";
			this->Z->Size = System::Drawing::Size(15, 15);
			this->Z->TabIndex = 29;
			this->Z->Text = L"Z";
			// 
			// A
			// 
			this->A->AutoSize = true;
			this->A->Location = System::Drawing::Point(26, 531);
			this->A->Name = L"A";
			this->A->Size = System::Drawing::Size(15, 15);
			this->A->TabIndex = 30;
			this->A->Text = L"A";
			// 
			// Coordinate
			// 
			this->Coordinate->FormattingEnabled = true;
			this->Coordinate->Items->AddRange(gcnew cli::array< System::Object^  >(2) {L"�ؽ�����", L"�ռ�����"});
			this->Coordinate->Location = System::Drawing::Point(570, 62);
			this->Coordinate->Name = L"Coordinate";
			this->Coordinate->Size = System::Drawing::Size(148, 23);
			this->Coordinate->TabIndex = 31;
			this->Coordinate->SelectedIndexChanged += gcnew System::EventHandler(this, &RobotForm::Coordinate_SelectedIndexChanged);
			// 
			// MoveToPos1
			// 
			this->MoveToPos1->Location = System::Drawing::Point(824, 19);
			this->MoveToPos1->Name = L"MoveToPos1";
			this->MoveToPos1->Size = System::Drawing::Size(66, 25);
			this->MoveToPos1->TabIndex = 32;
			this->MoveToPos1->Text = L"0.0";
			// 
			// MoveToPos2
			// 
			this->MoveToPos2->Location = System::Drawing::Point(917, 19);
			this->MoveToPos2->Name = L"MoveToPos2";
			this->MoveToPos2->Size = System::Drawing::Size(66, 25);
			this->MoveToPos2->TabIndex = 33;
			this->MoveToPos2->Text = L"-90.0";
			// 
			// MoveToPos3
			// 
			this->MoveToPos3->Location = System::Drawing::Point(1003, 19);
			this->MoveToPos3->Name = L"MoveToPos3";
			this->MoveToPos3->Size = System::Drawing::Size(66, 25);
			this->MoveToPos3->TabIndex = 34;
			this->MoveToPos3->Text = L"180.0";
			// 
			// MoveToPos4
			// 
			this->MoveToPos4->Location = System::Drawing::Point(1099, 19);
			this->MoveToPos4->Name = L"MoveToPos4";
			this->MoveToPos4->Size = System::Drawing::Size(66, 25);
			this->MoveToPos4->TabIndex = 35;
			this->MoveToPos4->Text = L"0.0";
			// 
			// MessageBox
			// 
			this->MessageBox->Location = System::Drawing::Point(754, 58);
			this->MessageBox->Name = L"MessageBox";
			this->MessageBox->Size = System::Drawing::Size(411, 25);
			this->MessageBox->TabIndex = 36;
			// 
			// PosLabel
			// 
			this->PosLabel->AutoSize = true;
			this->PosLabel->Location = System::Drawing::Point(751, 24);
			this->PosLabel->Name = L"PosLabel";
			this->PosLabel->Size = System::Drawing::Size(67, 15);
			this->PosLabel->TabIndex = 37;
			this->PosLabel->Text = L"�ؽ�λ��";
			// 
			// HomePosButton
			// 
			this->HomePosButton->Location = System::Drawing::Point(1310, 23);
			this->HomePosButton->Name = L"HomePosButton";
			this->HomePosButton->Size = System::Drawing::Size(85, 26);
			this->HomePosButton->TabIndex = 38;
			this->HomePosButton->Text = L"���λ��";
			this->HomePosButton->UseVisualStyleBackColor = true;
			this->HomePosButton->Click += gcnew System::EventHandler(this, &RobotForm::HomePosButton_Click);
			// 
			// TestPosButton
			// 
			this->TestPosButton->Location = System::Drawing::Point(1310, 61);
			this->TestPosButton->Name = L"TestPosButton";
			this->TestPosButton->Size = System::Drawing::Size(85, 26);
			this->TestPosButton->TabIndex = 39;
			this->TestPosButton->Text = L"����λ��";
			this->TestPosButton->UseVisualStyleBackColor = true;
			this->TestPosButton->Click += gcnew System::EventHandler(this, &RobotForm::TestPosButton_Click);
			// 
			// RatioBox
			// 
			this->RatioBox->Location = System::Drawing::Point(570, 20);
			this->RatioBox->Name = L"RatioBox";
			this->RatioBox->Size = System::Drawing::Size(47, 25);
			this->RatioBox->TabIndex = 40;
			this->RatioBox->Text = L"30";
			this->RatioBox->TextChanged += gcnew System::EventHandler(this, &RobotForm::RatioBox_TextChanged);
			// 
			// RatioLabel
			// 
			this->RatioLabel->AutoSize = true;
			this->RatioLabel->Location = System::Drawing::Point(619, 24);
			this->RatioLabel->Name = L"RatioLabel";
			this->RatioLabel->Size = System::Drawing::Size(37, 15);
			this->RatioLabel->TabIndex = 41;
			this->RatioLabel->Text = L"����";
			// 
			// RobotForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 15);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1475, 754);
			this->Controls->Add(this->RatioLabel);
			this->Controls->Add(this->RatioBox);
			this->Controls->Add(this->TestPosButton);
			this->Controls->Add(this->HomePosButton);
			this->Controls->Add(this->PosLabel);
			this->Controls->Add(this->MessageBox);
			this->Controls->Add(this->MoveToPos4);
			this->Controls->Add(this->MoveToPos3);
			this->Controls->Add(this->MoveToPos2);
			this->Controls->Add(this->MoveToPos1);
			this->Controls->Add(this->Coordinate);
			this->Controls->Add(this->A);
			this->Controls->Add(this->Z);
			this->Controls->Add(this->Y);
			this->Controls->Add(this->X);
			this->Controls->Add(this->J4);
			this->Controls->Add(this->J3);
			this->Controls->Add(this->J2);
			this->Controls->Add(this->J1);
			this->Controls->Add(this->SpaceA);
			this->Controls->Add(this->SpaceZ);
			this->Controls->Add(this->SpaceY);
			this->Controls->Add(this->SpaceX);
			this->Controls->Add(this->JointAngle4);
			this->Controls->Add(this->JointAngle3);
			this->Controls->Add(this->JointAngle2);
			this->Controls->Add(this->JointAngle1);
			this->Controls->Add(this->JointAccChart);
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
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->JointAccChart))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}

#pragma endregion
	private: System::Void clearChart()
			 {
				 this->mCurrentTime = 0;
				 for each (Series^ series in this->JointChart->Series)
					 series->Points->Clear();
				 for each (Series^ series in this->JointVelChart->Series)
					 series->Points->Clear();
				 for each (Series^ series in this->JointAccChart->Series)
					 series->Points->Clear();
				 for each (Series^ series in this->SpaceChart->Series)
					 series->Points->Clear();
			 }

	private: System::Void setChart(int axixnum, int count, double *jointpos, double *jointvel, double *jointacc, double *spacepos, bool allshow)
			{
				this->JointAngle1->Text = jointpos[0].ToString("0.#######"); this->JointAngle2->Text = jointpos[1].ToString("0.#######");
				this->JointAngle3->Text = jointpos[2].ToString("0.#######"); this->JointAngle4->Text = jointpos[3].ToString("0.#######");
				this->SpaceX->Text = spacepos[0].ToString("0.#######"); this->SpaceY->Text = spacepos[1].ToString("0.#######");
				this->SpaceZ->Text = spacepos[2].ToString("0.#######"); this->SpaceA->Text = spacepos[3].ToString("0.#######");
				
				if(!allshow)
				{
					this->JointChart->Series[axixnum]->Points->AddXY(count, jointpos[axixnum]);			// ��ӹؽ�1���ݵ�
					this->JointVelChart->Series[axixnum]->Points->AddXY(count, jointvel[axixnum]);	// ��ӹؽ��ٶ�1���ݵ�
					this->JointAccChart->Series[axixnum]->Points->AddXY(count, jointacc[axixnum]);// ��ӹؽ��ٶ�1���ݵ�
					this->SpaceChart->Series[axixnum]->Points->AddXY(count, spacepos[axixnum]);		// ��ӿռ�X�����ݵ�
				}
				else
				{
					this->JointChart->Series["�ؽ�1"]->Points->AddXY(count, jointpos[0]);			// ��ӹؽ�1���ݵ�
					this->JointChart->Series["�ؽ�2"]->Points->AddXY(count, jointpos[1]);			// ��ӹؽ�2���ݵ�
					this->JointChart->Series["�ؽ�3"]->Points->AddXY(count, jointpos[2]);			// ��ӹؽ�3���ݵ�
					this->JointChart->Series["�ؽ�4"]->Points->AddXY(count, jointpos[3]);			// ��ӹؽ�4���ݵ�

					this->JointVelChart->Series["�ؽ��ٶ�1"]->Points->AddXY(count, jointvel[0]);	// ��ӹؽ��ٶ�1���ݵ�
					this->JointVelChart->Series["�ؽ��ٶ�2"]->Points->AddXY(count, jointvel[1]);	// ��ӹؽ��ٶ�2���ݵ�
					this->JointVelChart->Series["�ؽ��ٶ�3"]->Points->AddXY(count, jointvel[2]);	// ��ӹؽ��ٶ�3���ݵ�
					this->JointVelChart->Series["�ؽ��ٶ�4"]->Points->AddXY(count, jointvel[3]);	// ��ӹؽ��ٶ�4���ݵ�

					this->JointAccChart->Series["�ؽڼ��ٶ�1"]->Points->AddXY(count, jointacc[0]);	// ��ӹؽڼ��ٶ�1���ݵ�
					this->JointAccChart->Series["�ؽڼ��ٶ�2"]->Points->AddXY(count, jointacc[1]);	// ��ӹؽڼ��ٶ�2���ݵ�
					this->JointAccChart->Series["�ؽڼ��ٶ�3"]->Points->AddXY(count, jointacc[2]);	// ��ӹؽڼ��ٶ�3���ݵ�
					this->JointAccChart->Series["�ؽڼ��ٶ�4"]->Points->AddXY(count, jointacc[3]);	// ��ӹؽڼ��ٶ�4���ݵ�

					this->SpaceChart->Series["�ռ�X��"]->Points->AddXY(count, spacepos[0]);			// ��ӿռ�X�����ݵ�
					this->SpaceChart->Series["�ռ�Y��"]->Points->AddXY(count, spacepos[1]);			// ��ӿռ�Y�����ݵ�
					this->SpaceChart->Series["�ռ�Z��"]->Points->AddXY(count, spacepos[2]);			// ��ӿռ�Z�����ݵ�
				}
			 }

	private: System::Void planJoint(int axisnum, bool dir, double *nowpos)
			 {
				 this->mJointMoveNum = axisnum;
				 this->motioncombine->planManual(axisnum, dir, this->mIsJointMove, nowpos);
				 this->JointMoveTimer->Start();
			 }

	private: System::Void stayJoint(int axisnum)
			 {
				 this->mCurrentTime = this->mCurrentTime + 1;
				 this->motioncombine->execManualIntMove(this->mJointPos, this->mJointVel, this->mJointAcc, this->mSpacePos);
				 this->setChart(axisnum, this->mCurrentTime, this->mJointPos, this->mJointVel, this->mJointAcc, this->mSpacePos, this->mAllAxisShow);
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
					 status = this->motioncombine->execManualIntMove(this->mJointPos, this->mJointVel, this->mJointAcc, this->mSpacePos);
					 this->setChart(axisnum, this->mCurrentTime, this->mJointPos, this->mJointVel, this->mJointAcc, this->mSpacePos, this->mAllAxisShow);
					 printf("stopJoint: %f %f %f %f %f %f \n",mJointPos[0],mJointPos[1],mJointPos[2],mJointPos[3],mJointPos[4],mJointPos[5]);
				 }
			 }

	private: System::Void RobotForm_Load(System::Object^  sender, System::EventArgs^  e) {}

	private: System::Void MoveToStart_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				double dMoveToPos[MaxAxisNum] = {0.0};
				double num = 0.0;
				hsc3::algo::HS_MStatus status = hsc3::algo::M_UnInit;

				try
				{
					dMoveToPos[0] = Double::Parse(this->MoveToPos1->Text); dMoveToPos[1] = Double::Parse(this->MoveToPos2->Text);
					dMoveToPos[2] = Double::Parse(this->MoveToPos3->Text); dMoveToPos[3] = Double::Parse(this->MoveToPos4->Text);
					printf("MoveToStart_Click--Pos %f %f %f %f \n", dMoveToPos[0], dMoveToPos[1], dMoveToPos[2], dMoveToPos[3]);
					if(this->mIsJointMove)
					{
						motioncombine->planJoint(dMoveToPos);
						while(status != hsc3::algo::M_Done)
						{
							status = this->motioncombine->execJointIntMove(this->mJointPos, this->mJointVel, this->mJointAcc,this->mSpacePos);
							this->setChart(0, this->mCurrentTime, this->mJointPos, this->mJointVel, this->mJointAcc, this->mSpacePos, true);
							//printf("status=%d, outPos: %f %f %f %f %f %f \n", status, dJointPos[0],dJointPos[1],dJointPos[2],dJointPos[3],dJointPos[4],dJointPos[5]);
							printf("planJoint--status=%d, outVelPos: %f %f %f %f %f %f \n", status,mJointAcc[0],mJointAcc[1],mJointAcc[2],mJointAcc[3],mJointAcc[4],mJointAcc[5]);
							this->mCurrentTime = this->mCurrentTime + 1;
						}
					}
					else
					{
						motioncombine->planSpace();
						while((status != hsc3::algo::M_Done) && (status != hsc3::algo::M_Error))
						{
							status = this->motioncombine->execSpaceIntMove(this->mJointPos, this->mJointVel, this->mJointAcc,this->mSpacePos);
							this->setChart(0, this->mCurrentTime, this->mJointPos, this->mJointVel, this->mJointAcc, this->mSpacePos, true);
							//printf("planSpace--status=%d, outJPos: %f %f %f %f %f %f \n", status,mJointPos[0],mJointPos[1],mJointPos[2],mJointPos[3],mJointPos[4],mJointPos[5]);
							//printf("planSpace--status=%d, outSPos: %f %f %f %f %f %f \n", status,dSpacePos[0],dSpacePos[1],dSpacePos[2],dSpacePos[3],dSpacePos[4],dSpacePos[5]);
							//printf("planSpace--status=%d, outVelPos: %f %f %f %f %f %f \n", status,mJointVel[0],mJointVel[1],mJointVel[2],mJointVel[3],mJointVel[4],mJointVel[5]);
							printf("planSpace--status=%d, outAccPos: %f %f %f %f %f %f \n", status,mJointAcc[0],mJointAcc[1],mJointAcc[2],mJointAcc[3],mJointAcc[4],mJointAcc[5]);
							this->mCurrentTime = this->mCurrentTime + 1;
						}
					}
				}
				catch (FormatException^ ex)
				{
					this->MessageBox->Text = "��λ����������󣡣���";
					printf("MoveToStart_Click Error\n");
				}
			 }

	private: System::Void JointMoveTimer_Tick(System::Object^  sender, System::EventArgs^  e) {
				this->stayJoint(this->mJointMoveNum);
			 }

	private: System::Void J1PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(0, true, this->mJointPos);
			 }

	private: System::Void J1PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(0);
			 }

	private: System::Void J1NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(0, false, this->mJointPos);
			 }

	private: System::Void J1NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(0);
			 }

	private: System::Void J2PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(1, true, this->mJointPos);
			 }

	private: System::Void J2PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(1);
			 }

	private: System::Void J2NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(1, false, this->mJointPos);
			 }

	private: System::Void J2NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(1);
			 }

	private: System::Void J3PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(2, true, this->mJointPos);
			 }

	private: System::Void J3PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(2);
			 }

	private: System::Void J3NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(2, false, this->mJointPos);
			 }

	private: System::Void J3NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(2);
			 }

	private: System::Void J4PosButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(3, true, this->mJointPos);
			 }

	private: System::Void J4PosButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(3);
			 }

	private: System::Void J4NegButton_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->planJoint(3, false, this->mJointPos);
			 }

	private: System::Void J4NegButton_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				this->stopJoint(3);
			 }

	private: System::Void ClearChart_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				 this->MessageBox->Text = " ";
				this->clearChart();
			 }

	private: System::Void AllAxisShow_CheckedChanged(System::Object^  sender, System::EventArgs^  e) 
			 {
				this->mAllAxisShow = AllAxisShow->Checked;
				printf("AllAxisShow_CheckedChanged %d \n", mAllAxisShow);
			 }

	private: System::Void Coordinate_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) 
			 {
				 this->clearChart();
				 if(this->Coordinate->SelectedIndex == 0)
				 {
					 this->mIsJointMove = true;
					 this->PosLabel->Text = "�ؽ�λ��";
					 this->J1PosButton->Text = "J1+"; this->J1NegButton->Text = "J1-";
					 this->J2PosButton->Text = "J2+"; this->J2NegButton->Text = "J2-";
					 this->J3PosButton->Text = "J3+"; this->J3NegButton->Text = "J3-";
					 this->J4PosButton->Text = "J4+"; this->J4NegButton->Text = "J4-";
				 }
				 else
				 {
					 this->mIsJointMove = false;
					 this->PosLabel->Text = "�ռ�λ��";
					 this->J1PosButton->Text = "X+"; this->J1NegButton->Text = "X-";
					 this->J2PosButton->Text = "Y+"; this->J2NegButton->Text = "Y-";
					 this->J3PosButton->Text = "Z+"; this->J3NegButton->Text = "Z-";
					 this->J4PosButton->Text = "A+"; this->J4NegButton->Text = "A-";
				 }
				 printf("Coordinate_SelectedIndexChanged--%s, index=%d \n", this->Coordinate->SelectedItem->ToString(), this->Coordinate->SelectedIndex);
			 }

	private: System::Void HomePosButton_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				 this->MoveToPos1->Text = "0.0";   this->MoveToPos2->Text = "-90.0";
				 this->MoveToPos3->Text = "180.0"; this->MoveToPos4->Text = "0.0";
			 }

	private: System::Void TestPosButton_Click(System::Object^  sender, System::EventArgs^  e) 
			 {
				 this->MoveToPos1->Text = "60.0";   this->MoveToPos2->Text = "-20.0";
				 this->MoveToPos3->Text = "100.0"; this->MoveToPos4->Text = "90.0";
			 }

	private: System::Void RatioBox_TextChanged(System::Object^  sender, System::EventArgs^  e) 
			 {
				 double dRatio = 30;
				 try
				 {
					 dRatio = Double::Parse(this->RatioBox->Text);
					 this->motioncombine->setRatio(dRatio);
				 }
				 catch (FormatException^ ex)
				 {
					 this->MessageBox->Text = "��������ʧ�ܣ�����";
					 printf("RatioBox_TextChanged Error\n");
				 }
			 }
};

}

