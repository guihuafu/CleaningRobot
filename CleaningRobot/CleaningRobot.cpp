// CleaningRobot.cpp: ����Ŀ�ļ���

#include "stdafx.h"
#include "RobotForm.h"

using namespace CleaningRobot;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
	// �ڴ����κοؼ�֮ǰ���� Windows XP ���ӻ�Ч��
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false); 

	// ���������ڲ�������
	Application::Run(gcnew RobotForm());
	return 0;
}
