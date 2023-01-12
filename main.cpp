#include"ALLHead.h"
#include <iostream>
#include <fstream>
#include <iomanip> 
using namespace std;
int main()
{

	/*�ļ�·��*/
	string Nfilename = "D:\\����\\obs_binex.21n";
	string Ofilename = "D:\\����\\obs_binex.21o";
	int PosSys;
	cout << "��ѡ��ʹ�õ�����ϵͳ��1ΪBDS  2ΪGPS  3ΪBDS/GPS" << endl;
	cin >> PosSys;
	cout << PosSys << endl;
	ofstream out;//���ջ��������
	ofstream out1;//�м�������
	if (PosSys==1)
	{
		out.open("D:\\����\\STABDSresult.txt", ios::out);
		out1.open("D:\\����\\BDS�м����.txt", ios::out);
	}
	else if(PosSys ==2)
	{
		out.open("D:\\����\\STAGPSresult.txt", ios::out);
		out1.open("D:\\����\\GPS�м����.txt", ios::out);
	}
	else
	{
		out.open("D:\\����\\STAresult.txt", ios::out);
		out1.open("D:\\����\\�м����.txt", ios::out);
	}
	if (!out.is_open()) {
		cerr << "Failed to write receiver coordinates to file." << endl;//cerr���������׼�����ostream���󣬳����ڳ��������Ϣ��
		exit(EXIT_FAILURE);
	}
	if (!out1.is_open()) {
		cerr << "Failed to write Intermediate variables to file." << endl;
		exit(EXIT_FAILURE);
	}
	int weight;
	cout << "��ѡ��ʹ�õĶ�Ȩ��ʽ��1Ϊ��Ȩ  2Ϊ�߶ȽǶ�Ȩ" << endl;
	cin >> weight;
	cout << weight << endl;
	//
	/*�����*/
	CFileRead fileread;
	SPP spp;

	ObsFile ofile;
	Navhead navhdr;
	vector<ObsrecordEph> vecAllOREC;
	ObsHead ohdr;
	vector<NavRecord> vecAllNREC;
	vector<ASatPosInOneEph> StorSatPosEveEpo;
	vector<Descartes> StorRecPos;
	out1 << "iepoch" << "\t" << "System" << "\t" << "PRN" << "\t" << "Time" << "\t" << "Satpos" << "\t" << "ELve" << "\t" << "am" << "\t" << "psr" << "\t" << "Tropcor" <<"SatClk " << endl;
	fileread.ReadNavFile(Nfilename, navhdr, vecAllNREC);
	fileread.ReadObsFile(Ofilename, ofile);

	spp.SinglePP(ofile, vecAllNREC, StorSatPosEveEpo, StorRecPos,out1,PosSys, weight);//�����StorRecPos��
	
	out << "APPORX POS��" << endl;
	out << setw(22) << std::setprecision(6)
		<< std::setiosflags(ios::showpoint | ios::fixed) << ofile.ohdr.apporx_position[0];
	out << setw(19) << std::setprecision(6)
		<< std::setiosflags(ios::showpoint | ios::fixed) << ofile.ohdr.apporx_position[1];
	out << setw(19) << std::setprecision(6)
		<< std::setiosflags(ios::showpoint | ios::fixed) << ofile.ohdr.apporx_position[2] << endl << endl;

	out.width(10);
	out << "Num";
	out.width(25);
	out << "X";
	out.width(25);
	out << "Y";
	out.width(19);
	out << "Z" << endl;

	for (unsigned int i = 0; i != StorRecPos.size(); i++) {
		out << std::setw(6) << std::left << i;
		out << setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << StorRecPos[i].X;
		out << setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << StorRecPos[i].Y;
		out << setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << StorRecPos[i].Z << endl;
	}
	out.close();
	out1.close();
	return 0;
}