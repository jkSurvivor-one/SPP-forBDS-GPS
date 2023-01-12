#include"ALLHead.h"
#include <iostream>
#include <fstream>
#include <iomanip> 
using namespace std;
int main()
{

	/*文件路径*/
	string Nfilename = "D:\\桌面\\obs_binex.21n";
	string Ofilename = "D:\\桌面\\obs_binex.21o";
	int PosSys;
	cout << "请选择使用的坐标系统：1为BDS  2为GPS  3为BDS/GPS" << endl;
	cin >> PosSys;
	cout << PosSys << endl;
	ofstream out;//接收机坐标输出
	ofstream out1;//中间变量输出
	if (PosSys==1)
	{
		out.open("D:\\桌面\\STABDSresult.txt", ios::out);
		out1.open("D:\\桌面\\BDS中间变量.txt", ios::out);
	}
	else if(PosSys ==2)
	{
		out.open("D:\\桌面\\STAGPSresult.txt", ios::out);
		out1.open("D:\\桌面\\GPS中间变量.txt", ios::out);
	}
	else
	{
		out.open("D:\\桌面\\STAresult.txt", ios::out);
		out1.open("D:\\桌面\\中间变量.txt", ios::out);
	}
	if (!out.is_open()) {
		cerr << "Failed to write receiver coordinates to file." << endl;//cerr：输出到标准错误的ostream对象，常用于程序错误信息；
		exit(EXIT_FAILURE);
	}
	if (!out1.is_open()) {
		cerr << "Failed to write Intermediate variables to file." << endl;
		exit(EXIT_FAILURE);
	}
	int weight;
	cout << "请选择使用的定权方式：1为等权  2为高度角定权" << endl;
	cin >> weight;
	cout << weight << endl;
	//
	/*类对象*/
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

	spp.SinglePP(ofile, vecAllNREC, StorSatPosEveEpo, StorRecPos,out1,PosSys, weight);//结果在StorRecPos中
	
	out << "APPORX POS：" << endl;
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