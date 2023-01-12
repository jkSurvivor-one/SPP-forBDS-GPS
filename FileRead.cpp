#include <cstdio>
#include"ALLHead.h"

#include <string>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <fstream>//文件流头
#include <cstdlib>

//定义类CFileRead中的公共函数
using namespace std;


CFileRead::CFileRead()
{
}//C++11之后的一个规范检查。 忽略其实问题不大。 

CFileRead::~CFileRead()
{
}

/*******************
 * Name		: ReadNavFile
 * Function	: 读取导航电文
 * argument : string nfilename 导航电文文件名/文件指针
 *			  Navhead& Navhdr  导航电文头文件记录
 *			  vector<NavRecord>& vecNrec  导航电文文件记录
 *
 * return   : none
 *
 *principle ：从文件指针读取全部的头文件之后，单个导航电文的读取（8行）之后将对应的文件存入对应的容器中
 *remark    ：分为BDS  GPS
 *******************/
void CFileRead::ReadNavFile(string nfilename, Navhead& Navhdr, vector<NavRecord>& vecNrec)
{
	
	ifstream file;//定义文件流

	std::cout << "Start reading navigation file:[" << nfilename << " ] " << endl;
	std::cout << "In the process of reading......" << endl;

	file.open(nfilename, ios::in | ios::out);//打开n文件

	if (!file)  std::cout << "n文件打开错误!" << endl;

	/*---------------读取导航电文文件头---------------*/
	while (!file.eof())
	{
		string str;
		getline(file, str);//将
		//file.getline(str, MAXCHAR, '\n');//str只能是数组 不能是字符指针
		/*********头文件读取********/
		//GPS
		if ("GPSA" == str.substr(0, 4) && "IONOSPHERIC CORR" == str.substr(60, 16))
		{
			Navhdr.ionA1 = atof((str.substr(5, 12)).c_str());
			Navhdr.ionA2 = atof((str.substr(17, 12)).c_str());
			Navhdr.ionA3 = atof((str.substr(29, 12)).c_str());
			Navhdr.ionA4 = atof((str.substr(41, 12)).c_str());
		}
		else if ("GPSB" == str.substr(0, 4) && "IONOSPHERIC CORR" == str.substr(60, 16))
		{
			Navhdr.ionB1 = atof((str.substr(5, 12)).c_str());
			Navhdr.ionB2 = atof((str.substr(17, 12)).c_str());
			Navhdr.ionB3 = atof((str.substr(29, 12)).c_str());
			Navhdr.ionB4 = atof((str.substr(41, 12)).c_str());
		}
		//BDS
		if ("BDSA" == str.substr(0, 4) && "IONOSPHERIC CORR" == str.substr(60, 16))
		{
			Navhdr.bdionA1 = atof((str.substr(5, 12)).c_str());
			Navhdr.bdionA2 = atof((str.substr(17, 12)).c_str());
			Navhdr.bdionA3 = atof((str.substr(29, 12)).c_str());
			Navhdr.bdionA4 = atof((str.substr(41, 12)).c_str());
		}
		else if ("BDSB" == str.substr(0, 4) && "IONOSPHERIC CORR" == str.substr(60, 16))
		{
			Navhdr.bdionB1 = atof((str.substr(5, 12)).c_str());
			Navhdr.bdionB2 = atof((str.substr(17, 12)).c_str());
			Navhdr.bdionB3 = atof((str.substr(29, 12)).c_str());
			Navhdr.bdionB4 = atof((str.substr(41, 12)).c_str());
		}
		if ("END OF HEADER"== str.substr( 60, 13))break;
	}

	/*---------------读取导航电文数据记录---------------*/

	while (!file.eof())
	{
		NavRecord nrec = { };//暂存信息
		string  oneline;
		int prn;
		getline(file,oneline);//转型是在传地址进去的时候，会发生转型

		/*读取BD导航数据*/
		if ("C"== oneline.substr( 0, 1)) {
			prn = atoi((oneline.substr(1, 2)).c_str());//字符串转换函数atoi
			if (prn == 31 || prn == 56 || prn == 57 || prn == 58)//剔除在轨试验卫星
				continue;
			for (int i = 1; i <= 8; i++) {
				switch (i) {
				case 1: {
					nrec.sprn.SYS = 1;
					nrec.sprn.PRN = atoi((oneline.substr( 1, 2)).c_str());//atoi()函数：把字符串转换成整型数
					nrec.TOC.year = atoi((oneline.substr(4, 4)).c_str());
					nrec.TOC.month = atoi((oneline.substr(9, 2)).c_str());
					nrec.TOC.day = atoi((oneline.substr(12, 2)).c_str());
					nrec.TOC.hour = atoi((oneline.substr(15, 2)).c_str());
					nrec.TOC.minute = atoi((oneline.substr(18, 2)).c_str());
					nrec.TOC.second = atoi((oneline.substr(21, 2)).c_str());
					TimeSystem timesys;
					GPSTIME gpst;
					timesys.UTC2GPST(nrec.TOC, gpst);

					nrec.NGPST.wn = gpst.wn;
					nrec.NGPST.sow = gpst.sow;

					nrec.a0 = atof((oneline.substr(24, 14)).c_str())*pow(10,atoi((oneline.substr(39, 3)).c_str()));//atof()函数：把字符串转换成浮点数，所使用的头文件为<stdlib.h>
					nrec.a1 = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.a2 = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}
				case 2: {
					getline(file, oneline);
					nrec.IODE = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Crs = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Deltan = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.M0 = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}
				case 3: {
					getline(file, oneline);
					nrec.Cuc = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.e = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Cus = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.sqrtA = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}
				case 4: {
					getline(file, oneline);
					nrec.TOE = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Cic = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Omeca = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.Cis = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}
				case 5: {
					getline(file, oneline);
					nrec.i0 = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Crc = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Omega = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.OmegaDot = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}
				case 6: {
					getline(file, oneline);
					nrec.IDOT = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					//nrec.spare1_BD1 = atof(substr(oneline, 23, 41));
					nrec.WEEK = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					//nrec.spare2_BD1 = atof(substr(oneline, 61, 79));
					break;
				}
				case 7: {
					getline(file, oneline);
					nrec.integrity = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.HS = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.TGD = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.ISC1 = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}
				case 8: {
					getline(file, oneline);
					nrec.TransTimeOfMaq = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.IODC = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					//nrec.spare3_BD1 = atof(substr(oneline, 42, 60)));
					//nrec.spare4_BD1 = atof(substr(oneline, 61, 79));
					break;
				}
				}//switch
			}//for
			vecNrec.push_back(nrec);
		}

		else if ("G" == oneline.substr(0, 1)) {
			for (int i = 1; i <= 8; i++) {
				switch (i) {
				case 1://获取第一行数据

				{
					nrec.sprn.SYS = 2;
					nrec.sprn.PRN = atoi((oneline.substr(1, 2)).c_str());//atoi()函数：把字符串转换成整型数
					nrec.TOC.year = atoi((oneline.substr(4, 4)).c_str());
					nrec.TOC.month = atoi((oneline.substr(9, 2)).c_str());
					nrec.TOC.day = atoi((oneline.substr(12, 2)).c_str());
					nrec.TOC.hour = atoi((oneline.substr(15, 2)).c_str());
					nrec.TOC.minute = atoi((oneline.substr(18, 2)).c_str());
					nrec.TOC.second = atoi((oneline.substr(21, 2)).c_str());
					TimeSystem timesys;
					GPSTIME gpst;
					timesys.UTC2GPST(nrec.TOC, gpst);
					nrec.NGPST.wn = gpst.wn;
					nrec.NGPST.sow = gpst.sow;

					nrec.a0 = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));//atof()函数：把字符串转换成浮点数，所使用的头文件为<stdlib.h>
					nrec.a1 = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.a2 = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 2://广播轨道1
				{
					getline(file, oneline);
					nrec.IODE = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Crs = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Deltan = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.M0 = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 3://广播轨道2
				{
					getline(file, oneline);
					nrec.Cuc = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.e = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Cus = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.sqrtA = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 4://广播轨道3
				{
					getline(file, oneline);
					nrec.TOE = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Cic = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Omeca = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.Cis = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 5://广播轨道4
				{
					getline(file, oneline);
					nrec.i0 = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Crc = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Omega = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.OmegaDot = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 6://广播轨道5
				{
					getline(file, oneline);
					nrec.IDOT = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					//nrec.CodesOnL2Channel = atof(substr(oneline, 23, 41));
					nrec.WEEK = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					//nrec.L2PDataFlag = atof(substr(oneline, 61, 79));
					break;
				}

				case 7://广播轨道6
				{
					getline(file, oneline);
					//nrec.SatAccuracy = atof(substr(oneline, 4, 22));
					//nrec.SatHealth = atof(substr(oneline, 23, 41));
					nrec.TGD = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					//nrec.IODC = atof(substr(oneline, 61, 79));
					break;
				}

				case 8://广播轨道7
				{
					getline(file, oneline);
					nrec.TransTimeOfMaq = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.FitInterval = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					break;
				}
				}//switch
			}//for
			vecNrec.push_back(nrec);//函数将一个新的元素加到vector的最后面，位置为当前最后一个元素的下一个元素
			
		}//if
	}//while

	/*验证导航信息是否成功存储*/
	//for (int i = 0; i < vecNrec.size(); i++){
	//	cout << vecNrec[i].sprn.SYS;
	//	cout << vecNrec[i].sprn.PRN << "     ";
	//	cout << vecNrec[i].Cuc<< endl;
	//}

	std::cout << "End of reading" << endl;
	file.close();
}

/*******************
 * Name		:ReadObsFile
 * Function	:读取全部的观测值文件
 * argument :string ofilename   文件名（路径）
 *			 ObsFile& ofile      OBS文件的结构体（文件的存储位置）
 *
 * return   :none
 * 原理     :从文件打开开始，先读取文件头，存入ObsFile中的头文件部分
 *			 读取每一个历元的观测值存入容器ofile.vecOrec.push_back(orec)
 *			在头文件部分还要读取对应的观测值类型，以便于在后面观测值类型时能够对应起来
 *******************/
void CFileRead::ReadObsFile(string ofilename, ObsFile& ofile)
{
	//clock_t start, finish;
	//start = clock();//设置开始时间

	ifstream file;//定义文件流

	std::cout << "Start reading observation file :[" << ofilename << " ] " << endl;

	std::cout << "In the process of reading....." << endl;

	file.open(ofilename, ios_base::in);//打开o文件
	if (!file.is_open())
	{
		cerr << "打开文件失败" << endl;
		exit(EXIT_FAILURE);
	}

	/*---------------读取观测值头文件---------------*/
	string buff;//创建string对象 用于存储文件读取的字符串   C中没有string只能通过char buff[] 建立字符数组
	//const string str = { };//建立string 用于临时存储  从buff字符串中截取的字符

	while (getline(file, buff))//getline(<字符数组chs>，<读取字符的个数n>，<终止符>)
		/*stream& getline ( istream &is , string &str , char delim );
		其中，istream& is 表示一个输入流，譬如cin；
		string& str表示把从输入流读入的字符串存放在这个字符串中（可以自己随便命名，str什么的都可以）；
		char delim表示遇到这个字符停止读入，在不设置的情况下系统默认该字符为'\n'，也就是回车换行符（遇到回车停止读入）。*/
	{
		if (buff.substr(60, 19) == "APPROX POSITION XYZ")//读取测站概略位置
		{
			//s.substr(pos, len)返回值： string，包含s中从pos开始的len个字符的拷贝（pos的默认值是0，len的默认值是s.size()
			ofile.ohdr.apporx_position[0] = atof((buff.substr(0, 14)).c_str());//记住不要直接操作c_str()返回的指针，在操作之前先通过strcpy()函数转化处理。如c = s.c_str();
			ofile.ohdr.apporx_position[1] = atof((buff.substr(14, 14)).c_str());//c_str() 以 char* 形式传回 string 内含字符串，如果一个函数要求char*参数，可以使用c_str()方法：
			ofile.ohdr.apporx_position[2] = atof(buff.substr(28, 14).c_str());
		}
		else if (buff.substr(60, 19) == "SYS / # / OBS TYPES")
		{
			//判断系统类型，继而读取观测值类型并存入数组GPSObsType
			if (buff.substr(0, 1) == "C") {
				ofile.ohdr.BDObsTypNum = atoi((buff.substr(4, 2)).c_str());

				for (int i = 0; i != ofile.ohdr.BDObsTypNum; i++)
				{
					if (i < 13) {
						ofile.ohdr.BDSObsType[i] = ((buff.substr(7 + (i * 4), 3))).c_str();//溢出的意思是结果将超出数值的最大值，
					}
					else if (i >= 13 && i < 26) {//针对多行情况
						if (i == 13) {
							getline(file, buff);
						}
						ofile.ohdr.BDSObsType[i] = (buff.substr(7 + (i - 13) * 4, 3)).c_str();
					}
					else if (i >= 26 && i < 39) {
						if (i == 26) {
							getline(file, buff);
						}
						ofile.ohdr.BDSObsType[i] = (buff.substr(7 + (i - 13 * 2) * 4, 3)).c_str();
					}
				}
				/*cout << endl;*/
				puts("");//puts()和printf()都能够用来输出字符串，但是两者有些许不同。puts()在输出字符串后，会自动换行，而printf不会自动换行。
						 //puts()函数仍然以’\0’来确定字符串的结尾。
						 //C和C ++中的printf()和cout函数在打印变量，数字，行等方面都很突出，但是在打印字符串（尤其是printf()时，它们最终会落后。 
						 //在这种情况下， puts()函数很方便。
						 //puts()函数只能够输出字符串，参数只能是字符指针。输入若为其他类型数据或指针，编译器报错，无法进行数据类型转换。
			}

			if (buff.substr(0, 1) == "G") {
				ofile.ohdr.GPSObsTypNum = atoi((buff.substr(4, 2)).c_str());
				for (int i = 0; i != ofile.ohdr.GPSObsTypNum; i++) {
					if (i < 13) {
						ofile.ohdr.GPSObsType[i] = ((buff.substr(7 + (i * 4), 3))).c_str();
					}

					else if (i >= 13 && i < 26) {
						if (i == 13) {
							getline(file, buff);
						}
						ofile.ohdr.GPSObsType[i] = ((buff.substr(7 + (i - 13) * 4, 3))).c_str();
					}

					else if (i >= 26 && i < 39) {
						if (i == 26) {
							getline(file, buff);
						}
						ofile.ohdr.GPSObsType[i] = ((buff.substr(7 + (i - 13 * 2) * 4, 3))).c_str();
					}
				}
				puts("");
			}
		}
		else if (buff.substr(60, 13) == "END OF HEADER") {
			break;
		}
	}

	/*---------------读取观测值记录文件---------------*/
	ObsrecordEph orec = { };

	while (getline(file, buff))//istream& getline ( istream &is , string &str , char delim );只有读到行末，或读到字符delim返回时并返回值才为真。其他情况返回都会被视为false。
	{//上一步同时还进行了读取
		string str;
		orec.epoch.year = atoi((buff.substr(2, 4)).c_str());//c_str()//返回的是一个与str相同的char指针
		orec.epoch.month = atoi((buff.substr(7, 2)).c_str());
		orec.epoch.day = atoi((buff.substr(10, 2)).c_str());
		orec.epoch.hour = atoi((buff.substr(13, 2)).c_str());
		orec.epoch.minute = atoi((buff.substr(16, 2)).c_str());
		orec.epoch.second = atof((buff.substr(19, 10)).c_str());

		TimeSystem timesys;//创建Timesystem对象
		GPSTIME gpst;
		timesys.UTC2GPST(orec.epoch, gpst);//读取历元时转化为GPS时存储
		orec.OGPST.wn = gpst.wn;
		orec.OGPST.sow = gpst.sow;
		orec.GNSSSatNum = atoi((buff.substr(33, 2)).c_str());//该历元所观测到的卫星数目
		int c = 0, g = 0;//c\g分别为对应的北斗和GPS
		for (int i = 0; i != orec.GNSSSatNum; i++) {//将该历元全部读完
			string buff = { };
			getline(file, buff);                          //每颗卫星读一行
			string Sys = buff.substr(0, 1);
			if (Sys == "C") {
				if (buff.length() != ofile.ohdr.BDObsTypNum * 16 + 3) { //字符串长度是否有缺失数据，字符串长度，避免数组访问越界
					for (int j = 0; j != abs((ofile.ohdr.BDObsTypNum * 16 + 3)) - buff.length(); j++) {
						buff += ' ';
					}
				}
				int prn = atoi((buff.substr(1, 2)).c_str());//PRN
				orec.BDPRN[c].SYS = 1;//标识为北斗
				if (prn == 31 || prn == 56 || prn == 57 || prn == 58)//剔除在轨试验卫星
					continue;//跳出for继续循环
				else
					orec.BDPRN[c].PRN = prn; //存储卫星PRN
				for (int k = 0; k != ofile.ohdr.BDObsTypNum; k++) {//循环主要是寻找是否有对应的观测值
					if (ofile.ohdr.BDSObsType[k] == "C2I") {//对应第一个伪距 与类型对应起来
						orec.BDvalue[c][0] = atof((buff.substr(3 + k * 16, 16)).c_str());
						break;
					}
				}
				for (int j = 0; j != ofile.ohdr.BDObsTypNum; j++) {
					if (ofile.ohdr.BDSObsType[j] == "C6I") {//J代表第几个  在观测值较多时使用更佳
						orec.BDvalue[c][1] = atof((buff.substr(3 + j * 16, 16)).c_str());
						break;
					}
				}
				c++;
			}
			if (Sys == "G") {
				if (buff.length() != ofile.ohdr.GPSObsTypNum * 16 + 3) {
					for (int j = 0; j != abs((ofile.ohdr.GPSObsTypNum * 16 + 3)) - buff.length(); j++) {
						buff += ' ';
					}
				}
				orec.GPSPRN[g].SYS = 2;
				orec.GPSPRN[g].PRN = atoi(buff.substr(1, 2).c_str());

				for (int k = 0; k != ofile.ohdr.GPSObsTypNum; k++) {
					if (ofile.ohdr.GPSObsType[k] == "C1C") {
						str = buff.substr(3 + k * 16, 16);
						orec.GPSvalue[g][0] = atof(str.c_str());
						/*cout << "C1C=" << setw(16) << setiosflags(ios::left) << orec.GPSvalue[g][0];*/
						break;
					}
				}
				for (int j = 0; j != ofile.ohdr.GPSObsTypNum; j++) {
					if (ofile.ohdr.GPSObsType[j] == "C2W") {
						str = buff.substr(3 + j * 16, 16);
						orec.GPSvalue[g][1] = atof(str.c_str());
						/*cout << "C2W=" << setw(16) << setiosflags(ios::left) << orec.GPSvalue[g][1] << endl;*/
						break;
					}
				}
				g++;
			}
		}
		orec.BDSatNum = c;
		orec.GPSSatNum = g;
		ofile.vecOrec.push_back(orec); //该容器创建在结构体中        //读取完一个历元观测数据，存到vector容器中//实际上不管用不用new，vector中内容都会存放在堆上
		orec = { };													//读取完一个历元观测数据,不初始化会造成内存不足
	}
	
	file.close();
	//finish = clock();
	std::cout << "End of reading" << endl;
	//std::cout << "It takes " << (double)(finish - start) / CLOCKS_PER_SEC << "s" << endl << endl;
}
//引用调用  ObsrecordEph& orec 使用引用调用，可以在子函数中对 形参 所做的更改对 主函数 中的 实参 有效。 


/*******************
 * Name		: GetBestNav
 * Function	: 获取最佳导航电文
 * argument : ObsrecordEph& orec		   当前历元的观测文件（获取时间卫星编号等）
 *			  SPRN sprn					   SYS&&PRN
 *			  vector<NavRecord>& vecNrec   所有的导航电文均存储在这里面
 *			  int& label                   将最好的星历的编号通过label返回出来
 *			  int&mark               当星历过期时（差值大于7200）为1
 * return : none
 *原理：在系统及PRN对应正确之后不断的，遍历对比obs时间与星历时间作对比获取间隔时间最小的星历
 * 注：是否需要设置一个最小的历元间隔检验最后的星历是否过期
 *******************/
void CFileRead::GetBestNav(ObsrecordEph& orec, SPRN sprn, vector<NavRecord>& vecNrec, int& label,int&mark)
{
	double min = 0;
	mark = 0;
	for (unsigned int i = 0; i != vecNrec.size(); i++)//size()函数除了跟length()函数一样可以获取字符串长度之外，还可以获取vector类型的长度。
	{
		if (vecNrec.at(i).sprn.SYS == sprn.SYS && vecNrec.at(i).sprn.PRN == sprn.PRN) {//判断系统和prn相同
			min = fabs(orec.OGPST.sow - vecNrec.at(i).NGPST.sow);//获取当前间隔时间
			label = i;
		}
	}

	for (unsigned int i = 0; i != vecNrec.size(); i++)
	{
		if (vecNrec.at(i).sprn.SYS == sprn.SYS && vecNrec.at(i).sprn.PRN == sprn.PRN) {
			if (fabs(orec.OGPST.sow - vecNrec.at(i).NGPST.sow) < min) {//at用法
				min = fabs(orec.OGPST.sow - vecNrec.at(i).NGPST.sow);  //更新最小时间
				
				label = i;
			}
		}
	}
	if (min >7200)
	{
		mark=1;
	}
}



