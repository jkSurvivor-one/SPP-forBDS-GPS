#include <cstdio>
#include"ALLHead.h"

#include <string>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <fstream>//�ļ���ͷ
#include <cstdlib>

//������CFileRead�еĹ�������
using namespace std;


CFileRead::CFileRead()
{
}//C++11֮���һ���淶��顣 ������ʵ���ⲻ�� 

CFileRead::~CFileRead()
{
}

/*******************
 * Name		: ReadNavFile
 * Function	: ��ȡ��������
 * argument : string nfilename ���������ļ���/�ļ�ָ��
 *			  Navhead& Navhdr  ��������ͷ�ļ���¼
 *			  vector<NavRecord>& vecNrec  ���������ļ���¼
 *
 * return   : none
 *
 *principle �����ļ�ָ���ȡȫ����ͷ�ļ�֮�󣬵����������ĵĶ�ȡ��8�У�֮�󽫶�Ӧ���ļ������Ӧ��������
 *remark    ����ΪBDS  GPS
 *******************/
void CFileRead::ReadNavFile(string nfilename, Navhead& Navhdr, vector<NavRecord>& vecNrec)
{
	
	ifstream file;//�����ļ���

	std::cout << "Start reading navigation file:[" << nfilename << " ] " << endl;
	std::cout << "In the process of reading......" << endl;

	file.open(nfilename, ios::in | ios::out);//��n�ļ�

	if (!file)  std::cout << "n�ļ��򿪴���!" << endl;

	/*---------------��ȡ���������ļ�ͷ---------------*/
	while (!file.eof())
	{
		string str;
		getline(file, str);//��
		//file.getline(str, MAXCHAR, '\n');//strֻ�������� �������ַ�ָ��
		/*********ͷ�ļ���ȡ********/
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

	/*---------------��ȡ�����������ݼ�¼---------------*/

	while (!file.eof())
	{
		NavRecord nrec = { };//�ݴ���Ϣ
		string  oneline;
		int prn;
		getline(file,oneline);//ת�����ڴ���ַ��ȥ��ʱ�򣬻ᷢ��ת��

		/*��ȡBD��������*/
		if ("C"== oneline.substr( 0, 1)) {
			prn = atoi((oneline.substr(1, 2)).c_str());//�ַ���ת������atoi
			if (prn == 31 || prn == 56 || prn == 57 || prn == 58)//�޳��ڹ���������
				continue;
			for (int i = 1; i <= 8; i++) {
				switch (i) {
				case 1: {
					nrec.sprn.SYS = 1;
					nrec.sprn.PRN = atoi((oneline.substr( 1, 2)).c_str());//atoi()���������ַ���ת����������
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

					nrec.a0 = atof((oneline.substr(24, 14)).c_str())*pow(10,atoi((oneline.substr(39, 3)).c_str()));//atof()���������ַ���ת���ɸ���������ʹ�õ�ͷ�ļ�Ϊ<stdlib.h>
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
				case 1://��ȡ��һ������

				{
					nrec.sprn.SYS = 2;
					nrec.sprn.PRN = atoi((oneline.substr(1, 2)).c_str());//atoi()���������ַ���ת����������
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

					nrec.a0 = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));//atof()���������ַ���ת���ɸ���������ʹ�õ�ͷ�ļ�Ϊ<stdlib.h>
					nrec.a1 = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.a2 = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 2://�㲥���1
				{
					getline(file, oneline);
					nrec.IODE = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Crs = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Deltan = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.M0 = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 3://�㲥���2
				{
					getline(file, oneline);
					nrec.Cuc = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.e = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Cus = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.sqrtA = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 4://�㲥���3
				{
					getline(file, oneline);
					nrec.TOE = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Cic = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Omeca = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.Cis = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 5://�㲥���4
				{
					getline(file, oneline);
					nrec.i0 = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.Crc = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					nrec.Omega = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					nrec.OmegaDot = atof((oneline.substr(62, 14)).c_str()) * pow(10, atoi((oneline.substr(77, 3)).c_str()));
					break;
				}

				case 6://�㲥���5
				{
					getline(file, oneline);
					nrec.IDOT = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					//nrec.CodesOnL2Channel = atof(substr(oneline, 23, 41));
					nrec.WEEK = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					//nrec.L2PDataFlag = atof(substr(oneline, 61, 79));
					break;
				}

				case 7://�㲥���6
				{
					getline(file, oneline);
					//nrec.SatAccuracy = atof(substr(oneline, 4, 22));
					//nrec.SatHealth = atof(substr(oneline, 23, 41));
					nrec.TGD = atof((oneline.substr(43, 14)).c_str()) * pow(10, atoi((oneline.substr(58, 3)).c_str()));
					//nrec.IODC = atof(substr(oneline, 61, 79));
					break;
				}

				case 8://�㲥���7
				{
					getline(file, oneline);
					nrec.TransTimeOfMaq = atof((oneline.substr(5, 14)).c_str()) * pow(10, atoi((oneline.substr(20, 3)).c_str()));
					nrec.FitInterval = atof((oneline.substr(24, 14)).c_str()) * pow(10, atoi((oneline.substr(39, 3)).c_str()));
					break;
				}
				}//switch
			}//for
			vecNrec.push_back(nrec);//������һ���µ�Ԫ�ؼӵ�vector������棬λ��Ϊ��ǰ���һ��Ԫ�ص���һ��Ԫ��
			
		}//if
	}//while

	/*��֤������Ϣ�Ƿ�ɹ��洢*/
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
 * Function	:��ȡȫ���Ĺ۲�ֵ�ļ�
 * argument :string ofilename   �ļ�����·����
 *			 ObsFile& ofile      OBS�ļ��Ľṹ�壨�ļ��Ĵ洢λ�ã�
 *
 * return   :none
 * ԭ��     :���ļ��򿪿�ʼ���ȶ�ȡ�ļ�ͷ������ObsFile�е�ͷ�ļ�����
 *			 ��ȡÿһ����Ԫ�Ĺ۲�ֵ��������ofile.vecOrec.push_back(orec)
 *			��ͷ�ļ����ֻ�Ҫ��ȡ��Ӧ�Ĺ۲�ֵ���ͣ��Ա����ں���۲�ֵ����ʱ�ܹ���Ӧ����
 *******************/
void CFileRead::ReadObsFile(string ofilename, ObsFile& ofile)
{
	//clock_t start, finish;
	//start = clock();//���ÿ�ʼʱ��

	ifstream file;//�����ļ���

	std::cout << "Start reading observation file :[" << ofilename << " ] " << endl;

	std::cout << "In the process of reading....." << endl;

	file.open(ofilename, ios_base::in);//��o�ļ�
	if (!file.is_open())
	{
		cerr << "���ļ�ʧ��" << endl;
		exit(EXIT_FAILURE);
	}

	/*---------------��ȡ�۲�ֵͷ�ļ�---------------*/
	string buff;//����string���� ���ڴ洢�ļ���ȡ���ַ���   C��û��stringֻ��ͨ��char buff[] �����ַ�����
	//const string str = { };//����string ������ʱ�洢  ��buff�ַ����н�ȡ���ַ�

	while (getline(file, buff))//getline(<�ַ�����chs>��<��ȡ�ַ��ĸ���n>��<��ֹ��>)
		/*stream& getline ( istream &is , string &str , char delim );
		���У�istream& is ��ʾһ����������Ʃ��cin��
		string& str��ʾ�Ѵ�������������ַ������������ַ����У������Լ����������strʲô�Ķ����ԣ���
		char delim��ʾ��������ַ�ֹͣ���룬�ڲ����õ������ϵͳĬ�ϸ��ַ�Ϊ'\n'��Ҳ���ǻس����з��������س�ֹͣ���룩��*/
	{
		if (buff.substr(60, 19) == "APPROX POSITION XYZ")//��ȡ��վ����λ��
		{
			//s.substr(pos, len)����ֵ�� string������s�д�pos��ʼ��len���ַ��Ŀ�����pos��Ĭ��ֵ��0��len��Ĭ��ֵ��s.size()
			ofile.ohdr.apporx_position[0] = atof((buff.substr(0, 14)).c_str());//��ס��Ҫֱ�Ӳ���c_str()���ص�ָ�룬�ڲ���֮ǰ��ͨ��strcpy()����ת��������c = s.c_str();
			ofile.ohdr.apporx_position[1] = atof((buff.substr(14, 14)).c_str());//c_str() �� char* ��ʽ���� string �ں��ַ��������һ������Ҫ��char*����������ʹ��c_str()������
			ofile.ohdr.apporx_position[2] = atof(buff.substr(28, 14).c_str());
		}
		else if (buff.substr(60, 19) == "SYS / # / OBS TYPES")
		{
			//�ж�ϵͳ���ͣ��̶���ȡ�۲�ֵ���Ͳ���������GPSObsType
			if (buff.substr(0, 1) == "C") {
				ofile.ohdr.BDObsTypNum = atoi((buff.substr(4, 2)).c_str());

				for (int i = 0; i != ofile.ohdr.BDObsTypNum; i++)
				{
					if (i < 13) {
						ofile.ohdr.BDSObsType[i] = ((buff.substr(7 + (i * 4), 3))).c_str();//�������˼�ǽ����������ֵ�����ֵ��
					}
					else if (i >= 13 && i < 26) {//��Զ������
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
				puts("");//puts()��printf()���ܹ���������ַ���������������Щ��ͬ��puts()������ַ����󣬻��Զ����У���printf�����Զ����С�
						 //puts()������Ȼ�ԡ�\0����ȷ���ַ����Ľ�β��
						 //C��C ++�е�printf()��cout�����ڴ�ӡ���������֣��еȷ��涼��ͻ���������ڴ�ӡ�ַ�����������printf()ʱ���������ջ���� 
						 //����������£� puts()�����ܷ��㡣
						 //puts()����ֻ�ܹ�����ַ���������ֻ�����ַ�ָ�롣������Ϊ�����������ݻ�ָ�룬�����������޷�������������ת����
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

	/*---------------��ȡ�۲�ֵ��¼�ļ�---------------*/
	ObsrecordEph orec = { };

	while (getline(file, buff))//istream& getline ( istream &is , string &str , char delim );ֻ�ж�����ĩ��������ַ�delim����ʱ������ֵ��Ϊ�档����������ض��ᱻ��Ϊfalse��
	{//��һ��ͬʱ�������˶�ȡ
		string str;
		orec.epoch.year = atoi((buff.substr(2, 4)).c_str());//c_str()//���ص���һ����str��ͬ��charָ��
		orec.epoch.month = atoi((buff.substr(7, 2)).c_str());
		orec.epoch.day = atoi((buff.substr(10, 2)).c_str());
		orec.epoch.hour = atoi((buff.substr(13, 2)).c_str());
		orec.epoch.minute = atoi((buff.substr(16, 2)).c_str());
		orec.epoch.second = atof((buff.substr(19, 10)).c_str());

		TimeSystem timesys;//����Timesystem����
		GPSTIME gpst;
		timesys.UTC2GPST(orec.epoch, gpst);//��ȡ��Ԫʱת��ΪGPSʱ�洢
		orec.OGPST.wn = gpst.wn;
		orec.OGPST.sow = gpst.sow;
		orec.GNSSSatNum = atoi((buff.substr(33, 2)).c_str());//����Ԫ���۲⵽��������Ŀ
		int c = 0, g = 0;//c\g�ֱ�Ϊ��Ӧ�ı�����GPS
		for (int i = 0; i != orec.GNSSSatNum; i++) {//������Ԫȫ������
			string buff = { };
			getline(file, buff);                          //ÿ�����Ƕ�һ��
			string Sys = buff.substr(0, 1);
			if (Sys == "C") {
				if (buff.length() != ofile.ohdr.BDObsTypNum * 16 + 3) { //�ַ��������Ƿ���ȱʧ���ݣ��ַ������ȣ������������Խ��
					for (int j = 0; j != abs((ofile.ohdr.BDObsTypNum * 16 + 3)) - buff.length(); j++) {
						buff += ' ';
					}
				}
				int prn = atoi((buff.substr(1, 2)).c_str());//PRN
				orec.BDPRN[c].SYS = 1;//��ʶΪ����
				if (prn == 31 || prn == 56 || prn == 57 || prn == 58)//�޳��ڹ���������
					continue;//����for����ѭ��
				else
					orec.BDPRN[c].PRN = prn; //�洢����PRN
				for (int k = 0; k != ofile.ohdr.BDObsTypNum; k++) {//ѭ����Ҫ��Ѱ���Ƿ��ж�Ӧ�Ĺ۲�ֵ
					if (ofile.ohdr.BDSObsType[k] == "C2I") {//��Ӧ��һ��α�� �����Ͷ�Ӧ����
						orec.BDvalue[c][0] = atof((buff.substr(3 + k * 16, 16)).c_str());
						break;
					}
				}
				for (int j = 0; j != ofile.ohdr.BDObsTypNum; j++) {
					if (ofile.ohdr.BDSObsType[j] == "C6I") {//J����ڼ���  �ڹ۲�ֵ�϶�ʱʹ�ø���
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
		ofile.vecOrec.push_back(orec); //�����������ڽṹ����        //��ȡ��һ����Ԫ�۲����ݣ��浽vector������//ʵ���ϲ����ò���new��vector�����ݶ������ڶ���
		orec = { };													//��ȡ��һ����Ԫ�۲�����,����ʼ��������ڴ治��
	}
	
	file.close();
	//finish = clock();
	std::cout << "End of reading" << endl;
	//std::cout << "It takes " << (double)(finish - start) / CLOCKS_PER_SEC << "s" << endl << endl;
}
//���õ���  ObsrecordEph& orec ʹ�����õ��ã��������Ӻ����ж� �β� �����ĸ��Ķ� ������ �е� ʵ�� ��Ч�� 


/*******************
 * Name		: GetBestNav
 * Function	: ��ȡ��ѵ�������
 * argument : ObsrecordEph& orec		   ��ǰ��Ԫ�Ĺ۲��ļ�����ȡʱ�����Ǳ�ŵȣ�
 *			  SPRN sprn					   SYS&&PRN
 *			  vector<NavRecord>& vecNrec   ���еĵ������ľ��洢��������
 *			  int& label                   ����õ������ı��ͨ��label���س���
 *			  int&mark               ����������ʱ����ֵ����7200��Ϊ1
 * return : none
 *ԭ����ϵͳ��PRN��Ӧ��ȷ֮�󲻶ϵģ������Ա�obsʱ��������ʱ�����ԱȻ�ȡ���ʱ����С������
 * ע���Ƿ���Ҫ����һ����С����Ԫ����������������Ƿ����
 *******************/
void CFileRead::GetBestNav(ObsrecordEph& orec, SPRN sprn, vector<NavRecord>& vecNrec, int& label,int&mark)
{
	double min = 0;
	mark = 0;
	for (unsigned int i = 0; i != vecNrec.size(); i++)//size()�������˸�length()����һ�����Ի�ȡ�ַ�������֮�⣬�����Ի�ȡvector���͵ĳ��ȡ�
	{
		if (vecNrec.at(i).sprn.SYS == sprn.SYS && vecNrec.at(i).sprn.PRN == sprn.PRN) {//�ж�ϵͳ��prn��ͬ
			min = fabs(orec.OGPST.sow - vecNrec.at(i).NGPST.sow);//��ȡ��ǰ���ʱ��
			label = i;
		}
	}

	for (unsigned int i = 0; i != vecNrec.size(); i++)
	{
		if (vecNrec.at(i).sprn.SYS == sprn.SYS && vecNrec.at(i).sprn.PRN == sprn.PRN) {
			if (fabs(orec.OGPST.sow - vecNrec.at(i).NGPST.sow) < min) {//at�÷�
				min = fabs(orec.OGPST.sow - vecNrec.at(i).NGPST.sow);  //������Сʱ��
				
				label = i;
			}
		}
	}
	if (min >7200)
	{
		mark=1;
	}
}



