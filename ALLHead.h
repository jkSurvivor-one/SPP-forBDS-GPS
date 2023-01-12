#pragma once
using namespace std;


#include <vector>
#include <string>

//�궨��
#define      _SQR(x)              ((x)*(x))           /*��ƽ��*/
#define      _ABS(x)              ((x)>0?(x):-(x))    /*�����ֵ*/
#define      MAX(x, y)            (((x)>(y))?(x):(y))
#define      MIN(x, y)            (((x)<(y))?(x):(y))
#define      ARR_SIZE(a)          (sizeof((a))/sizeof((a[0])))  //��������Ԫ�صĸ���

#define      MAXCHAR   100                 /*ÿһ�ж��������ַ�����*/
#define      SYS_NONE             0x00
#define      SYS_GPS              0x01
#define      SYS_BDS              0x02
#define      SYS_MIX              (SYS_GPS|SYS_BDS)

#define      MAXSAT               (NSATGPS+NSATBDS)

#define      MINPRNGPS            1                   /* min satellite PRN number of GPS */
#define      MAXPRNGPS            32                  /* max satellite PRN number of GPS */
#define      NSATGPS              32                  /* number of GPS satellites */

#define      MINPRNBDS            1                   /* min satellite sat number of Compass */
#define      MAXPRNBDS            61                  /* max satellite sat number of Compass */
#define      NSATBDS              61                  /* number of Compass satellites */

#define      FREG1                1.57542E9           /* L1/E1  frequency (Hz) */
#define      FREG2                1.22760E9           /* L2     frequency (Hz) */
#define      FREG5                1.17645E9           /* L5/E5a frequency (Hz) */

#define      FREC1                1.561098E9           /* L1/E1  frequency (Hz) */
#define      FREC2                1.207140E9           /* L2     frequency (Hz) */
#define      FREC5                1.268520E9           /* L3     frequency (Hz) */

constexpr double MINUTE_IN_SECOND = 60;
constexpr double HOUR_IN_MINUTE = 60;
constexpr double DAY_IN_HOUR = 24;
constexpr double WEEK_IN_DAY = 7;
constexpr double HOUR_IN_SECOND = HOUR_IN_MINUTE * MINUTE_IN_SECOND;
constexpr double DAY_IN_SECOND = DAY_IN_HOUR * HOUR_IN_SECOND;
constexpr double WEEK_IN_SECOND = WEEK_IN_DAY * DAY_IN_SECOND;
constexpr double PI = 3.1415926535897932384626433832795;
constexpr double CLIGHT = 299792458.0;
constexpr double PRECISION = 1.0e-10;
constexpr double D2R = PI / 180.0;   //�Ƕ�*deg_rad= ����
constexpr double R2D = 180.0 / PI;   //����*rad_deg= �Ƕ�

//std::string GPS = "G";
//std::string BD = "C";
//std::string GLONASS = "R";
//std::string Galileo = "E";//J��QZSS����I��IRNSS����S��SBAS��


/********************GPS********************/
//const double L1 = 1575420000.0;                 //GPS�ز�Ƶ��
//const double L2 = 1227600000.0;
constexpr double P1 = 2.545727780163160;              //˫Ƶ�������ϵ��
constexpr double P2 = 1.545727780163160;              //˫Ƶ�������ϵ��

//WGS84
constexpr double E1S = 0.00669437999013;              //��һ���ƫ����
//const double e2s = 0.00673949674227;
constexpr double GM_WGS = 3.986005e+14;             //������������
constexpr double OMGE_WGS = 7.2921151467e-5;          //������ת���ٶ�
constexpr double A = 6378137.0;                       //���򳤰���
constexpr double ellipsoidalb = 6356752.314245;	//WGS_84����̰���
constexpr double WGSe1 = 0.00669437999013;				//WGS_84Ϊ��Բ��һƫ����

/********************BDS********************/
constexpr double B1_2 = 1561098000.0;//BDS�ز�Ƶ��
constexpr double B3 = 1268520000.0;
constexpr int GEO_prn[8] = { 1,2,3,4,5,59,60,61 };//GEO���ǵ�

//CGCS2000
constexpr double GM_CGCS = 3.986004418e14;        //GM(BDS)
constexpr double OMGE_CGCS = 7.2921150e-5;
//const double e1s = 0.00669438002290;
//������ ȱʡ Ĭ��Ϊpublic
// 
// 
// 
// /************************************
// ʱ������ϵͳ
//ͨ��ʱ�䣨����ʱ�䣩
struct COMMONTIME
{
	int year, month, day, hour, minute;
	double second;
};


//GPSʱ����+������
struct GPSTIME
{
	long wn;
	double sow;
	//double sn;   //������������������
	//double tos;  //����������С������
};

//����ʱ��ϵͳ
class TimeSystem
{
public:
	TimeSystem();
	~TimeSystem();
	COMMONTIME comt;
	GPSTIME gpst;
	void UTC2GPST(COMMONTIME& comt, GPSTIME& gpst);
	void GPST2BDT(GPSTIME& gpst, GPSTIME& bdt);
};
 struct Descartes
{
	double X, Y, Z;
};

//�������
 struct Geodetic
{
	double B, L, H;
};
class Coorsys
{
public:
	Coorsys();
	~Coorsys();
	void XYZ2BLH(double XYZ[3], double blh[3]);
	void BLH2XYZ(Geodetic& blh, Descartes& XYZ);
private:

};
/*******************�������Ľṹ��******************/
//����������Ϣͷ
struct Navhead
{
	double ionA1, ionA2, ionA3, ionA4;
	double ionB1, ionB2, ionB3, ionB4;//GPS������8����������
	double bdionA1, bdionA2, bdionA3, bdionA4;
	double bdionB1, bdionB2, bdionB3, bdionB4;//BDS������8����������
	long double A0, A1;//���ڼ���UTCʱ����������������ʽϵ��
	int T, W��leap;//�ο�ʱ�̣��ο����������������ʱ���
};

struct SPRN
{
	int SYS;                               //����ϵͳ(1��ʾ������2��ʾGPS��
	int PRN;                               //����PRN�ţ���G30��30
};

//����������Ϣ��¼
struct NavRecord
{
	SPRN sprn;
	COMMONTIME TOC;                        //��Ԫ��TOC �����ӵĲο�ʱ��
	double a0, a1, a2;                     //�����Ӳ������Ư��������Ư�ٶ�

	GPSTIME NGPST;                         //��ϵͳ�����Ĳο���Ԫͳһ����GPSʱ��ʾ

	double IODE;                           //��������
	double Crs;                            //�㶯����
	double Deltan;                         //�㶯����
	double M0;                             //�ο�ʱ��TOEʱ��ƽ�����

	double Cuc;                            //�㶯����
	double e;                              //���ƫ����
	double Cus;                            //�㶯���� 
	double sqrtA;                          //AΪ���ǹ�����뾶

	double TOE;                            //��������Ĳο�ʱ��
	double Cic;                            //�㶯����
	double Omeca;                          //���Ƕ�Ӧ�ο�ʱ��TOE��������ྭ
	double Cis;                            //�㶯����

	double i0;                             //TOEʱ�̵Ĺ�����
	double Crc;                            //�㶯����
	double Omega;                          //���ص�Ǿ�
	double OmegaDot;                       //�������Ӧ��ʱ��仯��

	double IDOT;                           //�����ǵı仯��
	double CodesOnL2Channel, datasources, spare1_BD1;
	double WEEK;
	double L2PDataFlag, ADOT, spare2_BD1;

	double SatAccuracy, integrity;
	double SatHealth, HS;                  //���ǽ���״̬
	double TGD;                            //Ⱥ�ӳ� 
	double IODC, ISC1;                     //ʱ���������ڣ�Ƶ��ƫ������

	double TransTimeOfMaq;
	double FitInterval;                    //BD��IODC
	double spare1, Deltan0dot, spare3_BD1;
	double spare2, ISC2, spare4_BD1;       //Ƶ��������                   
};
/*****************�۲�ֵ�ṹ��*****************/
//�۲�ֵ��Ϣͷ�ṹ
struct ObsHead
{
	int GPSObsTypNum=0;//GPS observe type number.SYS / # / OBS TYPES
	int BDObsTypNum=0;//BDS observe type number.SYS / # / OBS TYPES
	double apporx_position[3]={};//��վ����λ�ã�WGS84��XYZ��
	std::string GPSObsType[30];
	std::string BDSObsType[30];
};

//�۲�ֵ��Ϣ��¼�ṹ:һ����Ԫ
struct ObsrecordEph
{
	COMMONTIME epoch;                                 //��Ԫʱ�̣�ͨ��ʱ��
	GPSTIME OGPST;                                    //��Ԫʱ�̣�ת��ΪGPSʱͳһ��ʾ��
	int GNSSSatNum;                                   //����Ԫ���۲⵽��GNSS������
	int GPSSatNum;                                    //����Ԫ���۲⵽��GPS������
	int BDSatNum;                                     //����Ԫ���۲⵽��BDS������

	SPRN BDPRN[60];
	SPRN GPSPRN[32];
	double BDvalue[60][2];							//����Ϊ60������  0����C2I  1ΪC6I
	double GPSvalue[32][2];
};
//�۲�ֵ�ļ��ܺͰ��� ͷ�ļ���Ϣ�۲����ݵ�
struct ObsFile
{
	ObsHead ohdr;
	std::vector<ObsrecordEph> vecOrec;//vector<����>��ʶ��
};
/***********************FIleRead����***************/
class CFileRead//������
{
public:
	CFileRead();
	~CFileRead();
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
	void ReadNavFile(string nfilename, Navhead& Navhdr, vector<NavRecord>& vecNrec);
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
	void ReadObsFile(string ofilename, ObsFile& ofile);

	//����С����ʱ�䣬��ʹ�������ο�ʱ���������ʱ�̵Ĳ�ֵ��С
	void GetBestNav(ObsrecordEph& orec, SPRN sprn, vector<NavRecord>& vecNre, int& label,int&mark);

	//char* Substr(const char* s, int n1, int n2);
private:

};

/***********************������Ϣ��**********************/
//���ǵ���ϢPRN�����ꡢ����ֵ���߶Ƚǵ�
 struct  SatInformation
{
	int SYS;
	int PRN;
	double X, Y, Z;
	double SatClk;             //�����Ӳ�
	double codeC1, codeP2;   //��ͬ�ز��ϵ�α��۲�ֵ���ͣ�GPS:L1,L2;BDS:B1,B3)
	double psr;              //�޵�����ӳٵ�������Ϲ۲�ֵ
	double dtrop;            //�������ӳٸ�����
	double diono;            //������ӳٸ������������޵����α����Ϲ۲�ֵʱ������Ϊ0��
	double drtcm;            //��α��Ĳ�ָ���ֵ���˴�Ϊ0
	double azel[2];             //���Ǹ߶Ƚ�  0��λ��  1�߶Ƚ�
};
//����(GPS/BDS)����ά����
 struct  GNSSSatPos
{
	int SYS;
	int PRN;
	double X, Y, Z;
};
//��ǰ��Ԫ��,�������ǵ����ꡢ��������
 struct ASatPosInOneEph
{
	int GPSSatNumEph;       //��ǰ��Ԫ,GPS������
	int BDSatNumEph;       //��ǰ��Ԫ,BDS������
	int SatNumEph;
	std::vector<GNSSSatPos> vecAGPSPosInOneEph;  //��ǰ��Ԫ,����GPS���ǵ�λ��
	std::vector<GNSSSatPos> vecABDSPosInOneEph;  //��ǰ��Ԫ,����BDS���ǵ�λ��
	std::vector<GNSSSatPos> vecASatPosInOneEph;  //��ǰ��Ԫ,����BDS+GPS���ǵ�λ��
};

/*******************�������*******************/
class Correction
{
public:
	Correction();
	~Correction();
	//�߶ȽǼ���
	void CALazel(struct ObsHead& re_postion, struct SatInformation& satxyz);
	//��������
	void IONCorr(double gtime_t, struct ObsHead& re_postion, SatInformation& SatPosTemp, Navhead& nav);
	//���������
	void TropCorr(struct ObsHead& re_postion, SatInformation& SatPosTemp);
	//������ת����
	void RotaCorr(GPSTIME& rec_t, GPSTIME& emi_t, SatInformation& SatPosTemp, GNSSSatPos& SingleTemp);
	void calsatclk(const struct GPSTIME* time, NavRecord& BestN, SatInformation& SatPosTemp,double&ES);
private:

};



/*************************SPP����**********************/
class SPP
{
public:
	SPP();
	~SPP();
	//�����������Ƿ���ʱ��
	void CalEmitTSATpos(NavRecord& BestN, GPSTIME& rec_t, GPSTIME& emi_t, SatInformation& SatPosTemp,
		GNSSSatPos& SingleTemp, Descartes& RecPos, double& Es, const int& PosSys);//����������
	void CALBDSatPos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es);
	void CALGPSSatpos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es);
	void CALRecPos(vector<SatInformation>& vecSatPos, Descartes& car_GetRecPos, double vx[], const int& PosSys, const int& weight);
	void CALepochSTA(ObsFile& ofile, vector<NavRecord>& vecNrec_epo, ASatPosInOneEph& SingleSyssatEpoch,
		Descartes& RecPos, int i, vector<SatInformation>& vecSatPos, ofstream& out1, const int& PosSys, const int& weight, const int& iepoch);
	void SinglePP(ObsFile& ofile, vector<NavRecord>& vecNrec_epo, vector<ASatPosInOneEph>& vecStorSatPosEveEpo, vector<Descartes>& vecStorRecPos, ofstream& out1, const int& PosSys, const int& weight);

private:

};
