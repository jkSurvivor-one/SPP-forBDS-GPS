#pragma once
using namespace std;


#include <vector>
#include <string>

//宏定义
#define      _SQR(x)              ((x)*(x))           /*求平方*/
#define      _ABS(x)              ((x)>0?(x):-(x))    /*求绝对值*/
#define      MAX(x, y)            (((x)>(y))?(x):(y))
#define      MIN(x, y)            (((x)<(y))?(x):(y))
#define      ARR_SIZE(a)          (sizeof((a))/sizeof((a[0])))  //返回数组元素的个数

#define      MAXCHAR   100                 /*每一行读入的最大字符个数*/
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
constexpr double D2R = PI / 180.0;   //角度*deg_rad= 弧度
constexpr double R2D = 180.0 / PI;   //弧度*rad_deg= 角度

//std::string GPS = "G";
//std::string BD = "C";
//std::string GLONASS = "R";
//std::string Galileo = "E";//J（QZSS）、I（IRNSS）、S（SBAS）


/********************GPS********************/
//const double L1 = 1575420000.0;                 //GPS载波频率
//const double L2 = 1227600000.0;
constexpr double P1 = 2.545727780163160;              //双频消电离层系数
constexpr double P2 = 1.545727780163160;              //双频消电离层系数

//WGS84
constexpr double E1S = 0.00669437999013;              //第一轨道偏心率
//const double e2s = 0.00673949674227;
constexpr double GM_WGS = 3.986005e+14;             //地球引力常数
constexpr double OMGE_WGS = 7.2921151467e-5;          //地球自转角速度
constexpr double A = 6378137.0;                       //地球长半轴
constexpr double ellipsoidalb = 6356752.314245;	//WGS_84椭球短半轴
constexpr double WGSe1 = 0.00669437999013;				//WGS_84为椭圆第一偏心率

/********************BDS********************/
constexpr double B1_2 = 1561098000.0;//BDS载波频率
constexpr double B3 = 1268520000.0;
constexpr int GEO_prn[8] = { 1,2,3,4,5,59,60,61 };//GEO卫星的

//CGCS2000
constexpr double GM_CGCS = 3.986004418e14;        //GM(BDS)
constexpr double OMGE_CGCS = 7.2921150e-5;
//const double e1s = 0.00669438002290;
//创建类 缺省 默认为public
// 
// 
// 
// /************************************
// 时间坐标系统
//通用时间（日历时间）
struct COMMONTIME
{
	int year, month, day, hour, minute;
	double second;
};


//GPS时：周+周内秒
struct GPSTIME
{
	long wn;
	double sow;
	//double sn;   //当日秒数的整数部分
	//double tos;  //当日秒数的小数部分
};

//建立时间系统
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

//大地坐标
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
/*******************导航电文结构体******************/
//导航电文信息头
struct Navhead
{
	double ionA1, ionA2, ionA3, ionA4;
	double ionB1, ionB2, ionB3, ionB4;//GPS历书中8个电离层参数
	double bdionA1, bdionA2, bdionA3, bdionA4;
	double bdionB1, bdionB2, bdionB3, bdionB4;//BDS历书中8个电离层参数
	long double A0, A1;//用于计算UTC时间的历书参数：多项式系数
	int T, W，leap;//参考时刻，参考周数，跳秒引起的时间差
};

struct SPRN
{
	int SYS;                               //卫星系统(1表示北斗；2表示GPS）
	int PRN;                               //卫星PRN号，如G30的30
};

//导航电文信息记录
struct NavRecord
{
	SPRN sprn;
	COMMONTIME TOC;                        //历元：TOC 卫星钟的参考时刻
	double a0, a1, a2;                     //卫星钟差，卫星钟漂，卫星钟漂速度

	GPSTIME NGPST;                         //多系统星历的参考历元统一采用GPS时表示

	double IODE;                           //数据龄期
	double Crs;                            //摄动参数
	double Deltan;                         //摄动参数
	double M0;                             //参考时刻TOE时的平近点角

	double Cuc;                            //摄动参数
	double e;                              //轨道偏心率
	double Cus;                            //摄动参数 
	double sqrtA;                          //A为卫星轨道长半径

	double TOE;                            //轨道参数的参考时刻
	double Cic;                            //摄动参数
	double Omeca;                          //并非对应参考时刻TOE的升交点赤经
	double Cis;                            //摄动参数

	double i0;                             //TOE时刻的轨道倾角
	double Crc;                            //摄动参数
	double Omega;                          //近地点角距
	double OmegaDot;                       //升交点对应的时间变化率

	double IDOT;                           //轨道倾角的变化率
	double CodesOnL2Channel, datasources, spare1_BD1;
	double WEEK;
	double L2PDataFlag, ADOT, spare2_BD1;

	double SatAccuracy, integrity;
	double SatHealth, HS;                  //卫星健康状态
	double TGD;                            //群延迟 
	double IODC, ISC1;                     //时钟数据龄期，频内偏修正项

	double TransTimeOfMaq;
	double FitInterval;                    //BD：IODC
	double spare1, Deltan0dot, spare3_BD1;
	double spare2, ISC2, spare4_BD1;       //频内修正项                   
};
/*****************观测值结构体*****************/
//观测值信息头结构
struct ObsHead
{
	int GPSObsTypNum=0;//GPS observe type number.SYS / # / OBS TYPES
	int BDObsTypNum=0;//BDS observe type number.SYS / # / OBS TYPES
	double apporx_position[3]={};//测站概略位置（WGS84：XYZ）
	std::string GPSObsType[30];
	std::string BDSObsType[30];
};

//观测值信息记录结构:一个历元
struct ObsrecordEph
{
	COMMONTIME epoch;                                 //历元时刻（通用时）
	GPSTIME OGPST;                                    //历元时刻（转化为GPS时统一表示）
	int GNSSSatNum;                                   //本历元所观测到的GNSS卫星数
	int GPSSatNum;                                    //本历元所观测到的GPS卫星数
	int BDSatNum;                                     //本历元所观测到的BDS卫星数

	SPRN BDPRN[60];
	SPRN GPSPRN[32];
	double BDvalue[60][2];							//依次为60颗卫星  0代表C2I  1为C6I
	double GPSvalue[32][2];
};
//观测值文件总和包括 头文件信息观测数据等
struct ObsFile
{
	ObsHead ohdr;
	std::vector<ObsrecordEph> vecOrec;//vector<类型>标识符
};
/***********************FIleRead函数***************/
class CFileRead//创建类
{
public:
	CFileRead();
	~CFileRead();
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
	void ReadNavFile(string nfilename, Navhead& Navhdr, vector<NavRecord>& vecNrec);
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
	void ReadObsFile(string ofilename, ObsFile& ofile);

	//求最小外推时间，即使得星历参考时刻与待计算时刻的差值最小
	void GetBestNav(ObsrecordEph& orec, SPRN sprn, vector<NavRecord>& vecNre, int& label,int&mark);

	//char* Substr(const char* s, int n1, int n2);
private:

};

/***********************卫星信息等**********************/
//卫星的信息PRN、坐标、改正值、高度角等
 struct  SatInformation
{
	int SYS;
	int PRN;
	double X, Y, Z;
	double SatClk;             //卫星钟差
	double codeC1, codeP2;   //不同载波上的伪距观测值类型（GPS:L1,L2;BDS:B1,B3)
	double psr;              //无电离层延迟的线性组合观测值
	double dtrop;            //对流层延迟改正量
	double diono;            //电离层延迟改正量（采用无电离层伪距组合观测值时，此项为0）
	double drtcm;            //对伪距的差分改正值，此处为0
	double azel[2];             //卫星高度角  0方位角  1高度角
};
//卫星(GPS/BDS)的三维坐标
 struct  GNSSSatPos
{
	int SYS;
	int PRN;
	double X, Y, Z;
};
//当前历元内,所有卫星的坐标、卫星总数
 struct ASatPosInOneEph
{
	int GPSSatNumEph;       //当前历元,GPS卫星数
	int BDSatNumEph;       //当前历元,BDS卫星数
	int SatNumEph;
	std::vector<GNSSSatPos> vecAGPSPosInOneEph;  //当前历元,所有GPS卫星的位置
	std::vector<GNSSSatPos> vecABDSPosInOneEph;  //当前历元,所有BDS卫星的位置
	std::vector<GNSSSatPos> vecASatPosInOneEph;  //当前历元,所有BDS+GPS卫星的位置
};

/*******************改正项函数*******************/
class Correction
{
public:
	Correction();
	~Correction();
	//高度角计算
	void CALazel(struct ObsHead& re_postion, struct SatInformation& satxyz);
	//电离层改正
	void IONCorr(double gtime_t, struct ObsHead& re_postion, SatInformation& SatPosTemp, Navhead& nav);
	//对流层改正
	void TropCorr(struct ObsHead& re_postion, SatInformation& SatPosTemp);
	//地球自转改正
	void RotaCorr(GPSTIME& rec_t, GPSTIME& emi_t, SatInformation& SatPosTemp, GNSSSatPos& SingleTemp);
	void calsatclk(const struct GPSTIME* time, NavRecord& BestN, SatInformation& SatPosTemp,double&ES);
private:

};



/*************************SPP解算**********************/
class SPP
{
public:
	SPP();
	~SPP();
	//迭代计算卫星发射时刻
	void CalEmitTSATpos(NavRecord& BestN, GPSTIME& rec_t, GPSTIME& emi_t, SatInformation& SatPosTemp,
		GNSSSatPos& SingleTemp, Descartes& RecPos, double& Es, const int& PosSys);//参数传引用
	void CALBDSatPos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es);
	void CALGPSSatpos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es);
	void CALRecPos(vector<SatInformation>& vecSatPos, Descartes& car_GetRecPos, double vx[], const int& PosSys, const int& weight);
	void CALepochSTA(ObsFile& ofile, vector<NavRecord>& vecNrec_epo, ASatPosInOneEph& SingleSyssatEpoch,
		Descartes& RecPos, int i, vector<SatInformation>& vecSatPos, ofstream& out1, const int& PosSys, const int& weight, const int& iepoch);
	void SinglePP(ObsFile& ofile, vector<NavRecord>& vecNrec_epo, vector<ASatPosInOneEph>& vecStorSatPosEveEpo, vector<Descartes>& vecStorRecPos, ofstream& out1, const int& PosSys, const int& weight);

private:

};
