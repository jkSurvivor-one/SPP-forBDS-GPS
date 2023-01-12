#include "ALLHead.h"
#include<Eigen/Dense>
#include<iostream>
using namespace Eigen;
using namespace std;

double  dot (const double* a, const double* b, int n)
{
	double c = 0.0;

	while (--n >= 0) c += a[n] * b[n];
	return c;
}
void Correction::CALazel(struct ObsHead& re_postion, struct SatInformation& satxyz)//要将数据传出来要么用指针要么用数组
{

	double staBLH[3];								//测站的大地经纬度和高程  sta测站   sat卫星

	double Trans[9];								//声明转置矩阵
	double satsta[3];								//测站与卫星的连线
	double satxyz1[3];								//卫星在站心坐标系的坐标

	Coorsys coorcovert;
	coorcovert.XYZ2BLH(re_postion.apporx_position, staBLH);

	staBLH[0] = staBLH[0] * PI / 180.0;					//将纬度的单位转为弧度rad
	staBLH[1] = staBLH[1] * PI / 180.0;					//将经度的单位转为弧度rad
	/*****xyz2enu******/

	Trans[0] = -sin(staBLH[1]);
	Trans[1] = cos(staBLH[1]);
	Trans[2] = 0;
	Trans[3] = -sin(staBLH[0]) * cos(staBLH[1]);
	Trans[4] = -sin(staBLH[0]) * sin(staBLH[1]);
	Trans[5] = cos(staBLH[0]);
	Trans[6] = cos(staBLH[0]) * cos(staBLH[1]);
	Trans[7] = cos(staBLH[0]) * sin(staBLH[1]);
	Trans[8] = sin(staBLH[0]);

	satsta[0] = satxyz.X - re_postion.apporx_position[0];
	satsta[1] = satxyz.Y - re_postion.apporx_position[1];
	satsta[2] = satxyz.Z - re_postion.apporx_position[2];
	/*********转为enu*********/
	typedef Matrix<double, 3, 3> Matrix33d;
	typedef Matrix<double, 3, 1> Matrix31d;
	Matrix33d Tran = Matrix33d::Zero(3, 3);
	Tran(0, 0) = Trans[0]; Tran(0, 1) = Trans[1]; Tran(0, 2) = Trans[2];
	Tran(1, 0) = Trans[3]; Tran(1, 1) = Trans[4]; Tran(1, 2) = Trans[5];
	Tran(2, 0) = Trans[6]; Tran(2, 1) = Trans[7]; Tran(2, 2) = Trans[8];
	Matrix31d satst = Matrix31d::Zero(3, 1);
	satst(0, 0) = satsta[0];
	satst(1, 0) = satsta[1];
	satst(2, 0) = satsta[2];
	Matrix31d result = Matrix31d::Zero(3, 1);
	result = Tran * satst;
	/**求角度值*/
	satxyz1[0] = result(0, 0); satxyz1[1] = result(1, 0); satxyz1[2] = result(2, 0);
	satxyz.azel[1] = asin(satxyz1[2] / (sqrt(satxyz1[0] * satxyz1[0] + satxyz1[1] * satxyz1[1] + satxyz1[2] * satxyz1[2])));		//高度角，rad     /
	satxyz.azel[0] = dot(satxyz1, satxyz1, 2) < 1E-12 ? 0.0 : atan2(satxyz1[0], satxyz1[1]);
	//satxyz->azel[0] = atan2(satxyz1[1], satxyz1[0]);											//方位角，rad
	if (satxyz.azel[0] < 0)
		satxyz.azel[0] = satxyz.azel[0] + PI * 2;
}
void Correction::RotaCorr(GPSTIME& rec_t, GPSTIME& emi_t, SatInformation& SatPosTemp,GNSSSatPos& SingleTemp)
{
	//在epoch函数中的for循环前将vecOrec[i]赋值给rec_time
	double Tau = rec_t.sow - emi_t.sow;//信号传播时间
	double Omega = OMGE_WGS * Tau;
	SingleTemp.X = cos(Omega) * SatPosTemp.X + sin(Omega) * SatPosTemp.Y;
	SingleTemp.Y = cos(Omega) * SatPosTemp.Y - sin(Omega) * SatPosTemp.X;
	SingleTemp.Z = SatPosTemp.Z;
}
void Correction::IONCorr( double gtime_t,struct ObsHead& re_postion,SatInformation& SatPosTemp, Navhead& nav)
{
	double BLH[3];
	Coorsys coorcovert;
	coorcovert.XYZ2BLH(re_postion.apporx_position, BLH);
	/*azel[0] = 142.6;第二颗卫星
	azel[1] = 20.55;*/
	//azel[0] = azel[0] * PI / 180;
	//azel[1] = azel[1] * PI / 180;//转弧度(不用转弧度，上一步输入直接就是弧度）
	BLH[0] = BLH[0] * PI / 180;
	BLH[1] = BLH[1] * PI / 180;
	double tt, f, psi, phi, lami, amp, per, x;//pi弧度等于半圆；半圆=弧度/pi   2pi弧度为一个整圆的
	psi = 0.0137 / (SatPosTemp.azel[1] / PI + 0.11) - 0.022;//ψ半圆
	phi = BLH[0] / PI + psi * cos(SatPosTemp.azel[0]);//ϕ u + ψcosA   单位为半圆，   cos及sin中角度只能是弧度
	if (phi > 0.416) phi = 0.416;
	else if (phi < -0.416) phi = -0.416;//无需对第一个不变的情况赋值
	lami = BLH[1] / PI + psi * sin(SatPosTemp.azel[0]) / cos(phi * PI);
	phi += 0.064 * cos((lami - 1.617) * PI); //地磁纬度
	tt = 43200.0 * lami + gtime_t;//tt计算公式中的t
	tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */
	f = 1.0 + 16.0 * pow(0.53 - SatPosTemp.azel[1] / PI, 3.0);
	if(SatPosTemp.SYS==1)//北斗
	{ 
		amp = nav.bdionA1 + phi * (nav.bdionA2 + phi * (nav.bdionA3 + phi * nav.bdionA4));//振幅
		per = nav.bdionB1 + phi * (nav.bdionB2 + phi * (nav.bdionB3 + phi * nav.bdionB4));//依次去括号就完成次方运算//周期
	}
	else if (SatPosTemp.SYS == 2)//GPS
	{
		amp = nav.ionA1 + phi * (nav.ionA2 + phi * (nav.ionA3 + phi * nav.ionA4));//振幅
		per = nav.ionB1 + phi * (nav.ionB2 + phi * (nav.ionB3 + phi * nav.ionB4));//依次去括号就完成次方运算//周期
	}
	amp = amp < 0.0 ? 0.0 : amp;
	per = per < 72000.0 ? 72000.0 : per;
	x = 2.0 * PI * (tt - 50400.0) / per;
	//printf("电离层改正为：%lf\n", CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9));
	SatPosTemp.diono=CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);//双目运算符判断 pi/2=1.570,乘光速得距离
}
void Correction ::TropCorr (struct ObsHead& re_postion, SatInformation& SatPosTemp)//对流层改正要用自转改正前的卫星坐标
{
	double BLH[3];
	Coorsys coorcovert;
	coorcovert.XYZ2BLH(re_postion.apporx_position, BLH);
	const double temp0 = 15.0; /* temparature at sea level */

	double hgt, pres, temp, te, z, Tdry, Twet;
	//if (re_postion.apporx_position[2] < -100.0 || 1E4 < re_postion.apporx_position[2] || SatPosTemp.azel[1] <= 0) cout<<"1\n"<<endl;
	hgt = BLH[2] < 0.0 ? 0.0 : BLH[2];//海拔高度
	double hrel = 0.7;//湿分量
	pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);//大气压强（hpa）
	temp = temp0 - 6.5E-3 * hgt + 273.16;//大气绝对温度（k）
	te = 6.108 * hrel * exp((17.15 * temp - 4684.0) / (temp - 38.45));//水汽压力
	z = PI / 2.0 - SatPosTemp.azel[1];//天顶角 
	Tdry = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * BLH[0]) - 0.00028 * hgt / 1E3) / cos(z);//cos(2.0 * pos[0])接收机纬度
	Twet = 0.002277 * (1255.0 / temp + 0.05) * te / cos(z);

	SatPosTemp.dtrop=Tdry + Twet;
}
Correction::Correction()
{
}

Correction::~Correction()
{
}
/*******************
 * Name		: calsatclk
 * Function	: 卫星钟差改正
 * argument : const struct GPSTIME* time   当前观测历元的时间
 *			  NavRecord& BestN			   对应的星历
 *			  SatInformation& SatPosTemp   该卫星的信息结构体（用于存钟差）
 *			  double&ES					   计算的偏近点角
 * return   : none
 *
 *principle ：
 *remark    ：考虑了相对论效应的改正，此处需要计算偏近点角，卫星位置计算也需，是否有重复计算嫌疑,迭代计算卫星位置相差巨大，试用
 *******************/
void Correction::calsatclk(const struct GPSTIME* time, NavRecord& BestN, SatInformation& SatPosTemp,double&ES)
{
	
	double deltaTr;					//相对论效应改正
	double F;

	int i = 0;
	double Mk;						//平近点角(rad)
	double E = 0;					//偏近点角
	double Ek = 0;					//偏近点角
	double N0;						//平均角速度
	double tk;						//输入时间和星历参考时间的差
	double N;

	N0 = sqrt(GM_WGS) / (BestN.sqrtA * BestN.sqrtA * BestN.sqrtA);
	tk = (time->wn - BestN.NGPST.wn) * 604800 + (static_cast <int>(time->sow) - BestN.TOE);//TOD是直接读出来的    通过转换之后TIME会有误差

	N = N0 + BestN.Deltan;			//对平均运动角速度进行改正(rad/s)
	Mk = BestN.M0 + N * tk;			//rad

	do
	{
		E = Ek;
		Ek = Mk + BestN.e * sin(E);
		i++;
	} while ((fabs(Ek - E) > 1e-12));		//偏近点角(rad)
	ES = Ek;
	F = ((-2) * sqrt(1.0 * GM_WGS)) / (CLIGHT * CLIGHT);
	deltaTr = F * BestN.e * sqrt(BestN.sqrtA * BestN.sqrtA) * sin(Ek);//课本P86
	SatPosTemp.SatClk = BestN.a0 + BestN.a1 * tk + BestN.a2 * tk * tk + deltaTr - BestN.TGD;
}