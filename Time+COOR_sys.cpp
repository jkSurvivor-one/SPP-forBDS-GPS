#include "ALLHead.h"



TimeSystem::TimeSystem()
{
}
TimeSystem::~TimeSystem()
{

}
Coorsys::Coorsys()
{
}

Coorsys::~Coorsys()
{
}
//UTC转换到GPS时
void TimeSystem::UTC2GPST(COMMONTIME& comt, GPSTIME& gpst)//该关键字告诉编译器，函数中不会发生异常
{
	double y, m, JD, UT;//年、月、儒略日、通用日
	long week;
	double week_sec;
	UT = comt.hour + comt.minute / 60.0 + comt.second / 3600.0;
	if (comt.month > 2)
	{
		y = comt.year;
		m = comt.month;
	}
	else
	{
		y = comt.year - 1;
		m = comt.month + 12;
	}
	JD = static_cast<int>(365.25 * y) + static_cast<int>(30.6001 * (m + 1)) + comt.day + UT / 24.0 + 1720981.5;//C风格的强制转换也是支持的
	week = static_cast<int>((JD - 2444244.5) / 7.0);
	gpst.wn = week;
	week_sec = (JD - static_cast<double>(7 * week) - 2444244.5) * 86400.0;
	gpst.sow = week_sec;
}
//GPS时转到北斗时
void TimeSystem::GPST2BDT( GPSTIME & gpst, GPSTIME& bdt)//不会标const
{
	bdt.wn = gpst.wn - 1356;;
	bdt.sow = gpst.sow - 14.0;;

	if (bdt.sow <= 0)
	{
		bdt.sow += static_cast<double>(7 * 24 * 3600);//C++样式强制转换
		bdt.wn--;
	}

	if (bdt.sow > 604800)
	{
		bdt.sow -= 604800;
		bdt.wn++;
	}
}

void Coorsys::BLH2XYZ(Geodetic&blh,Descartes &XYZ )//出现不兼容问题主要出在，在头文件中将结构体放在了函数声明后面，无法识别
{
	double e1 = 0;
	e1 = sqrt(A * A - ellipsoidalb * ellipsoidalb) / A;
	double L = blh.L * D2R;//角弧转换
	double B = blh.B * D2R;
	double H = blh.H;
	double N = A / sqrt(1 - e1 * e1 * sin(B) * sin(B));
	XYZ.X = (N + H) * cos(B) * cos(L);
	XYZ.Y = (N + H) * cos(B) * sin(L);
	XYZ.Z = (N * (1 - e1 * e1) + H) * sin(B);
}
void Coorsys::XYZ2BLH(double XYZ[3], double blh[3])
{
	double tmpX = XYZ[0];
	double temY = XYZ[1];
	double temZ = XYZ[2];
	double curB = 0;
	double N = 0;
	double calB = atan2(temZ, sqrt(tmpX * tmpX + temY * temY));
	int counter = 0;
	//printf("B初始值%lf\n", calB * r2d);
	while (fabs(curB - calB) * R2D > PRECISION && counter < 25)
	{
		curB = calB;
		N = A / sqrt(1 - WGSe1 * WGSe1 * sin(curB) * sin(curB));
		calB = atan2(temZ + N * WGSe1 * WGSe1 * sin(curB), sqrt(tmpX * tmpX + temY * temY));
		counter++;
		//printf("B迭代为%lf\n", calB * r2d);
	}
	blh[0] = curB * R2D;//B
	blh[1] = atan2(temY, tmpX) * R2D;//L
	blh[2] = temZ / sin(curB) - N * (1 - WGSe1 * WGSe1);//H
}
