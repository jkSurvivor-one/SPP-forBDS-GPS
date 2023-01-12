#include"ALLHead.h"
#include<Eigen/Dense>

#include<iostream>
#include <iomanip>
#include <fstream>//要将文件流传入函数使用<<运算符，必须要加上头文件才能使用，要传入string还要加上对应的头文件
using namespace Eigen;
using namespace std;
//int PosSys = 2;//系统
//int weight = 2;//权重1等权  2高度角定权
SPP::SPP()
{
}

SPP::~SPP()
{
}
/*******************
 * Name		: CalEmitTSATpos
 * Function	: 迭代计算卫星位置和卫星钟差
 * argument : NavRecord& BestN   与当前观测文件卫星最接近的卫星星历
 *			  GPSTIME& rec_t     观测文件中给出的观测时刻
 *			  SatInformation& SatPosTemp  卫星信息文件，用于存储调用卫星位置计算得到的卫星坐标
 *			  GNSSSatPos& SingleTemp 临时存储卫星位置，用于计算其他值：几何距离等
 *			  Descartes& RecPos      接收机位置，传入近似接收机坐标
 *			  double& Es             传出该卫星偏近点角
 * return   : none
 *
 *principle ：
 *remark    ：每次计算一颗卫星
 *******************/
void SPP::CalEmitTSATpos(NavRecord& BestN, GPSTIME& rec_t, GPSTIME& emi_t, SatInformation& SatPosTemp,GNSSSatPos& SingleTemp, Descartes& RecPos, double& Es,const int& PosSys)
{
	GPSTIME   emi_t0{ 99 };           //计算卫星信号发射时刻迭代过度变量
	emi_t0.wn = rec_t.wn;
	emi_t.wn = rec_t.wn;              //卫星信号发射时刻
	double L_RecSat{ 99.0 };          //定义站星距,几何距离( geometric range)

	//卫星到测站的信号传播时间大约位0.07s
	/*迭代法计算卫星信号发射时间*/
	emi_t.sow = rec_t.sow - 0.075;
	do {
		emi_t0.sow = emi_t.sow;

		if (PosSys == 1) {
			CALBDSatPos(BestN, emi_t0, SatPosTemp, Es);
		}
		else if (PosSys == 2) {
			CALGPSSatpos(BestN, emi_t0, SatPosTemp, Es);
		}
		else if (PosSys == 3) {
			if (BestN.sprn.SYS == 1) {
				CALBDSatPos(BestN, emi_t0, SatPosTemp, Es);
			}
			if (BestN.sprn.SYS == 2) {
				CALGPSSatpos(BestN, emi_t0, SatPosTemp, Es);
			}
		}
		Correction BDS;
		BDS.RotaCorr(rec_t, emi_t, SatPosTemp, SingleTemp);
		L_RecSat = sqrt(pow((SingleTemp.X - RecPos.X), 2) + pow((SingleTemp.Y - RecPos.Y), 2) +
			pow((SingleTemp.Z - RecPos.Z), 2));
		emi_t.sow = rec_t.sow - L_RecSat / CLIGHT;
		//在进行地球自转之前与结果相同，自转改正后结果不正确
	} while (fabs(emi_t0.sow - emi_t.sow) > PRECISION);//1e-12
}

/*******************
 * Name		: CALBDSatPos
 * Function	: 计算北斗卫星位置
 * argument : NavRecord& BestN      与当前观测文件卫星最接近的卫星星历
 *			  GPSTIME emi_t         信号发射时刻
 *			  SatInformation& SatPosTemp  卫星的各信息（坐标存储）
 *			  double& Es            放计算出的偏近点角，给其他函数用
 *
 * return   : none
 *
 *principle ：
 *remark    ：北斗主要需要分为两种计算方法
 *******************/
void SPP::CALBDSatPos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es)
{
	int i = 0;
	double n0, n;

	//1.计算卫星运动的平均角速度n
	double a;
	a = pow(BestN.sqrtA, 2);
	n0 = sqrt(GM_CGCS / pow(a, 3));
	n = n0 + BestN.Deltan;

	//2.计算归化时间tk，观测时刻周内秒-参考时刻周内秒                          
	double tk;
	GPSTIME BDemi_t;
	TimeSystem timesys;
	timesys.GPST2BDT(emi_t, BDemi_t);
	tk = (BDemi_t.wn * WEEK_IN_SECOND + BDemi_t.sow) - (BestN.WEEK * WEEK_IN_SECOND + BestN.TOE);
	if (tk < -302400.0)
	{
		tk = tk + WEEK_IN_SECOND;
	}
	else if (tk > 302400.0)
	{
		tk = tk - WEEK_IN_SECOND;
	}
	else
	{
		tk = tk;
	}

	//3.计算观测瞬间卫星的平近点角M
	double M;
	M = BestN.M0 + n * tk;

	//4.计算偏近点角Es      外部已计算
	double E0;
	double E;
	E = M;
	do
	{
		E0 = E;
		E = M + BestN.e * sin(E0);
	} while (fabs(E0 - E) > 1e-12);
	Es = E;

	//5.计算真近点角f
	double f;
	f = atan2((sqrt(1 - pow(BestN.e, 2)) * sin(Es)), (cos(Es) - BestN.e));

	//6.计算升交角距miu
	double miu;
	miu = f + BestN.Omega;

	//7.计算摄动改正项
	double Delta_u;
	double Delta_r;
	double Delta_i;
	Delta_u = BestN.Cuc * cos(2 * miu) + BestN.Cus * sin(2 * miu);
	Delta_r = BestN.Crc * cos(2 * miu) + BestN.Crs * sin(2 * miu);
	Delta_i = BestN.Cic * cos(2 * miu) + BestN.Cis * sin(2 * miu);

	//8.进行摄动改正
	double cor_u;      //升交角距
	double cor_r;      //卫星矢径
	double cor_i;      //轨道倾角
	cor_u = miu + Delta_u;
	cor_r = fabs((BestN.sqrtA * BestN.sqrtA) * (1 - BestN.e * cos(Es)) + Delta_r);
	cor_i = BestN.i0 + BestN.IDOT * tk + Delta_i;

	//9.计算卫星在轨道面坐标系中的位置
	double xSatOrb;
	double ySatOrb;
	xSatOrb = cor_r * cos(cor_u);
	ySatOrb = cor_r * sin(cor_u);

	//判断卫星类型：GEO,IGSO,MEO
	int j = 0;
	for (i = 0; i < 8; i++)
	{
		if (BestN.sprn.PRN == GEO_prn[i])
			j = 1;
	}

	if (!j)
	{
		//10.计算改正后的升交点经Ls
		double Ls;
		Ls = BestN.Omeca + (BestN.OmegaDot - OMGE_CGCS) * tk - OMGE_CGCS * BestN.TOE;

		//11.计算卫星在瞬时地球坐标系中的位置
		double x_ECEF;
		double y_ECEF;
		double z_ECEF;
		x_ECEF = xSatOrb * cos(Ls) - ySatOrb * cos(cor_i) * sin(Ls);
		y_ECEF = xSatOrb * sin(Ls) + ySatOrb * cos(cor_i) * cos(Ls);
		z_ECEF = ySatOrb * sin(cor_i);

		//存储坐标
		SatPosTemp.X = x_ECEF;
		SatPosTemp.Y = y_ECEF;
		SatPosTemp.Z = z_ECEF;
	}
	else//计算GEO卫星，采用坐标旋转发拟合广播星历轨道参数//参考文献：北斗导航卫星位置计算方法研究
	{
		//10.计算改正后的升交点经Ls（静轨不考虑地球自转）
		double Ls;
		Ls = BestN.Omeca + BestN.OmegaDot * tk - OMGE_CGCS * BestN.TOE;//惯性坐标系

		//11.计算GEO卫星在自定义坐标系下的位置
		typedef Matrix<double, 3, 1> Matrix31d;
		Matrix31d GK = Matrix31d::Zero(3, 1);//定义个3X1的矩阵并初始化
		GK(0, 0) = xSatOrb * cos(Ls) - ySatOrb * cos(cor_i) * sin(Ls);
		GK(1, 0) = xSatOrb * sin(Ls) + ySatOrb * cos(cor_i) * cos(Ls);
		GK(2, 0) = ySatOrb * sin(cor_i);

		//11.计算GEO卫星在瞬时地球坐标系中的位置
		//typedef Matrix<double, 3, 3> Matrix3d;
		MatrixXd Rz = MatrixXd::Zero(3, 3);//定义一个3X3的矩阵并初始化
		MatrixXd Rx = MatrixXd::Zero(3, 3);
		typedef Matrix<double, 3, 1> Matrix31d;
		Matrix31d ECEF = Matrix31d::Zero(3, 1);

		Rz(0, 0) = cos(OMGE_CGCS * tk);
		Rz(0, 1) = sin(OMGE_CGCS * tk);
		Rz(1, 0) = -sin(OMGE_CGCS * tk);
		Rz(1, 1) = cos(OMGE_CGCS * tk);
		Rz(2, 2) = 1.0;

		Rx(0, 0) = 1.0;
		Rx(1, 1) = cos(-5.0 * D2R);
		Rx(1, 2) = sin(-5.0 * D2R);
		Rx(2, 1) = -sin(-5.0 * D2R);
		Rx(2, 2) = cos(-5.0 * D2R);

		ECEF = Rz * Rx * GK;
		//存储坐标
		SatPosTemp.X = ECEF(0, 0);
		SatPosTemp.Y = ECEF(1, 0);
		SatPosTemp.Z = ECEF(2, 0);
	}
}
/*******************
 * Name		: CALGPSatPos
 * Function	: 计算GPS卫星位置
 * argument : NavRecord& BestN      与当前观测文件卫星最接近的卫星星历
 *			  GPSTIME emi_t         信号发射时刻
 *			  SatInformation& SatPosTemp  卫星的各信息（坐标存储）
 *			  double& Es            放计算出的偏近点角，给其他函数用
 *
 * return   : none
 *
 *principle ：
 *remark    ：
 *******************/
void SPP::CALGPSSatpos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es)
{
	int i = 0;
	double n0, n;

	//1.计算卫星运行的平均角速度n
	double a;
	a = pow(BestN.sqrtA, 2);
	n0 = sqrt(GM_WGS / pow(a, 3));//pow()函数：计算x的y次幂
	n = n0 + BestN.Deltan;

	//2.计算归化时间tk，观测时刻周内秒-参考时刻周内秒                          
	double tk;
	tk = (emi_t.wn * WEEK_IN_SECOND + emi_t.sow) - (BestN.WEEK * WEEK_IN_SECOND + BestN.TOE);
	if (tk < -302400.0)
	{
		tk = tk + WEEK_IN_SECOND;
	}
	else if (tk > 302400.0)
	{
		tk = tk - WEEK_IN_SECOND;
	}
	else
	{
		tk = tk;
	}

	//3.计算信号发射时刻卫星的平近点角M
	double M;
	M = BestN.M0 + n * tk;

	//4.迭代计算偏近点角Es
	double E0;
	double E;
	E = M;
	do
	{
		E0 = E;
		E = M + BestN.e * sin(E0);//用弧度表示的开普勒方程
	} while (fabs(E0 - E) > 1E-10);
	Es = E;//传出去

	//5.计算真近点角f
	double f;
	f = atan2((sqrt(1 - pow(BestN.e, 2)) * sin(E)), (cos(E) - BestN.e));

	//6.计算升交角距miu
	double miu;
	miu = f + BestN.Omega;

	//7.计算摄动改正项
	double Delta_u;
	double Delta_r;
	double Delta_i;
	Delta_u = BestN.Cuc * cos(2 * miu) + BestN.Cus * sin(2 * miu);
	Delta_r = BestN.Crc * cos(2 * miu) + BestN.Crs * sin(2 * miu);
	Delta_i = BestN.Cic * cos(2 * miu) + BestN.Cis * sin(2 * miu);

	//8.进行摄动改正
	double cor_u;      //升交角距
	double cor_r;      //卫星矢径
	double cor_i;      //轨道倾角
	cor_u = miu + Delta_u;
	cor_r = fabs((BestN.sqrtA * BestN.sqrtA) * (1 - BestN.e * cos(E)) + Delta_r);
	cor_i = BestN.i0 + BestN.IDOT * tk + Delta_i;

	//9.计算卫星在轨道面坐标系中的位置
	double xSatOrb;
	double ySatOrb;
	xSatOrb = cor_r * cos(cor_u);
	ySatOrb = cor_r * sin(cor_u);

	//10.计算改正后的升交点的经度Ls
	double Ls;
	Ls = BestN.Omeca + (BestN.OmegaDot - OMGE_WGS) * tk - OMGE_WGS * BestN.TOE;

	//11.计算卫星在瞬时地球坐标系中的位置
	double x_ECEF;
	double y_ECEF;
	double z_ECEF;
	x_ECEF = xSatOrb * cos(Ls) - ySatOrb * cos(cor_i) * sin(Ls);
	y_ECEF = xSatOrb * sin(Ls) + ySatOrb * cos(cor_i) * cos(Ls);
	z_ECEF = ySatOrb * sin(cor_i);

	//存储坐标
	SatPosTemp.X = x_ECEF;
	SatPosTemp.Y = y_ECEF;
	SatPosTemp.Z = z_ECEF;
}

/*******************
 * Name		: CALRecPos
 * Function	: 组误差方程及解算接收机坐标
 * argument : vector<SatInformation>& vecSatPos   当前历元的卫星信息（包括各种误差，高度角等）容器
 *			  Descartes& car_GetRecPos      笛卡尔坐标系结构体用于存储解算出的接收机坐标
 *			  double vx[]					用于存放解算出来的VX4个改正值
 *
 * return   : none
 *
 *principle ：通过设置的POSSYS提前设置定位模式，组误差方程计算出接收机位置，该函数只计算出对应的改正值，不判断是否达到要求的限差值。
 *remark    ：北斗与GPS的误差模式相同，其时间、坐标系统的不同在计算卫星位置时已设定好，
 *******************/
void SPP::CALRecPos(vector<SatInformation>& vecSatPos, Descartes& car_GetRecPos, double vx[], const int& PosSys, const int& weight)
{
	double a = 0.004, b = 0.003;                        //高度角定权模型相关经验值

	if (PosSys == 1 || PosSys == 2) {
		unsigned int N = vecSatPos.size();                  //该历元内卫星可用数
		double vt = 0;                                      //接收机钟差平差值

		/*定义所用矩阵并初始化*/
		typedef Matrix<double, Dynamic, 4> MatrixX4d;
		MatrixX4d A = MatrixX4d::Zero(N, 4);
		typedef Matrix<double, 4, Dynamic> Matrix4Xd;
		Matrix4Xd AT = Matrix4Xd::Zero(4, N);
		Matrix4Xd ATP = Matrix4Xd::Zero(4, N);
		typedef Matrix<double, 4, 4> Matrix4d;
		Matrix4d ATPA = Matrix4d::Zero(4, 4);
		Matrix4d NBB = Matrix4d::Zero(4, 4);
		typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
		MatrixXd P = MatrixXd::Zero(N, N);
		typedef Matrix<double, 4, 1> Matrix41d;
		Matrix41d ATPL = Matrix41d::Zero(4, 1);
		Matrix41d VX = Matrix41d::Zero(4, 1);
		typedef Matrix<double, Dynamic, 1> MatrixX1d;
		MatrixX1d L = MatrixX1d::Zero(N, 1);           //余数项
		MatrixX1d S = MatrixX1d::Zero(N, 1);           //站星距
		MatrixX1d ll = MatrixX1d::Zero(N, 1);          //方向余弦
		MatrixX1d mm = MatrixX1d::Zero(N, 1);          //方向余弦
		MatrixX1d nn = MatrixX1d::Zero(N, 1);          //方向余弦
		MatrixX1d psr = MatrixX1d::Zero(N, 1);         //无电离层组合观测值
		MatrixX1d dclk = MatrixX1d::Zero(N, 1);        //卫星钟差
		MatrixX1d dtrop = MatrixX1d::Zero(N, 1);       //对流层延迟改正
		MatrixX1d Elev = MatrixX1d::Zero(N, 1);        //卫星高度角

		/*利用该历元内的j颗卫星，解出接收机坐标*/
		for (unsigned int j = 0; j < N; j++)
		{
			/*几何距离*/
			S(j, 0) = sqrt(pow((vecSatPos[j].X - car_GetRecPos.X), 2) +
				pow((vecSatPos[j].Y - car_GetRecPos.Y), 2) +
				pow((vecSatPos[j].Z - car_GetRecPos.Z), 2));
			/*伪距钟差等分别放入矩阵*/
			psr(j, 0) = vecSatPos[j].psr;//无电离层延迟非差改正
			dclk(j, 0) = vecSatPos[j].SatClk;
			dtrop(j, 0) = vecSatPos[j].dtrop;

			/*求卫星j在观测方程中的余数项：站星几何距-消电离层后的伪距psr-卫星钟差改正+对流层延迟改正*/
			L(j, 0) = S(j, 0) - psr(j, 0) - CLIGHT * dclk(j, 0) + dtrop(j, 0);

			/*求卫星j的方向余弦*/
			ll(j, 0) = (vecSatPos[j].X - car_GetRecPos.X) / S(j, 0);
			mm(j, 0) = (vecSatPos[j].Y - car_GetRecPos.Y) / S(j, 0);
			nn(j, 0) = (vecSatPos[j].Z - car_GetRecPos.Z) / S(j, 0);

			A(j, 0) = ll(j, 0);
			A(j, 1) = mm(j, 0);
			A(j, 2) = nn(j, 0);
			A(j, 3) = -1;

			/*定权*/
			if (weight == 1) {
				P(j, j) = 1;//等权
			}
			if (weight == 2) {
				/*高度角定权：改进的正弦函数模型*/
				Elev(j, 0) = vecSatPos[j].azel[1];
				P(j, j) = 1.0 / (a * a + b * b / pow(sin(Elev(j, 0)), 2));
			}
		}
		/*间接平差*/
		AT = A.transpose();  //矩阵的转置
		ATP = AT * P;
		ATPA = ATP * A;
		NBB = ATPA.inverse();//矩阵的求逆
		ATP = AT * P;
		ATPL = ATP * L;
		VX = NBB * ATPL;

		car_GetRecPos.X = car_GetRecPos.X + VX(0, 0);
		car_GetRecPos.Y = car_GetRecPos.Y + VX(1, 0);
		car_GetRecPos.Z = car_GetRecPos.Z + VX(2, 0);

		vx[0] = VX(0, 0);
		vx[1] = VX(1, 0);
		vx[2] = VX(2, 0);
		vx[3] = VX(3, 0);
		vt = vx[3];

		if ((vx[0] * vx[0] + vx[1] * vx[1] + vx[2] * vx[2]) > 1.0e-10) {
			vecSatPos.clear();//删除 vector 容器中所有的元素，使其变成空的 vector 容器。该函数会改变 vector 的大小（变为 0），但不是改变其容量。
			//清除这个容器中的数据，即清除当前历元的数据
		}
	}

	if (PosSys == 3) {
		unsigned int N = vecSatPos.size();                  //该历元内卫星可用数
		double vt = 0;                                      //接收机钟差平差值
		double vt_sys;

		/*定义所用矩阵并初始化*/
		typedef Matrix<double, Dynamic, 5> MatrixX5d;
		MatrixX5d A = MatrixX5d::Zero(N, 5);
		typedef Matrix<double, 5, Dynamic> Matrix5Xd;
		Matrix5Xd AT = Matrix5Xd::Zero(5, N);
		Matrix5Xd ATP = Matrix5Xd::Zero(5, N);
		typedef Matrix<double, 5, 5> Matrix5d;
		Matrix5d ATPA = Matrix5d::Zero(5, 5);
		Matrix5d NBB = Matrix5d::Zero(5, 5);
		typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
		MatrixXd P = MatrixXd::Zero(N, N);
		typedef Matrix<double, 5, 1> Matrix51d;
		Matrix51d ATPL = Matrix51d::Zero(5, 1);
		Matrix51d VX = Matrix51d::Zero(5, 1);
		typedef Matrix<double, Dynamic, 1> MatrixX1d;
		MatrixX1d L = MatrixX1d::Zero(N, 1);           //余数项
		MatrixX1d S = MatrixX1d::Zero(N, 1);           //站星距
		MatrixX1d ll = MatrixX1d::Zero(N, 1);          //方向余弦
		MatrixX1d mm = MatrixX1d::Zero(N, 1);          //方向余弦
		MatrixX1d nn = MatrixX1d::Zero(N, 1);          //方向余弦
		MatrixX1d psr = MatrixX1d::Zero(N, 1);         //无电离层组合观测值
		MatrixX1d dclk = MatrixX1d::Zero(N, 1);        //卫星钟差
		MatrixX1d dtrop = MatrixX1d::Zero(N, 1);       //对流层延迟改正
		MatrixX1d Elev = MatrixX1d::Zero(N, 1);        //卫星高度角

		/*利用该历元内的j颗卫星，解出接收机坐标*/
		for (unsigned int j = 0; j < N; j++)
		{
			S(j, 0) = sqrt(pow((vecSatPos[j].X - car_GetRecPos.X), 2) +
				pow((vecSatPos[j].Y - car_GetRecPos.Y), 2) +
				pow((vecSatPos[j].Z - car_GetRecPos.Z), 2));

			psr(j, 0) = vecSatPos[j].psr;
			dclk(j, 0) = vecSatPos[j].SatClk;
			dtrop(j, 0) = vecSatPos[j].dtrop;

			/*求卫星j在观测方程中的余数项：站星几何距-消电离层后的伪距psr-卫星钟差改正+对流层延迟改正*/
			L(j, 0) = S(j, 0) - psr(j, 0) - CLIGHT * dclk(j, 0) + dtrop(j, 0);

			/*求卫星j的方向余弦*/
			ll(j, 0) = (vecSatPos[j].X - car_GetRecPos.X) / S(j, 0);
			mm(j, 0) = (vecSatPos[j].Y - car_GetRecPos.Y) / S(j, 0);
			nn(j, 0) = (vecSatPos[j].Z - car_GetRecPos.Z) / S(j, 0);

			A(j, 0) = ll(j, 0);
			A(j, 1) = mm(j, 0);
			A(j, 2) = nn(j, 0);

			if (vecSatPos[j].SYS == 1) {//北斗和GPS钟差不同需要分开求解
				A(j, 3) = 0;
				A(j, 4) = -1;
			}

			if (vecSatPos[j].SYS == 2) {
				A(j, 3) = -1;
				A(j, 4) = 0;
			}

			/*权阵*/
			if (weight == 1) {
				P(j, j) = 1;//等权
			}

			if (weight == 2) {
				/*高度角定权：改进的正弦函数模型*/
				Elev(j, 0) = vecSatPos[j].azel[1];
				P(j, j) = 1.0 / (a * a + b * b / pow(sin(Elev(j, 0)), 2));
			}

		}

		/*间接平差*/
		AT = A.transpose();  //矩阵的转置
		ATP = AT * P;
		ATPA = ATP * A;
		NBB = ATPA.inverse();//矩阵的求逆
		ATP = AT * P;
		ATPL = ATP * L;
		VX = NBB * ATPL;

		car_GetRecPos.X = car_GetRecPos.X + VX(0, 0);
		car_GetRecPos.Y = car_GetRecPos.Y + VX(1, 0);
		car_GetRecPos.Z = car_GetRecPos.Z + VX(2, 0);

		vx[0] = VX(0, 0);
		vx[1] = VX(1, 0);
		vx[2] = VX(2, 0);
		vx[3] = VX(3, 0);
		vx[4] = VX(4, 0);

		vt = vx[3];
		vt_sys = vx[4];

		if ((vx[0] * vx[0] + vx[1] * vx[1] + vx[2] * vx[2]) > 1.0e-10) {
			vecSatPos.clear();//clear()	删除 vector 容器中所有的元素，使其变成空的 vector 容器。该函数会改变 vector 的大小（变为 0），但不是改变其容量。sawp释放内存
		}
	}

}
/**********
* 名称：CALepochSTA
* 功能：计算单历元的接收机坐标（station position)
* 参数：ObsFile& ofile  观测值文件 包括头文件信息及观测值   引用类型
		vector<SatInformation>& vecSatPos   各卫星的其他信息
*		vector<NavRecord>& vecNrec_epo   导航电文文件类型容器   引用类型
*		 ASatPosInOneEph& SingleSyssatEpoch   当前历元的所有卫星位置
*		Descartes& RecPos   笛卡尔坐标系的引用类型  用于通过容器存储解算出来的接收机坐标
*		int i				用于判断第几历元
*返回值：无返回值
* 注：PosSys 选项代表计算系统的选择1BDS 2GPS 3BDS&&GPS
***/
void SPP::CALepochSTA(ObsFile& ofile, vector<NavRecord>& vecNrec_epo, ASatPosInOneEph& SingleSyssatEpoch,Descartes& RecPos, int i, vector<SatInformation>& vecSatPos, ofstream& out1, const int& PosSys, const int& weight,const int & iepoch )
{
	GPSTIME rec_t = { };                         //历元已经转化成GPS时，如果需要则转化成BD时
	Correction epochcorr;						//为改正值创建对象
	SPRN sprn = { };//个成功员初始为0的4种方法struct A = { 0 };struct A = {};memset(&A, 0, sizeof(struct));A = (struct){ 0 } // c99 中 compound literal
	SatInformation SatPosTemp = { };                 //存储单个卫星数据结构体
	NavRecord BestN;                                  //最佳历元所在的导航信息

	double tk = 0;                               //归化时间
	double dRel = 0;                             //相对论改正数


	RecPos.X = ofile.ohdr.apporx_position[0];
	RecPos.Y = ofile.ohdr.apporx_position[1];
	RecPos.Z = ofile.ohdr.apporx_position[2];

	Coorsys coorsys;
	double GeoCoor1[3];
	Geodetic GeoCoor;

	coorsys.XYZ2BLH(ofile.ohdr.apporx_position, GeoCoor1);
	GeoCoor.B = GeoCoor1[0];
	GeoCoor.L = GeoCoor1[1];
	GeoCoor.H = GeoCoor1[2];
	double BBp = GeoCoor.B;
	double LLp = GeoCoor.L;
	double HHp = GeoCoor.H;

	/****************BD******************/
	if (PosSys == 1) {
		double* Delta_V = new double[4]();           //平差误差项// 每个元素初始化为0,括号内不能写其他值，只能初始化为0
		do {
			SingleSyssatEpoch = { };
			GNSSSatPos SingleTemp{ };//该变量每次循环都会被释放内存
			int label = { };                                 //每个卫星初始化一次指数

			rec_t.wn = ofile.vecOrec[i].OGPST.wn;            //观测时间（GPS时）
			rec_t.sow = ofile.vecOrec[i].OGPST.sow;

			if (ofile.vecOrec[i].BDPRN->SYS == 1) {
				for (int j = 0; j != ofile.vecOrec[i].BDSatNum; j++)//在第i个历元,遍历所有北斗卫星
				{
					//第i历元内，剔除不同时具有双频伪距或伪距之差大于50m的观测值
					if (ofile.vecOrec[i].BDvalue[j][0] == 0 ||
						ofile.vecOrec[i].BDvalue[j][1] == 0 ||
						fabs(ofile.vecOrec[i].BDvalue[j][0] - ofile.vecOrec[i].BDvalue[j][1]) > 50) {
						continue;
					}
					sprn.PRN = ofile.vecOrec[i].BDPRN[j].PRN;//将卫星信息存入sprn结构中
					sprn.SYS = ofile.vecOrec[i].BDPRN[j].SYS;

					SatPosTemp.PRN = sprn.PRN;//将卫星信息存入SatInfoNow结构体中
					SatPosTemp.SYS = sprn.SYS;

					SatPosTemp.codeC1 = ofile.vecOrec[i].BDvalue[j][0];
					SatPosTemp.codeP2 = ofile.vecOrec[i].BDvalue[j][1];

					SingleTemp.PRN = sprn.PRN;
					SingleTemp.SYS = sprn.SYS;

					CFileRead fileread;
					int mark=0;
					fileread.GetBestNav(ofile.vecOrec[i], sprn, vecNrec_epo, label,mark);
					if (mark==1)
					{
						continue;
					}
					BestN = vecNrec_epo[label];

					GPSTIME emi_t = { }, BDemi_t;

					double     Es = 0;                             //偏近点角
					CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos,Es, PosSys);
					////spp.calsatclk();
					epochcorr.CALazel(ofile.ohdr, SatPosTemp);//计算高度角
					//CALBDSatPos(BestN,emi_t, SatPosTemp, Es);
					
					/*电离层延迟改正：双频改正模型*///or kl（Correction 对象中）
					if (SatPosTemp.azel[1] >= 10 * D2R) //剔除低高度角卫星(弧度）
					{
						/*相对论效应的改正*/
						dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

						TimeSystem timesys;
						timesys.GPST2BDT(emi_t, BDemi_t);
						/*卫星钟差：多项式模型（考虑相对论效应）*/
						SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (BDemi_t.sow - BestN.TOE)
							+ BestN.a2 * (pow((BDemi_t.sow - BestN.TOE), 2)) + dRel;//单频的加上- BestN.TGD

						/*对流层延迟改正：Saastamoinenmodel*/
						epochcorr.TropCorr(ofile.ohdr, SatPosTemp);//数组越界
						/*求无电离层延迟的线性组合观测值*/
						SatPosTemp.psr = (B1_2 * B1_2 * ofile.vecOrec[i].BDvalue[j][0]
							- B3 * B3 * ofile.vecOrec[i].BDvalue[j][1]) / (B1_2 * B1_2 - B3 * B3)
							- CLIGHT * B1_2 * B1_2 * BestN.TGD / (B1_2 * B1_2 - B3 * B3);//
						SatPosTemp.PRN = ofile.vecOrec[i].BDPRN[j].PRN;
						SatPosTemp.SYS = ofile.vecOrec[i].BDPRN[j].SYS;

						SatPosTemp.X = SingleTemp.X;
						SatPosTemp.Y = SingleTemp.Y;
						SatPosTemp.Z = SingleTemp.Z;
						//暂存一组观测值的卫星坐标
						vecSatPos.push_back(SatPosTemp);
						//cout << "BDS" << SatPosTemp.X <<"\t" << SatPosTemp.Y <<"\t"<< SatPosTemp.Z << endl;
						//赋值当前解算历元卫星坐标计算结果
						out1  << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2) << ofile.vecOrec[i].epoch.second <<
							setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Y << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0] * R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
						SingleSyssatEpoch.vecABDSPosInOneEph.push_back(SingleTemp);
					}
				}//for循环
			}

			SingleSyssatEpoch.BDSatNumEph = SingleSyssatEpoch.vecABDSPosInOneEph.size();//记录当前历元观测的卫星总数
			//个数大于4个开始解算接收机坐标
			if (vecSatPos.size() >= 4) {
				CALRecPos(vecSatPos, RecPos, Delta_V, PosSys,  weight);
			}
			else {
				break;
			}

		} while ((Delta_V[0] * Delta_V[0] + Delta_V[1] * Delta_V[1] + Delta_V[2] * Delta_V[2]) > PRECISION);
		//迭代计算前后两次接收机位置小于1.0e-10
	}

	/***************GPS*****************/
	if (PosSys == 2) {
		double* Delta_V = new double[4]();           //平差误差项
		do {
			SingleSyssatEpoch = { };                              //进入下一历元前归零
			int label = { };                                    //每个卫星初始化一次指数

			rec_t.wn = ofile.vecOrec[i].OGPST.wn;
			rec_t.sow = ofile.vecOrec[i].OGPST.sow;

			GNSSSatPos SingleTemp{ 99 };

			if (ofile.vecOrec[i].GPSPRN->SYS == 2) {
				for (int j = 0; j != ofile.vecOrec[i].GPSSatNum; j++)
				{
					if (ofile.vecOrec[i].GPSvalue[j][0] == 0 ||
						ofile.vecOrec[i].GPSvalue[j][1] == 0 ||
						fabs(ofile.vecOrec[i].GPSvalue[j][0] - ofile.vecOrec[i].GPSvalue[j][1]) > 50) {
						continue;
					}
					else {
						sprn.PRN = ofile.vecOrec[i].GPSPRN[j].PRN;
						sprn.SYS = ofile.vecOrec[i].GPSPRN[j].SYS;

						SatPosTemp.PRN = sprn.PRN;
						SatPosTemp.SYS = sprn.SYS;

						SatPosTemp.codeC1 = ofile.vecOrec[i].GPSvalue[j][0];
						SatPosTemp.codeP2 = ofile.vecOrec[i].GPSvalue[j][1];

						SingleTemp.PRN = sprn.PRN;
						SingleTemp.SYS = sprn.SYS;

						CFileRead fileread;
						int mark = 0;
						fileread.GetBestNav(ofile.vecOrec[i], sprn, vecNrec_epo, label,mark);
						if (mark == 1)//星历过期
						{
							continue;
						}
						BestN = vecNrec_epo[label];

						GPSTIME emi_t = { };
						double     Es = 0;                             //偏近点角
						CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos, Es, PosSys);
						Correction spp;
						////spp.calsatclk();
						epochcorr.CALazel(ofile.ohdr, SatPosTemp);//计算高度角
						

						if (SatPosTemp.azel[1] >= 10 * D2R) //卫星截止高度角10°有些历元会错误
						{
							/*相对论效应的改正*/
							dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

							/*卫星钟差：多项式模型*/
							SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (emi_t.sow - BestN.TOE)
								+ BestN.a2 * (pow((emi_t.sow - BestN.TOE), 2)) + dRel;
							/*对流层改正*/
							epochcorr.TropCorr(ofile.ohdr, SatPosTemp);
							/*双频改正模型,求无电离层延迟的线性组合观测值*/
							SatPosTemp.psr = P1 * ofile.vecOrec[i].GPSvalue[j][0]
								- P2 * ofile.vecOrec[i].GPSvalue[j][1];

							SatPosTemp.PRN = ofile.vecOrec[i].GPSPRN[j].PRN;
							SatPosTemp.SYS = ofile.vecOrec[i].GPSPRN[j].SYS;

							SatPosTemp.X = SingleTemp.X;
							SatPosTemp.Y = SingleTemp.Y;
							SatPosTemp.Z = SingleTemp.Z;
							//暂存一组观测值的卫星坐标
							vecSatPos.push_back(SatPosTemp);
							//cout << "GPS" << SatPosTemp.X << "\t" << SatPosTemp.Y << "\t" << SatPosTemp.Z << endl;
							//赋值当前解算历元卫星坐标计算结果
							out1 << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2)<< ofile.vecOrec[i].epoch.second <<
								 setw(19) << std::setprecision(6)<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X <<"\t"<<  std::setprecision(6)
								<< std::setiosflags(ios::showpoint | ios::fixed) <<SatPosTemp.Y << "\t" << std::setprecision(6)
								<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0]*R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
							SingleSyssatEpoch.vecAGPSPosInOneEph.push_back(SingleTemp);
						}
					}
				}//for循环
			}

			SingleSyssatEpoch.GPSSatNumEph = SingleSyssatEpoch.vecAGPSPosInOneEph.size();

			if (vecSatPos.size() >= 4) {
				CALRecPos(vecSatPos, RecPos, Delta_V,  PosSys, weight);
			}
			else {
				break;
			}

		} while ((Delta_V[0] * Delta_V[0] + Delta_V[1] * Delta_V[1] + Delta_V[2] * Delta_V[2]) > PRECISION);
	}

	/**************BD/GPS***************/
	if (PosSys == 3) {
		double* Delta_V = new double[5]();           //平差误差项
		do {
			SingleSyssatEpoch = { };
			int label = { };                                 //每个卫星初始化一次指数

			rec_t.wn = ofile.vecOrec[i].OGPST.wn;               //观测时间
			rec_t.sow = ofile.vecOrec[i].OGPST.sow;

			GNSSSatPos SingleTemp{ 99 };

			if (ofile.vecOrec[i].BDPRN->SYS == 1) {
				for (int j = 0; j != ofile.vecOrec[i].BDSatNum; j++)
				{
					if (ofile.vecOrec[i].BDvalue[j][0] == 0 ||
						ofile.vecOrec[i].BDvalue[j][1] == 0 ||
						fabs(ofile.vecOrec[i].BDvalue[j][0] - ofile.vecOrec[i].BDvalue[j][1]) > 50) {
						continue;
					}
					sprn.PRN = ofile.vecOrec[i].BDPRN[j].PRN;
					sprn.SYS = ofile.vecOrec[i].BDPRN[j].SYS;
					SatPosTemp.PRN = sprn.PRN;
					SatPosTemp.SYS = sprn.SYS;
					SatPosTemp.codeC1 = ofile.vecOrec[i].BDvalue[j][0];
					SatPosTemp.codeP2 = ofile.vecOrec[i].BDvalue[j][1];
					SingleTemp.PRN = sprn.PRN;
					SingleTemp.SYS = sprn.SYS;

					CFileRead fileread;
					int mark = 0;
					fileread.GetBestNav(ofile.vecOrec[i], sprn, vecNrec_epo, label, mark);
					if (mark == 1)
					{
						continue;
					}
					BestN = vecNrec_epo[label];

					GPSTIME emi_t = { }, BDemi_t;

					double     Es = 0;                             //偏近点角
					CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos, Es,  PosSys);
					epochcorr.CALazel(ofile.ohdr, SatPosTemp);//计算高度角
					
					/*电离层延迟改正：双频改正模型*/
					if (SatPosTemp.azel[1] >= 10 * D2R) //剔除低高度角卫星
					{
						/*相对论效应的改正*/
						dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

						TimeSystem timesys;
						timesys.GPST2BDT(emi_t, BDemi_t);
						/*卫星钟差：多项式模型（考虑相对论效应）*/
						SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (BDemi_t.sow - BestN.TOE)
							+ BestN.a2 * (pow((BDemi_t.sow - BestN.TOE), 2)) + dRel;//- BestN.TGD

						/*对流层延迟改正：简化Hopfield模型*/
						epochcorr.TropCorr(ofile.ohdr, SatPosTemp);//epochcorr.TropCorr(ofile.ohdr,vecSatPos[i] );SatPosTemp数组越界，就慢慢找一步一步调试终于发现

						/*求无电离层延迟的线性组合观测值*/
						SatPosTemp.psr = (B1_2 * B1_2 * ofile.vecOrec[i].BDvalue[j][0]
							- B3 * B3 * ofile.vecOrec[i].BDvalue[j][1]) / (B1_2 * B1_2 - B3 * B3)
							- CLIGHT * B1_2 * B1_2 * BestN.TGD / (B1_2 * B1_2 - B3 * B3);//B1要-TGD

						SatPosTemp.PRN = ofile.vecOrec[i].BDPRN[j].PRN;
						SatPosTemp.SYS = ofile.vecOrec[i].BDPRN[j].SYS;

						SatPosTemp.X = SingleTemp.X;
						SatPosTemp.Y = SingleTemp.Y;
						SatPosTemp.Z = SingleTemp.Z;
						//暂存一组观测值的卫星坐标
						vecSatPos.push_back(SatPosTemp);
						//cout << "BDS/GPS" << SatPosTemp.X << "\t" << SatPosTemp.Y << "\t" << SatPosTemp.Z << endl;
						//赋值当前解算历元卫星坐标计算结果
						out1 << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2) << ofile.vecOrec[i].epoch.second <<
							setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Y << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0] * R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
						SingleSyssatEpoch.vecASatPosInOneEph.push_back(SingleTemp);
					}//if else
				}//for循环
			}//if

			if (ofile.vecOrec[i].GPSPRN->SYS == 2) {
				for (int j = 0; j != ofile.vecOrec[i].GPSSatNum; j++)
				{//这个判断可不要
					if (ofile.vecOrec[i].GPSvalue[j][0] == 0 ||ofile.vecOrec[i].GPSvalue[j][1] == 0 ||fabs(ofile.vecOrec[i].GPSvalue[j][0] - ofile.vecOrec[i].GPSvalue[j][1]) > 50) {
						continue;
					}
					sprn.PRN = ofile.vecOrec[i].GPSPRN[j].PRN;
					sprn.SYS = ofile.vecOrec[i].GPSPRN[j].SYS;
					SatPosTemp.PRN = sprn.PRN;
					SatPosTemp.SYS = sprn.SYS;
					SatPosTemp.codeC1 = ofile.vecOrec[i].GPSvalue[j][0];
					SatPosTemp.codeP2 = ofile.vecOrec[i].GPSvalue[j][1];
					SingleTemp.PRN = sprn.PRN;
					SingleTemp.SYS = sprn.SYS;

					CFileRead fileread;
					int mark = 0;
					fileread.GetBestNav(ofile.vecOrec[i], sprn, vecNrec_epo, label, mark);
					if (mark == 1)
					{
						continue;
					}

					BestN = vecNrec_epo[label];

					GPSTIME emi_t = { };
					double     Es = 0;                             //偏近点角
					CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos, Es,PosSys);
					epochcorr.CALazel(ofile.ohdr, SatPosTemp);//计算高度角
					/*相对论效应的改正*/
					dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

					/*卫星钟差：多项式模型*/
					SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (emi_t.sow - BestN.TOE)
						+ BestN.a2 * (pow((emi_t.sow - BestN.TOE), 2)) + dRel;

					if (SatPosTemp.azel[1] >= 10 * D2R) //卫星截止高度角10°
					{
						
						epochcorr.TropCorr(ofile.ohdr, SatPosTemp);
						/*双频改正模型,求无电离层延迟的线性组合观测值*/
						SatPosTemp.psr = P1 * ofile.vecOrec[i].GPSvalue[j][0]
							- P2 * ofile.vecOrec[i].GPSvalue[j][1];

						SatPosTemp.PRN = ofile.vecOrec[i].GPSPRN[j].PRN;
						SatPosTemp.SYS = ofile.vecOrec[i].GPSPRN[j].SYS;

						SatPosTemp.X = SingleTemp.X;
						SatPosTemp.Y = SingleTemp.Y;
						SatPosTemp.Z = SingleTemp.Z;
						//暂存一组观测值的卫星坐标
						vecSatPos.push_back(SatPosTemp);
						//赋值当前解算历元卫星坐标计算结果
						out1 << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2) << ofile.vecOrec[i].epoch.second <<
							setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Y << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0] * R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
						SingleSyssatEpoch.vecASatPosInOneEph.push_back(SingleTemp);
					}
				}//for循环

			}
			SingleSyssatEpoch.SatNumEph = SingleSyssatEpoch.vecASatPosInOneEph.size();//记录当前历元观测的卫星总数

			if (vecSatPos.size() >= 5) {
				CALRecPos(vecSatPos, RecPos, Delta_V, PosSys,  weight);
			}
			else {
				break;
			}
			
		} while ((Delta_V[0] * Delta_V[0] + Delta_V[1] * Delta_V[1] + Delta_V[2] * Delta_V[2]) > PRECISION);
		//迭代计算前后两次接收机位置小于1.0e-10
	}
	cout << ofile.vecOrec[i].epoch.year <<"/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << ofile.vecOrec[i].epoch.second << "\t" ;
	cout << setw(20) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << RecPos.X;
	cout << setw(20) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << RecPos.Y;
	cout << setw(20) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << RecPos.Z << endl;
	
}

/*******************
 * Name		: SinglePP
 * Function	: 实现全部的单点定位程序
 * argument : ObsFile& ofile    读取到的观测值文件
 *vector<NavRecord>& vecNrec_epo      读取到的NAV导航文件
 *vector<ASatPosInOneEph>& vecStorSatPosEveEpo    存储每一个历元的卫星坐标数组
 * vector<Descartes>& vecStorRecPos      储存接收机坐标
 * return   : none
 *
 *principle ：
 *remark    ：
 *******************/
void SPP::SinglePP(ObsFile& ofile, vector<NavRecord>& vecNrec_epo,
	vector<ASatPosInOneEph>& vecStorSatPosEveEpo, vector<Descartes>& vecStorRecPos,ofstream&out1,const int & PosSys, const int& weight)
{
	double Dtrec = 0;                         //接收机钟差
	ASatPosInOneEph EpoSat;
	Descartes RecPos = { };
	int jiange;
	cout << "请输入时间间隔" << endl;
	cin >> jiange;
	cout << jiange << endl;

	vector<SatInformation> vecSatPos;                //单历元的所有GPS卫星的信息//当使用过小的历元间隔时会出现在10000以上历元时出现错误
	int iepoch = 0;
	for (unsigned int i = 12; i < ofile.vecOrec.size(); i=i+jiange) //循环所有历元  unsigned无符号只有0~   //i在之后的在计算时可用于识别对应的历元进行历元间隔的控制。//FOR循环步长值的设置
	{
		CALepochSTA(ofile, vecNrec_epo, EpoSat, RecPos, i, vecSatPos,out1,  PosSys,  weight,iepoch);
		vecStorRecPos.push_back(RecPos);                                //在vecEPOCH中插入recpos
		vecStorSatPosEveEpo.push_back(EpoSat);                          //存储每一个历元的卫星坐标数组
		vecSatPos.clear();                                              //一个历元计算完成，清除历元数组vecSatPos
		iepoch++;														//每次将iepoch加一，由于在外部定义，在每次循环之后其值不会被销毁  
	}
}