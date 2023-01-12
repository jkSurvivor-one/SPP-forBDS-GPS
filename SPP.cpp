#include"ALLHead.h"
#include<Eigen/Dense>

#include<iostream>
#include <iomanip>
#include <fstream>//Ҫ���ļ������뺯��ʹ��<<�����������Ҫ����ͷ�ļ�����ʹ�ã�Ҫ����string��Ҫ���϶�Ӧ��ͷ�ļ�
using namespace Eigen;
using namespace std;
//int PosSys = 2;//ϵͳ
//int weight = 2;//Ȩ��1��Ȩ  2�߶ȽǶ�Ȩ
SPP::SPP()
{
}

SPP::~SPP()
{
}
/*******************
 * Name		: CalEmitTSATpos
 * Function	: ������������λ�ú������Ӳ�
 * argument : NavRecord& BestN   �뵱ǰ�۲��ļ�������ӽ�����������
 *			  GPSTIME& rec_t     �۲��ļ��и����Ĺ۲�ʱ��
 *			  SatInformation& SatPosTemp  ������Ϣ�ļ������ڴ洢��������λ�ü���õ�����������
 *			  GNSSSatPos& SingleTemp ��ʱ�洢����λ�ã����ڼ�������ֵ�����ξ����
 *			  Descartes& RecPos      ���ջ�λ�ã�������ƽ��ջ�����
 *			  double& Es             ����������ƫ�����
 * return   : none
 *
 *principle ��
 *remark    ��ÿ�μ���һ������
 *******************/
void SPP::CalEmitTSATpos(NavRecord& BestN, GPSTIME& rec_t, GPSTIME& emi_t, SatInformation& SatPosTemp,GNSSSatPos& SingleTemp, Descartes& RecPos, double& Es,const int& PosSys)
{
	GPSTIME   emi_t0{ 99 };           //���������źŷ���ʱ�̵������ȱ���
	emi_t0.wn = rec_t.wn;
	emi_t.wn = rec_t.wn;              //�����źŷ���ʱ��
	double L_RecSat{ 99.0 };          //����վ�Ǿ�,���ξ���( geometric range)

	//���ǵ���վ���źŴ���ʱ���Լλ0.07s
	/*���������������źŷ���ʱ��*/
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
		//�ڽ��е�����ת֮ǰ������ͬ����ת������������ȷ
	} while (fabs(emi_t0.sow - emi_t.sow) > PRECISION);//1e-12
}

/*******************
 * Name		: CALBDSatPos
 * Function	: ���㱱������λ��
 * argument : NavRecord& BestN      �뵱ǰ�۲��ļ�������ӽ�����������
 *			  GPSTIME emi_t         �źŷ���ʱ��
 *			  SatInformation& SatPosTemp  ���ǵĸ���Ϣ������洢��
 *			  double& Es            �ż������ƫ����ǣ�������������
 *
 * return   : none
 *
 *principle ��
 *remark    ��������Ҫ��Ҫ��Ϊ���ּ��㷽��
 *******************/
void SPP::CALBDSatPos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es)
{
	int i = 0;
	double n0, n;

	//1.���������˶���ƽ�����ٶ�n
	double a;
	a = pow(BestN.sqrtA, 2);
	n0 = sqrt(GM_CGCS / pow(a, 3));
	n = n0 + BestN.Deltan;

	//2.����黯ʱ��tk���۲�ʱ��������-�ο�ʱ��������                          
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

	//3.����۲�˲�����ǵ�ƽ�����M
	double M;
	M = BestN.M0 + n * tk;

	//4.����ƫ�����Es      �ⲿ�Ѽ���
	double E0;
	double E;
	E = M;
	do
	{
		E0 = E;
		E = M + BestN.e * sin(E0);
	} while (fabs(E0 - E) > 1e-12);
	Es = E;

	//5.����������f
	double f;
	f = atan2((sqrt(1 - pow(BestN.e, 2)) * sin(Es)), (cos(Es) - BestN.e));

	//6.���������Ǿ�miu
	double miu;
	miu = f + BestN.Omega;

	//7.�����㶯������
	double Delta_u;
	double Delta_r;
	double Delta_i;
	Delta_u = BestN.Cuc * cos(2 * miu) + BestN.Cus * sin(2 * miu);
	Delta_r = BestN.Crc * cos(2 * miu) + BestN.Crs * sin(2 * miu);
	Delta_i = BestN.Cic * cos(2 * miu) + BestN.Cis * sin(2 * miu);

	//8.�����㶯����
	double cor_u;      //�����Ǿ�
	double cor_r;      //����ʸ��
	double cor_i;      //������
	cor_u = miu + Delta_u;
	cor_r = fabs((BestN.sqrtA * BestN.sqrtA) * (1 - BestN.e * cos(Es)) + Delta_r);
	cor_i = BestN.i0 + BestN.IDOT * tk + Delta_i;

	//9.���������ڹ��������ϵ�е�λ��
	double xSatOrb;
	double ySatOrb;
	xSatOrb = cor_r * cos(cor_u);
	ySatOrb = cor_r * sin(cor_u);

	//�ж��������ͣ�GEO,IGSO,MEO
	int j = 0;
	for (i = 0; i < 8; i++)
	{
		if (BestN.sprn.PRN == GEO_prn[i])
			j = 1;
	}

	if (!j)
	{
		//10.���������������㾭Ls
		double Ls;
		Ls = BestN.Omeca + (BestN.OmegaDot - OMGE_CGCS) * tk - OMGE_CGCS * BestN.TOE;

		//11.����������˲ʱ��������ϵ�е�λ��
		double x_ECEF;
		double y_ECEF;
		double z_ECEF;
		x_ECEF = xSatOrb * cos(Ls) - ySatOrb * cos(cor_i) * sin(Ls);
		y_ECEF = xSatOrb * sin(Ls) + ySatOrb * cos(cor_i) * cos(Ls);
		z_ECEF = ySatOrb * sin(cor_i);

		//�洢����
		SatPosTemp.X = x_ECEF;
		SatPosTemp.Y = y_ECEF;
		SatPosTemp.Z = z_ECEF;
	}
	else//����GEO���ǣ�����������ת����Ϲ㲥�����������//�ο����ף�������������λ�ü��㷽���о�
	{
		//10.���������������㾭Ls�����첻���ǵ�����ת��
		double Ls;
		Ls = BestN.Omeca + BestN.OmegaDot * tk - OMGE_CGCS * BestN.TOE;//��������ϵ

		//11.����GEO�������Զ�������ϵ�µ�λ��
		typedef Matrix<double, 3, 1> Matrix31d;
		Matrix31d GK = Matrix31d::Zero(3, 1);//�����3X1�ľ��󲢳�ʼ��
		GK(0, 0) = xSatOrb * cos(Ls) - ySatOrb * cos(cor_i) * sin(Ls);
		GK(1, 0) = xSatOrb * sin(Ls) + ySatOrb * cos(cor_i) * cos(Ls);
		GK(2, 0) = ySatOrb * sin(cor_i);

		//11.����GEO������˲ʱ��������ϵ�е�λ��
		//typedef Matrix<double, 3, 3> Matrix3d;
		MatrixXd Rz = MatrixXd::Zero(3, 3);//����һ��3X3�ľ��󲢳�ʼ��
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
		//�洢����
		SatPosTemp.X = ECEF(0, 0);
		SatPosTemp.Y = ECEF(1, 0);
		SatPosTemp.Z = ECEF(2, 0);
	}
}
/*******************
 * Name		: CALGPSatPos
 * Function	: ����GPS����λ��
 * argument : NavRecord& BestN      �뵱ǰ�۲��ļ�������ӽ�����������
 *			  GPSTIME emi_t         �źŷ���ʱ��
 *			  SatInformation& SatPosTemp  ���ǵĸ���Ϣ������洢��
 *			  double& Es            �ż������ƫ����ǣ�������������
 *
 * return   : none
 *
 *principle ��
 *remark    ��
 *******************/
void SPP::CALGPSSatpos(NavRecord& BestN, GPSTIME emi_t, SatInformation& SatPosTemp, double& Es)
{
	int i = 0;
	double n0, n;

	//1.�����������е�ƽ�����ٶ�n
	double a;
	a = pow(BestN.sqrtA, 2);
	n0 = sqrt(GM_WGS / pow(a, 3));//pow()����������x��y����
	n = n0 + BestN.Deltan;

	//2.����黯ʱ��tk���۲�ʱ��������-�ο�ʱ��������                          
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

	//3.�����źŷ���ʱ�����ǵ�ƽ�����M
	double M;
	M = BestN.M0 + n * tk;

	//4.��������ƫ�����Es
	double E0;
	double E;
	E = M;
	do
	{
		E0 = E;
		E = M + BestN.e * sin(E0);//�û��ȱ�ʾ�Ŀ����շ���
	} while (fabs(E0 - E) > 1E-10);
	Es = E;//����ȥ

	//5.����������f
	double f;
	f = atan2((sqrt(1 - pow(BestN.e, 2)) * sin(E)), (cos(E) - BestN.e));

	//6.���������Ǿ�miu
	double miu;
	miu = f + BestN.Omega;

	//7.�����㶯������
	double Delta_u;
	double Delta_r;
	double Delta_i;
	Delta_u = BestN.Cuc * cos(2 * miu) + BestN.Cus * sin(2 * miu);
	Delta_r = BestN.Crc * cos(2 * miu) + BestN.Crs * sin(2 * miu);
	Delta_i = BestN.Cic * cos(2 * miu) + BestN.Cis * sin(2 * miu);

	//8.�����㶯����
	double cor_u;      //�����Ǿ�
	double cor_r;      //����ʸ��
	double cor_i;      //������
	cor_u = miu + Delta_u;
	cor_r = fabs((BestN.sqrtA * BestN.sqrtA) * (1 - BestN.e * cos(E)) + Delta_r);
	cor_i = BestN.i0 + BestN.IDOT * tk + Delta_i;

	//9.���������ڹ��������ϵ�е�λ��
	double xSatOrb;
	double ySatOrb;
	xSatOrb = cor_r * cos(cor_u);
	ySatOrb = cor_r * sin(cor_u);

	//10.����������������ľ���Ls
	double Ls;
	Ls = BestN.Omeca + (BestN.OmegaDot - OMGE_WGS) * tk - OMGE_WGS * BestN.TOE;

	//11.����������˲ʱ��������ϵ�е�λ��
	double x_ECEF;
	double y_ECEF;
	double z_ECEF;
	x_ECEF = xSatOrb * cos(Ls) - ySatOrb * cos(cor_i) * sin(Ls);
	y_ECEF = xSatOrb * sin(Ls) + ySatOrb * cos(cor_i) * cos(Ls);
	z_ECEF = ySatOrb * sin(cor_i);

	//�洢����
	SatPosTemp.X = x_ECEF;
	SatPosTemp.Y = y_ECEF;
	SatPosTemp.Z = z_ECEF;
}

/*******************
 * Name		: CALRecPos
 * Function	: �����̼�������ջ�����
 * argument : vector<SatInformation>& vecSatPos   ��ǰ��Ԫ��������Ϣ�������������߶Ƚǵȣ�����
 *			  Descartes& car_GetRecPos      �ѿ�������ϵ�ṹ�����ڴ洢������Ľ��ջ�����
 *			  double vx[]					���ڴ�Ž��������VX4������ֵ
 *
 * return   : none
 *
 *principle ��ͨ�����õ�POSSYS��ǰ���ö�λģʽ�������̼�������ջ�λ�ã��ú���ֻ�������Ӧ�ĸ���ֵ�����ж��Ƿ�ﵽҪ����޲�ֵ��
 *remark    ��������GPS�����ģʽ��ͬ����ʱ�䡢����ϵͳ�Ĳ�ͬ�ڼ�������λ��ʱ���趨�ã�
 *******************/
void SPP::CALRecPos(vector<SatInformation>& vecSatPos, Descartes& car_GetRecPos, double vx[], const int& PosSys, const int& weight)
{
	double a = 0.004, b = 0.003;                        //�߶ȽǶ�Ȩģ����ؾ���ֵ

	if (PosSys == 1 || PosSys == 2) {
		unsigned int N = vecSatPos.size();                  //����Ԫ�����ǿ�����
		double vt = 0;                                      //���ջ��Ӳ�ƽ��ֵ

		/*�������þ��󲢳�ʼ��*/
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
		MatrixX1d L = MatrixX1d::Zero(N, 1);           //������
		MatrixX1d S = MatrixX1d::Zero(N, 1);           //վ�Ǿ�
		MatrixX1d ll = MatrixX1d::Zero(N, 1);          //��������
		MatrixX1d mm = MatrixX1d::Zero(N, 1);          //��������
		MatrixX1d nn = MatrixX1d::Zero(N, 1);          //��������
		MatrixX1d psr = MatrixX1d::Zero(N, 1);         //�޵������Ϲ۲�ֵ
		MatrixX1d dclk = MatrixX1d::Zero(N, 1);        //�����Ӳ�
		MatrixX1d dtrop = MatrixX1d::Zero(N, 1);       //�������ӳٸ���
		MatrixX1d Elev = MatrixX1d::Zero(N, 1);        //���Ǹ߶Ƚ�

		/*���ø���Ԫ�ڵ�j�����ǣ�������ջ�����*/
		for (unsigned int j = 0; j < N; j++)
		{
			/*���ξ���*/
			S(j, 0) = sqrt(pow((vecSatPos[j].X - car_GetRecPos.X), 2) +
				pow((vecSatPos[j].Y - car_GetRecPos.Y), 2) +
				pow((vecSatPos[j].Z - car_GetRecPos.Z), 2));
			/*α���Ӳ�ȷֱ�������*/
			psr(j, 0) = vecSatPos[j].psr;//�޵�����ӳٷǲ����
			dclk(j, 0) = vecSatPos[j].SatClk;
			dtrop(j, 0) = vecSatPos[j].dtrop;

			/*������j�ڹ۲ⷽ���е������վ�Ǽ��ξ�-���������α��psr-�����Ӳ����+�������ӳٸ���*/
			L(j, 0) = S(j, 0) - psr(j, 0) - CLIGHT * dclk(j, 0) + dtrop(j, 0);

			/*������j�ķ�������*/
			ll(j, 0) = (vecSatPos[j].X - car_GetRecPos.X) / S(j, 0);
			mm(j, 0) = (vecSatPos[j].Y - car_GetRecPos.Y) / S(j, 0);
			nn(j, 0) = (vecSatPos[j].Z - car_GetRecPos.Z) / S(j, 0);

			A(j, 0) = ll(j, 0);
			A(j, 1) = mm(j, 0);
			A(j, 2) = nn(j, 0);
			A(j, 3) = -1;

			/*��Ȩ*/
			if (weight == 1) {
				P(j, j) = 1;//��Ȩ
			}
			if (weight == 2) {
				/*�߶ȽǶ�Ȩ���Ľ������Һ���ģ��*/
				Elev(j, 0) = vecSatPos[j].azel[1];
				P(j, j) = 1.0 / (a * a + b * b / pow(sin(Elev(j, 0)), 2));
			}
		}
		/*���ƽ��*/
		AT = A.transpose();  //�����ת��
		ATP = AT * P;
		ATPA = ATP * A;
		NBB = ATPA.inverse();//���������
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
			vecSatPos.clear();//ɾ�� vector ���������е�Ԫ�أ�ʹ���ɿյ� vector �������ú�����ı� vector �Ĵ�С����Ϊ 0���������Ǹı���������
			//�����������е����ݣ��������ǰ��Ԫ������
		}
	}

	if (PosSys == 3) {
		unsigned int N = vecSatPos.size();                  //����Ԫ�����ǿ�����
		double vt = 0;                                      //���ջ��Ӳ�ƽ��ֵ
		double vt_sys;

		/*�������þ��󲢳�ʼ��*/
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
		MatrixX1d L = MatrixX1d::Zero(N, 1);           //������
		MatrixX1d S = MatrixX1d::Zero(N, 1);           //վ�Ǿ�
		MatrixX1d ll = MatrixX1d::Zero(N, 1);          //��������
		MatrixX1d mm = MatrixX1d::Zero(N, 1);          //��������
		MatrixX1d nn = MatrixX1d::Zero(N, 1);          //��������
		MatrixX1d psr = MatrixX1d::Zero(N, 1);         //�޵������Ϲ۲�ֵ
		MatrixX1d dclk = MatrixX1d::Zero(N, 1);        //�����Ӳ�
		MatrixX1d dtrop = MatrixX1d::Zero(N, 1);       //�������ӳٸ���
		MatrixX1d Elev = MatrixX1d::Zero(N, 1);        //���Ǹ߶Ƚ�

		/*���ø���Ԫ�ڵ�j�����ǣ�������ջ�����*/
		for (unsigned int j = 0; j < N; j++)
		{
			S(j, 0) = sqrt(pow((vecSatPos[j].X - car_GetRecPos.X), 2) +
				pow((vecSatPos[j].Y - car_GetRecPos.Y), 2) +
				pow((vecSatPos[j].Z - car_GetRecPos.Z), 2));

			psr(j, 0) = vecSatPos[j].psr;
			dclk(j, 0) = vecSatPos[j].SatClk;
			dtrop(j, 0) = vecSatPos[j].dtrop;

			/*������j�ڹ۲ⷽ���е������վ�Ǽ��ξ�-���������α��psr-�����Ӳ����+�������ӳٸ���*/
			L(j, 0) = S(j, 0) - psr(j, 0) - CLIGHT * dclk(j, 0) + dtrop(j, 0);

			/*������j�ķ�������*/
			ll(j, 0) = (vecSatPos[j].X - car_GetRecPos.X) / S(j, 0);
			mm(j, 0) = (vecSatPos[j].Y - car_GetRecPos.Y) / S(j, 0);
			nn(j, 0) = (vecSatPos[j].Z - car_GetRecPos.Z) / S(j, 0);

			A(j, 0) = ll(j, 0);
			A(j, 1) = mm(j, 0);
			A(j, 2) = nn(j, 0);

			if (vecSatPos[j].SYS == 1) {//������GPS�Ӳͬ��Ҫ�ֿ����
				A(j, 3) = 0;
				A(j, 4) = -1;
			}

			if (vecSatPos[j].SYS == 2) {
				A(j, 3) = -1;
				A(j, 4) = 0;
			}

			/*Ȩ��*/
			if (weight == 1) {
				P(j, j) = 1;//��Ȩ
			}

			if (weight == 2) {
				/*�߶ȽǶ�Ȩ���Ľ������Һ���ģ��*/
				Elev(j, 0) = vecSatPos[j].azel[1];
				P(j, j) = 1.0 / (a * a + b * b / pow(sin(Elev(j, 0)), 2));
			}

		}

		/*���ƽ��*/
		AT = A.transpose();  //�����ת��
		ATP = AT * P;
		ATPA = ATP * A;
		NBB = ATPA.inverse();//���������
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
			vecSatPos.clear();//clear()	ɾ�� vector ���������е�Ԫ�أ�ʹ���ɿյ� vector �������ú�����ı� vector �Ĵ�С����Ϊ 0���������Ǹı���������sawp�ͷ��ڴ�
		}
	}

}
/**********
* ���ƣ�CALepochSTA
* ���ܣ����㵥��Ԫ�Ľ��ջ����꣨station position)
* ������ObsFile& ofile  �۲�ֵ�ļ� ����ͷ�ļ���Ϣ���۲�ֵ   ��������
		vector<SatInformation>& vecSatPos   �����ǵ�������Ϣ
*		vector<NavRecord>& vecNrec_epo   ���������ļ���������   ��������
*		 ASatPosInOneEph& SingleSyssatEpoch   ��ǰ��Ԫ����������λ��
*		Descartes& RecPos   �ѿ�������ϵ����������  ����ͨ�������洢��������Ľ��ջ�����
*		int i				�����жϵڼ���Ԫ
*����ֵ���޷���ֵ
* ע��PosSys ѡ��������ϵͳ��ѡ��1BDS 2GPS 3BDS&&GPS
***/
void SPP::CALepochSTA(ObsFile& ofile, vector<NavRecord>& vecNrec_epo, ASatPosInOneEph& SingleSyssatEpoch,Descartes& RecPos, int i, vector<SatInformation>& vecSatPos, ofstream& out1, const int& PosSys, const int& weight,const int & iepoch )
{
	GPSTIME rec_t = { };                         //��Ԫ�Ѿ�ת����GPSʱ�������Ҫ��ת����BDʱ
	Correction epochcorr;						//Ϊ����ֵ��������
	SPRN sprn = { };//���ɹ�Ա��ʼΪ0��4�ַ���struct A = { 0 };struct A = {};memset(&A, 0, sizeof(struct));A = (struct){ 0 } // c99 �� compound literal
	SatInformation SatPosTemp = { };                 //�洢�����������ݽṹ��
	NavRecord BestN;                                  //�����Ԫ���ڵĵ�����Ϣ

	double tk = 0;                               //�黯ʱ��
	double dRel = 0;                             //����۸�����


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
		double* Delta_V = new double[4]();           //ƽ�������// ÿ��Ԫ�س�ʼ��Ϊ0,�����ڲ���д����ֵ��ֻ�ܳ�ʼ��Ϊ0
		do {
			SingleSyssatEpoch = { };
			GNSSSatPos SingleTemp{ };//�ñ���ÿ��ѭ�����ᱻ�ͷ��ڴ�
			int label = { };                                 //ÿ�����ǳ�ʼ��һ��ָ��

			rec_t.wn = ofile.vecOrec[i].OGPST.wn;            //�۲�ʱ�䣨GPSʱ��
			rec_t.sow = ofile.vecOrec[i].OGPST.sow;

			if (ofile.vecOrec[i].BDPRN->SYS == 1) {
				for (int j = 0; j != ofile.vecOrec[i].BDSatNum; j++)//�ڵ�i����Ԫ,�������б�������
				{
					//��i��Ԫ�ڣ��޳���ͬʱ����˫Ƶα���α��֮�����50m�Ĺ۲�ֵ
					if (ofile.vecOrec[i].BDvalue[j][0] == 0 ||
						ofile.vecOrec[i].BDvalue[j][1] == 0 ||
						fabs(ofile.vecOrec[i].BDvalue[j][0] - ofile.vecOrec[i].BDvalue[j][1]) > 50) {
						continue;
					}
					sprn.PRN = ofile.vecOrec[i].BDPRN[j].PRN;//��������Ϣ����sprn�ṹ��
					sprn.SYS = ofile.vecOrec[i].BDPRN[j].SYS;

					SatPosTemp.PRN = sprn.PRN;//��������Ϣ����SatInfoNow�ṹ����
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

					double     Es = 0;                             //ƫ�����
					CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos,Es, PosSys);
					////spp.calsatclk();
					epochcorr.CALazel(ofile.ohdr, SatPosTemp);//����߶Ƚ�
					//CALBDSatPos(BestN,emi_t, SatPosTemp, Es);
					
					/*������ӳٸ�����˫Ƶ����ģ��*///or kl��Correction �����У�
					if (SatPosTemp.azel[1] >= 10 * D2R) //�޳��͸߶Ƚ�����(���ȣ�
					{
						/*�����ЧӦ�ĸ���*/
						dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

						TimeSystem timesys;
						timesys.GPST2BDT(emi_t, BDemi_t);
						/*�����Ӳ����ʽģ�ͣ����������ЧӦ��*/
						SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (BDemi_t.sow - BestN.TOE)
							+ BestN.a2 * (pow((BDemi_t.sow - BestN.TOE), 2)) + dRel;//��Ƶ�ļ���- BestN.TGD

						/*�������ӳٸ�����Saastamoinenmodel*/
						epochcorr.TropCorr(ofile.ohdr, SatPosTemp);//����Խ��
						/*���޵�����ӳٵ�������Ϲ۲�ֵ*/
						SatPosTemp.psr = (B1_2 * B1_2 * ofile.vecOrec[i].BDvalue[j][0]
							- B3 * B3 * ofile.vecOrec[i].BDvalue[j][1]) / (B1_2 * B1_2 - B3 * B3)
							- CLIGHT * B1_2 * B1_2 * BestN.TGD / (B1_2 * B1_2 - B3 * B3);//
						SatPosTemp.PRN = ofile.vecOrec[i].BDPRN[j].PRN;
						SatPosTemp.SYS = ofile.vecOrec[i].BDPRN[j].SYS;

						SatPosTemp.X = SingleTemp.X;
						SatPosTemp.Y = SingleTemp.Y;
						SatPosTemp.Z = SingleTemp.Z;
						//�ݴ�һ��۲�ֵ����������
						vecSatPos.push_back(SatPosTemp);
						//cout << "BDS" << SatPosTemp.X <<"\t" << SatPosTemp.Y <<"\t"<< SatPosTemp.Z << endl;
						//��ֵ��ǰ������Ԫ�������������
						out1  << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2) << ofile.vecOrec[i].epoch.second <<
							setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Y << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0] * R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
						SingleSyssatEpoch.vecABDSPosInOneEph.push_back(SingleTemp);
					}
				}//forѭ��
			}

			SingleSyssatEpoch.BDSatNumEph = SingleSyssatEpoch.vecABDSPosInOneEph.size();//��¼��ǰ��Ԫ�۲����������
			//��������4����ʼ������ջ�����
			if (vecSatPos.size() >= 4) {
				CALRecPos(vecSatPos, RecPos, Delta_V, PosSys,  weight);
			}
			else {
				break;
			}

		} while ((Delta_V[0] * Delta_V[0] + Delta_V[1] * Delta_V[1] + Delta_V[2] * Delta_V[2]) > PRECISION);
		//��������ǰ�����ν��ջ�λ��С��1.0e-10
	}

	/***************GPS*****************/
	if (PosSys == 2) {
		double* Delta_V = new double[4]();           //ƽ�������
		do {
			SingleSyssatEpoch = { };                              //������һ��Ԫǰ����
			int label = { };                                    //ÿ�����ǳ�ʼ��һ��ָ��

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
						if (mark == 1)//��������
						{
							continue;
						}
						BestN = vecNrec_epo[label];

						GPSTIME emi_t = { };
						double     Es = 0;                             //ƫ�����
						CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos, Es, PosSys);
						Correction spp;
						////spp.calsatclk();
						epochcorr.CALazel(ofile.ohdr, SatPosTemp);//����߶Ƚ�
						

						if (SatPosTemp.azel[1] >= 10 * D2R) //���ǽ�ֹ�߶Ƚ�10����Щ��Ԫ�����
						{
							/*�����ЧӦ�ĸ���*/
							dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

							/*�����Ӳ����ʽģ��*/
							SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (emi_t.sow - BestN.TOE)
								+ BestN.a2 * (pow((emi_t.sow - BestN.TOE), 2)) + dRel;
							/*���������*/
							epochcorr.TropCorr(ofile.ohdr, SatPosTemp);
							/*˫Ƶ����ģ��,���޵�����ӳٵ�������Ϲ۲�ֵ*/
							SatPosTemp.psr = P1 * ofile.vecOrec[i].GPSvalue[j][0]
								- P2 * ofile.vecOrec[i].GPSvalue[j][1];

							SatPosTemp.PRN = ofile.vecOrec[i].GPSPRN[j].PRN;
							SatPosTemp.SYS = ofile.vecOrec[i].GPSPRN[j].SYS;

							SatPosTemp.X = SingleTemp.X;
							SatPosTemp.Y = SingleTemp.Y;
							SatPosTemp.Z = SingleTemp.Z;
							//�ݴ�һ��۲�ֵ����������
							vecSatPos.push_back(SatPosTemp);
							//cout << "GPS" << SatPosTemp.X << "\t" << SatPosTemp.Y << "\t" << SatPosTemp.Z << endl;
							//��ֵ��ǰ������Ԫ�������������
							out1 << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2)<< ofile.vecOrec[i].epoch.second <<
								 setw(19) << std::setprecision(6)<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X <<"\t"<<  std::setprecision(6)
								<< std::setiosflags(ios::showpoint | ios::fixed) <<SatPosTemp.Y << "\t" << std::setprecision(6)
								<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0]*R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
							SingleSyssatEpoch.vecAGPSPosInOneEph.push_back(SingleTemp);
						}
					}
				}//forѭ��
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
		double* Delta_V = new double[5]();           //ƽ�������
		do {
			SingleSyssatEpoch = { };
			int label = { };                                 //ÿ�����ǳ�ʼ��һ��ָ��

			rec_t.wn = ofile.vecOrec[i].OGPST.wn;               //�۲�ʱ��
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

					double     Es = 0;                             //ƫ�����
					CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos, Es,  PosSys);
					epochcorr.CALazel(ofile.ohdr, SatPosTemp);//����߶Ƚ�
					
					/*������ӳٸ�����˫Ƶ����ģ��*/
					if (SatPosTemp.azel[1] >= 10 * D2R) //�޳��͸߶Ƚ�����
					{
						/*�����ЧӦ�ĸ���*/
						dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

						TimeSystem timesys;
						timesys.GPST2BDT(emi_t, BDemi_t);
						/*�����Ӳ����ʽģ�ͣ����������ЧӦ��*/
						SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (BDemi_t.sow - BestN.TOE)
							+ BestN.a2 * (pow((BDemi_t.sow - BestN.TOE), 2)) + dRel;//- BestN.TGD

						/*�������ӳٸ�������Hopfieldģ��*/
						epochcorr.TropCorr(ofile.ohdr, SatPosTemp);//epochcorr.TropCorr(ofile.ohdr,vecSatPos[i] );SatPosTemp����Խ�磬��������һ��һ���������ڷ���

						/*���޵�����ӳٵ�������Ϲ۲�ֵ*/
						SatPosTemp.psr = (B1_2 * B1_2 * ofile.vecOrec[i].BDvalue[j][0]
							- B3 * B3 * ofile.vecOrec[i].BDvalue[j][1]) / (B1_2 * B1_2 - B3 * B3)
							- CLIGHT * B1_2 * B1_2 * BestN.TGD / (B1_2 * B1_2 - B3 * B3);//B1Ҫ-TGD

						SatPosTemp.PRN = ofile.vecOrec[i].BDPRN[j].PRN;
						SatPosTemp.SYS = ofile.vecOrec[i].BDPRN[j].SYS;

						SatPosTemp.X = SingleTemp.X;
						SatPosTemp.Y = SingleTemp.Y;
						SatPosTemp.Z = SingleTemp.Z;
						//�ݴ�һ��۲�ֵ����������
						vecSatPos.push_back(SatPosTemp);
						//cout << "BDS/GPS" << SatPosTemp.X << "\t" << SatPosTemp.Y << "\t" << SatPosTemp.Z << endl;
						//��ֵ��ǰ������Ԫ�������������
						out1 << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2) << ofile.vecOrec[i].epoch.second <<
							setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Y << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0] * R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
						SingleSyssatEpoch.vecASatPosInOneEph.push_back(SingleTemp);
					}//if else
				}//forѭ��
			}//if

			if (ofile.vecOrec[i].GPSPRN->SYS == 2) {
				for (int j = 0; j != ofile.vecOrec[i].GPSSatNum; j++)
				{//����жϿɲ�Ҫ
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
					double     Es = 0;                             //ƫ�����
					CalEmitTSATpos(BestN, rec_t, emi_t, SatPosTemp, SingleTemp, RecPos, Es,PosSys);
					epochcorr.CALazel(ofile.ohdr, SatPosTemp);//����߶Ƚ�
					/*�����ЧӦ�ĸ���*/
					dRel = -(2 * BestN.sqrtA * BestN.e * sqrt(GM_CGCS) * sin(Es)) / (CLIGHT * CLIGHT);

					/*�����Ӳ����ʽģ��*/
					SatPosTemp.SatClk = BestN.a0 + BestN.a1 * (emi_t.sow - BestN.TOE)
						+ BestN.a2 * (pow((emi_t.sow - BestN.TOE), 2)) + dRel;

					if (SatPosTemp.azel[1] >= 10 * D2R) //���ǽ�ֹ�߶Ƚ�10��
					{
						
						epochcorr.TropCorr(ofile.ohdr, SatPosTemp);
						/*˫Ƶ����ģ��,���޵�����ӳٵ�������Ϲ۲�ֵ*/
						SatPosTemp.psr = P1 * ofile.vecOrec[i].GPSvalue[j][0]
							- P2 * ofile.vecOrec[i].GPSvalue[j][1];

						SatPosTemp.PRN = ofile.vecOrec[i].GPSPRN[j].PRN;
						SatPosTemp.SYS = ofile.vecOrec[i].GPSPRN[j].SYS;

						SatPosTemp.X = SingleTemp.X;
						SatPosTemp.Y = SingleTemp.Y;
						SatPosTemp.Z = SingleTemp.Z;
						//�ݴ�һ��۲�ֵ����������
						vecSatPos.push_back(SatPosTemp);
						//��ֵ��ǰ������Ԫ�������������
						out1 << setw(4) << iepoch << " " << SatPosTemp.SYS << " " << setw(3) << SatPosTemp.PRN << " " << ofile.vecOrec[i].epoch.year << "/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << setw(4) << std::setprecision(2) << ofile.vecOrec[i].epoch.second <<
							setw(19) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.X << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Y << "\t" << std::setprecision(6)
							<< std::setiosflags(ios::showpoint | ios::fixed) << SatPosTemp.Z << "\t" << SatPosTemp.azel[0] * R2D << "\t" << SatPosTemp.azel[1] * R2D << "\t" << SatPosTemp.psr << "\t" << SatPosTemp.dtrop << "\t" << SatPosTemp.SatClk << endl;
						SingleSyssatEpoch.vecASatPosInOneEph.push_back(SingleTemp);
					}
				}//forѭ��

			}
			SingleSyssatEpoch.SatNumEph = SingleSyssatEpoch.vecASatPosInOneEph.size();//��¼��ǰ��Ԫ�۲����������

			if (vecSatPos.size() >= 5) {
				CALRecPos(vecSatPos, RecPos, Delta_V, PosSys,  weight);
			}
			else {
				break;
			}
			
		} while ((Delta_V[0] * Delta_V[0] + Delta_V[1] * Delta_V[1] + Delta_V[2] * Delta_V[2]) > PRECISION);
		//��������ǰ�����ν��ջ�λ��С��1.0e-10
	}
	cout << ofile.vecOrec[i].epoch.year <<"/" << ofile.vecOrec[i].epoch.month << "/" << ofile.vecOrec[i].epoch.day << "/" << ofile.vecOrec[i].epoch.hour << "/" << ofile.vecOrec[i].epoch.minute << "/" << ofile.vecOrec[i].epoch.second << "\t" ;
	cout << setw(20) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << RecPos.X;
	cout << setw(20) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << RecPos.Y;
	cout << setw(20) << std::setprecision(6) << std::setiosflags(ios::showpoint | ios::fixed) << RecPos.Z << endl;
	
}

/*******************
 * Name		: SinglePP
 * Function	: ʵ��ȫ���ĵ��㶨λ����
 * argument : ObsFile& ofile    ��ȡ���Ĺ۲�ֵ�ļ�
 *vector<NavRecord>& vecNrec_epo      ��ȡ����NAV�����ļ�
 *vector<ASatPosInOneEph>& vecStorSatPosEveEpo    �洢ÿһ����Ԫ��������������
 * vector<Descartes>& vecStorRecPos      ������ջ�����
 * return   : none
 *
 *principle ��
 *remark    ��
 *******************/
void SPP::SinglePP(ObsFile& ofile, vector<NavRecord>& vecNrec_epo,
	vector<ASatPosInOneEph>& vecStorSatPosEveEpo, vector<Descartes>& vecStorRecPos,ofstream&out1,const int & PosSys, const int& weight)
{
	double Dtrec = 0;                         //���ջ��Ӳ�
	ASatPosInOneEph EpoSat;
	Descartes RecPos = { };
	int jiange;
	cout << "������ʱ����" << endl;
	cin >> jiange;
	cout << jiange << endl;

	vector<SatInformation> vecSatPos;                //����Ԫ������GPS���ǵ���Ϣ//��ʹ�ù�С����Ԫ���ʱ�������10000������Ԫʱ���ִ���
	int iepoch = 0;
	for (unsigned int i = 12; i < ofile.vecOrec.size(); i=i+jiange) //ѭ��������Ԫ  unsigned�޷���ֻ��0~   //i��֮����ڼ���ʱ������ʶ���Ӧ����Ԫ������Ԫ����Ŀ��ơ�//FORѭ������ֵ������
	{
		CALepochSTA(ofile, vecNrec_epo, EpoSat, RecPos, i, vecSatPos,out1,  PosSys,  weight,iepoch);
		vecStorRecPos.push_back(RecPos);                                //��vecEPOCH�в���recpos
		vecStorSatPosEveEpo.push_back(EpoSat);                          //�洢ÿһ����Ԫ��������������
		vecSatPos.clear();                                              //һ����Ԫ������ɣ������Ԫ����vecSatPos
		iepoch++;														//ÿ�ν�iepoch��һ���������ⲿ���壬��ÿ��ѭ��֮����ֵ���ᱻ����  
	}
}