#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

class RobotArm
{
private:
	double p[6],v[6],a[6];
	double r[12];
	double x,y,z;
	double tx,ty,tz;
public:
	RobotArm();
	void setp(double pp[6], double vv[6], double aa[6]);
	void setR(double RR[12]);
	void EulerAngle();
	void InvEulerAngle();
	void khantei();
	void kinematic();
	void inverse_kinematic();
	void joint_space(double ep[6], double ev[6], double ea[6], double tf);
	void PDControl(double sp[6], double ep[6]);
	void CTM(double sp[6], double ep[6]);
	void R_out();
	void p_out();
};

#endif
