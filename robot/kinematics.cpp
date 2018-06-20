#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>  // abs() for integer
#include <cmath>    // abs() for double, and fabs()
#include "client.h"
#include "kinematics.h"

#define PI 3.141592
#define a1 20
#define a2 165
#define d4 165
#define PHz 100.0

using namespace std;

RobotArm::RobotArm(){
  for (int i = 0;i<6;i++){
  p[i] = 0;
  v[i] = 0;
  a[i] = 0;
  }
}

void RobotArm::setp(double pp[6], double vv[6], double aa[6]){
  for (int i=0;i<6;i++){
    p[i] = pp[i];
    v[i] = vv[i];
    a[i] = aa[i];
  }
}

void RobotArm::setR(double RR[12]){
  for (int i=0;i<12;i++){
    r[i] = RR[i];
  }
}

void RobotArm::EulerAngle(){
  if(abs(r[9] -1.0) < 0.001){
    tx = PI/2;
    ty = 0;
    tz = atan2(r[4], r[0]);
  }else if(abs(r[9]+1.0) < 0.001){
    tx = PI/2;
    ty = 0;
    tz = atan2(r[4], r[0]);
  }else{
    tx = asin(r[9]);
    ty = atan2(-r[8],r[10]);
    tz = atan2(-r[2], r[5]);
  }
}

void RobotArm::InvEulerAngle(){
  r[0] = cos(ty)*cos(tz) - sin(tx)*sin(ty)*sin(tz);
  r[1] = -cos(tx)*sin(tz);
  r[2] = sin(ty)*cos(tz) + sin(tx)*cos(ty)*sin(tz);
  r[4] = cos(ty)*sin(tz) + sin(tx)*sin(ty)*cos(tz);
  r[5] = cos(tx)*cos(tz);
  r[6] = sin(tz)*sin(ty) - sin(tz)*cos(ty)*cos(tz);
  r[8] = -cos(tx)*sin(ty);
  r[9] = sin(tx);
  r[10] = cos(tx)*cos(ty);
}

void RobotArm::khantei(){
  int flag = 0;
  //可動範囲の制限
  if(abs(p[0]) > 2.96706) flag = 1;
  if(p[1] > 1.5708 || p[1] < -1.48353) flag = 1;
  if(p[2] < 0 || p[2] > 2.44346) flag = 1;
  if(abs(p[3]) > 2.44346) flag = 1;
  if(p[4] > 3.66519 || p[4] <-0.523599) flag = 1;
  if(abs(p[5]) > 6.28319) flag = 1;
  if(flag == 1){
    cout << "可動範囲外です" << endl;
    exit(1);
  }
}

void RobotArm::kinematic(){
  r[0] = cos(p[0])*(cos(p[1]+p[2])*(cos(p[3])*cos(p[4])*cos(p[5])-sin(p[3])*sin(p[5]))-sin(p[1]+p[2])*sin(p[4])*cos(p[5]))-sin(p[0])*(sin(p[3])*cos(p[4])*cos(p[5])+cos(p[3])*sin(p[5]));
  r[4] = sin(p[0])*(cos(p[1]+p[2])*(cos(p[3])*cos(p[4])*cos(p[5])-sin(p[3])*sin(p[5]))-sin(p[1]+p[2])*sin(p[4])*cos(p[5]))+cos(p[0])*(sin(p[3])*cos(p[4])*cos(p[5])+cos(p[3])*sin(p[5]));
  r[8] = -sin(p[1]+p[2])*(cos(p[3])*cos(p[4])*cos(p[5])-sin(p[3])*sin(p[5]))-cos(p[1]+p[2])*sin(p[4])*cos(p[5]);
  r[1] = cos(p[0])*(-cos(p[1]+p[2])*(cos(p[3])*cos(p[4])*sin(p[5])+sin(p[3])*cos(p[5]))+sin(p[1]+p[2])*sin(p[4])*sin(p[5]))-sin(p[0])*(-sin(p[3])*cos(p[4])*sin(p[5])+cos(p[3])*cos(p[5]));
  r[5] = sin(p[0])*(-cos(p[1]+p[2])*(cos(p[3])*cos(p[4])*sin(p[5])+sin(p[3])*cos(p[5]))+sin(p[1]+p[2])*sin(p[4])*sin(p[5]))+cos(p[0])*(-sin(p[3])*cos(p[4])*sin(p[5])+cos(p[3])*cos(p[5]));
  r[9] = sin(p[1]+p[2])*(cos(p[3])*cos(p[4])*sin(p[5])+sin(p[3])*cos(p[5]))+cos(p[1]+p[2])*sin(p[4])*sin(p[5]);
  r[2] = cos(p[0])*(cos(p[1]+p[2])*cos(p[3])*sin(p[4])+sin(p[1]+p[2])*cos(p[4]))-sin(p[0])*sin(p[3])*sin(p[4]);
  r[6] = sin(p[0])*(cos(p[1]+p[2])*cos(p[3])*sin(p[4])+sin(p[1]+p[2])*cos(p[4]))+cos(p[0])*sin(p[3])*sin(p[4]);
  r[10] = -sin(p[1]+p[2])*cos(p[3])*sin(p[4])+cos(p[1]+p[2])*cos(p[4]);
  r[3] = cos(p[0])*(sin(p[1]+p[2])*d4+sin(p[1])*a2+a1);
  r[7] = sin(p[0])*(sin(p[1]+p[2])*d4+sin(p[1])*a2+a1);
  r[11] = cos(p[1]+p[2])*d4+cos(p[1])*a2;
}

void RobotArm::inverse_kinematic(){
  p[0] = atan2(r[7], r[3]);
  p[2] = acos((pow(r[3],2)+pow(r[7],2)+pow(r[11],2)-2*(cos(p[0])*r[3]+sin(p[0])*r[7])*a1+pow(a1,2)-pow(d4,2)-pow(a2,2))/(2*d4*a2));
  p[1] = atan2(a2*sin(p[2])*r[11]+(cos(p[0])*r[3]+sin(p[0])*r[7]-a1)*(a2*cos(p[2])+d4), (a2*cos(p[2])+d4)*r[11]-a2*sin(p[2])*(cos(p[0])*r[3]+sin(p[0])*r[7]-a1))-p[2];
  p[3] = atan2(-sin(p[0])*r[2]+cos(p[0])*r[6], cos(p[0])*cos(p[1]+p[2])*r[2]+sin(p[0])*cos(p[1]+p[2])*r[6]-sin(p[1]+p[2])*r[10]);
  p[4] = atan2(r[2]*(cos(p[0])*cos(p[1]+p[2])*cos(p[3])-sin(p[0])*sin(p[3]))+r[6]*(sin(p[0])*cos(p[1]+p[2])*cos(p[3])+cos(p[0])*sin(p[3]))-r[10]*(sin(p[1]+p[2])*cos(p[3])), r[2]*cos(p[0])*sin(p[1]+p[2])+r[6]*sin(p[0])*sin(p[1]+p[2])+r[10]*cos(p[1]+p[2]));
  p[5] = atan2(r[0]*(-cos(p[0])*cos(p[1]+p[2])*sin(p[3])-sin(p[0])*cos(p[3]))+r[4]*(-sin(p[0])*cos(p[1]+p[2])*sin(p[3])+cos(p[0])*cos(p[3]))+r[8]*sin(p[1]+p[2])*sin(p[3]), r[1]*(-cos(p[0])*cos(p[1]+p[2])*sin(p[3])-sin(p[0])*cos(p[3]))+r[5]*(-sin(p[0])*cos(p[1]+p[2])*sin(p[3])+cos(p[0])*cos(p[3]))+r[9]*sin(p[1]+p[2])*sin(p[3]));

  //可動範囲内に変換
  while(p[0] > 2.96706) p[0] -= 6.28319;
  while(p[0] < -2.96706) p[0] += 6.28319;
  while(p[1] > 1.5708) p[1] -= 3.14159;
  while(p[1] < -1.48353) p[1] += 3.14159;
  while(p[2] > 2.44346) p[2] -= 6.28319;
  while(p[2] < 0) p[2] += 6.28319;
  while(p[3] > 2.44346) p[3] -= 6.28319;
  while(p[3] < -2.44346) p[3] += 6.28319;
  while(p[4] > 3.66519) p[4] -= 6.28319;
  while(p[4] < -0.523599) p[4] += 6.28319;
}

void RobotArm::joint_space (double ep[6], double ev[6], double ea[6], double tf){
  double a[6];
  int t = 0;
  string fname;
  for (int i = 0; i < 6; i++){
    a[0] = p[i];
    a[1] = v[i];
    a[2] = a[i]/2;
    a[3] = (20*ep[i]-20*p[i]-(8*ev[i]+12*v[i])*tf-(3*a[i]-ea[i])*pow(tf,2))/(2*pow(tf,3));
    a[4] = (30*p[i]-30*ep[i]+(14*ev[i]+16*v[i])*tf+(3*a[i]-2*ea[i])*pow(tf,2))/(2*pow(tf,4));
    a[5] = (12*ep[i]-12*p[i]-(6*ev[i]+6*v[i])*tf-(a[i]-ea[i])*pow(tf,2))/(2*pow(tf,5));
    fname = "JS";
    fname = fname + char('1'+i);
    ofstream fout;
    fout.open(fname.c_str());
    if(fout.fail()){
      cout << "fileopen failed" << endl;
      exit(1);
    }
    t = 0;
    while (t < tf*PHz){
      fout << a[0]+a[1]*t/PHz+a[2]*pow(t/PHz,2)+a[3]*pow(t/PHz,3)+a[4]*pow(t/PHz,4)+a[5]*pow(t/PHz,5) << endl;
      t++;
    }
    fout.close();
  }
}

void RobotArm::PDControl(double sp[6], double ep[6]){
  double Kp = 0.1;
  double Kd = 0.2;
  double p[6] = {sp[0],sp[1],sp[2],sp[3],sp[4],sp[5]};
  double v[6] = {0,0,0,0,0,0};
  double a[6] = {0,0,0,0,0,0};
  double t[6] = {0,0,0,0,0,0};
  double I[6] = {1,1,1,1,1,1};
  string fname;
  fname = "PD";
  ofstream fout;
  fout.open(fname.c_str());
  if(fout.fail()){
    cout << "fileopen failed" << endl;
    exit(1);
  }
  for (int tim = 0; tim <100; tim++){
    for (int i=0; i<6; i++){
      t[i] = Kp*(ep[i]-p[i])-Kd*v[i];
      a[i] = I[i]*t[i];
      v[i] += a[i];
      p[i] += v[i];
    }
    fout << tim << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" << p[3] << "\t" << p[4] << "\t" << p[5] << "\t" << endl;
  }
  fout.close();
}

void RobotArm::CTM(double sp[6], double ep[6]){
  double Kp = -0.1;
  double Kv = 0.2;
  double p[6] = {sp[0],sp[1],sp[2],sp[3],sp[4],sp[5]};
  double v[6] = {0,0,0,0,0,0};
  double a[6] = {0,0,0,0,0,0};
  double t[6] = {0,0,0,0,0,0};
  double I[6] = {1,1,1,1,1,1};
  string fname;
  fname = "PD";
  ofstream fout;
  fout.open(fname.c_str());
  if(fout.fail()){
    cout << "fileopen failed" << endl;
    exit(1);
  }
  for (int tim = 0; tim <100; tim++){
    for (int i=0; i<6; i++){
      a[i] = -Kp*(ep[i]-p[i])-Kv*v[i];
      t[i] = a[i]/I[i];
      v[i] += a[i];
      p[i] += v[i];
    }
    fout << tim << "\t" << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" << p[3] << "\t" << p[4] << "\t" << p[5] << "\t" << endl;
  }
  fout.close();
}

void RobotArm::R_out(){
  cout << r[0] << "\t" << r[1] << "\t" << r[2] << "\t" << r[3] << endl;
  cout << r[4] << "\t" << r[5] << "\t" << r[6] << "\t" << r[7] << endl;
  cout << r[8] << "\t" << r[9] << "\t" << r[10] << "\t" << r[11] << endl;
  cout << 0 << "\t" << 0 << "\t" << 0 << "\t" << 1 <<endl;
}

void RobotArm::p_out(){
  cout << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" << p[3] << "\t"  << p[4] << "\t" << p[5] << "\t" << endl;
}

int main(){
  Timer tim;
  SocketCommunication so1;
  so1.Init();
  double coo[6] = {185,0,125,180,0,0};
  To_MotoMiniStruct coord;
  for (int i = 0; i < 6; i++){
    coord.elem[i] = coo[i];
  }
  int err = so1.ChangeMode(1);
  err += so1.ChangeSpd(100);
  if (err > 0) exit(1);
  for (int j= 0; j<100;j++){
    tim.StartLoopTime();
    so1.SendCoord(&coord);
    coord.elem[2] = 50*cos(2*PI*j/100)+25;
    coord.elem[0] = 10*sin(2*PI*j/100)+100;
    tim.SleepLoopTime(0.1);
  }
  so1.SendCoord(&coord);
  so1.Terminate();
  /*RobotArm Robot;
  double sp[6] = {2.7,-1.2,2.3,-2,-0.3,6};
  double sv[6] = {0,0,0,0,0,0};
  double sa[6] = {0,0,0,0,0,0};
  double ep[6] = {0,0,0,0,0,0};
  double ev[6] = {0,0,0,0,0,0};
  double ea[6] = {0,0,0,0,0,0};
  Robot.setp(sp, sv, sa);
  Robot.khantei();
  Robot.kinematic();
  Robot.inverse_kinematic();
  Robot.R_out();
  Robot.p_out();
  Robot.joint_space(ep,ev,ea, 3);
  Robot.PDControl(sp,ep);
  */
  return 0;
}

