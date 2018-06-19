#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>  // abs() for integer
#include <cmath>    // abs() for float, and fabs()
#include "Matrix3D.h"

#define a1 20
#define a2 165
#define d4 165
#define PHz 100.0

using namespace std;

void khantei(float p[6]){
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

void matrix_out(float r[12]){
  cout << r[0] << "\t" << r[1] << "\t" << r[2] << "\t" << r[3] << endl;
  cout << r[4] << "\t" << r[5] << "\t" << r[6] << "\t" << r[7] << endl;
  cout << r[8] << "\t" << r[9] << "\t" << r[10] << "\t" << r[11] << endl;
  cout << 0 << "\t" << 0 << "\t" << 0 << "\t" << 1 <<endl;
}

void vector_out(float p[6]){
  cout << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" << p[3] << "\t"  << p[4] << "\t" << p[5] << "\t" << endl;
}

void kinematic(float p[6], float r[12]){
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

void inverse_kinematic(float r[12], float p[6]){
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

void joint_space (float sp[6], float sv[6], float sa[6], float ep[6], float ev[6], float ea[6], float tf){
  float a[6];
  int t = 0;
  string fname;
  for (int i = 0; i < 6; i++){
    a[0] = sp[i];
    a[1] = sv[i];
    a[2] = sa[i]/2;
    a[3] = (20*ep[i]-20*sp[i]-(8*ev[i]+12*sv[i])*tf-(3*sa[i]-ea[i])*pow(tf,2))/(2*pow(tf,3));
    a[4] = (30*sp[i]-30*ep[i]+(14*ev[i]+16*sv[i])*tf+(3*sa[i]-2*ea[i])*pow(tf,2))/(2*pow(tf,4));
    a[5] = (12*ep[i]-12*sp[i]-(6*ev[i]+6*sv[i])*tf-(sa[i]-ea[i])*pow(tf,2))/(2*pow(tf,5));
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

void construct_matrix(float p[6], Matrix3D R[6], Vector3D P[6]){
  Matrix3D r0(	cos(p[0]), -sin(p[0]), 0,
		sin(p[0]), cos(p[0]), 0,
		0, 0, 1);
  Vector3D p0(0,0,0);
  R[0] = r0;
  P[0] = p0;
  Matrix3D r1(	sin(p[1]), cos(p[1]), 0,
		0, 0, 1,
		cos(p[1]), -sin(p[1]), 0);
  Vector3D p1(a1,0,0);
  R[1] = r1;
  P[1] = p1;
  Matrix3D r2(	-sin(p[2]), -cos(p[2]), 0,
		cos(p[2]), sin(p[2]), 0,
		0, 0, 1);
  Vector3D p2(a2,0,0);
  R[2] = r2;
  P[2] = p2;
  Matrix3D r3(	cos(p[3]), -sin(p[3]), 0,
		0, 0, -1,
		sin(p[3]), cos(p[3]), 0);
  Vector3D p3(0,-d4,0);
  R[3] = r3;
  P[3] = p3;
  Matrix3D r4(	cos(p[4]), -sin(p[4]), 0,
		0, 0, 1,
		-sin(p[4]), -cos(p[4]), 0);
  Vector3D p4(0,0,0);
  R[4] = r4;
  P[4] = p4;
  Matrix3D r5(	cos(p[5]), -sin(p[5]), 0,
		0, 0, -1,
		sin(p[5]), cos(p[5]), 0);
  Vector3D p5(0,0,0);
  R[5] = r5;
  P[5] = p5;
}

void INE(float p[6], float pd[6], float pdd[6]){
  Vector3D w[7], wd[7], vd[7], vCd[7], F[7], N[7];
  Matrix3D R[6];
  Vector3D P[6];
  Vector3D a(0,0,0);
  w[0] = a;
  wd[0] = a;
  vd[0] = a;
  vCd[0] = a;
  F[0] = a;
  N[0] = a;
  construct_matrix(p, R, P);
  for(int i; i < 6; i++){
    Vector3D pdZ(0,0,pd[i]);
    Vector3D pddZ(0,0,pdd[i]);
    w[i+1] = R[i]*w[i]+pdZ;
    wd[i+1] = R[i]*wd[i]+(R[i]*w[i])%pdZ+pddZ;
    vd[i+1] = R[i]*(wd[i]%P[i]+w[i]%(w[i]%P[i])+vd[i]);
  }
}

void PDControl(float sp[6], float ep[6]){
  float Kp = 0.1;
  float Kd = 0.2;
  float p[6] = {sp[0],sp[1],sp[2],sp[3],sp[4],sp[5]};
  float v[6] = {0,0,0,0,0,0};
  float a[6] = {0,0,0,0,0,0};
  float t[6] = {0,0,0,0,0,0};
  float I[6] = {1,1,1,1,1,1};
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

float CTM(float sp[6], float ep[6]){
  float Kp = -0.1;
  float Kv = 0.2;
  float p[6] = {sp[0],sp[1],sp[2],sp[3],sp[4],sp[5]};
  float v[6] = {0,0,0,0,0,0};
  float a[6] = {0,0,0,0,0,0};
  float t[6] = {0,0,0,0,0,0};
  float I[6] = {1,1,1,1,1,1};
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

int main(){
  float p[6] = {2.7,-1.2,2.3,-2,-0.3,6};
  float ip[6];
  float r[12];
  float sp[6] = {2.7,-1.2,2.3,-2,-0.3,6};
  float sv[6] = {0,0,0,0,0,0};
  float sa[6] = {0,0,0,0,0,0};
  float ep[6] = {0,0,0,0,0,0};
  float ev[6] = {0,0,0,0,0,0};
  float ea[6] = {0,0,0,0,0,0};
  khantei(p);
  kinematic(p, r);
  matrix_out(r);
  inverse_kinematic(r,ip);
  vector_out(ip);
  PDControl(sp,ep);
  joint_space (sp, sv, sa, ep, ev, ea, 3);
  return 0;
}

