#ifndef Matrix3D_H
#define Matrix3D_H
#include <stdexcept>
#include "Vector3D.h"

class Matrix3D{
public:
	static const int ROW = 3;
	static const int COL = 3;
	//メンバ変数
	union{
		struct{
			float	a,b,c,
				d,e,f,
				g,h,i;
		};
		float val[ROW][COL];
	};
	//コンストラクタ
	Matrix3D(){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]=0;
			}
		}
	}
	Matrix3D(float x00, float x01, float x02,
		  float x10, float x11, float x12,
		  float x20, float x21, float x22){
		  val[0][0]=x00;val[0][1]=x01;val[0][2]=x02;
		  val[1][0]=x10;val[1][1]=x11;val[1][2]=x12;
		  val[2][0]=x20;val[2][1]=x21;val[2][2]=x22;
	}
	//代入演算子
	Matrix3D& operator=(const Matrix3D& A){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
			 this->val[i][j]=A.val[i][j];
			}
		}
		return *this;
	}
	Matrix3D operator+(){return *this;}	//+Matrix3D
	Matrix3D operator-(){				//-Matrix3D
		Matrix3D A;
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				A.val[i][j]=-val[i][j];
			}
		}
		return A;
	}
	// +=
	Matrix3D& operator+=(const Matrix3D& A){
		for(int i=0;i<ROW;i++){
		   for(int j=0;j<COL;j++){
				val[i][j]+=A.val[i][j];
		   }
		}
		return *this;// IN:Matrix  OUT:this
	}
	// -=
	Matrix3D& operator-=(const Matrix3D& A){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]-=A.val[i][j];
			}
		}
		return *this;// IN:Matrix  OUT:this
	}
	// *=
	Matrix3D& operator*=(float k){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]*=k;
			}
		}
		return *this;// IN:scalar  OUT:this
	}
	// /=
	Matrix3D& operator/=(float k){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]/=k;
			}
		}
		return *this;// IN:scalar  OUT:this
	}
	//比較演算子
	bool operator==(const Matrix3D& A ) const{
		bool result=true;
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				result &= (val[i][j]==A.val[i][j]);
			}
		}
		return result;
	}
	bool operator!=(const Matrix3D& A ) const{
		return !((*this) == A);
	}
	//添え字演算子
	float* operator[](int i){
		return val[i];
	}
	float& operator()(int i,int j){
		return val[i][j];
	}

};
//Matrix3D+Matrix3D
inline Matrix3D operator+(const Matrix3D& A,const Matrix3D& B){//IN:Matrix3D Matrix3D OUT:Matrix3D
	Matrix3D C;
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++){
			C.val[i][j]=A.val[i][j]+B.val[i][j];
		}
	}
	return C;
}
//Matrix3D-Matrix3D
inline Matrix3D operator-(const Matrix3D& A,const Matrix3D& B){//IN:Matrix3D Matrix3D OUT:Matrix3D
	Matrix3D C;
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++){
			C.val[i][j]=A.val[i][j]-B.val[i][j];
		}
	}
	return C;
}
//float*Matrix3D
inline Matrix3D operator*(float k,const  Matrix3D& A){//IN:Matrix3D Matrix3D OUT:Matrix3D
	Matrix3D B;
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++){
			B.val[i][j]=A.val[i][j]*k;
		}
	}
	return B;
}
//Matrix3D*float
inline Matrix3D operator*(const Matrix3D& A,float k){//IN:Matrix3D Matrix3D OUT:Matrix3D
	Matrix3D B;
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++){
			B.val[i][j]=A.val[i][j]*k;
		}
	}
	return B;
}
//Matrix3D/float
inline Matrix3D operator/(const Matrix3D& A,float k){//IN:Matrix3D Matrix3D OUT:Matrix3D
	Matrix3D B;
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++){
			B.val[i][j]=A.val[i][j]/k;
		}
	}
	return B;
}
//product C=AB//Matrix3D*Matrix3D
inline Matrix3D operator*(const Matrix3D& A,const Matrix3D& B){//IN:Matrix3D Matrix3D OUT:Matrix3D
	Matrix3D C;//C=O
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++){
			for(int k=0;k<Matrix3D::ROW;k++)C.val[i][j]+=A.val[i][k]*B.val[k][j];
		}
	}
	return C;
}//operator*
//product w=Av//Matrix3D*Vector3D
inline Vector3D operator*(const Matrix3D& A,const Vector3D& v){//IN:Vector3D Matrix3D OUT:Vector3D 
	Vector3D w;//w=O

	for(int i=0;i<Matrix3D::ROW;i++){
		w.val[i] = 0;
		for(int j=0;j<Matrix3D::COL;j++){
			w.val[i] += v.val[j]*A.val[i][j];
		}
	}
	return w;
}//operator*
//トレースを取得する
inline float trace(const Matrix3D& A){
	float tr=0;
	for(int i=0;i<Matrix3D::ROW;i++){
		tr+=A.val[i][i];
	}
	return tr;
}
//転置行列を取得する
inline Matrix3D transpose(const Matrix3D& A){
	Matrix3D AT;
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++){
			AT.val[i][j]=A.val[j][i];
		}
	}
	return AT;
}
//行列式を取得する
inline float det(const Matrix3D& A){
	//Sarrusの方法
	float d = 0;
	d+=A.val[0][0]*A.val[1][1]*A.val[2][2];
	d+=A.val[0][1]*A.val[1][2]*A.val[2][0];
	d+=A.val[0][2]*A.val[1][0]*A.val[2][1];
	d-=A.val[0][2]*A.val[1][1]*A.val[2][0];
	d-=A.val[0][0]*A.val[1][2]*A.val[2][1];
	d-=A.val[0][1]*A.val[1][0]*A.val[2][2];
	return d;
}
//逆行列を取得する
inline Matrix3D inverse(const Matrix3D& A){
	Matrix3D Ai;
	float d=det(A);

	if(d == 0){
		throw std::overflow_error("inverse():逆行列は存在しません");//error処理
	}
	//余因子展開,cramerの公式
	Ai.val[0][0]=(A.val[1][1]*A.val[2][2]-A.val[1][2]*A.val[2][1])/d;
	Ai.val[0][1]=-(A.val[0][1]*A.val[2][2]-A.val[0][2]*A.val[2][1])/d;
	Ai.val[0][2]=(A.val[0][1]*A.val[1][2]-A.val[1][1]*A.val[0][2])/d;

	Ai.val[1][0]=-(A.val[1][0]*A.val[2][2]-A.val[1][2]*A.val[2][0])/d;
	Ai.val[1][1]=(A.val[0][0]*A.val[2][2]-A.val[0][2]*A.val[2][0])/d;
	Ai.val[1][2]=-(A.val[0][0]*A.val[1][2]-A.val[1][0]*A.val[0][2])/d;

	Ai.val[2][0]=(A.val[1][0]*A.val[2][1]-A.val[1][1]*A.val[2][0])/d;
	Ai.val[2][1]=-(A.val[0][0]*A.val[2][1]-A.val[0][1]*A.val[2][0])/d;
	Ai.val[2][2]=(A.val[0][0]*A.val[1][1]-A.val[0][1]*A.val[1][0])/d;
	return Ai;
}
//出力
#include <iostream>
inline std::ostream& operator<<(std::ostream& s, const Matrix3D& A){
	for(int i=0;i<Matrix3D::ROW;i++){
		for(int j=0;j<Matrix3D::COL;j++)s <<A.val[i][j]<<"\t";
			s <<"\n";
		}
	return s;
}
#endif
