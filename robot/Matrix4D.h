#ifndef Matrix4D_H
#define Matrix4D_H
#include <stdexcept>

class Matrix4D{
public:
	static const int ROW = 4;
	static const int COL = 4;
	//メンバ変数
	union{
		struct{
			float	a,b,c,d,
				e,f,g,h,
				i,j,k,l;
		};
		float val[ROW][COL];
	};
	//コンストラクタ
	Matrix4D(){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]=0;
			}
		}
	}
	Matrix4D(float x00, float x01, float x02, float x03,
		  float x10, float x11, float x12, float x13,
		  float x20, float x21, float x22, float x23,
		  float x30, float x31, float x32, float x33){
		  val[0][0]=x00;val[0][1]=x01;val[0][2]=x02;val[0][3]=x03;
		  val[1][0]=x10;val[1][1]=x11;val[1][2]=x12;val[1][3]=x13;
		  val[2][0]=x20;val[2][1]=x21;val[2][2]=x22;val[2][3]=x23;
		  val[3][0]=x30;val[3][1]=x31;val[3][2]=x32;val[3][3]=x33;
	}
	//代入演算子
	Matrix4D& operator=(const Matrix4D& A){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
			 this->val[i][j]=A.val[i][j];
			}
		}
		return *this;
	}
	Matrix4D operator+(){return *this;}	//+Matrix4D
	Matrix4D operator-(){				//-Matrix4D
		Matrix4D A;
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				A.val[i][j]=-val[i][j];
			}
		}
		return A;
	}
	// +=
	Matrix4D& operator+=(const Matrix4D& A){
		for(int i=0;i<ROW;i++){
		   for(int j=0;j<COL;j++){
				val[i][j]+=A.val[i][j];
		   }
		}
		return *this;// IN:Matrix  OUT:this
	}
	// -=
	Matrix4D& operator-=(const Matrix4D& A){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]-=A.val[i][j];
			}
		}
		return *this;// IN:Matrix  OUT:this
	}
	// *=
	Matrix4D& operator*=(float k){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]*=k;
			}
		}
		return *this;// IN:scalar  OUT:this
	}
	// /=
	Matrix4D& operator/=(float k){
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				val[i][j]/=k;
			}
		}
		return *this;// IN:scalar  OUT:this
	}
	//比較演算子
	bool operator==(const Matrix4D& A ) const{
		bool result=true;
		for(int i=0;i<ROW;i++){
			for(int j=0;j<COL;j++){
				result &= (val[i][j]==A.val[i][j]);
			}
		}
		return result;
	}
	bool operator!=(const Matrix4D& A ) const{
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
//Matrix4D+Matrix4D
inline Matrix4D operator+(const Matrix4D& A,const Matrix4D& B){//IN:Matrix4D Matrix4D OUT:Matrix4D
	Matrix4D C;
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++){
			C.val[i][j]=A.val[i][j]+B.val[i][j];
		}
	}
	return C;
}
//Matrix4D-Matrix4D
inline Matrix4D operator-(const Matrix4D& A,const Matrix4D& B){//IN:Matrix4D Matrix4D OUT:Matrix4D
	Matrix4D C;
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++){
			C.val[i][j]=A.val[i][j]-B.val[i][j];
		}
	}
	return C;
}
//float*Matrix4D
inline Matrix4D operator*(float k,const  Matrix4D& A){//IN:Matrix4D Matrix4D OUT:Matrix4D
	Matrix4D B;
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++){
			B.val[i][j]=A.val[i][j]*k;
		}
	}
	return B;
}
//Matrix4D*float
inline Matrix4D operator*(const Matrix4D& A,float k){//IN:Matrix4D Matrix4D OUT:Matrix4D
	Matrix4D B;
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++){
			B.val[i][j]=A.val[i][j]*k;
		}
	}
	return B;
}
//Matrix4D/float
inline Matrix4D operator/(const Matrix4D& A,float k){//IN:Matrix4D Matrix4D OUT:Matrix4D
	Matrix4D B;
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++){
			B.val[i][j]=A.val[i][j]/k;
		}
	}
	return B;
}
//product C=AB//Matrix4D*Matrix4D
inline Matrix4D operator*(const Matrix4D& A,const Matrix4D& B){//IN:Matrix4D Matrix4D OUT:Matrix4D
	Matrix4D C;//C=O
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++){
			for(int k=0;k<Matrix4D::ROW;k++)C.val[i][j]+=A.val[i][k]*B.val[k][j];
		}
	}
	return C;
}//operator*
//product w=Av//Matrix4D*Vector4D
//トレースを取得する
inline float trace(const Matrix4D& A){
	float tr=0;
	for(int i=0;i<Matrix4D::ROW;i++){
		tr+=A.val[i][i];
	}
	return tr;
}
//転置行列を取得する
inline Matrix4D transpose(const Matrix4D& A){
	Matrix4D AT;
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++){
			AT.val[i][j]=A.val[j][i];
		}
	}
	return AT;
}
//行列式を取得する
inline float det(const Matrix4D& A){
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
//出力
#include <iostream>
inline std::ostream& operator<<(std::ostream& s, const Matrix4D& A){
	for(int i=0;i<Matrix4D::ROW;i++){
		for(int j=0;j<Matrix4D::COL;j++)s <<A.val[i][j]<<"\t";
			s <<"\n";
		}
	return s;
}
#endif
