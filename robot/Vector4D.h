#ifndef Vector4D_H
#define Vector4D_H

class Vector4D{
public:
	static const int ROW = 4;
	//メンバ変数
	union{
		struct{
			float	x,
				y,
				z,
				w;
		};
		float val[ROW];
	};
	//コンストラクタ
	Vector4D();
	Vector4D(float x,float y,float z, float w);
	//代入演算子
	Vector4D& operator=(const Vector4D& v);
	//単項演算子
	Vector4D& operator+=(const Vector4D& v);
	Vector4D& operator-=(const Vector4D& v);
	Vector4D& operator*=(float k);
	Vector4D& operator/=(float k);
	Vector4D operator+()const;
	Vector4D operator-()const;
	//添え字演算子
	float& operator[](int i);
    //比較演算子
	bool operator==(const Vector4D& v ) const;
	bool operator!=(const Vector4D& v ) const;
	//べクトルの長さ
	float norm()const;
	//正規化
	void normalize();
};
//ベクトル演算
//Vector4D+Vector4D
Vector4D operator+(const Vector4D& u,const Vector4D& v);
//Vector4D-Vector4D
Vector4D operator-(const Vector4D& u,const Vector4D& v);
//float*Vector4D
Vector4D operator*(float k,const  Vector4D& v);
//Vector4D*float
Vector4D operator*(const Vector4D& v,float k);
//Vector4D/float
Vector4D operator/(const Vector4D& v,float k);
//内積 Vector4D*Vector4D
float operator*(const Vector4D& u,const Vector4D& v);
//外積 Vector4D%Vector4D
Vector4D operator%(const Vector4D& u,const Vector4D& v);
//2つのベクトルのなす角度
float angle(const Vector4D& u,const Vector4D& v);
//出力
#include <iostream>
inline std::ostream& operator<<(std::ostream& s, const Vector4D& v);
//*----------------------メンバ関数の実装--------------------------*//
#include <cmath>
//コンストラクタ
inline Vector4D::Vector4D(){ x = y = z = 0; }
inline Vector4D::Vector4D(float x,float y,float z){
	this->x=x;		this->y=y;		this->z=z;
}
//代入演算子
inline Vector4D& Vector4D::operator=(const Vector4D& v){
	this->x=v.x;	this->y=v.y;	this->z=v.z;
	return *this;
}
//単項演算子
inline Vector4D& Vector4D::operator+=(const Vector4D& v){
	 this->x += v.x;	this->y += v.y;		this->z += v.z;
	 return *this;
}
inline Vector4D& Vector4D::operator-=(const Vector4D& v){
	 this->x -= v.x;	this->y -= v.y;		this->z -= v.z;
	 return *this;
}
inline Vector4D& Vector4D::operator*=(float k){
	 this->x *= k;		this->y *= k;		this->z *= k;
	 return *this;
}
inline Vector4D& Vector4D::operator/=(float k){
	this->x /= k;		this->y /= k;		this->z /= k;
	return *this;
}
inline Vector4D Vector4D::operator+()const{		//+Vector4D
	return *this;
}
inline Vector4D Vector4D::operator-()const{		//-Vector4D
	return Vector4D(-x,-y,-z);
}
//添え字演算子
inline float& Vector4D::operator[](int i){
	if(i == 0){
		return x;
	}
	else if(i == 1){
		return y;
	}
	else if(i == 2){
		return z;
	}
	else{
		return x;
	}
}
//比較演算子
inline bool Vector4D::operator==(const Vector4D& v ) const{
    return (x == v.x) && (y == v.y) && (z == v.z);
}
inline bool Vector4D::operator!=(const Vector4D& v ) const{
    return !(*this == v);
}
//べクトルの長さ
inline float Vector4D::norm()const{
	return pow(x*x+y*y+z*z,0.5f);
}
//正規化
inline void Vector4D::normalize(){
	*this /= norm();
}
//*----------------------グローバル関数の実装--------------------------*//
//二項演算子の定義
//Vector4D+Vector4D
inline Vector4D operator+(const Vector4D& u,const Vector4D& v){
	Vector4D w;
	w.x=u.x+v.x;
	w.y=u.y+v.y;
	w.z=u.z+v.z;
	return w;
}
//Vector4D-Vector4D
inline Vector4D operator-(const Vector4D& u,const Vector4D& v){
	Vector4D w;
	w.x=u.x-v.x;
	w.y=u.y-v.y;
	w.z=u.z-v.z;
	return w;
}
//float*Vector4D
inline Vector4D operator*(float k,const  Vector4D& v){
	return Vector4D(k*v.x,k*v.y,k*v.z);
}
//Vector4D*float
inline Vector4D operator*(const Vector4D& v,float k){
	return Vector4D(v.x*k,v.y*k,v.z*k);
}
//Vector4D/float
inline Vector4D operator/(const Vector4D& v,float k){
	return Vector4D(v.x/k,v.y/k,v.z/k);
}
//内積 Vector4D*Vector4D
inline float operator*(const Vector4D& u,const Vector4D& v){
	return u.x*v.x+u.y*v.y+u.z*v.z;
}
//外積 Vector4D%Vector4D
inline Vector4D operator%(const Vector4D& u,const Vector4D& v){
	Vector4D w;
	w.x=u.y*v.z-u.z*v.y;
	w.y=u.z*v.x-u.x*v.z;
	w.z=u.x*v.y-u.y*v.x;
	return w;
}
//画面への表示
#include <iostream>
inline std::ostream& operator<<(std::ostream& s, const Vector4D& v){
	return s <<'('<<v.x<<","<<v.y<<","<<v.z<<')';
}

#define PI 3.1415926535
//2つのベクトルのなす角
inline float angle(const Vector4D& u,const Vector4D& v){
	float cos =u*v/(u.norm()*v.norm());
	return float(acos(cos)/PI*180);
}

#endif
