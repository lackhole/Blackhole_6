#pragma once

#include <math.h>
#include <iostream>
#include <Windows.h>

namespace BH_MATRIX {
	typedef double e_type;

	class Vector4;
	class Matrix4;

	typedef Vector4 Point3;

	namespace CONSTANTS {
		const double PI = atan(1.) * 4.;
	}

	inline double rad2deg(double rad) { return (180. / CONSTANTS::PI) * rad; }
	inline double deg2rad(double deg) { return (CONSTANTS::PI / 180.) * deg; }

#define SIZE_MATRIX 16
#define SIZE_VECTOR 4
const int SIZE_ELEM4=sizeof(e_type) * 4;

	class Vector4 {
	public:
		e_type elem[4];

		Vector4() :elem{ 0 } {}
		Vector4(e_type x, e_type y, e_type z, e_type w = 0) : elem{ x,		y,		z,		w } {};
		Vector4(e_type arr[3], e_type w) : elem{ arr[0], arr[1], arr[2], w } {
		};
		Vector4(e_type arr[4]){
			memcpy(elem, arr, sizeof(elem));
		};

		Vector4(const Vector4& vec) : elem{ 0 } {
			memcpy(elem, vec.elem, sizeof(elem));
		}
		Vector4(const Vector4&& vec) : elem{ 0 } {
			memcpy(elem, vec.elem, sizeof(elem));
		}
		Vector4& operator=(const Vector4& vec) {
			memcpy(elem, vec.elem, sizeof(elem));
			return *this;
		}
		Vector4& operator=(const Vector4&& vec) {
			memcpy(elem, vec.elem, sizeof(elem));
			return *this;
		}

		e_type operator[](int index) const { return elem[index]; }

		Vector4 operator-(const Vector4& vec) const {
			e_type res[4] = { 0 };
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i) {
				res[i] = elem[i] - vec.elem[i];
			}
			return Vector4(res);
		}
		Vector4 operator+(const Vector4& vec) const {
			e_type res[4] = { 0 };
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i) {
				res[i] = elem[i] + vec.elem[i];
			}
			return Vector4(res);
		}

		Vector4& operator+=(const Vector4& vec) {
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				elem[i] += vec.elem[i];
			return *this;
		}
		Vector4& operator-=(const Vector4& vec) {
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				elem[i] -= vec.elem[i];
			return *this;
		}

		Vector4 operator+(e_type num) const {
			e_type res[4] = { 0 };
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				res[i] = elem[i] + num;
			return Vector4(res);
		}
		Vector4 operator-(e_type num) const {
			e_type res[4] = { 0 };
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				res[i] = elem[i] - num;
			return Vector4(res);
		}
		Vector4 operator*(e_type num) const {
			e_type res[4] = { 0 };
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				res[i] = elem[i] * num;
			return Vector4(res);
		}
		Vector4 operator/(e_type num) const {
			e_type res[4] = { 0 };
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				res[i] = elem[i] / num;
			return Vector4(res);
		}

		Vector4& operator+=(e_type num) {
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)	elem[i] += num;
			return *this;
		}
		Vector4& operator-=(e_type num) {
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				elem[i] -= num;
			return *this;
		}
		Vector4& operator*=(e_type num) {
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				elem[i] *= num;
			return *this;
		}
		Vector4& operator/=(e_type num) {
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				elem[i] /= num;
			return *this;
		}
		

		double Dot(const Vector4& vec) const {
			double w = 0.;
#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				w += elem[i] * vec.elem[i];
			return w;
		}
		Vector4 Product(const Vector4& vec) const {
			e_type arr[3] = { 0 };

#pragma loop(hint_parallel(4))
			for (int i = 0, l = 1, r = 2; i < 3; ++i, ++r %= 3, ++l %= 3) {
				arr[i] = elem[l] * vec.elem[r] - elem[r] * vec.elem[l];
			}

			return (Vector4(arr, 0));
		}
		Vector4& Normalize3() {
			e_type sum = 0;

			#pragma loop(hint_parallel(3))
			for (int i = 0; i < 3; ++i)
				sum += elem[i] * elem[i];

			e_type size = sqrt(sum);
			for (int i = 0; i < 3; ++i)
				elem[i] /= size;
			
			return *this;
		}
		Vector4& Normalize4() {
			return operator/=(Size());
		}

		double Size3() const {
			e_type sum = 0;
			for (int i = 0; i < 3; ++i)
				sum += elem[i] * elem[i];
			return sqrt(sum);
		}

		double Size() const {
			e_type sum = 0;
#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				sum += elem[i] * elem[i];
			return sqrt(sum);
		}

		double AngleWith(const Vector4& vec) const {
			return acos(Dot(vec) / (Size() * vec.Size()));
		}

		void print() const {
#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i) {
				printf("%8f ", elem[i]);
			}
			std::cout << std::endl;
		}

	};

	class Matrix4 {
	private:
		inline int MONE_POW_IJ(int i, int j) const {return 1 - 2*((i+j)%2);};
	public:
		e_type elem[SIZE_MATRIX];

		Matrix4() :elem{ 0 } {};
		Matrix4(
			e_type e11, e_type e12, e_type e13,
			e_type e21, e_type e22, e_type e23,
			e_type e31, e_type e32, e_type e33
		) :elem{
			e11,e12,e13,0,
			e21,e22,e23,0,
			e31,e32,e33,0,
			0  ,0  ,0  ,1 }
		{}
		Matrix4(
			e_type e11, e_type e12, e_type e13, e_type e14,
			e_type e21, e_type e22, e_type e23, e_type e24,
			e_type e31, e_type e32, e_type e33, e_type e34,
			e_type e41, e_type e42, e_type e43, e_type e44)
			:elem{
			e11,e12,e13,e14,
			e21,e22,e23,e24,
			e31,e32,e33,e34,
			e41,e42,e43,e44 }
		{}
		Matrix4(e_type arr[SIZE_MATRIX]){
			memcpy(elem, arr, sizeof(elem));
		}

		Matrix4(const Matrix4& mat){
			memcpy(elem, mat.elem, sizeof(elem));
		}
		Matrix4(const Matrix4&& mat) noexcept{
			memcpy(elem, mat.elem, sizeof(elem));
		}

		static Matrix4 Zeros() {
			static Matrix4 matrix(
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0);
			return matrix;
		}
		static Matrix4 Ones() {
			static Matrix4 matrix(
				1,1,1,1,
				1,1,1,1,
				1,1,1,1,
				1,1,1,1);
			return matrix;
		}
		static Matrix4 Eye() {
			static Matrix4 matrix(
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1);
			return matrix;
		}


		double det() const {
			return 
				(  elem[0]*(elem[5]*elem[10] - elem[6]*elem[9])
				- elem[1]*(elem[4]*elem[10] - elem[6]*elem[8])
				+ elem[2]*(elem[4]*elem[9]  - elem[5]*elem[8]));
		}
		Matrix4 reverse() const {
			return (Matrix4(
				  elem[5]*elem[10] - elem[9]*elem[6],  -(elem[1]*elem[10] - elem[9]*elem[2]),   elem[1]*elem[6] - elem[5]*elem[2],
				-(elem[4]*elem[10] - elem[8]*elem[6]),   elem[0]*elem[10] - elem[8]*elem[2],  -(elem[0]*elem[6] - elem[4]*elem[2]),
				  elem[4]*elem[9]  - elem[8]*elem[5],  -(elem[0]*elem[9]  - elem[8]*elem[1]),   elem[0]*elem[5] - elem[4]*elem[1])/this->det());
		}

		Matrix4& operator=(const Matrix4& mat) {
			memcpy(elem, mat.elem, sizeof(elem));
			return *this;
		}
		Matrix4& operator=(const Matrix4&& mat) noexcept{
			memcpy(elem, mat.elem, sizeof(elem));
			return *this;
		}

		Matrix4 operator*(e_type number) const {
			Matrix4 matrix(*this);

			#pragma loop(hint_parallel(4))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				matrix.elem[i] *= number;
			return matrix;
		}
		Matrix4 operator*(const Matrix4& mat) const {
			Matrix4 matrix;

			for (int i = 0; i < SIZE_MATRIX; ++i) {
				#pragma loop(hint_parallel(4))
				for (int j = 0, row = (i / 4) * 4, col = i % 4; j < 4; ++j) {
					matrix.elem[i] += elem[row + j] * mat.elem[col + 4 * j];
				}
			}

			return matrix;
		}
		Vector4 operator*(const Vector4& vec) const {
			e_type res[4] = { 0 };

			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i) {
				res[i / 4] += elem[i] * vec.elem[i % 4];
			}

			return Vector4(res);
		}

		Matrix4& operator*=(e_type number) {
			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				elem[i] *= number;

			return *this;
		}
		Matrix4& operator*=(const Matrix4& mat) {
			e_type m_elem[SIZE_MATRIX] = { 0 };

			for (int i = 0; i < SIZE_MATRIX; ++i) {
				#pragma loop(hint_parallel(4))
				for (int j = 0, row = (i / 4) * 4, col = i % 4; j < 4; ++j) {
					m_elem[i] += elem[row + j] * mat.elem[col + 4 * j];
				}
			}

			memcpy(elem, m_elem, sizeof(elem));

			return *this;
		}

		Matrix4 operator+(e_type number) const {
			Matrix4 matrix(*this);

			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				matrix.elem[i] += number;
			return matrix;
		}
		Matrix4& operator+=(e_type number) {
			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				elem[i] += number;

			return *this;
		}

		Matrix4 operator-(e_type number) const {
			Matrix4 matrix(*this);

			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				matrix.elem[i] -= number;
			return matrix;
		}
		Matrix4& operator-=(e_type number) {
			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				elem[i] -= number;

			return *this;
		}

		Matrix4 operator/(e_type number) const {
			Matrix4 matrix(*this);

			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				matrix.elem[i] /= number;
			return matrix;
		}
		Matrix4& operator/=(e_type number) {
			#pragma loop(hint_parallel(SIZE_MATRIX))
			for (int i = 0; i < SIZE_MATRIX; ++i)
				elem[i] /= number;

			return *this;
		}

		Matrix4& rotate(double x, double y, double z) {



			(*this) *= (
				Matrix4(
					cos(z), -sin(z), 0,
					sin(z), cos(z), 0,
					0, 0, 1)
				*
				Matrix4(
					cos(y), 0, sin(y),
					0, 1, 0,
					-sin(y), 0, cos(y))
				*
				Matrix4(
					1, 0, 0,
					0, cos(x), -sin(x),
					0, sin(x), cos(x)));


			return *this;
		}

		static Matrix4 rotateAroundAxis(Vector4 basisVector, double rad) {
			static Matrix4 temp;
			double sin_rad = sin(rad), cos_rad = cos(rad);

			basisVector.Normalize3();
			temp = Matrix4::Zeros();

			for (int i = 0; i < 3; ++i) {
				#pragma loop(hint_parallel(3))
				for (int j = 0; j < 3; ++j) {
					temp.elem[i*4+j] += basisVector.elem[i] * basisVector.elem[j] * (1 - cos_rad);
				}
			}

			temp.elem[0]+=cos_rad;
			temp.elem[1]-=basisVector[2]*sin_rad;
			temp.elem[2]+=basisVector[1]*sin_rad;

			temp.elem[4]+=basisVector[2]*sin_rad;
			temp.elem[5]+=cos_rad;
			temp.elem[6]-=basisVector[0]*sin_rad;

			temp.elem[8]-=basisVector[1]*sin_rad;
			temp.elem[9]+=basisVector[0]*sin_rad;
			temp.elem[10]+=cos_rad;

			return temp;
		}


		void print() const {
			for (int i = 0; i < 4; ++i) {
				#pragma loop(hint_parallel(4))
				for (int j = 0; j < 4; ++j) {
					printf("%8lf ", (double)(elem[i * 4 + j]));
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
		}
	};

	/*
	Rotates target vector match as source vector
	*/
	Matrix4 conversionMatrix(const Vector4& target,const Vector4& src) {
		Vector4 vector_product = target.Product(src);
		vector_product.Normalize3();
		const double
			& l = vector_product[0],
			& m = vector_product[1],
			& n = vector_product[2];

		double angle = target.AngleWith(src);
		double c = 1.-cos(angle);

		return Matrix4 (
			l*l*c +   cos(angle),	m*l*c - n*sin(angle),	n*l*c + m*sin(angle),
			l*m*c + n*sin(angle),	m*m*c +   cos(angle),	n*m*c - l*sin(angle),
			l*n*c - m*sin(angle),	m*n*c + l*sin(angle),	n*n*c +   cos(angle)
			);		
	}
	
	double distanceBetweenLineSegmentAndPoint(const Vector4& l1, const Vector4& l2, const Vector4 point) {
		Vector4 p_close = l1, p_far = l1;
		if((l1 - point).Size3() > (l2 - point).Size3())
			p_close = l2;
		else
			p_far = l2;

		Vector4 u = p_close - p_far, vec_l1 = p_far - point;
		double lineSegmentLength = u.Size3();
		if(isnan(lineSegmentLength))
			return -1;

		double theta = u.AngleWith(vec_l1);
		double r = vec_l1.Size3();
		double d = sin(theta) * r;
		Vector4 vec_l1_d = (p_close - p_far).Normalize3() * r*cos(theta);
		if((vec_l1_d.Size3() > u.Size3())){
			return -1;
		}
		return d;
	}
};
