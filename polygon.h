#pragma once

#include <iostream>
#include <math.h>
#include <vector>

#include "opencv2\highgui\highgui.hpp" 
#include "opencv2\core\core.hpp" 
#include "opencv2\opencv.hpp"

#include "matrix.h"
#include "physics.h"

using namespace BH_MATRIX;
using std::cout;
using std::endl;
using std::string;
using cv::imread;
//using cv::Vec3b;


namespace bh_poly {
	enum type {
		TRIANGLE, RECTANGLE
	};

	const int UCHAR3 = sizeof(uchar) * 3;
	const int UCHAR4 = sizeof(uchar) * 3;
	const double SMALL_DOUBLE = 1e-5;

	typedef double v_type;
	class Rectangle;
	class Material;
	class Object;
	class Object3D;
	class ObjectManager;

	void inject(v_type* Dst_verticeArray, const Vector4& src) {
		e_type const* elem = src.elem;
		for (int i = 0; i < 3; ++i)
			Dst_verticeArray[i] = elem[i];
	}


	class Material {
	public:

		cv::Mat texture;

		Material() : texture() {
		}

		int setTexture(const std::string& path, int flag) {
			cv::Mat temp = cv::imread(path, flag);
			texture = temp;

			if (texture.empty() || texture.data == NULL) {
				printf("  ! Error: ");
				cout << "File " << path << " not found!" << endl;
				return 0;
			}
			return 1;
		}

		int setColor(const cv::Vec3b& color) {
			texture = cv::Mat3b(100, 100, CV_8UC3);
			texture = color;
			return 1;
		}

	};

	// Virtual calss
	class Object {
	public:
		std::function<void(long double)> motion;
		Material material;
		bool isCurved;

		Object()
			:motion([](long double t)->void{return;}), material(), isCurved(false)
		{}

		virtual bool isInPlane(double x, double y, double z, long double t) const = 0;
		virtual bool isInPlane(const Vector4& vec) const = 0;
		virtual double calculate(double x, double y, double z, long double t) const = 0;
		virtual double calculate(const Vector4& vec) const = 0;
		virtual cv::Vec3b GetPixel(double x, double y, double z, long double t) const = 0;
		virtual cv::Vec3b GetPixel(const Vector4& vec) const = 0;
		virtual void Move(double x, double y, double z) = 0;
		virtual void Move(const Vector4& vec) = 0;
		virtual void Rotate(double x, double y, double z) = 0;
		virtual void Rotate(const Vector4& vec) = 0;
		virtual void Scale(double x, double y, double z) = 0;
		//virtual void alpha_area_test(double x, double y, double z, double radius) = 0;

	};

	




	

	class Sphere : public Object {
	public:
		Vector4 pos;
		double radius;
		Material material;

		bool doesIntersect(const Vector4& p1, const Vector4& p2) {
			double d1 = (p1-pos).Size3(), d2 = (p2-pos).Size3();
			//if((d1-radius)*(d2-radius) < radius*radius)
			if((d1-radius)*(d2-radius) < 0)
				return true;
			else if (d1 < radius && d2 < radius)
				return false;
			double d = distanceBetweenLineSegmentAndPoint(p1, p2, pos);
			if(d >= 0 && d <= radius)
				return true;
			else
				return false;
		}
		inline double distance(const Vector4& vec) const { return (vec - pos).Size3(); }

		Sphere(double x, double y, double z, double radius) : pos(x, y, z), radius(radius) {
			isCurved = true;
		};
		void setColor(const cv::Vec3b& color) {
			material.setColor(color);
		}

		bool isInPlane(double x, double y, double z, long double t) const override {
			return distance(Vector4(x, y, z)) <= radius;
		}

		bool isInPlane(const Vector4& vec) const override {
			return distance(vec) <= radius;
		}
		double calculate(double x, double y, double z, long double t) const override {
			return (x-pos[0])*(x-pos[0]) + (y-pos[1])*(y-pos[1]) + (z-pos[2])*(z-pos[2]) - radius*radius;
		}
		double calculate(const Vector4& vec) const override {
			return (vec[0] - pos[0]) * (vec[0] - pos[0]) + (vec[1] - pos[1]) * (vec[1] - pos[1]) + (vec[2] - pos[2]) * (vec[2] - pos[2]) - radius * radius;
		}
		cv::Vec3b GetPixel(double x, double y, double z, long double t) const override {
			return material.texture.at<cv::Vec3b>(0,0);
		}
		cv::Vec3b GetPixel(const Vector4& vec) const override {
			return material.texture.at<cv::Vec3b>(0, 0);
		}
		void Move(double x, double y, double z) override {
			pos += Vector4(x,y,z);
		}
		void Move(const Vector4& vec) override {
			pos += vec;
		}
		void Rotate(double x, double y, double z) override {
			return;
		}
		void Rotate(const Vector4& vec) override {
			return;
		}
		void Scale(double x, double y, double z) override {
			radius *= x;
		}
	};


	class Rectangle : public Object {
	private:
		bool isFlat() const {
			double sum = 0;
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 3; ++i)
				sum += vertices[0].elem[i] * coefficient.elem[i];
			sum += coefficient.elem[3];

			return (-SMALL_DOUBLE < sum) && (sum < SMALL_DOUBLE);
		}
		void setEdgeVector() {
			edgeVector_test[0] = vertices[1] - vertices[0];
			edgeVector_test[1] = vertices[3] - vertices[0];
		}
		void setCoefficient() {
			coefficient = (vertices[1] - vertices[0]).Product(vertices[3] - vertices[0]).Normalize3();
			#pragma loop(hint_parallel(3))
			for (int i = 0; i < 3; ++i)
				coefficient.elem[3] -= coefficient.elem[i] * vertices[0].elem[i];
		}
		void setCenter() {
			std::fill_n(center.elem, 4, 0.);
			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				center += vertices[i];
			center /= 4;
		}
		void setWidth() {
			width = (vertices[3] - vertices[0]).Size3();
		}
		void setHeight() {
			height = (vertices[1] - vertices[0]).Size3();
		}
		void reInitialize() {
			setEdgeVector();
			setCoefficient();
			setCenter();
			setWidth();
			setHeight();
		}
	public:
		/*
			1a	4d
			2b	3c
		*/
		Vector4 vertices[4], coefficient, center;
		const Vector4 vertices_origin[4], coefficient_origin, center_origin;
		Vector4 edgeVector_test[2];// v_12, v_14
		double width, height;

		Rectangle(
			double v11, double v12, double v13,
			double v21, double v22, double v23,
			double v31, double v32, double v33,
			double v41, double v42, double v43)
			:
			vertices		{Vector4(v11,v12,v13), Vector4(v21,v22,v23), Vector4(v31,v32,v33), Vector4(v41,v42,v43) },
			edgeVector_test	{ (vertices[1] - vertices[0]),(vertices[3] - vertices[0]) },
			coefficient		((vertices[1] - vertices[0]).Product(vertices[3] - vertices[0]).Normalize3()),
			width			((vertices[3] - vertices[0]).Size3()),
			height			((vertices[1] - vertices[0]).Size3())
		{
			#pragma loop(hint_parallel(3))
			for (int i = 0; i < 3; ++i)	coefficient.elem[3] -= coefficient.elem[i] * vertices[0].elem[i];

			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)	center += vertices[i];
			center /= 4;

			for(int i=0;i<4;++i)
				memcpy(const_cast<e_type*>(vertices_origin[i].elem), vertices[i].elem, SIZE_ELEM4);
			memcpy(const_cast<e_type*>(coefficient_origin.elem), coefficient.elem, SIZE_ELEM4);
			memcpy(const_cast<e_type*>(center_origin.elem), center.elem, SIZE_ELEM4);

			

			if (!isFlat()) {
				cout << "Unflat polygon! (" << this << ")" << endl;
				exit(-1);
			}
		}

		Rectangle(const Vector4& v1, const Vector4& v2, const Vector4& v3, const Vector4& v4) :
			vertices		{v1,v2,v3,v4 },
			edgeVector_test	{v2 - v1,v4 - v1 },
			coefficient		((v2 - v1).Product(v4 - v1).Normalize3()),
			width			((v4 - v1).Size3()),
			height			((v2 - v1).Size3())
		{
			#pragma loop(hint_parallel(3))
			for (int i = 0; i < 3; ++i)
				coefficient.elem[3] -= coefficient.elem[i] * vertices[0].elem[i];

			#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				center += vertices[i];
			center /= 4;

			for (int i = 0; i < 4; ++i)
				memcpy(const_cast<e_type*>(vertices_origin[i].elem), vertices[i].elem, SIZE_ELEM4);
			memcpy(const_cast<e_type*>(coefficient_origin.elem), coefficient.elem, SIZE_ELEM4);
			memcpy(const_cast<e_type*>(center_origin.elem), center.elem, SIZE_ELEM4);

			if (!isFlat()) {
				cout << "Unflat polygon! (" << this << ")" << endl;
				exit(-1);
			}
		}

		void setColor(const cv::Vec3b& color) {
			material.setColor(color);
		}

		virtual bool isInPlane(double x, double y, double z, long double t) const override {
			double
				sum = calculate(x, y, z, t),
				dot_12 = (Vector4(x, y, z) - vertices[0]).Dot(edgeVector_test[0]),
				dot_14 = (Vector4(x, y, z) - vertices[0]).Dot(edgeVector_test[1]);



			return
				(-SMALL_DOUBLE < sum) && (sum < SMALL_DOUBLE) &&	// on plane
				(0 < dot_12 && dot_12 < height * height) &&			// in rectangle
				(0 < dot_14 && dot_14 < width * width);
		}
		virtual bool isInPlane(const Vector4& vec) const override {
			double
				sum = calculate(vec),
				dot_12 = (vec - vertices[0]).Dot(edgeVector_test[0]),
				dot_14 = (vec - vertices[0]).Dot(edgeVector_test[1]);

			return
				(-SMALL_DOUBLE < sum) && (sum < SMALL_DOUBLE) &&	// on plane
				(0. < dot_12 && dot_12 < (double)(height * height)) &&			// in rectangle
				(0. < dot_14 && dot_14 < (double)(width * width));
		}
		virtual double calculate(double x, double y, double z, long double t) const override {
			return
				x * coefficient.elem[0] +
				y * coefficient.elem[1] +
				z * coefficient.elem[2] +
				coefficient.elem[3];
		}
		virtual double calculate(const Vector4& vec) const override {
			double sum = 0;
#pragma loop(hint_parallel(3))
			for (int i = 0; i < 3; ++i)
				sum += vec.elem[i] * coefficient.elem[i];
			sum += coefficient.elem[3];
			return sum;
		}


		int SetTexture(const std::string& path, int flag) {
			return material.setTexture(path, flag);
		}

		virtual cv::Vec3b GetPixel(double x, double y, double z, long double t) const {

			Vector4 pixelVector(Vector4(x, y, z) - vertices[0]);

			double theta = pixelVector.AngleWith(vertices[3] - vertices[0]);
			double r = pixelVector.Size();

			isnan(theta) ? theta = 0 : 1;


			int px_w = (int)((r * cos(theta) / width) * material.texture.cols);
			int px_h = (int)((r * sin(theta) / height) * material.texture.rows);

			// minus 1 when same
			px_w -= !(~(px_w | ~material.texture.cols));
			px_h -= !(~(px_h | ~material.texture.rows));

			uchar* ptr = (material.texture.data + ((px_h * material.texture.cols + px_w) * 3));


			return cv::Vec3b(ptr[0], ptr[1], ptr[2]);
		}
		virtual cv::Vec3b GetPixel(const Vector4& vec) const {

			Vector4 pixelVector(vec - vertices[0]);

			double theta = pixelVector.AngleWith(vertices[3] - vertices[0]);
			double r = pixelVector.Size();

			isnan(theta) ? theta = 0 : 1;


			int px_w = (int)((r * cos(theta) / width) * material.texture.cols);
			int px_h = (int)((r * sin(theta) / height) * material.texture.rows);

			// minus 1 when same
			px_w -= !(~(px_w | ~material.texture.cols));
			px_h -= !(~(px_h | ~material.texture.rows));

			uchar* ptr = (material.texture.data + ((px_h * material.texture.cols + px_w) * 3));


			return cv::Vec3b(ptr[0], ptr[1], ptr[2]);
		}

		void Move(double x, double y, double z) {
			Vector4 moveVector(x, y, z);

#pragma loop(hint_parallel(4))
			for(int i=0;i<4;++i)
				vertices[i]+=moveVector;
			center+=moveVector;
			reInitialize();
		}
		void Move(const Vector4& vec) {
#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i)
				vertices[i] += vec;
			center += vec;
			reInitialize();
		}

		void Rotate(double x, double y, double z) {
			Vector4 center_backup = center;
			Matrix4 rotationMatrix = Matrix4::Eye().rotate(x,y,z);

			Move(center*(-1.));
#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i) {
				vertices[i] = rotationMatrix * vertices[i];
			}
			Move(center);

			reInitialize();
		}
		void Rotate(const Vector4& vec) {
			Vector4 center_backup = center;
			Matrix4 rotationMatrix = Matrix4::Eye().rotate(vec[0], vec[1], vec[2]);

			Move(center * (-1.));
#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i) {
				vertices[i] = rotationMatrix * vertices[i];
			}
			Move(center);
			reInitialize();
		}

		void MoveTo(const Vector4& vec) {
#pragma loop(hint_parallel(4))
			for (int i = 0; i < 4; ++i) {
				vertices[i] -= center;
				vertices[i] += vec;
			}

			center = vec;
			reInitialize();
		}
		void RotateTo(const Vector4& vec) {

		}

		void Scale(double x, double y, double z) {
			for (int i = 0; i < 4; ++i) {
				(vertices[i]).elem[0] *= x;
				(vertices[i]).elem[1] *= y;
				(vertices[i]).elem[2] *= z;
			}

			reInitialize();
		}
	};

	class Annulus : public Rectangle {
	private:
		inline double distance(const Vector4& vec) const { return (vec - center).Size3(); }
	public:
		double radius_outer, radius_inner;

		Annulus(
			double v11, double v12, double v13,
			double v21, double v22, double v23,
			double v31, double v32, double v33,
			double v41, double v42, double v43,
			double radius_outer, double radius_inner)
			:
			Rectangle( 
				v11,  v12,  v13,
				v21,  v22,  v23,
				v31,  v32,  v33,
				v41,  v42,  v43),
			radius_outer(radius_outer), radius_inner(radius_inner)
		{}

		virtual bool isInPlane(double x, double y, double z, long double t) const override{
			double sum = calculate(x, y, z, t),
				dist = distance(Vector4(x,y,z));
			return
				(-SMALL_DOUBLE < sum) & (sum < SMALL_DOUBLE) &
				(radius_inner <= dist) & (dist <= radius_outer);
		}
		virtual bool isInPlane(const Vector4& vec) const override{
			double sum = calculate(vec),
				dist = distance(vec);
			return
				(-SMALL_DOUBLE < sum) & (sum < SMALL_DOUBLE) &
				(radius_inner <= dist) & (dist <= radius_outer);
		}


	};

	class AccretionDisk : public Annulus {
	public:

		AccretionDisk(
			double v11, double v12, double v13,
			double v21, double v22, double v23,
			double v31, double v32, double v33,
			double v41, double v42, double v43,
			double radius_outer, double radius_inner)
			:
			Annulus(
				v11, v12, v13,
				v21, v22, v23,
				v31, v32, v33,
				v41, v42, v43,
				radius_outer, radius_inner)
		{}

		virtual cv::Vec3b GetPixel(double x, double y, double z, long double t) const override {

			double r_bh = (center - Vector4(x,y,z)).Size3() * constants::coefficient_length;
			double angle_velocity = sqrt(constants::G * constants::coefficient_mass * M) * pow(r_bh, -1.5);
			double theta_bh = angle_velocity * t;
			Vector4 rot_xyz = Matrix4::rotateAroundAxis(coefficient, theta_bh)*Vector4(x,y,z);
			x=rot_xyz.elem[0];
			t=rot_xyz.elem[0];
			z=rot_xyz.elem[0];

			Vector4 pixelVector(Vector4(x, y, z) - vertices[0]);

			double theta = pixelVector.AngleWith(vertices[3] - vertices[0]);
			double r = pixelVector.Size();

			isnan(theta) ? theta = 0 : 1;


			int px_w = (int)((r * cos(theta) / width) * material.texture.cols);
			int px_h = (int)((r * sin(theta) / height) * material.texture.rows);

			/*
			double r = sqrt((px_w -w_center)*(px_w - w_center) + (px_h - h_center)*(px_h - h_center)) * constants::coefficient_length;
			double angle_velocity = sqrt(constants::G * constants::coefficient_mass * M) * pow(r, -1.5);
			double theta = angle_velocity *t;
			Matrix4 rot = Matrix4::rotateAroundAxis(coefficient, theta);
			*/

			// minus 1 when same
			px_w -= !(~(px_w | ~material.texture.cols));
			px_h -= !(~(px_h | ~material.texture.rows));

			uchar* ptr = (material.texture.data + ((px_h * material.texture.cols + px_w) * 3));


			return cv::Vec3b(ptr[0], ptr[1], ptr[2]);
		}
		virtual cv::Vec3b GetPixel(const Vector4& vec) const override{

			double r_bh = (center - vec).Size3() * constants::coefficient_length;
			double angle_velocity = sqrt(constants::G * constants::coefficient_mass * M) * pow(r_bh, -1.5) * 0.1;
			double theta_bh = angle_velocity * vec[3];
			Vector4 rot_xyz = Matrix4::rotateAroundAxis(coefficient, theta_bh) * vec;

			Vector4 pixelVector(rot_xyz - vertices[0]);

			double theta = pixelVector.AngleWith(vertices[3] - vertices[0]);
			double r = pixelVector.Size();

			isnan(theta) ? theta = 0 : 1;


			int px_w = (int)((r * cos(theta) / width) * material.texture.cols);
			int px_h = (int)((r * sin(theta) / height) * material.texture.rows);

			// minus 1 when same
			px_w -= !(~(px_w | ~material.texture.cols));
			px_h -= !(~(px_h | ~material.texture.rows));

			uchar* ptr = (material.texture.data + ((px_h * material.texture.cols + px_w) * 3));


			return cv::Vec3b(ptr[0], ptr[1], ptr[2]);
		}
	};

	class Object3D {
	public:
		unsigned const int side_num;
		Vector4 center;
		Object *obj_ptr;

		Object3D(unsigned const int side_num) :
			side_num(side_num), center(0), obj_ptr(0){}

		Object3D(unsigned const int side_num, Object* v_ptr, const Vector4& center) :
			side_num(side_num), obj_ptr(v_ptr), center(center) {}

		virtual void Rotate(const Vector4& angle_vec) = 0;
		virtual void Move(const Vector4& trans_vec) = 0;
	};

	/*
	class Cube : public Object3D {
	public:
		Rectangle rect[6];
		Cube(
			Vector4 v1, Vector4 v2, Vector4 v3, Vector4 v4,
			Vector4 v5, Vector4 v6, Vector4 v7, Vector4 v8)
			:
			rect{
				Rectangle(v1,v2,v3,v4),
				Rectangle(v1,v5,v6,v2),
				Rectangle(v2,v6,v7,v3),
				Rectangle(v3,v7,v8,v4),
				Rectangle(v4,v8,v5,v1),
				Rectangle(v5,v8,v6,v7)
			},
			Object3D(6, rect, (v1+v2+v3+v4+v5+v6+v7+v8)/8.)
		{}

		void Rotate(const Vector4& angle_vec) override {

		}

		void Move(const Vector4& trans_vec) override {

		}
	};
	*/


	class ObjectManager : public _Singleton<ObjectManager> {
	public:
		static std::vector<Object*> objectArray;


		void insertObject(Object& obj) {
			objectArray.push_back(&obj);
		}

		void insertObject(Object3D& obj) {
			for (int i = 0; i < obj.side_num; ++i) {
				//objectArray.push_back()
			}
		}

	};

	std::vector<Object*> ObjectManager::objectArray(0);
}