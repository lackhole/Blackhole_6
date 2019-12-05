#pragma once

#include <cmath>
#include "matrix.h"

using namespace BH_MATRIX;
class Camera {
private:
	double _h;
	double angle_vertical, angle_horizontal;
public:
	int width, height;
	Vector4 focus, lookat;
	double fovy;

	Camera(int width, int height, Vector4 focus, Vector4 lookat, double fovy)
		: width(width), height(height), focus(focus), lookat(lookat), fovy(fovy)
	{
		_h = (width / 2.) / tan(fovy / 2.);
		Vector4 lookVector = lookat - focus;
		angle_vertical = lookVector.AngleWith(Vector4(lookVector[0], lookVector[1], 0)) * (lookVector[2] >= 0 ? 1 : -1);
		angle_horizontal = Vector4(lookVector[0], lookVector[1], 0).AngleWith(Vector4(1,0,0)) * (lookVector[1] >= 0 ? 1 : -1);
	}

	// tan(fovy/2) = (w/2) / h

	Vector4 pixelVector(int w, int h) {
		Vector4 pixelVector = Vector4(width / 2. - w, -_h, height / 2. - h);
		pixelVector = Matrix4::Eye().rotate(0, -angle_vertical, 0).rotate(0, 0, angle_horizontal)* pixelVector;
		pixelVector += focus;
		return pixelVector.Normalize3();
	}
};
