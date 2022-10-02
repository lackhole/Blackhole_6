
#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <filesystem>

#include "opencv2/opencv.hpp"

#include "config.h"
#include "properties.h"
#include "matrix.h"
#include "polygon.h"
#include "raytracing.h"
#include "physics.h"
#include "camera.h"

#define STRINGIFY_IMPL(x) #x
#define STRINGIFY(x) STRINGIFY_IMPL(x)

#define kPWD STRINGIFY(PWD)

/*
 coordinate system

                    z theta
                    |__  /
                    |  \/
                    |  /\
                    | /  v_* P
                    |/ _/''|
--------------------+-/-r--|----- y
                   /|\     |
                  / | \    |
                 /  |  \   |
                /---+-->\  |
               / phi|    \ |
              /     |     \|
             x
*/

clock_t t_begin = clock();



#define DISTANCE3(x,y,z)	sqrt((x)*(x)+(y)*(y)+(z)*(z))

#define G(b, u)				( (u)*(u)*(2.*M*(u) - 1.) + 1. / ((b)*(b)) )
#define FUNC(b, u)			(1. / sqrt((u)*(u)*(M2*(u) - 1.) + 1. / ((b) * (b))))

const double pi = 3.141592653589793238462643383279502884197169399375105820974944;

void btncbf(int event, int x, int y, int flags, void* userdata);
double findSolutionBinary(const double b) {
	// y = 2Mx^2 - x^2 + 1/b^2
	static const double dx = 0.0000001;
	static const int count = 20;

	if (b < b_c) return -1;	//just check if signed

	double l = 0. + dx;
	double r = 1. / (3. * M);
	double mid;


	const double b_invsq = 1. / (b * b);

	for (int i = 0; i < count; ++i) {
		mid = (l + r) / 2.;

		if (G(b, mid) > 0.) {
			l = mid;
		}
		else {
			r = mid;
		}
	}

	return l;
}

int main(void) {
  namespace fs = std::filesystem;
  namespace chrono = std::chrono;
	cout << "Program Started" << endl;

	// Object Manager handles objects
	ObjectManager& objectManager = bh_poly::ObjectManager::getInstance();

	// Folder name as current time
	const auto cur_time = std::to_string(chrono::system_clock::to_time_t(chrono::system_clock::now()));

	bh_poly::Rectangle rect(
		1000, 0, 1000,
		1000, 0, -1000,
		-1000, 0, -1000,
		-1000, 0, 1000
	);
	bh_poly::Rectangle rect2(
		0, 1000, 1000,
		0, -1000, 1000,
		0, -1000, -1000,
		0, 1000, -1000
	);
	bh_poly::Rectangle rect3(
		1000, 1000, 0,
		-1000, 1000, 0,
		-1000, -1000, 0,
		1000, -1000, 0
	);
	bh_poly::Sphere b_hole(0, 0, 0, diameter);
	bh_poly::Annulus disc(
		1000, 1000, 0,
		-1000, 1000, 0,
		-1000, -1000, 0,
		1000, -1000, 0,
		1000, diameter * 10
	);
	bh_poly::AccretionDisk acc_disk(1000, 1000, 0,
		-1000, 1000, 0,
		-1000, -1000, 0,
		1000, -1000, 0,
		1000, diameter * 10);

	Vector4 vec_ball(0, -400, 300), vec_ball_norm = Matrix4::rotateAroundAxis(Vector4(1, 0, 0), -pi / 2.) * vec_ball;

	bh_poly::Sphere ball(600, -400, 300, radius * 10);
	//ball.pos = Matrix4::rotateAroundAxis(vec_ball_norm, (pi) * 1) * ball.pos;
	//ball.pos = Matrix4::rotateAroundAxis(vec_ball_norm, (pi / 360) * 60)*ball.pos;
	ball.pos = Matrix4::rotateAroundAxis(vec_ball_norm, (pi / 360) * 0) * ball.pos;

  bh_poly::Rectangle background(
      2000, -2000, 2000,
      -2000, -2000, 2000,
      -2000, -2000, -2000,
      2000, -2000, -2000
      );

  background.setColor({0, 0, 0});
//	background.SetTexture("/Users/yonggyulee/Downloads/dsp.jpg");
	//rect.SetTexture("C:/Users/cosge/Desktop/white.png", cv::IMREAD_UNCHANGED);
//	disc.setColor({0,0,180});
	disc.SetTexture(std::string(kPWD) + "/resource/acc_disc.png");
//	disc.SetTexture("/Users/yonggyulee/Downloads/Abell-39.jpg");
//	rect.setColor(cv::Vec3b(255, 0, 0));
//	rect2.setColor(cv::Vec3b(0, 255, 0));
	//rect3.setColor(cv::Vec3b(0 , 0, 255));
//	rect3.SetTexture("C:/Users/cosge/Desktop/accretiondisk.jpg", cv::IMREAD_COLOR);
	//rect.setColor(cv::Vec3b(255, 0, 0));
//	acc_disk.SetTexture("C:/Users/cosge/Desktop/accretiondisk.jpg", cv::IMREAD_COLOR);

    objectManager.insertObject(background);
	objectManager.insertObject(disc);
//	objectManager.insertObject(rect);
//	objectManager.insertObject(rect2);
	//objectManager.insertObject(rect3);
	//objectManager.insertObject(ball);
	//objectManager.insertObject(acc_disk);

	ball.setColor(cv::Vec3b(255, 255, 255));
	b_hole.setColor(cv::Vec3b(0, 0, 0));
	disc.motion = [&, disc](long double t) {
		auto disc_t = disc;

	};

	disc.Rotate(-0.01, 0, 0);
	//rect.Move(0, 10, 0);
	rect2.Rotate(0, 0, 0.1);
	rect.Rotate(0, 0.1, 0);
	rect3.Rotate(-0.03, 0, 0);
	acc_disk.Rotate(-0.01, 0, 0);

	Collision collision_info;

	// Image set to Black
	const int screen_w = 1600 / 2, screen_h = 900 / 2;
	cv::Mat3b image(screen_h, screen_w, CV_8UC3);
	image = cv::Vec3b(0, 0, 0);

	// Create directories
  const auto base_path = fs::path(kPWD)/"output"/cur_time;

  fs::create_directories(base_path/"video");
  fs::create_directories(base_path/"image");
	cv::VideoWriter out_capture((base_path/"video/video.avi").string(),
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 29, cv::Size(screen_w, screen_h), true);
	
	

	clock_t begin = clock();

	Vector4 cameraFocus(0, 20000, 0);
	Vector4 cameraLookat(0, 0, 0);
	double fovy = 3.14159265 / 40.;
	double h = screen_w / (2. * tan(fovy / 2.));
	/*
		Lookup table that stores ray's traces if realative position between camera and the blackhole is fixed through time.
		Current camera is fixed in y - axis.
		Do not change x or z value.
		Only change y value for fast calculations.
	*/
	Ray** ray_lookups = 0;
	{
		ray_lookups = (Ray**)calloc(screen_h, sizeof(Ray*));
		if (ray_lookups == 0) {
			cout<<"Memory allocation has failed while building the lookup table"<<endl;
			goto SKIP_LOOKUP;
		}
		for (int j = 0; j < screen_h; ++j) {
			ray_lookups[j] = (Ray*)calloc(screen_w, sizeof(Ray));
			if (ray_lookups[j] == 0) {
				cout << "Memory allocation has failed while building the lookup table" << endl;
				for (int k = 0; k < j; ++k)
					free(ray_lookups[k]);
				free(ray_lookups);
				ray_lookups = 0;
				goto SKIP_LOOKUP;
			}
		}
	}


	for (int px_h = 0; px_h < screen_h; ++px_h) {
		for (int px_w = 0; px_w < screen_w; ++px_w) {

			// lookup ray for each pixels
			Ray* ray = &ray_lookups[px_h][px_w];
			ray->setInit(cameraFocus);
			ray->buildLookup_test(nstep);

			// direction of the pixel
			Vector4 pixelPoint = (cameraFocus + Vector4(screen_w / 2. - px_w + 0.5, -h, screen_h / 2. - px_h + 0.5));

			// normalized unit vector
			Vector4 x_v, y_v, z_v;
			y_v = (cameraFocus - pixelPoint).Normalize3();
			z_v = (pixelPoint.Product(y_v)).Normalize3();
			if (isnan(z_v[0])) {
				z_v = Vector4(1, 0, 0);
			}
			x_v = y_v.Product(z_v).Normalize3();

			// conversion matrix for new coordinates
			Matrix4 matrix_reconversion = Matrix4(
				z_v[0], y_v[0], x_v[0],
				z_v[1], y_v[1], x_v[1],
				z_v[2], y_v[2], x_v[2]);
			Matrix4 matrix_conversion = matrix_reconversion.reverse();

			Vector4 convertedCameraFocus = matrix_conversion * cameraFocus;
			Vector4 convertedPixel = matrix_conversion * pixelPoint;

			/*
				b         |   impact parameter
				sol       |   solution for motion equation
				periapsis |   periapsis(1/3M for non-solvable)
			*/
			const double b = convertedCameraFocus[2];
			const double sol = findSolutionBinary(b);
			const double periapsis = sol > 0 ? sol : 1. / (3 * M);//1./diameter;//1E+300;
			double r0 = convertedCameraFocus.Size3();

			register double
				phi = atan(convertedCameraFocus[2] / convertedCameraFocus[1]),
				dphi = 0,
				dphi_prev = 0,
				u = 1. / r0,
				r = r0;

			const double
				integrate_begin = u,
				integrate_end = periapsis;

			register double
				du = (integrate_end - integrate_begin) * nstep_inverse;
			register const double
				du_h = du / 2.,
				b_invsq = 1. / (b * b);

			// vector of light
			// x = 0
			register Vector4
				lightVector = convertedCameraFocus,
				lightVector_prev = convertedCameraFocus;

			register double time_local = world_time;

			// trapezoid integration
			for (register int i = 0; i < nstep_safe; ++i) {
				u += du;
				dphi = FUNC(b, u);

				phi += (dphi_prev + dphi) * du_h;
				r = 1. / u;

				lightVector = { 0, r * cos(phi), r * sin(phi), 0 };
				time_local -= ((lightVector_prev - lightVector).Size3() * constants::coefficient_length) / constants::c;
				lightVector.elem[3] = time_local;

				dphi_prev = dphi;
				ray->prograde(matrix_reconversion * lightVector);

				lightVector_prev = lightVector;
			}
			{
				u += du * 0.9;
				dphi = FUNC(b, u);

				phi += (dphi_prev + dphi) * du_h;
				r = 1. / u;

				lightVector = { 0, r * cos(phi), r * sin(phi), 0 };
				time_local -= ((lightVector_prev - lightVector).Size3() * constants::coefficient_length) / constants::c;
				lightVector.elem[3] = time_local;

				dphi_prev = dphi;
				ray->prograde(matrix_reconversion * lightVector);

				lightVector_prev = lightVector;
			}

			if (sol < 0) {
				ray->builded = true;
				continue;
			}

			for (register int i = 0; i < nstep_safe; ++i) {
				u -= du;
				dphi = FUNC(b, u);

				phi += (dphi_prev + dphi) * du_h;
				r = 1. / u;

				lightVector = { 0, r * cos(phi), r * sin(phi), 0 };
				time_local -= ((lightVector_prev - lightVector).Size3() * constants::coefficient_length) / constants::c;
				lightVector.elem[3] = time_local;

				dphi_prev = dphi;

				ray->prograde(matrix_reconversion * lightVector);

				lightVector_prev = lightVector;
			}

			ray->builded = true;
		}


		cout << "\rBuilding Lookup: " << px_h + 1 << "/" << screen_h;
	}
	cout << endl;


	SKIP_LOOKUP:
	for (int time = 0; time < 360; ++time) {
		clock_t begin_image = clock();
    std::memset(image.data, 0, image.rows * image.cols * 3);


		for (int px_h = 0; px_h < screen_h; ++px_h) {
			for (int px_w = 0; px_w < screen_w; ++px_w) {
				Ray* ray = &ray_lookups[px_h][px_w];
				if (ray->builded) {
					for (int i = 0; i < buffer_count - 1; ++i)
						// add time
						ray->position[i].elem[3] += world_time;
					collision_info = ray->calcRemains();
					if (collision_info.collided) {
						// copy texture data to pixel
						memcpy(image.data + ((int)px_h * screen_w + px_w) * 3, collision_info.object->GetPixel(collision_info.point).val, sizeof(uchar) * 3);
					}
					continue;
				}

				Vector4 pixelVector = (cameraFocus + Vector4(screen_w / 2. - px_w, -h, screen_h / 2. - px_h));


				Vector4 x_v, y_v, z_v;
				y_v = (cameraFocus - pixelVector).Normalize3();
				z_v = (pixelVector.Product(cameraFocus - pixelVector)).Normalize3();
				x_v = y_v.Product(z_v).Normalize3();


				Matrix4 matrix_conversion = Matrix4(
					z_v[0], y_v[0], x_v[0],
					z_v[1], y_v[1], x_v[1],
					z_v[2], y_v[2], x_v[2]).reverse();

				Vector4 convertedCameraFocus = matrix_conversion * cameraFocus;
				Vector4 convertedPixel = matrix_conversion * pixelVector;


				//const double b = pixelPoint.Product(y_v).Size3();
				//const double b = convertedPixel[2];
				const double b = convertedCameraFocus[2];




				const double sol = findSolutionBinary(b);
				const double periapsis = sol > 0 ? sol : 1./diameter;

				double r0 = cameraFocus.Size3();

				register double
					phi = atan(convertedCameraFocus[2] / convertedCameraFocus[1]),
					dphi = 0,
					dphi_prev = 0,
					u = 1. / r0,
					r = r0;

				const double
					integrate_begin = u,
					integrate_end = periapsis;

				register double
					du = (integrate_end - integrate_begin) * nstep_inverse,
					du_h = du / 2.,
					b_invsq = 1. / (b * b);

				// vector of light
				// x = 0
				register Vector4
					lightVector = convertedCameraFocus,
					lightVector_prev = convertedCameraFocus;

				double time_local = world_time;
				
				for (int i = 0; i < nstep_safe; i++) {
					u += du;
					dphi = FUNC(b, u);

					phi += (dphi_prev + dphi) * du_h;
					r = 1. / u;

					lightVector = { 0, r * cos(phi), r * sin(phi), 0 };
					time_local -= ((lightVector_prev - lightVector).Size3()*constants::coefficient_length) / constants::c;
					lightVector.elem[3] = time_local;

					dphi_prev = dphi;

					collision_info = ray->prograde(matrix_conversion * lightVector);
					if (collision_info.collided) {
						memcpy(image.data + ((int)px_h * screen_w + px_w)*3, collision_info.object->GetPixel(collision_info.point).val, sizeof(uchar) * 3);
						goto ENDLOOP;
					}

					lightVector_prev = lightVector;
				}
				collision_info = ray->calcRemains();
				if (collision_info.collided) {
					memcpy(image.data + ((int)px_h * screen_w + px_w) * 3, collision_info.object->GetPixel(collision_info.point).val, sizeof(uchar) * 3);
					goto ENDLOOP;
				}

				for (register int i = 0; i < nstep_safe; i++) {
					u -= du;
					dphi = FUNC(b, u);

					phi += (dphi_prev + dphi) * du_h;
					r = 1. / u;

					lightVector = { 0, r * cos(phi), r * sin(phi), 0 };
					time_local -= ((lightVector_prev - lightVector).Size3() * constants::coefficient_length) / constants::c;
					lightVector.elem[3] = time_local;

					dphi_prev = dphi;

					collision_info = ray->prograde(matrix_conversion * lightVector);
					if (collision_info.collided) {
						memcpy(image.data + ((int)px_h * screen_w + px_w) * 3, collision_info.object->GetPixel(collision_info.point).val, sizeof(uchar) * 3);
						goto ENDLOOP;
					}

					lightVector_prev = lightVector;
				}
				collision_info = ray->calcRemains();
				if (collision_info.collided) {
					memcpy(image.data + ((int)px_h * screen_w + px_w) * 3, collision_info.object->GetPixel(collision_info.point).val, sizeof(uchar) * 3);
					goto ENDLOOP;
				}


			ENDLOOP:
				;
			}

			if (px_h % 50 == 0) {
//				cv::imshow("test2", image);
//				cv::waitKey(1);
				std::cout << "\r" << px_h + 1 << "/" << screen_h;
			}

		}
		std::cout << "\r" << screen_h << "/" << screen_h;

		std::vector<int> comp;
		comp.push_back(cv::IMWRITE_PNG_COMPRESSION);
		comp.push_back(9);
//		cv::imwrite(std::string(kPWD) + "/" + cur_time + "/image/" + "t=" + std::to_string(time) + ".jpg", image);
		cout << "Writed: " << time << ", elapsed time: " << ((double)clock() - (double)begin_image) / CLOCKS_PER_SEC << "s" << endl;

		// write to video
		out_capture.write(image);

    cv::putText(image, std::string("T= ") + std::to_string(world_time), {5, 10}, cv::FONT_HERSHEY_PLAIN, 1, {0, 200, 0}, 1);
    cv::putText(image, std::string("Rendering= ") + std::to_string(((double)clock() - (double)begin_image) / CLOCKS_PER_SEC * 1000) + "ms", {5, 20}, cv::FONT_HERSHEY_PLAIN, 1, {0, 200, 0}, 1);
		cv::imshow("Blackhole: Ray-Tracing C++", image);
//		cv::setMouseCallback("Blackhole: Ray-Tracing C++", btncbf);
		if(cv::waitKey(1) == 27) break;

		world_time += 0.000001;
		//ball.pos = Matrix4::rotateAroundAxis(vec_ball_norm, (pi / 180) * 1) * ball.pos;
		//ball.Move(Vector4(-5,0,0));
		disc.Rotate(-(pi / 1800.), 0, 0);

	}
	cout << "Time: " << ((double)clock() - (double)begin) / CLOCKS_PER_SEC << "s" << endl;

	return 0;
}

void btncbf(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		cout << x << ", " << y << endl;
	}
}
