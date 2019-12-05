#pragma once

#include <iostream>
#include <math.h>
#include <vector>

#include "matrix.h"
#include "polygon.h"
#include "properties.h"

using namespace bh_poly;
using namespace BH_MATRIX;


class Ray;
struct Collision;

struct Collision {
	bool collided;
	Object* object;
	Vector4 point;
};

class Ray {
private:
	struct CollideCandidate {
		Collision collision;
		double rank;
	};

	Collision detectCollision(const Ray& ray) const {

		double distance1, distance2;
		static std::vector<CollideCandidate> collidedCandidate{};
		collidedCandidate.clear();

		const int index_end = ray.end - 1;
		for (int i = ray.start; i < index_end; ++i) {
			for (const auto& obj : bh_poly::ObjectManager::objectArray) {
				if (obj->isCurved) {
					if (((Sphere*)obj)->doesIntersect(ray.position[i], ray.position[i + 1])) {
						/*
						double rank = distanceBetweenLineSegmentAndPoint(ray.position[i], ray.position[i+1], ((Sphere*)obj)->pos);
						if(rank < 0)
							continue;
						if (rank <= ((Sphere*)obj)->radius)
							rank = 0;
						else
							rank -= ((Sphere*)obj)->radius;
							*/
						double rank = distanceBetweenLineSegmentAndPoint(ray.position[i], ray.position[i + 1], ((Sphere*)obj)->pos);
						const double radius = ((Sphere*)obj)->radius;
						if (rank > radius)
							continue;
						rank = ((Sphere*)obj)->distance(ray.position[i]) - radius;

						Vector4 fake_inter = ((Sphere*)obj)->pos;
						fake_inter.elem[0] += radius;
						collidedCandidate.push_back(CollideCandidate{ Collision{true, obj, fake_inter},  rank});
					}
				}
				else {
					distance1 = obj->calculate(ray.position[i]);
					distance2 = obj->calculate(ray.position[(i + 1)]);

					if ((distance1 * distance2) <= 0) {
						distance1 = abs(distance1);
						distance2 = abs(distance2);

						double k = distance1 / (distance1 + distance2);
						Vector4 intersection = (ray.position[i] * (1 - k)) + (ray.position[i + 1] * k);

						if (obj->isInPlane(intersection)) {
							collidedCandidate.push_back(CollideCandidate{ Collision{true, obj, intersection}, (ray.position[i] - intersection).Size3()});
							continue;
						}

					}
				}
			}

			// calculate when the collision occurs;
			if(collidedCandidate.size()) {

				CollideCandidate& nearest = collidedCandidate[0];
				for (int i = 1; i < collidedCandidate.size(); ++i) {
					if (collidedCandidate[i].rank < nearest.rank)
						nearest = collidedCandidate[i];
				}

				const_cast<Ray*>(this)->reset();
				return nearest.collision;
			}
		}

		const_cast<Ray*>(this)->reset();
		return Collision{ 0 };

	}

	void reset() {
		if(keepdata)
			return;
		position[0] = position[end-1];
		end = 1;
	}
public:
	Vector4 position[buffer_count];	//Stores before, current position of ray and time {x, y, z, t}
	int start, end;		//which is first
	bool keepdata;
	bool builded;

	void setInit(const Vector4& init_pos) {
		start = 0;
		end = 1;
		keepdata = false;
		builded = false;
		position[0] = init_pos;
	}

	Ray() : start(0), end(1), keepdata(false), builded(false) {}

	Ray(const Vector4& init_pos) : start(0), end(1),keepdata(false),builded(false) {
		position[0] = init_pos;
	}
	Collision prograde(const Vector4& next) {
		position[end] = next;
		++end;
		if (end == buffer_count) {
			return detectCollision(*this);
		}
		else {
			return Collision{0};
		}
	}
	Collision calcRemains() {
		return detectCollision(*this);
	}

	void buildLookup_test(int step_num) {
		if (buffer_count <= step_num * 2) {
			std::cout<<"buffer count must be more than two times bigger than step number"<<std::endl;
			exit(-1);
		}
		keepdata = true;
	}
};


