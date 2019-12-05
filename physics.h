#pragma once
#include <cmath>

long double world_time = 0.014;

const double M = 10.;
const double M2 = M * 2;
const double diameter = M;
const double radius = 2. * M;
const double b_c = 3. * sqrt(3) * M;

namespace constants {
	const long double m_sun = 1.989 * pow(10, 30);
	const long double c = 299792458;
	const long double G = 6.67408 *pow(10, -11);
	const double pi = 3.141592653589793238462643383279502884197169399375105820974944;

	//coefficient * unit = default unit   (m, kg, s)

	/*
	| unit in code|   | unit in real
	|-------------|---|-----------
	|   [length]  | 1 | 1477m
	|    [mass]   | 1 | 1M_sun
	|    [time]   | 1 | 1/(c*1477)
	*/

	const long double coefficient_length = (m_sun*G)/(c*c);
	const long double coefficient_mass = m_sun;
	const long double coefficient_time = 1./(coefficient_length*c);
}