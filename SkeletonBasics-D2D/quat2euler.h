#ifndef QUAT2EULER_H
#define QUAT2EULER_H

#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>

using namespace std;

#ifdef __cplusplus
namespace Quat2Euler {
#endif
	///////////////////////////////
	// Quaternion struct
	// Simple incomplete quaternion struct for demo purpose
	///////////////////////////////
	struct Quaternion{
		Quaternion():x(0), y(0), z(0), w(1){};
		Quaternion(double x, double y, double z, double w):x(x), y(y), z(z), w(w){};

		void normalize(){
			double norm = std::sqrt(x*x + y*y + z*z + w*w);
			x /= norm;
			y /= norm;
			z /= norm;
			w /= norm;
		}

		double norm(){
			return std::sqrt(x*x + y*y + z*z + w*w);
		}

		double x;
		double y;
		double z;
		double w;  

	};

	///////////////////////////////
	// Quaternion to Euler
	///////////////////////////////
	enum RotSeq{zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy,xzx};

	INLINE void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
		res[0] = atan2( r11, r12 );
		res[1] = acos ( r21 );
		res[2] = atan2( r31, r32 );
	}

	INLINE void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
		res[0] = atan2( r31, r32 );
		res[1] = asin ( r21 );
		res[2] = atan2( r11, r12 );
	}

	INLINE void quaternion2Euler(const Quaternion& q, double res[], RotSeq rotSeq)
	{
		switch(rotSeq){
		case zyx:
			threeaxisrot( 2*(q.x*q.y + q.w*q.z),
				q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				-2*(q.x*q.z - q.w*q.y),
				2*(q.y*q.z + q.w*q.x),
				q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				res);
			break;

		case zyz:
			twoaxisrot( 2*(q.y*q.z - q.w*q.x),
				2*(q.x*q.z + q.w*q.y),
				q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				2*(q.y*q.z + q.w*q.x),
				-2*(q.x*q.z - q.w*q.y),
				res);
			break;

		case zxy:
			threeaxisrot( -2*(q.x*q.y - q.w*q.z),
				q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				2*(q.y*q.z + q.w*q.x),
				-2*(q.x*q.z - q.w*q.y),
				q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				res);
			break;

		case zxz:
			twoaxisrot( 2*(q.x*q.z + q.w*q.y),
				-2*(q.y*q.z - q.w*q.x),
				q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				2*(q.x*q.z - q.w*q.y),
				2*(q.y*q.z + q.w*q.x),
				res);
			break;

		case yxz:
			threeaxisrot( 2*(q.x*q.z + q.w*q.y),
				q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				-2*(q.y*q.z - q.w*q.x),
				2*(q.x*q.y + q.w*q.z),
				q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				res);
			break;

		case yxy:
			twoaxisrot( 2*(q.x*q.y - q.w*q.z),
				2*(q.y*q.z + q.w*q.x),
				q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				2*(q.x*q.y + q.w*q.z),
				-2*(q.y*q.z - q.w*q.x),
				res);
			break;

		case yzx:
			threeaxisrot( -2*(q.x*q.z - q.w*q.y),
				q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				2*(q.x*q.y + q.w*q.z),
				-2*(q.y*q.z - q.w*q.x),
				q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				res);
			break;

		case yzy:
			twoaxisrot( 2*(q.y*q.z + q.w*q.x),
				-2*(q.x*q.y - q.w*q.z),
				q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				2*(q.y*q.z - q.w*q.x),
				2*(q.x*q.y + q.w*q.z),
				res);
			break;

		case xyz:
			threeaxisrot( -2*(q.y*q.z - q.w*q.x),
				q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				2*(q.x*q.z + q.w*q.y),
				-2*(q.x*q.y - q.w*q.z),
				q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				res);
			break;

		case xyx:
			twoaxisrot( 2*(q.x*q.y + q.w*q.z),
				-2*(q.x*q.z - q.w*q.y),
				q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				2*(q.x*q.y - q.w*q.z),
				2*(q.x*q.z + q.w*q.y),
				res);
			break;

		case xzy:
			threeaxisrot( 2*(q.y*q.z + q.w*q.x),
				q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				-2*(q.x*q.y - q.w*q.z),
				2*(q.x*q.z + q.w*q.y),
				q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				res);
			break;

		case xzx:
			twoaxisrot( 2*(q.x*q.z - q.w*q.y),
				2*(q.x*q.y + q.w*q.z),
				q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				2*(q.x*q.z + q.w*q.y),
				-2*(q.x*q.y - q.w*q.z),
				res);
			break;
		default:
			std::cout << "Unknown rotation sequence" << std::endl;
			break;
		}
	}

	///////////////////////////////
	// Helper functions
	///////////////////////////////
	INLINE Quaternion operator*(Quaternion& q1, Quaternion& q2){
		Quaternion q;
		q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
		q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
		q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
		q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
		return q;
	}

	INLINE ostream& operator <<(std::ostream& stream, const Quaternion& q) {
		cout << q.w << " "<< showpos << q.x << "i " << q.y << "j " << q.z << "k"; 
		cout << noshowpos;
		return stream;
	}

	INLINE double rad2deg(double rad){
		return rad*180.0/M_PI;
	}

	/*
	///////////////////////////////
	// Main
	///////////////////////////////
	int main(){

	Quaternion q; // x,y,z,w
	Quaternion qx45(sin(M_PI/8), 0,0, cos(M_PI/8) );
	Quaternion qy45(0, sin(M_PI/8), 0, cos(M_PI/8));
	Quaternion qz45(0, 0, sin(M_PI/8), cos(M_PI/8));
	Quaternion qx90(sin(M_PI/4), 0,0, cos(M_PI/4) );
	Quaternion qy90(0, sin(M_PI/4), 0, cos(M_PI/4));
	Quaternion qz90(0, 0, sin(M_PI/4), cos(M_PI/4));

	double res[3];

	q = qz45*qx45;
	q.normalize();
	quaternion2Euler(q, res, zyx);
	cout << "Rotation sequence: X->Y->Z" << endl;
	cout << "x45 -> z45" << endl;
	cout << "q: " << q << endl;
	cout << "x: " << rad2deg(res[0]) << " y: " << rad2deg(res[1]) << " z: " << rad2deg(res[2]) << endl << endl;

	q = qz90*qx90;
	q.normalize();
	quaternion2Euler(q, res, zyx);
	cout << "Rotation sequence: X->Y->Z" << endl;
	cout << "x90 -> z90" << endl;
	cout << "q: " << q << endl;
	cout << "x: " << rad2deg(res[0]) << " y: " << rad2deg(res[1]) << " z: " << rad2deg(res[2]) << endl << endl;

	q = qx90*qz90;
	q.normalize();
	quaternion2Euler(q, res, xyz);
	cout << "Rotation sequence: Z->Y->X" << endl;
	cout << "z90 -> x90" << endl;
	cout << "q: " << q << endl;
	cout << "x: " << rad2deg(res[0]) << " y: " << rad2deg(res[1]) << " z: " << rad2deg(res[2]) << endl;
	}
	*/

#ifdef __cplusplus
} // namespace Quat2Euler {
#endif

#endif // QUAT2EULER_H
