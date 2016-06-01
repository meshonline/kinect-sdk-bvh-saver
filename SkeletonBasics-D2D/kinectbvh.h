#ifndef KINECTBVH_H
#define KINECTBVH_H

#define SCALE 1.f
#define FPS 0.033333

#include <windows.h>
#include <NuiApi.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include "vec_math.h"
#include "quat2euler.h"

using namespace std;

struct KinectJoint
{
	Vector4 quat;
};

/**
* Manipule les donn閑s de la Kinect que la classe KinectSkeleton a envoy?et g閚鑢e le fichier BVH r閟ultant de la capture de mouvements
*/
class KinectBVH
{
public:
	/**
	* Constructeur
	*/
	KinectBVH();
	/**
	* Destructeur
	*/
	~KinectBVH();

	/**
	* Ajoute un offset ?la description du BVH
	*/
	void AddOffset(int, const Vector4 &);
	/**
	* Ajoute un squelette et ses informations pour les donn閑s de la capture de mouvements
	*/
	void AddBonesOrientation(KinectJoint *);
	/**
	* Ajoute une position du joint Hip Center pour les donn閑s de la capture de mouvements
	*/
	void AddPosition(const Vector4 &);
	/**
	* Incr閙ente le nombre de frames
	*/
	void IncrementNbFrames();
	/**
	* Cr殚 un nouveau fichier en fonction du nom re鐄 en param鑤re, renvoi true si r閡ssi sinon false
	*/
	bool CreateBVHFile(string);
	/**
	* G閚鑢e le fichier BVH
	*/
	void FillBVHFile();
private:
	// Fichier
	ofstream m_pFile;

	// pre-calculated quaternions
	Vec_Math::Quaternion q_z_30;
	Vec_Math::Quaternion q_z_n30;
	Vec_Math::Quaternion q_z_43;
	Vec_Math::Quaternion q_z_n43;

	// Offsets
	vector<Vector4> m_aOffsets;
	// Positions du Hip Center
	vector<Vector4> m_vPositions;
	// Squelettes et leurs informations des joints
	vector<KinectJoint> m_vBonesOrientation;

	// Nombre de frames
	int m_nbFrame;

	void GetAngles(KinectJoint *, int, double []);

	/**
	* G閚鑢e la description du squelette pour le BVH
	*/
	void CreateSkeletonInformation();
	/**
	* G閚鑢e les donn閑s des mouvements pour le BVH
	*/
	void CreateMotionInformation();
};

#endif // KINECTBVH_H
