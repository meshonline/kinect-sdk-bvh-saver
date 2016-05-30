#include "kinectbvh.h"

/**
* Constructeur
*/
KinectBVH::KinectBVH()
{
	q_z_40 = Vec_Math::quat_from_axis_anglef(0.f, 0.f, 1.f, 40.f*Vec_Math::kDegToRad);
	q_z_320 = Vec_Math::quat_from_axis_anglef(0.f, 0.f, 1.f, 320.f*Vec_Math::kDegToRad);
	q_z_30 = Vec_Math::quat_from_axis_anglef(0.f, 0.f, 1.f, 30.f*Vec_Math::kDegToRad);
	q_z_330 = Vec_Math::quat_from_axis_anglef(0.f, 0.f, 1.f, 330.f*Vec_Math::kDegToRad);
}

/**
* Destructeur
*/
KinectBVH::~KinectBVH()
{
	if (m_pFile.is_open())
	{
		m_pFile.close();
	}
}

/**
* Ajoute un offset ?la description du BVH
*/
void KinectBVH::AddOffset(int i, const Vector4 &offset)
{
	Vector4 one_offset;
	one_offset.x = offset.x * SCALE;
	one_offset.y = offset.y * SCALE;
	one_offset.z = offset.z * SCALE;
	one_offset.w = offset.w * SCALE;
	m_aOffsets.push_back(one_offset);
}

/**
* Créé un nouveau fichier en fonction du nom reçu en paramètre, renvoi true si réussi sinon false
*/
bool KinectBVH::CreateBVHFile(string filename)
{
	m_nbFrame = 0;

	m_aOffsets.clear();
	m_vPositions.clear();
	m_vBonesOrientation.clear();

	if (m_pFile.is_open())
	{
		m_pFile.close();
	}

	m_pFile.open(filename.c_str());

	return m_pFile.is_open();
}

/**
* Génère le fichier BVH
*/
void KinectBVH::FillBVHFile()
{
	CreateSkeletonInformation();
	CreateMotionInformation();
	m_pFile.close();
}

/**
* Génère la description du squelette pour le BVH
*/
void KinectBVH::CreateSkeletonInformation()
{
	stringstream flux;

	// ROOT
	flux << "HIERARCHY" << endl;
	flux << "ROOT Hip" << endl;
	flux << "{" << endl;

		// Spine
		flux << "\tOFFSET " << -m_aOffsets[0].x << " " << m_aOffsets[0].y << " " << m_aOffsets[0].z << endl;
		flux << "\tCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation" << endl;
		flux << "\tJOINT Spine" << endl;
		flux << "\t{" << endl;

			// Shoulder Center
			flux << "\t\tOFFSET " << -m_aOffsets[1].x << " " << m_aOffsets[1].y << " " << m_aOffsets[1].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
			flux << "\t\tJOINT ShoulderCenter" << endl;
			flux << "\t\t{" << endl;
				// Head
				flux << "\t\t\tOFFSET " << -m_aOffsets[2].x << " " << m_aOffsets[2].y << " " << m_aOffsets[2].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
				flux << "\t\t\tJOINT Head" << endl;
				flux << "\t\t\t{" << endl;
					// End Site
					flux << "\t\t\t\tOFFSET " << -m_aOffsets[3].x << " " << m_aOffsets[3].y << " " << m_aOffsets[3].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					flux << "\t\t\t\tEnd Site" << endl;
					flux << "\t\t\t\t{" << endl;
						flux << "\t\t\t\t\tOFFSET 0.0 " << m_aOffsets[3].y<< " 0.0" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;

				// Shoulder Left
				flux << "\t\t\tJOINT ShoulderLeft" << endl;
				flux << "\t\t\t{" << endl;
					// Elbow Left
					flux << "\t\t\t\tOFFSET " << -m_aOffsets[4].x << " " << m_aOffsets[4].y << " " << m_aOffsets[4].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowLeft" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Left
						flux << "\t\t\t\t\tOFFSET " << -m_aOffsets[5].x << " " << m_aOffsets[5].y << " " << m_aOffsets[5].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristLeft" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Left
							flux << "\t\t\t\t\t\tOFFSET " << -m_aOffsets[6].x << " " << m_aOffsets[6].y << " " << m_aOffsets[6].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
							flux << "\t\t\t\t\t\tJOINT HandLeft" << endl;
							flux << "\t\t\t\t\t\t{" << endl;
								// End Site
								flux << "\t\t\t\t\t\t\tOFFSET " << -m_aOffsets[7].x << " " << m_aOffsets[7].y << " " << m_aOffsets[7].z << endl;
								flux << "\t\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
								flux << "\t\t\t\t\t\t\tEnd Site" << endl;
								flux << "\t\t\t\t\t\t\t{" << endl;
									flux << "\t\t\t\t\t\t\t\tOFFSET 0.0 0.0 0.0" << endl;
								flux << "\t\t\t\t\t\t\t}" << endl;
							flux << "\t\t\t\t\t\t}" << endl;
						flux << "\t\t\t\t\t}" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;

				// Shoulder Right
				flux << "\t\t\tJOINT ShoulderRight" << endl;
				flux << "\t\t\t{" << endl;
					// Elbow Right
					flux << "\t\t\t\tOFFSET " << -m_aOffsets[8].x << " " << m_aOffsets[8].y << " " << m_aOffsets[8].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowRight" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Right
						flux << "\t\t\t\t\tOFFSET " << -m_aOffsets[9].x << " " << m_aOffsets[9].y << " " << m_aOffsets[9].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristRight" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Right
							flux << "\t\t\t\t\t\tOFFSET " << -m_aOffsets[10].x << " " << m_aOffsets[10].y << " " << m_aOffsets[10].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
							flux << "\t\t\t\t\t\tJOINT HandRight" << endl;
							flux << "\t\t\t\t\t\t{" << endl;
								// End Site
								flux << "\t\t\t\t\t\t\tOFFSET " << -m_aOffsets[11].x << " " << m_aOffsets[11].y << " " << m_aOffsets[11].z << endl;
								flux << "\t\t\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
								flux << "\t\t\t\t\t\t\tEnd Site" << endl;
								flux << "\t\t\t\t\t\t\t{" << endl;
									flux << "\t\t\t\t\t\t\t\tOFFSET 0.0 0.0 0.0" << endl;
								flux << "\t\t\t\t\t\t\t}" << endl;
							flux << "\t\t\t\t\t\t}" << endl;
						flux << "\t\t\t\t\t}" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;
			
			flux << "\t\t}" << endl;

		flux << "\t}" << endl;

		// Hip Left
		flux << "\tJOINT HipLeft" << endl;
		flux << "\t{" << endl;

			// Knee Left
			flux << "\t\tOFFSET " << -m_aOffsets[12].x << " " << m_aOffsets[12].y << " " << m_aOffsets[12].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
			flux << "\t\tJOINT KneeLeft" << endl;
			flux << "\t\t{" << endl;

				// Ankle Left
				flux << "\t\t\tOFFSET " << -m_aOffsets[13].x << " " << m_aOffsets[13].y << " " << m_aOffsets[13].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
				flux << "\t\t\tJOINT AnkleLeft" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Left
					flux << "\t\t\t\tOFFSET " << -m_aOffsets[14].x << " " << m_aOffsets[14].y << " " << m_aOffsets[14].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					flux << "\t\t\t\tJOINT FootLeft" << endl;
					flux << "\t\t\t\t{" << endl;
					
						// End Site
						flux << "\t\t\t\t\tOFFSET " << -m_aOffsets[15].x << " " << m_aOffsets[15].y << " " << m_aOffsets[15].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tEnd Site" << endl;
						flux << "\t\t\t\t\t{" << endl;
							flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 0.0" << endl;
						flux << "\t\t\t\t\t}" << endl;	
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;
			flux << "\t\t}" << endl;
		flux << "\t}" << endl;

		// Hip Right
		flux << "\tJOINT HipRight" << endl;
		flux << "\t{" << endl;

			// Knee Right
			flux << "\t\tOFFSET " << -m_aOffsets[16].x << " " << m_aOffsets[16].y << " " << m_aOffsets[16].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
			flux << "\t\tJOINT KneeRight" << endl;
			flux << "\t\t{" << endl;

				// Ankle Right
				flux << "\t\t\tOFFSET " << -m_aOffsets[17].x << " " << m_aOffsets[17].y << " " << m_aOffsets[17].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
				flux << "\t\t\tJOINT AnkleRight" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Right
					flux << "\t\t\t\tOFFSET " << -m_aOffsets[18].x << " " << m_aOffsets[18].y << " " << m_aOffsets[18].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
					flux << "\t\t\t\tJOINT FootRight" << endl;
					flux << "\t\t\t\t{" << endl;
					
						// End Site
						flux << "\t\t\t\t\tOFFSET " << -m_aOffsets[19].x << " " << m_aOffsets[19].y << " " << m_aOffsets[19].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Xrotation Yrotation" << endl;
						flux << "\t\t\t\t\tEnd Site" << endl;
						flux << "\t\t\t\t\t{" << endl;
							flux << "\t\t\t\t\t\tOFFSET 0.0 0.0 0.0" << endl;
						flux << "\t\t\t\t\t}" << endl;	
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;
			flux << "\t\t}" << endl;
		flux << "\t}" << endl;

	flux << "}" << endl;

	m_pFile << flux.str();
}

/**
* Incrémente le nombre de frames
*/
void KinectBVH::IncrementNbFrames()
{
	++m_nbFrame;
}

/**
* Ajoute un squelette et ses informations pour les données de la capture de mouvements
*/
void KinectBVH::AddBonesOrientation(KinectJoint *joints)
{
	for(int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++) {
		m_vBonesOrientation.push_back(joints[i]);
	}
}

/**
* Ajoute une position du joint Hip Center pour les données de la capture de mouvements
*/
void KinectBVH::AddPosition(const Vector4 &position)
{
	Vector4 pos;
	pos.x = position.x * 100.f * SCALE;
	pos.y = position.y * 100.f * SCALE;
	pos.z = position.z * 100.f * SCALE;
	pos.w = position.w;
	m_vPositions.push_back(pos);
}

void KinectBVH::GetAngles(KinectJoint *joints, int idx, double angles[])
{
	Vec_Math::Quaternion q;

	// set identity quaternion on bone tip
	if (idx==NUI_SKELETON_POSITION_HEAD||
		idx==NUI_SKELETON_POSITION_HAND_LEFT||
		idx==NUI_SKELETON_POSITION_HAND_RIGHT||
		idx==NUI_SKELETON_POSITION_FOOT_LEFT||
		idx==NUI_SKELETON_POSITION_FOOT_RIGHT) {
		q = Vec_Math::quat_identity;
	} else {
		// get bone's quaternion from bone tip
		q = Vec_Math::vec4_create(
			joints[idx+1].quat.x,
			joints[idx+1].quat.y,
			joints[idx+1].quat.z,
			joints[idx+1].quat.w);
	}


	// correct some bones
	Vec_Math::Quaternion q_delta;
	if (idx == NUI_SKELETON_POSITION_SHOULDER_RIGHT) {
		q_delta = Vec_Math::quat_multiply(q_z_30, q);
	} else if (idx == NUI_SKELETON_POSITION_SHOULDER_LEFT) {
		q_delta = Vec_Math::quat_multiply(q_z_330, q);
	} else if (idx == NUI_SKELETON_POSITION_HIP_RIGHT) {
		q_delta = Vec_Math::quat_multiply(q_z_320, q);
	} else if (idx == NUI_SKELETON_POSITION_HIP_LEFT) {
		q_delta = Vec_Math::quat_multiply(q_z_40, q);
	} else {
		q_delta = q;
	}

	// convert the quaternion to euler angles by ZXY order
	Quat2Euler::Quaternion q_to_convert(q_delta.x, q_delta.y, q_delta.z, q_delta.w);
	Quat2Euler::quaternion2Euler(q_to_convert, angles, Quat2Euler::zxy);

	if (idx == NUI_SKELETON_POSITION_SHOULDER_RIGHT) {
		// flip yaw data
		angles[0] = -angles[0];
	}

	if (idx == NUI_SKELETON_POSITION_SHOULDER_LEFT) {
		// turn from back to front
		angles[0] = angles[0] + M_PI;
	}

	if (idx == NUI_SKELETON_POSITION_ELBOW_LEFT) {
		// swap yaw and roll
		double t = angles[0];
		angles[0] = -angles[2];
		angles[2] = -t;
	}

	if (idx == NUI_SKELETON_POSITION_ELBOW_RIGHT) {
		// swap yaw and roll
		double t = angles[0];
		angles[0] = angles[2];
		angles[2] = -t;
	}

	if (idx == NUI_SKELETON_POSITION_WRIST_RIGHT) {
		// flip roll data
		angles[2] = -angles[2];
	}

	if (idx == NUI_SKELETON_POSITION_HIP_RIGHT) {
		// swap pitch and yaw
		double t = angles[0];
		angles[0] = -angles[1];
		angles[1] = t;
	}

	if (idx == NUI_SKELETON_POSITION_HIP_LEFT) {
		// flip yaw data
		angles[1] = -angles[1];
	}

	if (idx == NUI_SKELETON_POSITION_KNEE_LEFT) {
		// swap pitch and yaw
		double t = angles[0];
		angles[0] = -angles[1];
		angles[1] = -t;
	}

	if (idx == NUI_SKELETON_POSITION_KNEE_RIGHT) {
		// swap pitch and yaw
		double t = angles[0];
		angles[0] = -angles[1];
		angles[1] = -t;
	}

	if (idx == NUI_SKELETON_POSITION_ANKLE_LEFT) {
		// swap pitch ,yaw and roll
		double t = angles[0];
		angles[0] = angles[1];
		angles[1] = angles[2];
		angles[2] = t;
	}

	if (idx == NUI_SKELETON_POSITION_ANKLE_RIGHT) {
		// swap pitch ,yaw and roll
		double t = angles[0];
		angles[0] = angles[1];
		angles[1] = angles[2];
		angles[2] = t;
	}

	// clamp to valid range
	if (angles[0] > M_PI) {
		angles[0] -= Vec_Math::k2Pi;
	}
	if (angles[0] < -M_PI) {
		angles[0] += Vec_Math::k2Pi;
	}
	if (angles[1] > M_PI) {
		angles[1] -= Vec_Math::k2Pi;
	}
	if (angles[1] < -M_PI) {
		angles[1] += Vec_Math::k2Pi;
	}
	if (angles[2] > M_PI) {
		angles[2] -= Vec_Math::k2Pi;
	}
	if (angles[2] < -M_PI) {
		angles[2] += Vec_Math::k2Pi;
	}
}

/**
* Génère les données des mouvements pour le BVH
*/
void KinectBVH::CreateMotionInformation()
{
	stringstream flux;

	flux << "\nMOTION" << endl;
	flux << "Frames: " << m_nbFrame << endl;
	flux << "Frame Time: " << FPS << endl;

	for (int i = 0; i < static_cast<int>(m_vPositions.size()); i++) {
		flux << -m_vPositions[i].x << " " << m_vPositions[i].y << " " << -m_vPositions[i].z << " ";
		KinectJoint *joints = &m_vBonesOrientation[i * NUI_SKELETON_POSITION_COUNT];
		for(int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++) {
			double angles[3];
			GetAngles(joints, j, angles);
			if(_isnan(angles[0])) angles[0] = 0.;
			if(_isnan(angles[1])) angles[1] = 0.;
			if(_isnan(angles[2])) angles[2] = 0.;
			flux << angles[2] * Vec_Math::kRadToDeg << " " << angles[0] * Vec_Math::kRadToDeg << " " << angles[1] * Vec_Math::kRadToDeg << " ";
		}
		flux << endl;
	}

	m_pFile << flux.str();
}
