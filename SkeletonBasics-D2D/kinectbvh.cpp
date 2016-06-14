#include "kinectbvh.h"

/**
* Constructeur
*/
KinectBVH::KinectBVH()
{
    // fill parent joint map
    parent_joint_map[NUI_SKELETON_POSITION_HIP_CENTER] = NUI_SKELETON_POSITION_HIP_CENTER;
    parent_joint_map[NUI_SKELETON_POSITION_SPINE] = NUI_SKELETON_POSITION_HIP_CENTER;
    parent_joint_map[NUI_SKELETON_POSITION_SHOULDER_CENTER] = NUI_SKELETON_POSITION_SPINE;
    parent_joint_map[NUI_SKELETON_POSITION_HEAD] = NUI_SKELETON_POSITION_SHOULDER_CENTER;
    parent_joint_map[NUI_SKELETON_POSITION_SHOULDER_LEFT] = NUI_SKELETON_POSITION_SHOULDER_CENTER;
    parent_joint_map[NUI_SKELETON_POSITION_ELBOW_LEFT] = NUI_SKELETON_POSITION_SHOULDER_LEFT;
    parent_joint_map[NUI_SKELETON_POSITION_WRIST_LEFT] = NUI_SKELETON_POSITION_ELBOW_LEFT;
    parent_joint_map[NUI_SKELETON_POSITION_HAND_LEFT] = NUI_SKELETON_POSITION_WRIST_LEFT;
    parent_joint_map[NUI_SKELETON_POSITION_SHOULDER_RIGHT] = NUI_SKELETON_POSITION_SHOULDER_CENTER;
    parent_joint_map[NUI_SKELETON_POSITION_ELBOW_RIGHT] = NUI_SKELETON_POSITION_SHOULDER_RIGHT;
    parent_joint_map[NUI_SKELETON_POSITION_WRIST_RIGHT] = NUI_SKELETON_POSITION_ELBOW_RIGHT;
    parent_joint_map[NUI_SKELETON_POSITION_HAND_RIGHT] = NUI_SKELETON_POSITION_WRIST_RIGHT;
    parent_joint_map[NUI_SKELETON_POSITION_HIP_LEFT] = NUI_SKELETON_POSITION_HIP_CENTER;
    parent_joint_map[NUI_SKELETON_POSITION_KNEE_LEFT] = NUI_SKELETON_POSITION_HIP_LEFT;
    parent_joint_map[NUI_SKELETON_POSITION_ANKLE_LEFT] = NUI_SKELETON_POSITION_KNEE_LEFT;
    parent_joint_map[NUI_SKELETON_POSITION_FOOT_LEFT] = NUI_SKELETON_POSITION_ANKLE_LEFT;
    parent_joint_map[NUI_SKELETON_POSITION_HIP_RIGHT] = NUI_SKELETON_POSITION_HIP_CENTER;
    parent_joint_map[NUI_SKELETON_POSITION_KNEE_RIGHT] = NUI_SKELETON_POSITION_HIP_RIGHT;
    parent_joint_map[NUI_SKELETON_POSITION_ANKLE_RIGHT] = NUI_SKELETON_POSITION_KNEE_RIGHT;
    parent_joint_map[NUI_SKELETON_POSITION_FOOT_RIGHT] = NUI_SKELETON_POSITION_ANKLE_RIGHT;
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
	CorrectKinect();
    CreateQuaternionInformation();
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
		flux << "\tOFFSET " << m_aOffsets[0].x << " " << m_aOffsets[0].y << " " << m_aOffsets[0].z << endl;
		flux << "\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation" << endl;
		flux << "\tJOINT Spine" << endl;
		flux << "\t{" << endl;

			// Shoulder Center
			flux << "\t\tOFFSET " << m_aOffsets[1].x << " " << m_aOffsets[1].y << " " << m_aOffsets[1].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
			flux << "\t\tJOINT ShoulderCenter" << endl;
			flux << "\t\t{" << endl;
				// Head
				flux << "\t\t\tOFFSET " << m_aOffsets[2].x << " " << m_aOffsets[2].y << " " << m_aOffsets[2].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
				flux << "\t\t\tJOINT Head" << endl;
				flux << "\t\t\t{" << endl;
					// End Site
					flux << "\t\t\t\tOFFSET " << m_aOffsets[3].x << " " << m_aOffsets[3].y << " " << m_aOffsets[3].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tEnd Site" << endl;
					flux << "\t\t\t\t{" << endl;
						flux << "\t\t\t\t\tOFFSET 0.0 " << m_aOffsets[3].y<< " 0.0" << endl;
					flux << "\t\t\t\t}" << endl;
				flux << "\t\t\t}" << endl;

				// Shoulder Left
				flux << "\t\t\tJOINT ShoulderLeft" << endl;
				flux << "\t\t\t{" << endl;
					// Elbow Left
					flux << "\t\t\t\tOFFSET " << m_aOffsets[4].x << " " << m_aOffsets[4].y << " " << m_aOffsets[4].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowLeft" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Left
						flux << "\t\t\t\t\tOFFSET " << m_aOffsets[5].x << " " << m_aOffsets[5].y << " " << m_aOffsets[5].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristLeft" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Left
							flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[6].x << " " << m_aOffsets[6].y << " " << m_aOffsets[6].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
							flux << "\t\t\t\t\t\tJOINT HandLeft" << endl;
							flux << "\t\t\t\t\t\t{" << endl;
								// End Site
								flux << "\t\t\t\t\t\t\tOFFSET " << m_aOffsets[7].x << " " << m_aOffsets[7].y << " " << m_aOffsets[7].z << endl;
								flux << "\t\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
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
					flux << "\t\t\t\tOFFSET " << m_aOffsets[8].x << " " << m_aOffsets[8].y << " " << m_aOffsets[8].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tJOINT ElbowRight" << endl;
					flux << "\t\t\t\t{" << endl;
						// Wrist Right
						flux << "\t\t\t\t\tOFFSET " << m_aOffsets[9].x << " " << m_aOffsets[9].y << " " << m_aOffsets[9].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
						flux << "\t\t\t\t\tJOINT WristRight" << endl;
						flux << "\t\t\t\t\t{" << endl;
							// Hand Right
							flux << "\t\t\t\t\t\tOFFSET " << m_aOffsets[10].x << " " << m_aOffsets[10].y << " " << m_aOffsets[10].z << endl;
							flux << "\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
							flux << "\t\t\t\t\t\tJOINT HandRight" << endl;
							flux << "\t\t\t\t\t\t{" << endl;
								// End Site
								flux << "\t\t\t\t\t\t\tOFFSET " << m_aOffsets[11].x << " " << m_aOffsets[11].y << " " << m_aOffsets[11].z << endl;
								flux << "\t\t\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
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
			flux << "\t\tOFFSET " << m_aOffsets[12].x << " " << m_aOffsets[12].y << " " << m_aOffsets[12].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
			flux << "\t\tJOINT KneeLeft" << endl;
			flux << "\t\t{" << endl;

				// Ankle Left
				flux << "\t\t\tOFFSET " << m_aOffsets[13].x << " " << m_aOffsets[13].y << " " << m_aOffsets[13].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
				flux << "\t\t\tJOINT AnkleLeft" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Left
					flux << "\t\t\t\tOFFSET " << m_aOffsets[14].x << " " << m_aOffsets[14].y << " " << m_aOffsets[14].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tJOINT FootLeft" << endl;
					flux << "\t\t\t\t{" << endl;
					
						// End Site
						flux << "\t\t\t\t\tOFFSET " << m_aOffsets[15].x << " " << m_aOffsets[15].y << " " << m_aOffsets[15].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
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
			flux << "\t\tOFFSET " << m_aOffsets[16].x << " " << m_aOffsets[16].y << " " << m_aOffsets[16].z << endl;
			flux << "\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
			flux << "\t\tJOINT KneeRight" << endl;
			flux << "\t\t{" << endl;

				// Ankle Right
				flux << "\t\t\tOFFSET " << m_aOffsets[17].x << " " << m_aOffsets[17].y << " " << m_aOffsets[17].z << endl;
				flux << "\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
				flux << "\t\t\tJOINT AnkleRight" << endl;
				flux << "\t\t\t{" << endl;

					// Foot Right
					flux << "\t\t\t\tOFFSET " << m_aOffsets[18].x << " " << m_aOffsets[18].y << " " << m_aOffsets[18].z << endl;
					flux << "\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
					flux << "\t\t\t\tJOINT FootRight" << endl;
					flux << "\t\t\t\t{" << endl;
					
						// End Site
						flux << "\t\t\t\t\tOFFSET " << m_aOffsets[19].x << " " << m_aOffsets[19].y << " " << m_aOffsets[19].z << endl;
						flux << "\t\t\t\t\tCHANNELS 3 Zrotation Yrotation Xrotation" << endl;
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

Vec_Math::Vec3 KinectBVH::GetEulers(KinectJoint *joints, int idx)
{
    // get parent's quaternion
    Vec_Math::Quaternion q_parent;
    if (idx == NUI_SKELETON_POSITION_HIP_CENTER) {
        q_parent = Vec_Math::quat_identity;
    } else {
        q_parent = Vec_Math::vec4_create(joints[parent_joint_map[idx]].quat.x,
                                         joints[parent_joint_map[idx]].quat.y,
                                         joints[parent_joint_map[idx]].quat.z,
                                         joints[parent_joint_map[idx]].quat.w);
    }

    // get joint's quaternion
    Vec_Math::Quaternion q_current = Vec_Math::vec4_create(joints[idx].quat.x,
                                                           joints[idx].quat.y,
                                                           joints[idx].quat.z,
                                                           joints[idx].quat.w);

    // calculate between quaternion
	Vec_Math::Quaternion q_delta = Vec_Math::quat_left_multiply(q_current, Vec_Math::quat_inverse(q_parent));

	// convert the quaternion to euler angles by roll->yaw->pitch order, which roll is outer, pitch is inner.
    Vec_Math::Vec3 ret = Vec_Math::euler_from_quat(q_delta);
    
    return ret;
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
		flux << m_vPositions[i].x << " " << m_vPositions[i].y << " " << m_vPositions[i].z << " ";
		KinectJoint *joints = &m_vBonesOrientation[i * NUI_SKELETON_POSITION_COUNT];
		for(int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++) {
			Vec_Math::Vec3 angles = GetEulers(joints, j);
			flux << angles.z * Vec_Math::kRadToDeg << " " << angles.y * Vec_Math::kRadToDeg << " " << angles.x * Vec_Math::kRadToDeg << " ";
		}
		flux << endl;
	}

	m_pFile << flux.str();
}

void KinectBVH::CorrectKinect()
{
	// kinect's pitch angle
	const float kinect_angle = -6.f;
	Vec_Math::Mat3 correct_matrix = Vec_Math::mat3_rotation_x(kinect_angle * Vec_Math::kDegToRad);

	Vec_Math::Vec3 one_pos;
	for (int i = 0; i < static_cast<int>(m_vPositions.size()); i++) {
		// correct body's position
		one_pos.x = m_vPositions[i].x;
		one_pos.y = m_vPositions[i].y;
		one_pos.z = m_vPositions[i].z;
		one_pos = Vec_Math::mat3_mul_vector(one_pos, correct_matrix);
		m_vPositions[i].x = one_pos.x;
		m_vPositions[i].y = one_pos.y;
		m_vPositions[i].z = one_pos.z;

		// correct joint's position
		KinectJoint *joints = &m_vBonesOrientation[i * NUI_SKELETON_POSITION_COUNT];
		for(int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++) {
			one_pos.x = joints[j].pos.x;
			one_pos.y = joints[j].pos.y;
			one_pos.z = joints[j].pos.z;
			one_pos = Vec_Math::mat3_mul_vector(one_pos, correct_matrix);
			joints[j].pos.x = one_pos.x;
			joints[j].pos.y = one_pos.y;
			joints[j].pos.z = one_pos.z;
		}
	}
}

void KinectBVH::CreateQuaternionInformation()
{
    // we save last stable x axis for each joint to avoid trembling
    Vec_Math::Vec3 last_stable_vx[NUI_SKELETON_POSITION_COUNT];
    for (int i=0; i<NUI_SKELETON_POSITION_COUNT; i++) {
        last_stable_vx[i] = Vec_Math::vec3_zero;
    }

    // loop through all records
    for (int i = 0; i < static_cast<int>(m_vPositions.size()); i++) {
        KinectJoint *joints = &m_vBonesOrientation[i * NUI_SKELETON_POSITION_COUNT];

        const float MAX_STABLE_DOT = 0.9f;
        float dot;
        Vector4 p1, p2;
        Vec_Math::Vec3 v1, v2;
        Vec_Math::Vec3 vx, vy, vz;
        Vec_Math::Vec3 v_body_x;
        Vec_Math::Mat3 m, mr;
        Vec_Math::Quaternion q;

        // NUI_SKELETON_POSITION_HIP_CENTER
        p1 = joints[NUI_SKELETON_POSITION_HIP_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_HIP_RIGHT].pos;
        vx = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_HIP_CENTER].pos;
        p2 = joints[NUI_SKELETON_POSITION_SPINE].pos;
        vy = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_HIP_CENTER].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_HIP_CENTER].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_HIP_CENTER].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_HIP_CENTER].quat.w = q.w;
        
		// save body's x axis for later use
        v_body_x = vx;
        
        // NUI_SKELETON_POSITION_SPINE
        p1 = joints[NUI_SKELETON_POSITION_HIP_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_HIP_RIGHT].pos;
        vx = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_SPINE].pos;
        p2 = joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].pos;
        vy = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_SPINE].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_SPINE].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_SPINE].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_SPINE].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_SHOULDER_CENTER
        p1 = joints[NUI_SKELETON_POSITION_HIP_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_HIP_RIGHT].pos;
        vx = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].pos;
        p2 = joints[NUI_SKELETON_POSITION_HEAD].pos;
        vy = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_HEAD
        joints[NUI_SKELETON_POSITION_HEAD].quat = joints[NUI_SKELETON_POSITION_SHOULDER_CENTER].quat;

        // NUI_SKELETON_POSITION_SHOULDER_LEFT
        p1 = joints[NUI_SKELETON_POSITION_SHOULDER_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ELBOW_LEFT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_ELBOW_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_WRIST_LEFT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_SHOULDER_LEFT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_SHOULDER_LEFT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_SHOULDER_LEFT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_SHOULDER_LEFT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_SHOULDER_LEFT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_SHOULDER_LEFT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_ELBOW_LEFT
        p1 = joints[NUI_SKELETON_POSITION_ELBOW_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_WRIST_LEFT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_WRIST_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_HAND_LEFT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_ELBOW_LEFT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_ELBOW_LEFT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_ELBOW_LEFT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_ELBOW_LEFT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_ELBOW_LEFT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_ELBOW_LEFT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_WRIST_LEFT
        p1 = joints[NUI_SKELETON_POSITION_ELBOW_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_WRIST_LEFT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_WRIST_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_HAND_LEFT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_ELBOW_LEFT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_ELBOW_LEFT] = vx;
        }
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_WRIST_LEFT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_WRIST_LEFT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_WRIST_LEFT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_WRIST_LEFT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_HAND_LEFT
        joints[NUI_SKELETON_POSITION_HAND_LEFT].quat = joints[NUI_SKELETON_POSITION_WRIST_LEFT].quat;
        
        // NUI_SKELETON_POSITION_SHOULDER_RIGHT
        p1 = joints[NUI_SKELETON_POSITION_SHOULDER_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_WRIST_RIGHT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_SHOULDER_RIGHT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(-Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_SHOULDER_RIGHT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_SHOULDER_RIGHT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_SHOULDER_RIGHT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_SHOULDER_RIGHT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_ELBOW_RIGHT
        p1 = joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_WRIST_RIGHT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_WRIST_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_HAND_RIGHT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_ELBOW_RIGHT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_ELBOW_RIGHT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(-Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_WRIST_RIGHT
        p1 = joints[NUI_SKELETON_POSITION_ELBOW_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_WRIST_RIGHT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_WRIST_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_HAND_RIGHT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_ELBOW_RIGHT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_ELBOW_RIGHT] = vx;
        }
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(-Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_WRIST_RIGHT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_WRIST_RIGHT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_WRIST_RIGHT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_WRIST_RIGHT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_HAND_RIGHT
        joints[NUI_SKELETON_POSITION_HAND_RIGHT].quat = joints[NUI_SKELETON_POSITION_WRIST_RIGHT].quat;
        
        // NUI_SKELETON_POSITION_HIP_LEFT
        p1 = joints[NUI_SKELETON_POSITION_HIP_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_KNEE_LEFT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_KNEE_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ANKLE_LEFT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_HIP_LEFT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            // reverse the direction because knees can only bend to back
            vx = Vec_Math::vec3_negate(vx);
            dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v_body_x), Vec_Math::vec3_normalize(vx));
            // knees bend forward
            if (dot > 0) {
                vx = Vec_Math::vec3_negate(vx);
            }
            last_stable_vx[NUI_SKELETON_POSITION_HIP_LEFT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_HIP_LEFT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_HIP_LEFT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_HIP_LEFT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_HIP_LEFT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_KNEE_LEFT
        p1 = joints[NUI_SKELETON_POSITION_KNEE_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ANKLE_LEFT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_ANKLE_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_FOOT_LEFT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_KNEE_LEFT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_KNEE_LEFT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_KNEE_LEFT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_KNEE_LEFT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_KNEE_LEFT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_KNEE_LEFT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_ANKLE_LEFT
        p1 = joints[NUI_SKELETON_POSITION_KNEE_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ANKLE_LEFT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_ANKLE_LEFT].pos;
        p2 = joints[NUI_SKELETON_POSITION_FOOT_LEFT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_ANKLE_LEFT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_ANKLE_LEFT] = vx;
        }
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_x(Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_ANKLE_LEFT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_ANKLE_LEFT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_ANKLE_LEFT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_ANKLE_LEFT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_FOOT_LEFT
        joints[NUI_SKELETON_POSITION_FOOT_LEFT].quat = joints[NUI_SKELETON_POSITION_ANKLE_LEFT].quat;
        
        // NUI_SKELETON_POSITION_HIP_RIGHT
        p1 = joints[NUI_SKELETON_POSITION_HIP_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_KNEE_RIGHT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_KNEE_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_HIP_RIGHT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            // reverse the direction because knees can only bend to back
            vx = Vec_Math::vec3_negate(vx);
            dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v_body_x), Vec_Math::vec3_normalize(vx));
            // knees bend forward
            if (dot > 0) {
                vx = Vec_Math::vec3_negate(vx);
            }
            last_stable_vx[NUI_SKELETON_POSITION_HIP_RIGHT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_HIP_RIGHT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_HIP_RIGHT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_HIP_RIGHT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_HIP_RIGHT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_KNEE_RIGHT
        p1 = joints[NUI_SKELETON_POSITION_KNEE_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_FOOT_RIGHT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_KNEE_RIGHT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_KNEE_RIGHT] = vx;
        }
        vy = v1;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_z(Vec_Math::kPi));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_KNEE_RIGHT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_KNEE_RIGHT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_KNEE_RIGHT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_KNEE_RIGHT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_ANKLE_RIGHT
        p1 = joints[NUI_SKELETON_POSITION_KNEE_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].pos;
        v1 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        p1 = joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].pos;
        p2 = joints[NUI_SKELETON_POSITION_FOOT_RIGHT].pos;
        v2 = Vec_Math::vec3_create(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        dot = Vec_Math::vec3_dot(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
		// cross product will be unstable
        if (fabsf(dot) > MAX_STABLE_DOT) {
            vx = last_stable_vx[NUI_SKELETON_POSITION_ANKLE_RIGHT];
        } else {
            vx = Vec_Math::vec3_cross(Vec_Math::vec3_normalize(v1), Vec_Math::vec3_normalize(v2));
            last_stable_vx[NUI_SKELETON_POSITION_ANKLE_RIGHT] = vx;
        }
        vy = v2;
        vz = Vec_Math::vec3_zero;
        m = Vec_Math::mat3_from_axis(vx, vy, vz);
		// inverse bind pose
        mr = Vec_Math::mat3_inverse(Vec_Math::mat3_rotation_x(Vec_Math::kPiDiv2));
        m = Vec_Math::mat3_multiply(mr, m);
        q = Vec_Math::quat_from_mat3(m);
		joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].quat.x = q.x;
		joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].quat.y = q.y;
		joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].quat.z = q.z;
		joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].quat.w = q.w;
        
        // NUI_SKELETON_POSITION_FOOT_RIGHT
        joints[NUI_SKELETON_POSITION_FOOT_RIGHT].quat = joints[NUI_SKELETON_POSITION_ANKLE_RIGHT].quat;
    }
}
