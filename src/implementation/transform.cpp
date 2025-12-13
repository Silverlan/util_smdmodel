// SPDX-FileCopyrightText: (c) 2024 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

module se_smd.model;

static Mat4 euler_angles_to_matrix(const EulerAngles &ang)
{
	auto r = Mat4(1.f);
	r = glm::gtc::rotate(r, -ang.r, uvec::RIGHT);
	r = glm::gtc::rotate(r, -ang.y, uvec::UP);
	r = glm::gtc::rotate(r, ang.p, uvec::FORWARD);
	return r;
}

static void translate_matrix(Mat4 &m, const Vector3 &offset)
{
	m[3][0] = offset.x;
	m[3][1] = offset.y;
	m[3][2] = offset.z;
}

static Mat4 mul_matrix(const Mat4 &m1, const Mat4 &m2)
{
	auto r = Mat4(1.f);
	for(auto i = 0; i <= 3; i++) {
		for(auto j = 0; j <= 3; j++) {
			auto sum = 0.f;
			for(auto k = 0; k <= 3; k++)
				sum += m1[i][k] * m2[k][j];
			r[i][j] = sum;
		}
	}
	return r;
}

static Vector3 get_translation(const Mat4 &m) { return Vector3(m[3][0], m[3][1], m[3][2]); }

static Quat get_rotation(const Mat4 &m)
{
	auto q = Quat(1.f, 0.f, 0.f, 0.f);
	auto trace = m[0][0] + m[1][1] + m[2][2];
	if(trace > 0.f) {
		auto s = 0.5f / sqrtf(trace + 1.f);
		q.w = 0.25f / s;
		q.x = (m[2][1] - m[1][2]) * -s;
		q.y = (m[0][2] - m[2][0]) * s;
		q.z = (m[1][0] - m[0][1]) * s;
	}
	else {
		if(m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
			auto s = 2.f * sqrtf(1.f + m[0][0] - m[1][1] - m[2][2]);
			q.w = (m[2][1] - m[1][2]) / s;
			q.x = 0.25f * -s;
			q.y = (m[0][1] + m[1][0]) / s;
			q.z = (m[0][2] + m[2][0]) / s;
		}
		else if(m[1][1] > m[2][2]) {
			auto s = 2.f * sqrtf(1.0 + m[1][1] - m[0][0] - m[2][2]);
			q.w = (m[0][2] - m[2][0]) / -s;
			q.x = (m[0][1] + m[1][0]) / s;
			q.y = 0.25f * -s;
			q.z = (m[1][2] + m[2][1]) / -s;
		}
		else {
			auto s = 2.f * sqrtf(1.0 + m[2][2] - m[0][0] - m[1][1]);
			q.w = (m[1][0] - m[0][1]) / -s;
			q.x = (m[0][2] + m[2][0]) / s;
			q.y = (m[1][2] + m[2][1]) / -s;
			q.z = 0.25f * -s;
		}
	}
	return q;
}

void source_engine::smd::SMDModel::BuildNodeMatrix(Frame &frame, UInt id, std::vector<Mat4> &matrices)
{
	auto &nodes = GetNodes();
	auto &node = nodes[id];
	auto &t = frame.transforms[id];
	auto &pos = t.position;
	auto &ang = t.angles;
	auto m = euler_angles_to_matrix(ang);
	translate_matrix(m, pos);
	if(node.parent != -1)
		m = mul_matrix(m, matrices[node.parent]);
	matrices[id] = m;
	for(auto it = node.children.begin(); it != node.children.end(); ++it)
		BuildNodeMatrix(frame, *it, matrices);
}

static void rotation_to_axis_angle(Quat &rot, Vector3 &axis, float &angle)
{
	angle = 2.f * pragma::math::acos(rot.w);
	auto sqr = sqrtf(1.f - rot.w * rot.w);
	if(sqr == 0.f) {
		axis = Vector3(0.f, 0.f, 0.f);
		return;
	}
	axis = Vector3(rot.x / sqr, rot.y / sqr, rot.z / sqr);
}

static void axis_angle_to_rotation(Vector3 &axis, float angle, Quat &rot)
{
	auto s = pragma::math::sin(angle / 2.f);
	if(s == 0.f) {
		rot = Quat(1.f, 0.f, 0.f, 0.f);
		return;
	}
	rot = Quat(pragma::math::cos(angle / 2.f), axis.x * s, axis.y * s, axis.z * s);
}

static void convert_rotation(Quat &rot)
{
	Vector3 axis;
	float angle;
	rotation_to_axis_angle(rot, axis, angle);
	// Equivalent to QuatXYZToXZY in MaxScript
	auto xAxis = axis.x;
	axis.x = -axis.y;
	axis.y = -axis.z;
	axis.z = -xAxis;
	//
	axis_angle_to_rotation(axis, angle, rot);

	// This is to comply to an accidental mistake in axis shift when loading the wrmi-file
	auto w = rot.w;
	auto x = rot.x;
	auto y = rot.y;
	auto z = rot.z;
	rot.x = w;
	rot.y = x;
	rot.z = y;
	rot.w = z;
}

void source_engine::smd::SMDModel::ConvertCoordinateSystem()
{
	auto &nodes = GetNodes();
	auto &frames = GetFrames();
	auto &skeleton = GetSkeleton();
	for(auto it = frames.begin(); it != frames.end(); ++it) {
		auto &frame = *it;
		std::vector<Mat4> matrices(nodes.size());
		for(auto it = skeleton.rootNodes.begin(); it != skeleton.rootNodes.end(); ++it) {
			auto rootNode = *it;
			BuildNodeMatrix(frame, rootNode, matrices);
		}
		for(UInt i = 0; i < frame.transforms.size(); i++) {
			auto &t = frame.transforms[i];
			auto &m = matrices[i];
			t.position = get_translation(m);
			auto y = t.position.y;
			t.position.y = t.position.z;
			t.position.z = -y;
			t.position.x = -t.position.x;

			t.rotation = get_rotation(m);
			convert_rotation(t.rotation);
			t.angles = EulerAngles(t.rotation);
		}
	}
	auto &meshes = GetMeshes();
	for(auto it = meshes.begin(); it != meshes.end(); ++it) {
		auto &mesh = *it;
		for(auto it = mesh.triangles.begin(); it != mesh.triangles.end(); ++it) {
			auto &tri = *it;
			for(auto i = 0; i < 3; i++) {
				auto &v = tri.vertices[i];
				auto y = v.position.y;
				v.position.y = v.position.z;
				v.position.z = -y;

				y = v.normal.y;
				v.normal.y = v.normal.z;
				v.normal.z = -y;
			}
		}
	}
}
