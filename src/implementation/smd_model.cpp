// SPDX-FileCopyrightText: (c) 2024 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

module se_smd.model;

using namespace source_engine::smd;

#define MAX_VERT_LENGTH 524'288

SMDModel::Node::Node() : id(-1), parent(-1) {}

SMDModel::Skeleton::Skeleton() : startTime(-1) {}

SMDModel::Frame::Frame() {}

SMDModel::FrameTransform::FrameTransform() {}

SMDModel::Vertex::Vertex() : bone(-1) {}

SMDModel::Triangle::Triangle() {}

SMDModel::Mesh::Mesh(const std::string &tex) : texture(tex) {}

SMDModel::Mesh::Mesh() : Mesh("") {}

////////////////////////////////////////

SMDModel::SMDModel() {}

SMDModel::~SMDModel() {}

void SMDModel::ReadNodeBlock(pragma::fs::VFilePtr &f)
{
	std::string l;
	while(ReadBlockLine(f, l) == true) {
		std::vector<std::string> args;
		pragma::string::explode_whitespace(l, args);
		if(args.size() >= 3) {
			for(auto i = 0; i < 3; i++)
				pragma::string::remove_quotes(args[i]);
			m_nodes.push_back(Node());
			auto &node = m_nodes.back();
			node.id = pragma::string::to_int(args[0]);
			node.name = args[1];
			node.parent = pragma::string::to_int(args[2]);
		}
	}
}

void SMDModel::ReadSkeletonBlock(pragma::fs::VFilePtr &f)
{
	std::string l;
	int time = -1;
	while(ReadBlockLine(f, l) == true) {
		std::vector<std::string> args;
		pragma::string::explode_whitespace(l, args);
		if(args.size() > 1) {
			pragma::string::to_lower(args[0]);
			if(args[0] == "time") {
				auto frame = pragma::string::to_int(args[1]);
				if(time != -1 && frame != (time + 1))
					return;
				if(time == -1)
					m_skeleton.startTime = frame;
				time = frame;
				m_frames.push_back(Frame());
			}
			else if(time != -1 && args.size() >= 7) {
				auto &frame = m_frames.back();
				auto boneId = pragma::string::to_int(args[0]);
				if(boneId >= 0) {
					if(frame.transforms.size() < (boneId + 1))
						frame.transforms.resize(boneId + 1);
					auto &t = frame.transforms[boneId];
					t.position = Vector3(-pragma::string::to_float(args[1]), pragma::string::to_float(args[2]), pragma::string::to_float(args[3]));
					t.angles = EulerAngles(pragma::string::to_float(args[4]), pragma::string::to_float(args[5]), pragma::string::to_float(args[6]));
					//t.rotation = uquat::create(t.angles);
				}
			}
		}
	}
}

void SMDModel::ReadTriangleBlock(pragma::fs::VFilePtr &f)
{
	std::string l;
	auto vertId = -1;
	Mesh *mesh = nullptr;
	Triangle *tri = nullptr;
	auto bInvalid = false;
	while(ReadBlockLine(f, l) == true) {
		if(vertId == -1) {
			bInvalid = false;
			auto tex = l;
			auto pos = tex.find_last_of('.');
			if(pos != pragma::string::NOT_FOUND)
				tex = tex.substr(0, pos);
			auto it = std::find_if(m_meshes.begin(), m_meshes.end(), [tex](Mesh &mesh) { return (mesh.texture == tex) ? true : false; });
			if(it == m_meshes.end()) {
				m_meshes.push_back(Mesh(tex));
				it = m_meshes.end() - 1;
			}
			mesh = &(*it);
			vertId = 0;
			mesh->triangles.push_back(Triangle());
			tri = &mesh->triangles.back();
		}
		else {
			std::vector<std::string> args;
			pragma::string::explode_whitespace(l, args);
			if(args.size() >= 9) {
				auto &v = tri->vertices[vertId];
				v.bone = pragma::string::to_int(args[0]);
				v.position = Vector3(pragma::string::to_float(args[1]), pragma::string::to_float(args[2]), pragma::string::to_float(args[3]));
				auto l = uvec::length(v.position);
				if(l > MAX_VERT_LENGTH)
					bInvalid = true;
				v.normal = Vector3(pragma::string::to_float(args[4]), pragma::string::to_float(args[5]), pragma::string::to_float(args[6]));
				v.uv = Vector2(pragma::string::to_float(args[7]), pragma::string::to_float(args[8]));
				if(args.size() >= 10) {
					auto numLinks = pragma::string::to_int(args[9]);
					auto sum = 0.f;
					for(auto i = 0; i < numLinks; i++) {
						auto boneId = pragma::string::to_int(args[10 + i * 2]);
						auto weight = pragma::string::to_float(args[11 + i * 2]);
						v.weights.insert(std::unordered_map<int, float>::value_type(boneId, weight));
						sum += weight;
					}
					if(sum < 0.999f)
						v.weights.insert(std::unordered_map<int, float>::value_type(v.bone, 1.f - sum));
				}
			}
			vertId++;
			if(vertId == 3) {
				vertId = -1;
				if(bInvalid == true)
					mesh->triangles.erase(mesh->triangles.end() - 1);
			}
		}
	}
}

Bool SMDModel::ReadLine(pragma::fs::VFilePtr &f, std::string &l, Bool skipEmptyLines)
{
	if(f->Eof())
		return false;
	l = f->ReadLine();
	pragma::string::remove_whitespace(l);
	if((skipEmptyLines == true && l.empty()) || (!l.empty() && (l.front() == '#' || l.front() == ';')))
		return ReadLine(f, l);
	return true;
}

Bool SMDModel::ReadBlockLine(pragma::fs::VFilePtr &f, std::string &l)
{
	auto r = ReadLine(f, l, false);
	auto ll = l;
	pragma::string::to_lower(ll);
	if(r == false || ll == "end")
		return false;
	return true;
}

std::unique_ptr<SMDModel> SMDModel::Load(pragma::fs::VFilePtr &f)
{
	auto mdl = std::unique_ptr<SMDModel>(new SMDModel);
	std::string l;
	while(ReadLine(f, l) == true) {
		pragma::string::to_lower(l);
		if(l == "nodes")
			mdl->ReadNodeBlock(f);
		else if(l == "skeleton")
			mdl->ReadSkeletonBlock(f);
		else if(l == "triangles")
			mdl->ReadTriangleBlock(f);
	}
	mdl->BuildSkeleton();
	mdl->ConvertCoordinateSystem();
	return mdl;
}

std::unique_ptr<SMDModel> SMDModel::Load(const std::string &file)
{
	auto f = pragma::fs::open_file(file.c_str(), pragma::fs::FileMode::Read);
	if(f == nullptr)
		f = pragma::fs::open_system_file(file.c_str(), pragma::fs::FileMode::Read);
	if(f == nullptr)
		return nullptr;
	return Load(f);
}

std::vector<SMDModel::Node> &SMDModel::GetNodes() { return m_nodes; }
std::vector<SMDModel::Frame> &SMDModel::GetFrames() { return m_frames; }
std::vector<SMDModel::Mesh> &SMDModel::GetMeshes() { return m_meshes; }
SMDModel::Skeleton &SMDModel::GetSkeleton() { return m_skeleton; }

void SMDModel::BuildSkeleton()
{
	auto &nodes = GetNodes();
	auto &skeleton = GetSkeleton();
	for(UInt i = 0; i < nodes.size(); ++i) {
		auto &node = nodes[i];
		if(node.parent == -1)
			skeleton.rootNodes.push_back(i);
		else
			nodes[node.parent].children.push_back(i);
	}
}
