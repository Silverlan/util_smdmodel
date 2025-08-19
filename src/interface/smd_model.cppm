// SPDX-FileCopyrightText: (c) 2024 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <string>
#include <memory>
#include <unordered_map>
#include <mathutil/umath.h>
#include <mathutil/uvec.h>
#include <mathutil/uquat.h>
#include <fsys/vfileptr.h>
#include "smd_definitions.hpp"

export module se_smd.model;

export namespace source_engine::smd {
	class DLLSMDMDL SMDModel {
	  public:
		struct DLLSMDMDL Node {
			Node();
			std::string name;
			int id;
			int parent;
			std::vector<UInt> children;
		};
		struct DLLSMDMDL FrameTransform {
			FrameTransform();
			Vector3 position;
			EulerAngles angles;
			Quat rotation;
		};
		struct DLLSMDMDL Frame {
			Frame();
			std::vector<FrameTransform> transforms;
		};
		struct DLLSMDMDL Skeleton {
			Skeleton();
			int startTime;
			std::vector<UInt> rootNodes;
		};
		struct DLLSMDMDL Vertex {
			Vertex();
			int bone;
			std::unordered_map<int, float> weights;
			Vector3 position;
			Vector3 normal;
			Vector2 uv;
		};
		struct DLLSMDMDL Triangle {
			Triangle();
			Vertex vertices[3];
		};
		struct DLLSMDMDL Mesh {
			Mesh();
			Mesh(const std::string &tex);
			std::string texture;
			std::vector<Triangle> triangles;
		};
	  private:
		SMDModel();
		std::vector<Node> m_nodes;
		std::vector<Frame> m_frames;
		std::vector<Mesh> m_meshes;
		Skeleton m_skeleton;
		void ReadNodeBlock(VFilePtr &f);
		void ReadSkeletonBlock(VFilePtr &f);
		void ReadTriangleBlock(VFilePtr &f);
		void BuildNodeMatrix(Frame &frame, UInt id, std::vector<Mat4> &matrices);
		void BuildSkeleton();
		void ConvertCoordinateSystem();
		static Bool ReadLine(VFilePtr &f, std::string &l, Bool skipEmptyLines = true);
		static Bool ReadBlockLine(VFilePtr &f, std::string &l);
	  public:
		~SMDModel();
		static std::unique_ptr<SMDModel> Load(const std::string &file);
		static std::unique_ptr<SMDModel> Load(VFilePtr &file);
		std::vector<Node> &GetNodes();
		std::vector<Frame> &GetFrames();
		std::vector<Mesh> &GetMeshes();
		Skeleton &GetSkeleton();
	};
};
