#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "base.hpp"
#include "collision.hpp"


namespace NDSAT {
	namespace Models {
		enum OBJMode
		{
			null,
			objname,
			vertex,
			edge,
			face
		};

		struct Face {
			std::vector<size_t> vertex;

			Face() {};

			template<typename... Args>
			Face(Args... args) : vertex{ static_cast<size_t>(args)... } {}
		};

		struct Edge {
			size_t a;
			size_t b;
		};

		struct PointWE {
			Point point;
			std::vector<size_t> edges;
		};

		// BIGGEST
		struct Model
		{
			std::string name = "obj";
			std::vector<PointWE> verticies;
			std::vector<Edge> edges;
			std::vector<Face> faces;

			void reset() {
				verticies.clear();
				edges.clear();
			}

			void addEdge(size_t a, size_t b) {
				size_t edgeCount = edges.size();

				Edge newEdge;
				newEdge.a = a;
				newEdge.b = b;

				verticies[newEdge.a].edges.push_back(edgeCount);
				verticies[newEdge.b].edges.push_back(edgeCount);

				edges.push_back(newEdge);
			}

			bool edgeExists(size_t a, size_t b) {
				if (verticies[a].edges.size() < 0) return false;
				if (verticies[b].edges.size() < 0) return false;

				for (size_t i = 0; i < verticies[a].edges.size(); i++) {
					if (edges[verticies[a].edges[i]].a == a && edges[verticies[a].edges[i]].b == b) return true;
					if (edges[verticies[a].edges[i]].a == b && edges[verticies[a].edges[i]].b == a) return true;
				}

				return false;
			}

			void dissolveFacesToEdges() {
				for (size_t f = 0; f < faces.size(); f++) {
					for (size_t i = 0; i < faces[f].vertex.size()-1; i++) {
						if (!edgeExists(faces[f].vertex[i], faces[f].vertex[i+1])) addEdge(faces[f].vertex[i], faces[f].vertex[i + 1]);
						//std::cout << faces[f].vertex[i] << " " << faces[f].vertex[i + 1] << " ";
					}
					//std::cout << std::endl;
				}
				faces.clear();
			}

			void importFromFile(std::string filename) {
				std::ifstream file(filename);

				if (!file.is_open()) return;

				OBJMode curMode = null;

				std::string line;
				while (std::getline(file, line)) {
					std::stringstream ss(line);
					std::string type;
					ss >> type;
					if (type == "v") {
						double x, y, z;
						ss >> x >> y >> z;
						PointWE newPoint;
						newPoint.point = Point(x, y, z);
						verticies.push_back(newPoint);
					}
					else if (type == "o") {
						ss.ignore();
						std::getline(ss, this->name);
					}
					else if (type == "l") {
						size_t a, b;
						ss >> a >> b;
						addEdge(a-1, b-1);
					}
					else if (type == "f") {
						Face newFace;

						size_t v = 0;
						while (ss >> v) {
							//std::cout << v << std::endl;
							newFace.vertex.push_back(v-1);
						} 
						faces.push_back(newFace);
					}
				}

				file.close();
			}

			void exportToOBJ(std::string filename) {
				std::ofstream file(filename);

				file << "o" << " " << this->name << std::endl;

				for (auto it : verticies) {
					file << "v" << " " << it.point.dim[0] << " " << it.point.dim[1] << " " << it.point.dim[2] << std::endl;
				}

				for (auto it : edges) {
					file << "l" << " " << it.a + 1 << " " << it.b + 1 << std::endl;
				}


				file.close();
			}

			void registerEdgesInVerticies() {
				// clean all up
				for (size_t i = 0; i < verticies.size(); i++) {
					verticies[i].edges.clear();
				}

				for (size_t i = 0; i < edges.size(); i++) {
					verticies[edges[i].a].edges.push_back(i);
					verticies[edges[i].b].edges.push_back(i);
				}
			}

			// Topology stuff
			size_t addVertex(Point vert) {
				PointWE newVert;
				newVert.point = vert;

				verticies.push_back(newVert);

				return verticies.size() - 1;
			}

			// Different stuff
			void setMinDimensionsTo(long dim) {
				for (size_t i = 0; i < verticies.size(); i++) {
					for (int j = 0; j < (dim - verticies[i].point.dim.size()); j++) {
						verticies[i].point.dim.push_back(0);
					}
				}
			}

			void moveObject(std::vector<double> vec) {
				for (size_t i = 0; i < verticies.size(); i++) {
					for (size_t j = 0; j < vec.size(); j++) {
						verticies[i].point.dim[j] += vec[j];
					}
				}
			}

			void scaleObject(std::vector<double> vec) {
				for (size_t i = 0; i < verticies.size(); i++) {
					for (size_t j = 0; j < vec.size(); j++) {
						verticies[i].point.dim[j] *= vec[j];
					}
				}
			}

			// Perpective Projection
			void perspectiveProjectionDecrement(double D = 6, size_t dimensions = 4) {
				for (size_t i = 0; i < verticies.size(); i++) {
					double scale = 1 / (D - verticies[i].point.dim[dimensions-1]);

					for (size_t d = 0; d < dimensions - 1; d++) {
						verticies[i].point.dim[d] = verticies[i].point.dim[d] * scale;
					}

					verticies[i].point.dim[dimensions-1] = 0;
				}
			}

			// now this is fun.
			void rotateObject(size_t dim1, size_t dim2, double rad) {
				double cos = std::cos(rad);
				double sin = std::sin(rad);

				register double old1, old2;
				for (size_t i = 0; i < this->verticies.size(); i++) {
					old1 = verticies[i].point.dim[dim1];
					old2 = verticies[i].point.dim[dim2];

					verticies[i].point.dim[dim1] = cos * old1 - sin * old2;
					verticies[i].point.dim[dim2] = sin * old1 + cos * old2;
				}
			}

			// Much more fun
			void dimensionCollapse(double dimVal, size_t targetDimension = 2, size_t totalDimensions = 3) {
				std::vector<PointWE> verticiesCopy = verticies;
				std::cout << verticies.size() << std::endl;


				// Dimension
				for (size_t edge = 0; edge < edges.size(); edge++) {
					for (size_t dim = 0; dim < totalDimensions; dim++) {
						if (dim == targetDimension) continue;
						Point A = verticies[edges[edge].a].point;
						Point B = verticies[edges[edge].b].point;


							Collision::D2Point pA = Collision::D2Point(A.dim[dim], A.dim[targetDimension]);
							Collision::D2Point pB = Collision::D2Point(B.dim[dim], B.dim[targetDimension]);

							double delta = (dimVal - pA.y) / (pB.y - pA.y);
							
							double interpolatedCoord = Collision::linearExtrapolation(A.dim[dim], B.dim[dim], delta);

							if (!(interpolatedCoord != interpolatedCoord)) {
								verticiesCopy[edges[edge].a].point.dim[dim] = interpolatedCoord;
								verticiesCopy[edges[edge].b].point.dim[dim] = interpolatedCoord;
							}
					}
				}

				for (size_t i = 0; i < verticiesCopy.size(); i++) {
					verticiesCopy[i].point.dim[targetDimension] = 0;
				}

				this->verticies = verticiesCopy;
			}

			template <typename Func>
			void dimensionCollapseDynamic(Func getDimensionCoordinate, size_t targetDimension = 2, size_t totalDimensions = 3) {
				std::vector<PointWE> verticiesCopy = verticies;

				// Dimension XY
				for (size_t edge = 0; edge < edges.size(); edge++) {
					for (size_t dim = 0; dim < totalDimensions; dim++) {
						if (dim == targetDimension) continue;
						Point A = verticies[edges[edge].a].point;
						Point B = verticies[edges[edge].b].point;

						Collision::D2Point pA = Collision::D2Point(A.dim[dim], A.dim[targetDimension]);
						Collision::D2Point pB = Collision::D2Point(B.dim[dim], B.dim[targetDimension]);

						double delta = (getDimensionCoordinate(A,B) - pA.y) / (pB.y - pA.y);

						double interpolatedCoord = Collision::linearExtrapolation(A.dim[dim], B.dim[dim], delta);

						if (interpolatedCoord != interpolatedCoord) interpolatedCoord = 0;

						//std::cout << interpolatedCoord << std::endl;

						verticiesCopy[edges[edge].a].point.dim[dim] = interpolatedCoord;
						verticiesCopy[edges[edge].b].point.dim[dim] = interpolatedCoord;
					}
				}

				for (size_t i = 0; i < verticiesCopy.size(); i++) {
					verticiesCopy[i].point.dim[targetDimension] = 0;
				}

				this->verticies = verticiesCopy;
			}

		};

	}
}