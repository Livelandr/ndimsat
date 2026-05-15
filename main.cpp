#include <iostream>
#include <fstream>
#include <vector>

#include "ndimintersections.hpp"

int main()
{

	// 3D intersection
	{
		NDSAT::Point a(0, 0, 0), b(10, 0, 0), c(5, 10, 0);
		NDSAT::Point n(5, 5, -5), m(5, 5, 5);

		std::cout << "3D PLANE INTERSECTION 1: " << std::boolalpha << NDSAT::doEdgeTrigonIntersect(a, b, c, n, m) << " ";
		std::cout << NDSAT::countProjectionTrigonEdgeIntersections(a, b, c, n, m) << "/" << (3) << std::endl;
	}

	// 3D non intersection
	{
		NDSAT::Point a(0, 0, 0), b(10, 0, 0), c(5, 10, 0);
		NDSAT::Point n(5, 5, 10), m(6, 6, 15);

		std::cout << "3D PLANE NON-INTERSECTION: " << std::boolalpha << NDSAT::doEdgeTrigonIntersect(a, b, c, n, m) << " ";
		std::cout << NDSAT::countProjectionTrigonEdgeIntersections(a, b, c, n, m) << "/" << (3) << std::endl;
	}

	// 4D intersection
	{
		NDSAT::Point a(0, 0, 0, 5), b(10, 0, 0, 5), c(5, 10, 0, 5);
		NDSAT::Point n(5, 5, -5, 5), m(5, 5, 5, 5);

		std::cout << "4D PLANE INTERSECTION: " << std::boolalpha << NDSAT::doEdgeTrigonIntersect(a, b, c, n, m, 4) << " ";
		std::cout << NDSAT::countProjectionTrigonEdgeIntersections(a, b, c, n, m, 4) << "/" << (4 * 3 / 2) << std::endl;
	}

	// 4D non intersection, intersects in 3D, but in wrong W
	{
		NDSAT::Point a(0, 0, 0, 0), b(10, 0, 0, 0), c(5, 10, 0, 0);
		NDSAT::Point n(5, 5, -5, 10), m(5, 5, 5, 10);

		std::cout << "4D PLANE NON-INTERSECTION: " << std::boolalpha << NDSAT::doEdgeTrigonIntersect(a, b, c, n, m, 4) << " ";
		std::cout << NDSAT::countProjectionTrigonEdgeIntersections(a, b, c, n, m, 4) << "/" << (4 * 3 / 2) << std::endl;
	}

	return 0;
}
