// Copyright (C) 2026 Livelandr
// This file is part of NDIM-SAT.
// NDIM-SAT is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation...

#pragma once

#include "base.hpp"

namespace NDSAT {
	namespace Collision {

	struct D2Point {
		double x;
		double y;

		D2Point() {
			x = 0;
			y = 0;
		}

		D2Point(double _x, double _y) {
			x = _x;
			y = _y;
		}
	};

	double linearExtrapolation(double a, double b, double d) {
		return a + (b - a) * d;
	}

	inline double linear(D2Point a, D2Point b, double x) {
		return a.y + (b.y - a.y) / (b.x - a.x) * (x - a.x);
	}

	inline double findIntersectionX(D2Point a, D2Point b, D2Point c, D2Point d) {
		return -(linear(a, b, 0) - linear(c, d, 0)) / ((b.y - a.y) / (b.x - a.x) - (d.y - c.y) / (d.x - c.x));
	}

	bool check2DPointOnEdge(D2Point a, D2Point b, D2Point p) {

		if (a.x == p.x && a.y == p.y) return true;
		if (b.x == p.x && b.y == p.y) return true;

		if (p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y)) {
			return (linear(a, b, a.x) == linear(a, p, a.x));
		}


		return false;
	}

	bool check2DCollision(D2Point a, D2Point b, D2Point c, D2Point d) {
		double interX = findIntersectionX(a, b, c, d);

		if (interX != interX) {
			if (check2DPointOnEdge(a, b, c) &&
				check2DPointOnEdge(a, b, d) &&
				check2DPointOnEdge(c, d, a) &&
				check2DPointOnEdge(c, d, b)
				) {
				return 1;
			}
		}

		if (a.x == b.x) {
			interX = a.x;

			double cdVal = linear(c, d, interX);
			if (cdVal < std::min(a.y, b.y) || cdVal > std::max(a.y, b.y)) return false;
		}
		if (c.x == d.x) {
			interX = c.x;
			double abVal = linear(a, b, interX);
			if (abVal < std::min(c.y, d.y) || abVal > std::max(c.y, d.y)) return false;
		}

		bool abMin = std::min(a.x, b.x) <= interX;
		bool abMax = std::max(a.x, b.x) >= interX;
		bool cdMin = std::min(c.x, d.x) <= interX;
		bool cdMax = std::max(c.x, d.x) >= interX;

		return (abMin && abMax && cdMin && cdMax);
	}

	bool checkTrigonEdgeIntersection(D2Point a, D2Point b, D2Point c, D2Point x, D2Point y) {
		if (check2DCollision(x, y, a, b)) return true;
		if (check2DCollision(x, y, b, c)) return true;
		if (check2DCollision(x, y, c, a)) return true;

		double barA = ((b.y - c.y) * (x.x - c.x) + (c.x - b.x) * (x.y - c.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
		double barB = ((c.y - a.y) * (x.x - c.x) + (a.x - c.x) * (x.y - c.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
		double barC = 1 - barA - barB;

		if ((0 <= barA) && (barA <= 1) && (0 <= barB) && (barB <= 1) && (0 <= barC) && (barC <= 1)) return true;

		barA = ((b.y - c.y) * (y.x - c.x) + (c.x - b.x) * (y.y - c.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
		barB = ((c.y - a.y) * (y.x - c.x) + (a.x - c.x) * (y.y - c.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
		barC = 1 - barA - barB;

		if ((0 <= barA) && (barA <= 1) && (0 <= barB) && (barB <= 1) && (0 <= barC) && (barC <= 1)) return true;

		return false;
	}

	Point findPlaneIntersection(Point A, Point B, Point C, Point D, Point n, Point m, size_t dimensions = 3) {
		return Point(0, 0, 0);
	}

	size_t countProjectionEdgesIntersections(Point a, Point b, Point c, Point d, size_t dimensions = 3) {
		D2Point pA, pB, pC, pD;
		size_t projections = 0;

		for (size_t i = 0; i < dimensions; i++) {
			for (size_t j = i + 1; j < dimensions; j++) {
				pA.x = a.dim[i];
				pA.y = a.dim[j];

				pB.x = b.dim[i];
				pB.y = b.dim[j];

				pC.x = c.dim[i];
				pC.y = c.dim[j];

				pD.x = d.dim[i];
				pD.y = d.dim[j];

				projections += check2DCollision(pA, pB, pC, pD);
			}
		}

		return projections;
	}

	size_t countProjectionTrigonEdgeIntersections(Point a, Point b, Point c, Point n, Point m, size_t dimensions = 3) {
		D2Point pA, pB, pC, pN, pM;
		size_t projections = 0;

		for (size_t i = 0; i < dimensions; i++) {
			for (size_t j = i + 1; j < dimensions; j++) {
				pA.x = a.dim[i];
				pA.y = a.dim[j];

				pB.x = b.dim[i];
				pB.y = b.dim[j];

				pC.x = c.dim[i];
				pC.y = c.dim[j];

				pN.x = n.dim[i];
				pN.y = n.dim[j];

				pM.x = m.dim[i];
				pM.y = m.dim[j];

				projections += checkTrigonEdgeIntersection(pA, pB, pC, pN, pM);
			}
		}
		return projections;
	}

	bool doEdgesIntersect(Point a, Point b, Point c, Point d, size_t dimensions = 3) {
		return countProjectionEdgesIntersections(a, b, c, d, dimensions) == (dimensions * (dimensions - 1) / 2);
	}

	bool doEdgeTrigonIntersect(Point a, Point b, Point c, Point n, Point m, size_t dimensions = 3) {
		return countProjectionTrigonEdgeIntersections(a, b, c, n, m, dimensions) == (dimensions * (dimensions - 1) / 2);
	}

	bool doEdgePlainIntersect(Point A, Point B, Point C, Point D, Point n, Point m, size_t dimensions = 3) {
		bool Half1 = doEdgeTrigonIntersect(A, B, C, n, m, dimensions);
		bool Half2 = doEdgeTrigonIntersect(A, D, C, n, m, dimensions);

		return Half1 || Half2;
	}

	/*
	int onWhichSideWhenIntersecting(Point A, Point B, Point C, Point D, Point n, Point m, double d, size_t dimensions = 3) {
		Point extrapolatedPoint;
		for (size_t i = 0; i < dimensions; i++) {
			// n -> m
			extrapolatedPoint.dim.push_back( linearExtrapolation(n.dim[i], m.dim[i], d) );
		}

		std::cout << "INTERSECTING: " << doEdgePlainIntersect(A, B, C, D, n, m, dimensions) << std::endl;

		for (size_t dim = 0; dim < dimensions; dim++) {
			for (size_t dim1 = dim+1; dim1 < dimensions; dim1++) {
				D2Point pN = D2Point(n.dim[dim], n.dim[dim1]);
				D2Point pM = D2Point(m.dim[dim], m.dim[dim1]);

				D2Point pA = D2Point(A.dim[dim], A.dim[dim1]);
				D2Point pB = D2Point(B.dim[dim], B.dim[dim1]);
				D2Point pC = D2Point(C.dim[dim], C.dim[dim1]);
				D2Point pD = D2Point(D.dim[dim], D.dim[dim1]);

			}
		}

		for (size_t i = 0; i < extrapolatedPoint.dim.size(); i++) {
			std::cout << i << ": " << extrapolatedPoint.dim[i] << std::endl;
		}

		return 0;
	}
	*/
}
}
