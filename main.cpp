// Copyright (C) 2026 Livelandr
// This file is part of NDIM-SAT.
// NDIM-SAT is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation...

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "models.hpp"
#include "collision.hpp"


void spawn4DHyperSphere(NDSAT::Models::Model& mdl, double radiusOuter, double radiusInner, double wOffset, int rings, int sectors) {
    int zeroVertex = static_cast<int>(mdl.verticies.size());

    double pi = std::acos(-1.0);
    int sphereSize = (rings + 1) * (sectors + 1);

    for (int r = 0; r <= rings; ++r) {
        double phi = pi * double(r) / double(rings);
        double y = radiusOuter * std::cos(phi);
        double sinPhi = std::sin(phi);

        for (int s = 0; s <= sectors; ++s) {
            double theta = 2.0 * pi * double(s) / double(sectors);
            double x = radiusOuter * sinPhi * std::cos(theta);
            double z = radiusOuter * sinPhi * std::sin(theta);
            mdl.addVertex(NDSAT::Point(x, y, z, 0.0));
        }
    }

    for (int r = 0; r <= rings; ++r) {
        double phi = pi * double(r) / double(rings);
        double y = radiusInner * std::cos(phi);
        double sinPhi = std::sin(phi);

        for (int s = 0; s <= sectors; ++s) {
            double theta = 2.0 * pi * double(s) / double(sectors);
            double x = radiusInner * sinPhi * std::cos(theta);
            double z = radiusInner * sinPhi * std::sin(theta);
            mdl.addVertex(NDSAT::Point(x, y, z, wOffset));
        }
    }

    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {

            int p0_out = zeroVertex + r * (sectors + 1) + s;
            int p1_out = p0_out + 1;
            int p2_out = zeroVertex + (r + 1) * (sectors + 1) + s;
            int p3_out = p2_out + 1;

            int p0_in = p0_out + sphereSize;
            int p1_in = p1_out + sphereSize;
            int p2_in = p2_out + sphereSize;
            int p3_in = p3_out + sphereSize;

            mdl.addEdge(p0_out, p1_out);
            mdl.addEdge(p0_out, p2_out);

            mdl.addEdge(p0_in, p1_in);
            mdl.addEdge(p0_in, p2_in);
        }
    }

    for (int i = 0; i < sphereSize; ++i) {
        mdl.addEdge(zeroVertex + i, zeroVertex + i + sphereSize);
    }
}

int main()
{
    // Radians
    double rotation = 3.14159;

    NDSAT::Models::Model mdl;

    spawn4DHyperSphere(mdl, 6, 3, 5, 16, 16);

    mdl.rotateObject(2, 3, rotation);

    mdl.perspectiveProjectionDecrement(6, 4);

    std::string name = "hypersphere" + std::to_string(i) + ".obj";
    mdl.exportToOBJ(name);
    
	return 0;
}
