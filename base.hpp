// Copyright (C) 2026 Livelandr
// This file is part of NDIM-SAT.
// NDIM-SAT is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation...

#pragma once

namespace NDSAT {
	struct Point {
		std::vector<double> dim;

		Point() {};

		template<typename... Args>
		Point(Args... args) : dim{ static_cast<double>(args)... } {}
	};

	struct DimVector {
		std::vector<double> dim;

		DimVector() {};

		template<typename... Args>
		DimVector(Args... args) : dim{ static_cast<double>(args)... } {}
	};
}
