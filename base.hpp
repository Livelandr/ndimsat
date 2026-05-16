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