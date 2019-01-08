#ifndef GRAPHICS2D_WORLD2D_H
#define GRAPHICS2D_WORLD2D_H

#include "stdafx.h"

class Helper2d;

namespace lib2d 
{
	class world2d
	{
	public:
		world2d() = default;
		~world2d() = default;

		void step(Helper2d * helper);
	};
}

#endif
