#include "world2d.h"
#include "Helper2d.h"

namespace lib2d
{
	void world2d::step(Helper2d * helper)
	{
		helper->paint_line(helper->get_rect().topLeft().x(), helper->get_rect().topLeft().y(), helper->get_rect().bottomRight().x(), helper->get_rect().bottomRight().y()); //≤‚ ‘
	}
}