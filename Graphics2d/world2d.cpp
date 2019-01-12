#include "world2d.h"
#include "Helper2d.h"

namespace lib2d
{
	v2::v2(double _x, double _y) : x(_x), y(_y) {}

	v2 v2::operator* (double d) const
	{
		return v2(x * d, y * d);
	}

	v2 v2::operator/ (double d) const
	{
		return v2(x / d, y / d);
	}

	v2 v2::operator+ (double d) const
	{
		return v2(x + d, y + d);
	}

	v2 v2::operator- (double d) const
	{
		return v2(x - d, y - d);
	}

	v2 v2::operator+ (const v2 &v) const
	{
		return v2(x + v.x, y + v.y);
	}

	v2 v2::operator- (const v2 &v) const
	{
		return v2(x - v.x, y - v.y);
	}

	v2 &v2::operator+= (const v2 &v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}

	v2 &v2::operator-= (const v2 &v)
	{
		x -= v.x;
		y -= v.y;
		return *this;
	}

	v2 v2::operator- () const
	{
		return{ -x, -y };
	}

	v2 operator* (double d, const v2 &v)
	{
		return { d * v.x, d * v.y };
	}

	double v2::cross(const v2 &v) const
	{
		return x * v.y - y * v.x;
	}

	double v2::dot(const v2 &v) const
	{
		return x * v.x + y * v.y;
	}

	double v2::magnitude() const
	{
		return std::sqrt(x * x + y * y);
	}

	double v2::magnitude_square() const
	{
		return x * x + y * y;
	}

	v2 v2::normalize() const
	{
		return *this / magnitude();
	}

	v2 v2::normal() const
	{
		return N().normalize();
	}

	v2 v2::N() const
	{
		return v2{ y, -x };
	}

	bool v2::zero(double d) const
	{
		return std::abs(x) < d && std::abs(y) < d;
	}

	v2 v2::rotate(double theta) const
	{
		const auto _sin = std::sin(theta);
		const auto _cos = std::cos(theta);
		return{ _cos * x - _sin * y, _sin * x + _cos * y };
	}

	//----------------------------------------------

	m2::m2(double _x1, double _y1, double _x2, double _y2) : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {}

	m2::m2(double d) : x1(d), y1(0), x2(0), y2(d) {}

	m2 m2::operator+ (const m2 &m) const
	{
		return{ x1 + m.x1, y1 + m.y1 , x2 + m.x2 , y2 + m.y2 };
	}

	m2 m2::operator* (double d) const
	{
		return{ x1 * d, y1 * d, x2 * d, y2 * d };
	}

	v2 m2::operator* (const v2 &v) const
	{
		return{ x1 * v.x + y1 * v.y , x2 * v.x + y2 * v.y };
	}

	m2 operator* (double d, const m2 &m)
	{
		return m*d;
	}

	const m2 m2::rotate(double theta)
	{
		const auto _sin = std::sin(theta);
		const auto _cos = std::cos(theta);
		*this = m2{ _cos, -_sin, _sin, _cos };  //构造临时对象通过 拷贝赋值运算符 给本结构体赋值
		return *this;
	}

	v2 m2::rotate(const v2 &v)const
	{
		//|x1,y1| * |v.x| = |x1 * v.x + y1 * v.y|
		//|x2,y2|   |v.y|   |x2 * v.x + y2 * v.y|
		return v2{ x1 * v.x + y1 * v.y, x2 * v.x + y2 * v.y };
	}

	double m2::det() const
	{
		return x1 * y2 - x2 * y1;
	}

	m2 m2::inv() const
	{
		auto _det = det();
		return _det == 0 ? m2(inf, inf, inf, inf) : (m2(y2, -x2, -y1, x1) * (1 / _det));
	}

	//---------------------------------------------------

	//void world2d::step(Helper2d * helper)
	//{
	//	//helper->paint_line(helper->get_rect().topLeft().x(), helper->get_rect().topLeft().y(), helper->get_rect().bottomRight().x(), helper->get_rect().bottomRight().y()); //测试
	//	
	//	auto now = QTime::currentTime();

	//}
}