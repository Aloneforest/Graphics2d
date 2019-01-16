#include "stdafx.h"
#include "world2d.h"
#include "Helper2d.h"

namespace lib2d
{
	//-------------------v2------------------------------------

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
		return{ d * v.x, d * v.y };
	}

	double v2::cross(const v2 &v) const     //叉乘
	{
		return x * v.y - y * v.x;
	}

	double v2::dot(const v2 &v) const       //点乘
	{
		return x * v.x + y * v.y;
	}

	double v2::magnitude() const
	{
		return std::sqrt(x * x + y * y);
	}

	double v2::magnitudeSquare() const
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

	//----------------m2------------------------------

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

	//------------polygon2d--------------------------------------

	polygon2d::polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices) :body2d(_id, _mass, _pos), vertices(_vertices), verticesWorld(_vertices) 
	{
		init();
	}

    double polygon2d::calcPolygonArea()
    {
        double area = 0;
        auto size = vertices.size();
        for (size_t i = 0; i < size; ++i)
        {
            auto j = (i + 1) % size;
            area += vertices[i].cross(vertices[j]);     //叉乘计算面积
        }
        return area / 2;
    }

    v2 polygon2d::calcPolygonCentroid()
    {
        v2 gc;
        auto size = vertices.size();
        for (size_t i = 0; i < size; ++i)
        {
            auto j = (i + 1) % size;                                            //多边形重心 = （各三角形重心 * 其面积） / 总面积
            gc += (vertices[i] + vertices[j]) * vertices[i].cross(vertices[j]); //三角形重心 = 两向量之和 / 3
        }
        return gc / 6.0 / calcPolygonArea();
    }

    double polygon2d::calcPolygonInertia()
    {
        double acc0 = 0;
        double acc1 = 0;
        auto size = vertices.size();
        for (size_t i = 0; i < size; ++i)       //转动惯量 = m / 6 * Σ((Pn+1 x Pn)(Pn+1·Pn+1 + Pn+1·Pn + Pn·Pn))/（Σ(Pn+1 x Pn)）
        {
            auto a = vertices[i], b = vertices[(i + 1) % size];
            auto _cross = std::abs(a.cross(b));
            acc0 += _cross * (a.dot(a) + b.dot(b) + a.dot(b));
            acc1 += _cross;
        }
        return mass * acc0 / 6 / acc1;
    }

    void polygon2d::calcPolygonBounds()
    {
        boundMin = boundMax = verticesWorld[0];
        for (size_t i = 1; i < verticesWorld.size(); ++i)
        {
            boundMin.x = std::min(boundMin.x, verticesWorld[i].x);
            boundMin.y = std::min(boundMin.y, verticesWorld[i].y);
            boundMax.x = std::max(boundMax.x, verticesWorld[i].x);
            boundMax.y = std::max(boundMax.y, verticesWorld[i].y);
        }
    }

    bool polygon2d::containsInBound(const v2 & pt)
    {
        return boundMin.x < pt.x &&
            boundMax.x > pt.x &&
            boundMin.y < pt.y &&
            boundMax.y > pt.y;
    }

    bool polygon2d::containsInPolygon(const v2 & pt)
    {
        const auto size = verticesWorld.size();
        if (size < 3) return false;
        if ((pt - verticesWorld[0]).cross(verticesWorld[1] - verticesWorld[0]) > 0)
            return false;
        if ((pt - verticesWorld[0]).cross(verticesWorld[size - 1] - verticesWorld[0]) < 0)
            return false;

        int i = 2, j = size - 1;
        int line = -1;

        while (i <= j)
        {
            int mid = (i + j) >> 1;

            if ((pt - verticesWorld[0]).cross(verticesWorld[mid] - verticesWorld[0]) > 0)
            {
                line = mid;
                j = mid - 1;
            }
            else
            {
                i = mid + 1;
            }
        }
        return (pt - verticesWorld[line - 1]).cross(verticesWorld[line] - verticesWorld[line - 1]) < 0;
    }

    bool polygon2d::contains(const v2 & pt)
    { 
        auto a = containsInBound(pt);
        auto b = containsInPolygon(pt);
        return containsInBound(pt) && containsInPolygon(pt);    //先判断是否在外包框内，再判断是否在多边形内
    }

    void polygon2d::drag(const v2 & pt, const v2 & offset)
    {
        V += 1.0 / mass * offset;                                       //速度 += 力矩 / 质量
        angleV += 1.0 / inertia * (pt - pos - center).cross(offset);    //角速度 += 转动半径 x 力矩 / 转动惯量
    }

	void polygon2d::init()
    {
        inertia = calcPolygonInertia();
        center = calcPolygonCentroid();

        R.rotate(angle);
        for (size_t i = 0; i < verticesWorld.size(); ++i)
        {
            auto v = R.rotate(vertices[i] - center) + center;
            verticesWorld[i] = pos + v;
        }

        calcPolygonBounds();
	}

    void polygon2d::setStatic()
    {
        isStatic = true;
    }

	void polygon2d::update(const v2 gravity, int n)
	{
        switch (n)
        {
        case 0:
        {
            F = gravity * mass;     //默认受到重力
        }
            break;
        case 1:
        {
            V += F / mass * world2d::dt;     //速度增量 = 加速度 * 时间间隔
        }
            break;
        case 2:
        {
            pos += V * world2d::dt;
            angle += angleV * world2d::dt;
            R.rotate(angle);
            for (size_t i = 0; i < vertices.size(); i++)
            {
                auto v = R.rotate(vertices[i] - center) + center;
                verticesWorld[i] = pos + v;
            }
        }
            break;
        default:
            break;
        }
	}

	void polygon2d::draw(Helper2d * helper)
	{
		helper->paintPolygon(verticesWorld);
	}

	//-------------world2d--------------------------------------

	QTime world2d::last_clock = QTime::currentTime();
	double world2d::dt = 1.0 / 30;
	double world2d::dt_inv = 30;

	polygon2d * world2d::makePolygon(const double mass, const std::vector<v2> &vertices, const v2 &pos, const bool statics = false)
	{
		auto polygon = std::make_unique<polygon2d>(global_id++, mass, pos, vertices);
		auto obj = polygon.get();
        if (statics)
        {
            polygon->setStatic();
            static_bodies.push_back(std::move(polygon));
        }
        else
        {
            bodies.push_back(std::move(polygon));
        }
		return obj;
	}

	polygon2d * world2d::makeRect(const double mass, double w, double h, const v2 &pos, bool statics = false)
	{
		w = std::abs(w);
		h = std::abs(h);
		std::vector<v2> vertices =
		{
			{ w / 2, h / 2 },
			{ -w / 2, h / 2 },
			{ -w / 2, -h / 2 },
			{ w / 2, -h / 2 },
		};
		return makePolygon(mass, vertices, pos, statics);
	}

	void world2d::step(Helper2d * helper)
	{
		auto now = QTime::currentTime();
		dt = last_clock.msecsTo(now)*0.001;	//计算距离时间t的毫秒数，如果t早于当前时间，则为负
		dt = std::min(dt, 1.0 / 30);
		dt_inv = 1.0 / dt;
		last_clock = now;

		helper->clear();

		for (auto &body : bodies)
		{
			body->update(gravity, 2);
		}

		for (auto &body : bodies)
		{
			body->draw(helper);
		}

        if (true == mouse_drag)
        {
            auto dist = global_drag + global_drag_offset;
            helper->paintLine(global_drag, dist);
        }
	}

	void world2d::clear()
	{
		bodies.clear();
	}

	void world2d::init()
	{
		clear();
		std::vector<v2> vertices =
		{
			{ -0.05, 0 },
			{ 0.05, 0 },
			{ 0, 0.05 }
		};
		auto p1 = makePolygon(2, vertices, { -0.05, -0.09 });
        //p1->V = v2(0.2, 0);
        //p1->angleV = 0.8;
        auto p2 = makePolygon(2, vertices, { 0.05, -0.09 });
        //p2->V = v2(-0.2, 0);
        //p2->angleV = -0.8;
	}

    body2d * world2d::findBody(const v2 & pos)               //查找点所在图形
    {
        auto body = std::find_if(bodies.begin(), bodies.end(), [&](auto & b){return b->contains(pos);});
        if (body != bodies.end())
        {
            return (*body).get();
        }
        return nullptr;
    }

    void world2d::offset(const v2 & pt, const v2 & offset)   //计算点所在图形受力变换
    {
        auto body = findBody(pt);
        if (body)
        {
            body->drag(pt, offset);
        }
    }

    void world2d::mouse(const v2 & pt, bool down)            //鼠标坐标捕获
    {
        if (true == down)
        {
            mouse_drag = true;
            global_drag_offset.x = 0;
            global_drag_offset.y = 0;
            global_drag.x = pt.x;
            global_drag.y = pt.y;
        }
        else
        {
            mouse_drag = false;
            global_drag_offset.x = (pt.x - global_drag.x);
            global_drag_offset.y = (pt.y - global_drag.y);
            offset(global_drag, global_drag_offset);
            global_drag.x = pt.x;
            global_drag.y = pt.y;
        }
    }

    void world2d::motion(const v2 & pt)                      //鼠标移动矢量
    {
        if (true == mouse_drag)
        {
            global_drag_offset.x = (pt.x - global_drag.x);
            global_drag_offset.y = (pt.x - global_drag.y);
        }
    }

	void world2d::world2d::setHelper(Helper2d * helper)
	{
		this->helper = helper;
	}
}