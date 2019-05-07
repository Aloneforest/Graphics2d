#include "stdafx.h"
#include "body2d.h"
#include "Helper2d.h"

namespace lib2d
{
    //------------body2d--------------------------------------

    v2 body2d::rotate(const v2 &v) const
    {
        return m2().rotate(angle).rotate(v);
    }

    v2 body2d::world() const
    {
        return pos + center;
    }

    //------------polygon2d--------------------------------------

    polygon2d::polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices) :body2d(_id, _mass, _pos)
    {
        init(_vertices);
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

    double polygon2d::calcPolygonInertia(double mass)
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
        aabb.boundMin = aabb.boundMax = verticesWorld[0];
        for (size_t i = 1; i < verticesWorld.size(); ++i)
        {
            aabb.boundMin.x = std::min(aabb.boundMin.x, verticesWorld[i].x);
            aabb.boundMin.y = std::min(aabb.boundMin.y, verticesWorld[i].y);
            aabb.boundMax.x = std::max(aabb.boundMax.x, verticesWorld[i].x);
            aabb.boundMax.y = std::max(aabb.boundMax.y, verticesWorld[i].y);
        }
    }

    bool polygon2d::containsInBound(const v2 & pt)
    {
        return aabb.boundMin.x < pt.x &&
            aabb.boundMax.x > pt.x &&
            aabb.boundMin.y < pt.y &&
            aabb.boundMax.y > pt.y;
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
        int line = 2;

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
        V += mass.inv * offset;                                       //速度 += 动量（力） / 质量
        angleV += inertia.inv * (pt - pos - center).cross(offset);    //角速度 += 角动量（力矩（重心到受力点向量 x 力）） / 转动惯量
    }

    void polygon2d::init(const std::vector<v2> &_vertices)
    {
        vertices = _vertices;
        verticesWorld = _vertices;

        inertia.set(calcPolygonInertia(mass.value));
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

    void polygon2d::impulse(const v2 & p, const v2 & r)
    {
        auto _p = p * world2d::dtInv;
        F += _p;
        Fa += _p;
        M += r.cross(_p);
    }

    body2dType polygon2d::type() const
    {
        return POLYGON;
    }

    v2 polygon2d::min() const
    {
        return aabb.boundMin;
    }

    v2 polygon2d::max() const
    {
        return aabb.boundMax;
    }

    void polygon2d::update(int n)
    {
        if (isStatic) return;
        if (isSleep) return;

        switch (n)
        {
        case INIT_FORCE_AND_TORQUE:                                     // 初始化力和力矩
        {
            F.x = F.y = 0;
            M = 0;
        }
        break;
        case CALC_VELOCITY_AND_ANGULAR_VELOCITY:                        // 计算速度和角速度
        {
            V += F * mass.inv * world2d::dt;                            //速度增量 = 力 * 时间间隔 / 质量
            angleV += M * inertia.inv * world2d::dt;                    //角速度增量 = 力矩 * 时间间隔 / 转动惯量
        }
        break;
        case CALC_DISPLACEMENT_AND_ANGLE:                               // 计算位移和角度
        {
            pos += V * world2d::dt;
            angle += angleV * world2d::dt;
            R.rotate(angle);
            for (size_t i = 0; i < vertices.size(); i++)
            {
                auto v = R.rotate(vertices[i] - center) + center;
                verticesWorld[i] = pos + v;
            }
            calcPolygonBounds();
        }
        break;
        case ADD_GRAVITY:                                               // 添加重力
        {
            F += world2d::gravity * mass.value;
            Fa += F;
        }
        break;
        case RESET_NET_FORCE:                                           // 合外力累计清零
        {
            Fa.x = Fa.y = 0;
        }
        break;
        case DETERMINE_DORMANCY:                                                         // 当合外力和速度为零时，判定休眠
        {
            if (Fa.zero(1e-2) && V.zero(1e-2) && std::abs(angleV) < 1e-2) {
                V.x = 0;
                V.y = 0;
                angleV = 0;
                update(INIT_FORCE_AND_TORQUE);
                update(RESET_NET_FORCE);
                collNum = 0;
                isSleep = true;
            }
        }
        break;
        default:
            break;
        }
    }

    void polygon2d::draw(Helper2d * helper, int color)
    {
        helper->paintPolygon(verticesWorld, color);

        auto p = pos + center;

        auto F = v2((Fa.x >= 0 ? 0.05 : -0.05) * std::log10(1 + std::abs(Fa.x) * 5), (Fa.y >= 0 ? 0.05 : -0.05) * std::log10(1 + std::abs(Fa.y) * 5)); // 力向量
                                                                                                                                                       //auto D = v2(R.x1 * 0.05, R.x2 * 0.05);                //旋转方向

        if (false == isStatic)
        {
            helper->paintLine(p, p + F, helper->dragYellow);        //力向量
            helper->paintLine(p, p + V, helper->dragRed);           //速度向量
        }
    }

    v2 polygon2d::edge(const size_t idx) const           //向量|idx+1, idx|
    {
        return verticesWorld[(idx + 1) % verticesWorld.size()] - verticesWorld[idx];
    }

    //-------------circle2d----------------------------------------

    circle2d::circle2d(uint16_t _id, double _mass, v2 _pos, double _r) : body2d(_id, _mass, _pos), r(_r)
    {
        init();
    }

    void circle2d::calcCircleBounds()
    {
        aabb.boundMin = pos - r;
        aabb.boundMax = pos + r;
    }

    void circle2d::init()
    {
        inertia.set(mass.value * r * r * 0.5); //质量 * 半径平方 /2
        int n = int(std::floor(std::pow(2.0, std::log2(M_PI * 2 * r * 64))));
        auto delta = 2.0 / n;

        for (auto i = 0; i < n; ++i)
        {
            vertices.push_back(v2(r * std::cos(i * delta * M_PI), r * std::sin(i * delta * M_PI)));
        }
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            verticesWorld.push_back(vertices[i] + pos);
        }
        calcCircleBounds();
    }

    body2dType circle2d::type() const
    {
        return CIRCLE;
    }

    void circle2d::setStatic()
    {
        isStatic = true;
    }

    void circle2d::impulse(const v2 & p, const v2 & r)
    {
        if (isStatic) return;
        auto _p = p * world2d::dtInv;
        F += _p;
        Fa += _p;
        M += r.cross(_p);
    }

    void circle2d::drag(const v2 & pt, const v2 & offset)
    {
        V += mass.inv * offset;
        angleV += inertia.inv * (pt - pos - center).cross(offset);
    }

    bool circle2d::contains(const v2 & pt)
    {
        const auto delta = pos + center - pt;
        return delta.magnitudeSquare() < r * r;
    }

    v2 circle2d::min() const
    {
        return pos - r;
    }

    v2 circle2d::max() const
    {
        return pos + r;
    }

    void circle2d::update(int n)
    {
        if (isStatic) return;
        if (isSleep) return;

        switch (n)
        {
        case INIT_FORCE_AND_TORQUE:                                     // 初始化力和力矩
        {
            F.x = F.y = 0;
            M = 0;
        }
        break;
        case CALC_VELOCITY_AND_ANGULAR_VELOCITY:                        // 计算速度和角速度
        {
            V += F * mass.inv * world2d::dt;                            //速度增量 = 力 * 时间间隔 / 质量
            angleV += M * inertia.inv * world2d::dt;                    //角速度增量 = 力矩 * 时间间隔 / 转动惯量
        }
        break;
        case CALC_DISPLACEMENT_AND_ANGLE:                               // 计算位移和角度
        {
            pos += V * world2d::dt;
            angle += angleV * world2d::dt;
            R.rotate(angle);
            for (size_t i = 0; i < vertices.size(); i++)
            {
                auto v = R.rotate(vertices[i] - center) + center;
                verticesWorld[i] = pos + v;
            }
            calcCircleBounds();
        }
        break;
        case ADD_GRAVITY:                                               // 添加重力
        {
            F += world2d::gravity * mass.value;
            Fa += F;
        }
        break;
        case RESET_NET_FORCE:                                           // 合外力累计清零
        {
            Fa.x = Fa.y = 0;
        }
        break;
        case DETERMINE_DORMANCY:                                        // 当合外力和速度为零时，判定休眠
        {
            if (Fa.zero(1e-2) && V.zero(1e-2) && std::abs(angleV) < 1e-2) {
                V.x = 0;
                V.y = 0;
                angleV = 0;
                update(INIT_FORCE_AND_TORQUE);
                update(RESET_NET_FORCE);
                collNum = 0;
                isSleep = true;
            }
        }
        break;
        default:
            break;
        }
    }

    void circle2d::draw(Helper2d * helper, int color)
    {
        helper->paintCircle(pos, r, color);

        auto p = pos + center;

        auto F = v2((Fa.x >= 0 ? 0.05 : -0.05) * std::log10(1 + std::abs(Fa.x) * 5), (Fa.y >= 0 ? 0.05 : -0.05) * std::log10(1 + std::abs(Fa.y) * 5)); // 力向量

        if (false == isStatic)
        {
            helper->paintLine(p, p + F, helper->dragYellow);        //力向量
            helper->paintLine(p, p + V, helper->dragRed);           //速度向量
        }
    }

    v2 circle2d::edge(const size_t idx) const
    {
        return verticesWorld[(idx + 1) % verticesWorld.size()] - verticesWorld[idx];
    }
}