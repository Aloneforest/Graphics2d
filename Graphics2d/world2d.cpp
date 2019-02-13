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

    double v2::cross(const v2 &v) const     //���
    {
        return x * v.y - y * v.x;
    }

    double v2::dot(const v2 &v) const       //���
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
        *this = m2{ _cos, -_sin, _sin, _cos };  //������ʱ����ͨ�� ������ֵ����� �����ṹ�帳ֵ
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
            area += vertices[i].cross(vertices[j]);     //��˼������
        }
        return area / 2;
    }

    v2 polygon2d::calcPolygonCentroid()
    {
        v2 gc;
        auto size = vertices.size();
        for (size_t i = 0; i < size; ++i)
        {
            auto j = (i + 1) % size;                                            //��������� = �������������� * ������� / �����
            gc += (vertices[i] + vertices[j]) * vertices[i].cross(vertices[j]); //���������� = ������֮�� / 3
        }
        return gc / 6.0 / calcPolygonArea();
    }

    double polygon2d::calcPolygonInertia(double mass)
    {
        double acc0 = 0;
        double acc1 = 0;
        auto size = vertices.size();
        for (size_t i = 0; i < size; ++i)       //ת������ = m / 6 * ��((Pn+1 x Pn)(Pn+1��Pn+1 + Pn+1��Pn + Pn��Pn))/����(Pn+1 x Pn)��
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
        return containsInBound(pt) && containsInPolygon(pt);    //���ж��Ƿ���������ڣ����ж��Ƿ��ڶ������
    }

    void polygon2d::drag(const v2 & pt, const v2 & offset)
    {
        V += mass.inv * offset;                                       //�ٶ� += ���������� / ����
        angleV += inertia.inv * (pt - pos - center).cross(offset);    //���ٶ� += �Ƕ��������أ����ĵ����������� x ������ / ת������
    }

    void polygon2d::init()
    {
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

    v2 polygon2d::world() const 
    {
        return pos + center;
    }

    body2dType polygon2d::type() const
    {
        return POLYGON;
    }

    v2 polygon2d::min() const
    {
        return boundMin;
    }

    v2 polygon2d::max() const
    {
        return boundMax;
    }

    void polygon2d::update(int n)
    {
        switch (n)
        {
        case 0:                                         // ��ʼ����������
        {
            F.x = F.y = 0;
            M = 0;
        }
            break;
        case 1:                                         // �����������أ��ó��ٶȺͽ��ٶ�
        {
            V += F * mass.inv * world2d::dt;            //�ٶ����� = ���ٶ� * ʱ����
            angleV += M * inertia.inv * world2d::dt;    //���ٶ����� = �Ǽ��ٶ� * ʱ����
        }
            break;
        case 2:                                         // ����λ�ƺͽǶ�
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
        case 3:                                         // �������
        {
            F += world2d::gravity * mass.value * world2d::dt;
            Fa += F;
        }
            break;
        case 4:                                         // �������ۼ�����
        {
            Fa.x = Fa.y = 0;
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

    v2 polygon2d::edge(const size_t idx) const           //����|idx+1, idx|
    {
        return verticesWorld[(idx + 1) % verticesWorld.size()] - verticesWorld[idx];
    }

    //-------------world2d--------------------------------------

    QTime world2d::lastClock = QTime::currentTime();
    double world2d::dt = 1.0 / 30;
    double world2d::dtInv = 30;
    v2 world2d::gravity = { 0,0 };//{ 0, -0.2 };

    polygon2d * world2d::makePolygon(const double mass, const std::vector<v2> &vertices, const v2 &pos, const bool statics = false)
    {
        auto polygon = std::make_unique<polygon2d>(globalId++, mass, pos, vertices);
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
        dt = lastClock.msecsTo(now)*0.001;    //�������ʱ��t�ĺ����������t���ڵ�ǰʱ�䣬��Ϊ��
        dt = std::min(dt, 1.0 / 30);
        dtInv = 1.0 / dt;
        lastClock = now;

        helper->clear();

        collisionCalc::collisionDetection(helper->getWorld());  //������ײ�ṹ

        for (auto & col : collisions)
        {
            collisionCalc::collisionPrepare(col.second);        //������ײ���ϵ��
        }

        for (auto &body : bodies)
        {
            body->update(4);
        }

        for (auto i = 0; i < 10; ++i)
        {
            for (auto & coll : collisions)
            {
                collisionCalc::collisionStateUpdate(coll.second);              //��������״̬
            }
        }

        for (auto &body : bodies)
        {
            body->update(0);
            body->update(3);
            body->update(1);
            body->update(2);
        }

        for (auto &body : bodies)
        {
            body->draw(helper);
        }

        for (auto &coll : collisions)
        {
            collisionCalc::drawCollision(helper, coll.second);
        }

        if (true == mouse_drag)
        {
            auto dist = globalDrag + globalDragOffset;
            helper->paintLine(globalDrag, dist, helper->dragYellow);
        }

        auto w = helper->getSize().width();
        auto h = helper->getSize().height();
        
        helper->paintText({ 10, 20 }, QString().sprintf("num: %d", collisions.size()));
    }

    void world2d::clear()
    {
        bodies.clear();
    }

    void world2d::init()
    {
        clear();
        std::vector<v2> vertices1 =
        {
            { -0.2, 0 },
            { 0.2, 0 },
            { 0, 0.3 }
        };
        std::vector<v2> vertices2 =
        {
            { -0.2, 0 },
            { 0.2, 0 },
            { 0.2, 0.2 },
            { -0.2, 0.2 }
        };
        auto p1 = makePolygon(2, vertices1, { -0.2, 0 });
        //p1->V = v2(0.2, 0);
        //p1->angleV = 0.8;
        auto p2 = makePolygon(2, vertices2, { 0.2, 0 });
        //p2->V = v2(-0.2, 0);
        //p2->angleV = -0.8;
    }

    body2d * world2d::findBody(const v2 & pos)               //���ҵ�����ͼ��
    {
        auto body = std::find_if(bodies.begin(), bodies.end(), [&](auto & b){return b->contains(pos);});
        if (body != bodies.end())
        {
            return (*body).get();
        }
        return nullptr;
    }

    void world2d::offset(const v2 & pt, const v2 & offset)   //���������ͼ�������任
    {
        auto body = findBody(pt);
        if (body)
        {
            body->drag(pt, offset);
        }
    }

    void world2d::mouse(const v2 & pt, bool down)            //������겶��
    {
        if (true == down)
        {
            mouse_drag = true;
            globalDragOffset.x = 0;
            globalDragOffset.y = 0;
            globalDrag.x = pt.x;
            globalDrag.y = pt.y;
        }
        else
        {
            mouse_drag = false;
            globalDragOffset.x = (pt.x - globalDrag.x);
            globalDragOffset.y = (pt.y - globalDrag.y);
            offset(globalDrag, globalDragOffset);
            globalDrag.x = pt.x;
            globalDrag.y = pt.y;
        }
    }

    void world2d::motion(const v2 & pt)                      //����ƶ�ʸ��
    {
        if (true == mouse_drag)
        {
            globalDragOffset.x = (pt.x - globalDrag.x);
            globalDragOffset.y = (pt.y - globalDrag.y);
        }
    }

    void world2d::world2d::setHelper(Helper2d * helper)
    {
        this->helper = helper;
    }

    //-------------collisionCalc-------------------------------------

    uint32_t collisionCalc::makeId(uint16_t a, uint16_t b)
    {
        return std::min(a, b) << 16 | std::max(a, b);
    }

    bool collisionCalc::separatingAxis(const body2d::ptr &bodyA, const body2d::ptr &bodyB, size_t &idx, double &sat)   //�������㷨
    {
        auto a = std::dynamic_pointer_cast<polygon2d>(bodyA);
        auto b = std::dynamic_pointer_cast<polygon2d>(bodyB);

        sat = -inf;

        for (size_t i = 0; i < a->vertices.size(); ++i)
        {
            auto va = a->verticesWorld[i];
            auto N = a->edge(i).normal();                           //ÿ���ߵķ�����
            auto sep = inf;                                         //��϶
            for (size_t j = 0; j < b->verticesWorld.size(); ++j)
            {
                auto vb = b->verticesWorld[j];
                sep = std::min(sep, (vb - va).dot(N));              //����|bodyB[j], bodyA[i]|ͶӰ������|bodyA[i+1], bodyA[i]|�ķ�������
            }

            if (sep > sat)
            {
                sat = sep;
                idx = i;
            }
        }
        return sat > 0;                                             //��϶����0��һ��û�нӴ�������0���ܷ�����ײ
    }

    bool collisionCalc::boundCollition(const body2d::ptr &bodyA, const body2d::ptr &bodyB)   //��������ཻ�ж�
    {
        auto a = std::dynamic_pointer_cast<polygon2d>(bodyA);
        auto b = std::dynamic_pointer_cast<polygon2d>(bodyB);
        auto centerA = (a->boundMax + a->boundMin) / 2;     //�����������
        auto centerB = (b->boundMax + b->boundMin) / 2;
        auto sizeA = (a->boundMax - a->boundMin);           //�������߳�
        auto sizeB = (b->boundMax - b->boundMin);
        return std::abs(centerB.x - centerA.x) / 2 <= (sizeA.x + sizeB.x) && std::abs(centerB.y - centerA.y) / 2 <= (sizeA.y + sizeB.y);
    }

    size_t collisionCalc::incidentEdge(const v2 & N, polygon2d::ptr bodyPtr)
    {
        size_t idx = SIZE_MAX;
        auto minDot = inf;
        auto body = std::dynamic_pointer_cast<polygon2d>(bodyPtr);
        for (size_t i = 0; i < body->verticesWorld.size(); ++i)
        {
            auto edgeNormal = body->edge(i).normal();   //����|body[i+1], body[i]|�ķ�����
            auto dot = edgeNormal.dot(N);               //����|body[i+1], body[i]|�ķ����� ͶӰ������ N ��
            if (dot < minDot)                           //������С����
            {
                minDot = dot;
                idx = i;
            }
        }
        return idx;                                     //������С��϶��Ӧ��
    }

    size_t collisionCalc::clip(std::vector<contact> & out, const std::vector<contact> & in, size_t i, const v2 & p1, const v2 & p2)    //Sutherland-Hodgman������βü���
    {
        size_t numOut = 0;
        auto N = (p2 - p1).normal();
        auto dist0 = N.dot(in[0].pos - p1);     //ͶӰ����
        auto dist1 = N.dot(in[1].pos - p1);

        if (dist0 <= 0) out[numOut++] = in[0];  //ͶӰС��0����������A��
        if (dist1 <= 0) out[numOut++] = in[1];

        if (dist0 * dist1 < 0)
        {
            auto interp = dist0 / (dist0 - dist1);
            out[numOut].pos = in[0].pos + interp * (in[1].pos - in[0].pos); //����|p1,p2|��|in1,in2|����
            out[numOut].idxA = -(int)i - 1;
            ++numOut;
        }

        return numOut;
    }

    bool collisionCalc::solveCollition(collision & coll)   //������ײ
    {
        //A��SAT���ӽ�0
        if (coll.satA < coll.satB)
        {
            std::swap(coll.bodyA, coll.bodyB);
            std::swap(coll.idxA, coll.idxB);
            std::swap(coll.satA, coll.satB);
        }

        auto bodyA = std::dynamic_pointer_cast<polygon2d>(coll.bodyA);
        auto bodyB = std::dynamic_pointer_cast<polygon2d>(coll.bodyB);
        auto bodyASize = bodyA->verticesWorld.size();
        auto bodyBSize = bodyB->verticesWorld.size();

        coll.N = bodyA->edge(coll.idxA).normal();       //����SAT��ķ���
        coll.idxB = incidentEdge(coll.N, bodyB);        //��ȡ��A���������B����ı�

        decltype(coll.contacts) contacts;               //��ȡ����

        //�ٶ������Ӵ��㣨��idxB���˵㣩
        contacts.emplace_back(bodyB->verticesWorld[coll.idxB % bodyBSize], coll.idxB % bodyBSize + 1);
        contacts.emplace_back(bodyB->verticesWorld[(coll.idxB + 1) % bodyBSize], (coll.idxB + 1) % bodyBSize + 1);
        auto tmp = contacts;

        for (size_t i = 0; i < bodyA->verticesWorld.size(); ++i)
        {
            if (i == coll.idxA)
            {
                continue;
            }
            if (clip(tmp, contacts, i, bodyA->verticesWorld[i % bodyASize], bodyA->verticesWorld[(i + 1) % bodyASize]) < 2)
            {
                return false;
            }
            contacts = tmp;
        }

        auto va = bodyA->verticesWorld[coll.idxA % bodyASize];

        for (auto & contact : contacts) //���㣺contact.pos    �ο��㣺�Ӵ��߶˵�va
        {
            auto sep = (contact.pos - va).dot(coll.N);
            if (sep <= 0)                       //����bodyA�ڵĽӴ���
            {
                contact.sep = sep;              //sepԽС���˵�va������pos�����߶ε�б��Խ�ӽ�����N
                contact.ra = contact.pos - coll.bodyA->pos - coll.bodyA->center;
                contact.rb = contact.pos - coll.bodyB->pos - coll.bodyB->center;
                coll.contacts.push_back(contact);
            }
        }

        return true;
    }

    void collisionCalc::collisionUpdate(collision & coll, const collision & oldColl)
    {
        auto & a = *coll.bodyA;
        auto & b = *coll.bodyB;

        const auto & oldContacts = oldColl.contacts;
        for (auto & newContact : coll.contacts)
        {
            auto oldContact = std::find(oldContacts.begin(), oldContacts.end(), newContact);
            if (oldContact != oldContacts.end())                            //ͬһ��ײ��ĸ���
            {
                newContact.pn = oldContact->pn;
                newContact.pt = oldContact->pt;

                auto tangent = coll.N.normal();                             //���߷���
                auto p = newContact.pn * coll.N + newContact.pt * tangent;  //���� = ���߷������ + ���߷������
                a.impulse(-p, newContact.ra);                               //ʩ������
                b.impulse(p, newContact.rb);
            }
        }
    }

    void collisionCalc::collisionDetection(const body2d::ptr &bodyA, const body2d::ptr &bodyB, world2d &world)
    {
        auto id = makeId(bodyA->id, bodyB->id);

        collision coll;
        coll.bodyA = bodyA;
        coll.bodyB = bodyB;

        //���ټ��
        if (!boundCollition(bodyA, bodyB)
            || separatingAxis(bodyA, bodyB, coll.idxA, coll.satA) || separatingAxis(bodyB, bodyA, coll.idxB, coll.satB))
        {
            auto prev = world.collisions.find(id);
            if (prev != world.collisions.end())
            {
                world.collisions.erase(prev);
                bodyA->collNum--;
                bodyB->collNum--;
            }
            return;
        }

        auto prev = world.collisions.find(id);
        if (prev == world.collisions.end())     //����
        {
            if (solveCollition(coll))
            {
                world.collisions.insert(std::make_pair(id, coll));
                bodyA->collNum++;
                bodyB->collNum++;
            }
        }
        else                                    //����
        {
            if (solveCollition(coll))
            {
                collisionUpdate(coll, world.collisions[id]);
                world.collisions[id] = coll;
            }
            else
            {
                world.collisions.erase(prev);
                bodyA->collNum--;
                bodyB->collNum--;
            }
        }
    }

    void collisionCalc::collisionDetection(world2d &world)
    {
        auto size = world.bodies.size();
        for (size_t i = 0; i < size; ++i)           //�������������������ɶ�����;�̬��������ײ���
        {
            for (size_t j = 0; j < size; ++j)
            {
                if (i < j)
                {
                    collisionDetection(world.bodies[i], world.bodies[j], world);
                }
            }
            for (auto &body : world.static_bodies)
            {
                collisionDetection(world.bodies[i], body, world);
            }
        }
        return;
    }

    void collisionCalc::collisionPrepare(collision & coll)
    {
        const double kBiasFactor = 0.2;             //��ײϵ��
        const auto & a = *coll.bodyA;
        const auto & b = *coll.bodyB;
        auto tangent = coll.N.normal();

        for (auto & contact : coll.contacts)
        {
            auto nA = contact.ra.cross(coll.N);     //���ط��߷���ͶӰ
            auto nB = contact.rb.cross(coll.N);
            auto kn = a.mass.inv + b.mass.inv + std::abs(a.inertia.inv) * nA * nA + std::abs(b.inertia.inv) * nB * nB;
            contact.massNormal = kn > 0 ? 1.0 / kn : 0.0;

            auto tA = contact.ra.cross(tangent);    //�������߷���ͶӰ
            auto tB = contact.rb.cross(tangent);
            auto kt = a.mass.inv + b.mass.inv + std::abs(a.inertia.inv) * tA * tA + std::abs(b.inertia.inv) * tB * tB;
            contact.massTangent = kt > 0 ? 1.0 / kt : 0.0;

            contact.bias = -kBiasFactor * world2d::dtInv * std::min(0.0, contact.sep);      //������ײ�����ཻ����
        }
    }

    void collisionCalc::collisionStateUpdate(collision & coll)
    {
        auto & a = *coll.bodyA;
        auto & b = *coll.bodyB;

        auto tangent = coll.N.normal();
        for (auto & contact : coll.contacts)
        {
            auto dv = (b.V + (-b.angleV * contact.rb.N())) - (a.V + (-a.angleV * contact.ra.N()));  //�Ӵ�����ٶ�=��B����ƽ���ٶ�+B�������ٶȣ�-��A����ƽ���ٶ�+A�������ٶȣ�

            auto vn = dv.dot(coll.N);                                                       //���߷����ٶȷ���
            auto dpn = (-vn + contact.bias) * contact.massNormal;                           //�����������
            if (contact.pn + dpn < 0)
            {
                dpn = -contact.pn;
            }

            auto vt = dv.dot(tangent);                                                      //���߷����ٶȷ���
            auto dpt = -vt * contact.massTangent;                                           //�����������
            auto friction = sqrt(a.f * b.f) * contact.pn;                                   //Ħ���� = Ħ��ϵ�� * ѹ��
            dpt = std::max(-friction, std::min(friction, contact.pt + dpt)) - contact.pt;   //�ۼƺ����ķ�Χ�޶���[-friction, friction]��

            a.update(0);
            b.update(0);

            auto p = dpn * coll.N + dpt * tangent;
            a.impulse(-p, contact.ra);
            b.impulse(p, contact.rb);
            contact.pn += dpn;
            contact.pt += dpt;

            a.update(1);
            b.update(1);
        }
    }

    void collisionCalc::drawCollision(Helper2d * helper, const collision & coll)
    {
        const auto typeA = coll.bodyA->type();
        const auto typeB = coll.bodyB->type();
        auto showA = false;
        auto showB = false;
        if (POLYGON == typeA)
        {
            showA = true;
            auto bodyA = std::dynamic_pointer_cast<polygon2d>(coll.bodyA);
            auto ptA1 = bodyA->verticesWorld[coll.idxA % bodyA->verticesWorld.size()];
            auto ptA2 = bodyA->verticesWorld[(coll.idxA + 1) % bodyA->verticesWorld.size()];
            helper->paintLine(ptA1, ptA2, helper->dragRed);
        }
        if (POLYGON == typeB)
        {
            showB = true;
            auto bodyB = std::dynamic_pointer_cast<polygon2d>(coll.bodyB);
            auto ptB1 = bodyB->verticesWorld[coll.idxB % bodyB->verticesWorld.size()];
            auto ptB2 = bodyB->verticesWorld[(coll.idxB + 1) % bodyB->verticesWorld.size()];
            helper->paintLine(ptB1, ptB2, helper->dragRed);
        }
        if (showA && showB)
        {
            for (auto & contact : coll.contacts)
            {
                helper->paintPoint(contact.pos, helper->dragWhite);
            }
        }
    }

}