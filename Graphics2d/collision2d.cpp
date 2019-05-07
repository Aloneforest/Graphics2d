#include "stdafx.h"
#include "collision2d.h"
#include "Helper2d.h"

namespace lib2d
{
    //-------------contact----------------------------------------

    bool contact::operator == (const contact & other) const
    {
        if (idxA == other.idxA && idxB == other.idxB)
        {
            return true;
        }
        return idxA == other.idxB && idxB == other.idxA;
    }

    bool contact::operator != (const contact & other) const
    {
        return !(*this == other);
    }

    //-------------collisionCalc-------------------------------------

    uint32_t collisionCalc::makeId(uint16_t a, uint16_t b)
    {
        return std::min(a, b) << 16 | std::max(a, b);
    }

    bool collisionCalc::separatingAxis(const body2d::ptr &bodyA, const body2d::ptr &bodyB, size_t &idx, double &sat)   //�������㷨
    {
        auto a = bodyA;// std::dynamic_pointer_cast<polygon2d>(bodyA);
        auto b = bodyB;// std::dynamic_pointer_cast<polygon2d>(bodyB);

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
        auto a = bodyA;//std::dynamic_pointer_cast<polygon2d>(bodyA);
        auto b = bodyB;//std::dynamic_pointer_cast<polygon2d>(bodyB);
        auto centerA = (a->aabb.boundMax + a->aabb.boundMin) / 2;     //�����������
        auto centerB = (b->aabb.boundMax + b->aabb.boundMin) / 2;
        auto sizeA = (a->aabb.boundMax - a->aabb.boundMin);           //�������߳�
        auto sizeB = (b->aabb.boundMax - b->aabb.boundMin);
        return std::abs(centerB.x - centerA.x) / 2 <= (sizeA.x + sizeB.x) && std::abs(centerB.y - centerA.y) / 2 <= (sizeA.y + sizeB.y);
    }

    size_t collisionCalc::incidentEdge(const v2 & N, polygon2d::ptr bodyPtr)
    {
        size_t idx = SIZE_MAX;
        auto minDot = inf;
        auto body = bodyPtr;// std::dynamic_pointer_cast<polygon2d>(bodyPtr);
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

        auto bodyA = coll.bodyA;// std::dynamic_pointer_cast<polygon2d>(coll.bodyA);
        auto bodyB = coll.bodyB;// std::dynamic_pointer_cast<polygon2d>(coll.bodyB);
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

        //����ϵ������
        auto &pos0 = contacts[0].pos;
        auto &pos1 = contacts[1].pos;
        const auto CO = bodyA->CO * bodyB->CO;
        auto dist = std::abs((va - pos0).dot(coll.N));
        auto bias = std::log10(1 + dist) * CO;
        pos0 -= coll.N * bias * 10;
        dist = std::abs((va - pos1).dot(coll.N));
        bias = std::log10(1 + dist) * CO;
        pos1 -= coll.N * bias * 10;

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
                bodyA->isSleep = false;
                bodyB->isSleep = false;
            }
        }
        else                                    //����
        {
            if (solveCollition(coll))
            {
                //collisionUpdate(coll, world.collisions[id]);
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
            for (auto &body : world.staticBodies)
            {
                collisionDetection(world.bodies[i], body, world);
            }
        }
        return;
    }

    void collisionCalc::collisionPrepare(collision & coll)
    {
        const double kBiasFactor = 0.2;             //����ϵ��
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

            contact.bias = -kBiasFactor * world2d::dtInv * std::min(0.0, contact.sep);      //������ײ�����ٶ� = ����ϵ�� * ��λʱ�� * ��ײ�����ཻ����
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

            a.update(INIT_FORCE_AND_TORQUE);
            b.update(INIT_FORCE_AND_TORQUE);

            auto p = dpn * coll.N + dpt * tangent;
            a.impulse(-p, contact.ra);
            b.impulse(p, contact.rb);
            contact.pn += dpn;
            contact.pt += dpt;

            a.update(CALC_VELOCITY_AND_ANGULAR_VELOCITY);
            b.update(CALC_VELOCITY_AND_ANGULAR_VELOCITY);
        }
    }

    void collisionCalc::collisionRemoveSleep(world2d &world) {
        erase_if(world.collisions, [&](auto &c) {
            if (c.second.bodyA->isStatic)
                return c.second.bodyB->isSleep;
            if (c.second.bodyB->isStatic)
                return c.second.bodyA->isSleep;
            return c.second.bodyA->isSleep && c.second.bodyB->isSleep;
        });
    }

    void collisionCalc::drawCollision(Helper2d * helper, const collision & coll)
    {
        const auto typeA = coll.bodyA->type();
        const auto typeB = coll.bodyB->type();
        auto showA = false;
        auto showB = false;
        {
            showA = true;
            auto bodyA = coll.bodyA;// std::dynamic_pointer_cast<polygon2d>(coll.bodyA);
            auto ptA1 = bodyA->verticesWorld[coll.idxA % bodyA->verticesWorld.size()];
            auto ptA2 = bodyA->verticesWorld[(coll.idxA + 1) % bodyA->verticesWorld.size()];
            helper->paintLine(ptA1, ptA2, helper->dragRed);
        }
        {
            showB = true;
            auto bodyB = coll.bodyB;//std::dynamic_pointer_cast<polygon2d>(coll.bodyB);
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