#include "stdafx.h"
#include "joint2d.h"
#include "Helper2d.h"

namespace lib2d
{
    //-------------revoluteJoint----------------------------------------

    revoluteJoint::revoluteJoint(body2d::ptr &_a, body2d::ptr &_b, const v2 & _anchor) : joint(_a, _b), anchor(_anchor)
    {
        localAchorA = m2().rotate(-a->angle).rotate(anchor - a->world());
        localAchorB = m2().rotate(-b->angle).rotate(anchor - b->world());
    }

    void revoluteJoint::prepare()
    {
        static const auto kBiasFactor = 0.2;             //补偿系数
        auto a = this->a;
        auto b = this->b;
        ra = a->rotate(localAchorA);
        rb = b->rotate(localAchorB);
        auto k = m2(a->mass.inv + b->mass.inv)
            + m2(a->inertia.inv * m2(ra.y * ra.y, -ra.y * ra.x, -ra.y * ra.x, ra.x * ra.x)) + m2(b->inertia.inv * m2(rb.y * rb.y, -rb.y * rb.x, -rb.y * rb.x, rb.x * rb.x));
        mass = k.inv();
        bias = -kBiasFactor * world2d::dtInv * (b->world() + rb - a->world() - ra); //物体A和物体B之间锚点位移修正（两锚点应该重合，两描点世界坐标相减）
    }

    void revoluteJoint::update()
    {
        auto a = this->a;
        auto b = this->b;
        auto dv = (a->V + (-a->angleV * ra.N())) - (b->V + (-b->angleV * rb.N()));

        p = mass * (dv + bias);
        if (!p.zero(1e-6))
        {
            pAcc = p;

            a->update(INIT_FORCE_AND_TORQUE);
            b->update(INIT_FORCE_AND_TORQUE);

            a->impulse(-p, ra);
            b->impulse(p, rb);

            a->update(CALC_VELOCITY_AND_ANGULAR_VELOCITY);
            b->update(CALC_VELOCITY_AND_ANGULAR_VELOCITY);
        }
    }

    void revoluteJoint::draw(Helper2d * helper)
    {
        auto centerA = a->world();
        auto anchorA = worldAnchorA();
        auto centerB = b->world();
        auto anchorB = worldAnchorB();

        if (!a->isStatic)
        {
            helper->paintLine(centerA, anchorA, helper->dragYellow);
        }
        if (!b->isStatic)
        {
            helper->paintLine(centerB, anchorB, helper->dragYellow);
        }
    }

    v2 revoluteJoint::worldAnchorA() const
    {
        return a->rotate(localAchorA) + a->world();
    }

    v2 revoluteJoint::worldAnchorB() const
    {
        return b->rotate(localAchorB) + b->world();
    }
}