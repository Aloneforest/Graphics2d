#include "stdafx.h"
#include "world2d.h"
#include "collision2d.h"
#include "Helper2d.h"

namespace lib2d
{
    QTime world2d::lastClock = QTime::currentTime();
    double world2d::dt = 1.0 / 30;
    double world2d::dtInv = 30;
    v2 world2d::gravity = { 0, -0.5 };

    int world2d::makePolygon(const double mass, const std::vector<v2> &vertices, const v2 &pos, const bool statics = false)
    {
        auto polygon = std::make_unique<polygon2d>(globalId++, mass, pos, vertices);
        if (statics)
        {
            polygon->mass.set(inf);
            polygon->setStatic();
            staticBodies.push_back(std::move(polygon));
            return staticBodies.size() - 1;
        }
        else
        {
            bodies.push_back(std::move(polygon));
            return bodies.size() - 1;
        }
    }

    int world2d::makeRect(const double mass, double w, double h, const v2 &pos, bool statics = false)
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

    int world2d::makeCircle(const double mass, double r, const v2 &pos, const bool statics = false)
    {
        auto circle = std::make_unique<circle2d>(globalId++, mass, pos, r);
        if (statics)
        {
            circle->mass.set(inf);
            circle->setStatic();
            staticBodies.push_back(std::move(circle));
            return staticBodies.size() - 1;
        }
        else
        {
            bodies.push_back(std::move(circle));
            return bodies.size() - 1;
        }
    }

    void world2d::makeRevoluteJoint(body2d::ptr &a, body2d::ptr &b, const v2 &anchor)
    {
        auto joint = std::make_shared<revoluteJoint>(a, b, anchor);
        joints.push_back(std::move(joint));
    }

    void world2d::step(Helper2d * helper)
    {
        auto now = QTime::currentTime();
        dt = lastClock.msecsTo(now)*0.001;    //计算距离时间t的毫秒数，如果t早于当前时间，则为负
        dt = std::min(dt, 1.0 / 30);
        dtInv = 1.0 / dt;
        lastClock = now;
        sleepNum = 0;

        helper->clear();

        collisionCalc::collisionDetection(helper->getWorld());  //构造碰撞结构

        //碰撞预处理
        for (auto & col : collisions)
        {
            collisionCalc::collisionPrepare(col.second);        //计算碰撞相关系数
        }

        //关节预处理
        for (auto & joint : joints)
        {
            joint->prepare();
        }

        //碰撞外力重设
        for (auto &body : bodies)
        {
            body->update(RESET_NET_FORCE);
        }

        //更新物体应碰撞改变的受力状态
        for (auto i = 0; i < 10; ++i)
        {
            for (auto & coll : collisions)
            {
                collisionCalc::collisionStateUpdate(coll.second);              
            }

            for (auto & joint : joints)
            {
                joint->update();
            }
        }

        //更新物体速度位置等
        for (auto &body : bodies)
        {
            body->update(INIT_FORCE_AND_TORQUE);
            body->update(ADD_GRAVITY);
            body->update(CALC_VELOCITY_AND_ANGULAR_VELOCITY);
            body->update(CALC_DISPLACEMENT_AND_ANGLE);
            body->update(DETERMINE_DORMANCY);
            if (body->isSleep)
                sleepNum++;
        }

        collisionCalc::collisionRemoveSleep(helper->getWorld());

        //绘制刚体
        for (auto &body : bodies)
        {
            body->draw(helper, YELLOW);
        }

        //绘制静态刚体
        for (auto &body : staticBodies)
        {
            body->draw(helper, WHITE);
        }

        //绘制碰撞情况
        for (auto &coll : collisions)
        {
            collisionCalc::drawCollision(helper, coll.second);
        }
        
        //绘制关节
        for (auto &joint : joints)
        {
            joint->draw(helper);
        }

        if (true == mouse_drag)
        {
            auto dist = globalDrag + globalDragOffset;
            helper->paintLine(globalDrag, dist, helper->dragYellow);
        }

        auto w = helper->getSize().width();
        auto h = helper->getSize().height();
        
        helper->paintText({ 10, 20 }, QString().sprintf("collnum: %d, sleepnum: %d", collisions.size(), sleepNum));
        helper->paintText({ (double)w - 100, 20 }, QString().sprintf("FPS: %.0lf", 1 / dt));
    }

    void world2d::clear()
    {
        globalId = 1;
        bodies.clear();
        staticBodies.clear();
        joints.clear();
        collisions.clear();
    }

    void world2d::makeBound()
    {
        makeRect(inf, 2, 0.02, { 0, 0.9 }, true);
        makeRect(inf, 2, 0.02, { 0, -0.97 }, true);
        makeRect(inf, 0.02, 1.95, { 0.97, -0.05 }, true);
        makeRect(inf, 0.02, 1.95, { -0.97, -0.05 }, true);
    }

    void world2d::init()
    {
        scene(1);
    }

    void world2d::scene(int i)
    {
        clear();
        switch (i)
        {
        case 1:
        {
            makeBound();
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
            //bodies[p1]->CO = 0.55;
            //p1->V = v2(0.2, 0);
            //p1->angleV = 0.8;
            auto p2 = makePolygon(2, vertices2, { 0.21, 0 });
            //bodies[p2]->CO = 0.55;
            //p2->V = v2(-0.2, 0);
            //p2->angleV = -0.8;
            auto p3 = makeCircle(2, 0.1, { 0.5, 0.5 });
            auto p4 = makeCircle(2, 0.1, { 0.5, 0.71 });
            //bodies[p3]->CO = 0.99;
            //bodies[p4]->CO = 0.99;
        }
            break;
        case 2:
        {
            auto ground = makeRect(inf, 1.5, 0.01, { 0, -0.9 }, true);
            auto box1 = makeCircle(2, 0.099, { 1.4, 0.8 });
            bodies[box1]->CO = 0.99;
            makeRevoluteJoint(staticBodies[ground], bodies[box1], { 0.3, 0.8 });
            //for (size_t i = 0; i < 4; i += 2)
            //{
            //    auto box2 = makeRect(2, 0.199, 0.199, { 0.1 - i * 0.2, -0.3 });
            //        bodies[box2]->CO = 0.99;
            //    makeRevoluteJoint(staticBodies[ground], bodies[box2], { 0.1 - i * 0.2, 0.8 });
            //}
            for (size_t i = 0; i < 4; i ++)
            {
                auto box2 = makeCircle(2, 0.099, { 0.1 - i * 0.2, -0.3 });
                    bodies[box2]->CO = 0.99;
                makeRevoluteJoint(staticBodies[ground], bodies[box2], { 0.1 - i * 0.2, 0.8 });
            }
        }
            break;
        case 3:
        {
            auto ground = makeRect(inf, 0.2, 0.2, { -0.3,-0.3 }, true);
            const double mass = 10.0;
            auto last = ground;
            auto box = makeRect(mass, 0.1, 0.02, { 0.05, 0.8 });
            makeRevoluteJoint(staticBodies[last], bodies[0], { 0, 0.8 });
            for (int i = 1; i < 15; ++i)
            {
                auto box = makeRect(mass, 0.1, 0.02, { 0.05 + 0.11 * i, 0.8 });
                makeRevoluteJoint(bodies[last], bodies[box], { 0.11*i, 0.8 });
                last = box;
            }
        }
            break;
        case 4:
        {
            makeBound();
            v2 x{ -0.45, -0.45 };
            v2 y;
            int n = 10;
            for (auto i = 0; i < n; ++i) {
                y = x;
                for (auto j = i; j < n; ++j) {
                    makeRect(1, 0.1, 0.1, y);
                    y += {0.11, 0.0};
                }
                x += {0.051, 0.11};
            }
        }
            break;
        case 5:
        {
            makeBound();
            v2 x{ -0.45, -0.45 };
            v2 y;
            int n = 10;
            for (auto i = 0; i < n; ++i) {
                y = x;
                for (auto j = i; j < n; ++j) {
                    switch ((i+j+13) % 3)
                    {
                    case 0:
                        makeRect(1, 0.1, 0.1, y);
                        break;
                    case 1:
                        makeCircle(1, 0.05, y);
                        break;
                    case 2:
                    {
                        std::vector<v2> vertices1 =
                        {
                            { 0.05, 0 },
                            { 0.02, 0.05 },
                            { -0.02, 0.05 },
                            { -0.05, 0 },
                            { -0.02, -0.05 },
                            { 0.02, -0.05 }
                        };
                        makePolygon(1, vertices1, y);
                    }
                        break;
                    }
                    y += {0.11, 0.0};
                }
                x += {0.051, 0.11};
            }
        }
        break;
        default:
            break;
        }
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
            body->isSleep = false;
            body->drag(pt, offset);
        }
    }

    void world2d::mouse(const v2 & pt, bool down)            //鼠标坐标捕获
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

    void world2d::motion(const v2 & pt)                      //鼠标移动矢量
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

    void world2d::enableGravity()
    {
        for (int i = 0; i < bodies.size(); i++)
        {
            bodies[i]->isSleep = false;
        }
    }
}