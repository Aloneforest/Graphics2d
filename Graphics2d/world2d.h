#ifndef GRAPHICS2D_WORLD2D_H
#define GRAPHICS2D_WORLD2D_H

#include "body2d.h"
#include "joint2d.h"
#include "collision2d.h"

class Helper2d;

namespace lib2d
{
    //屏幕
    class world2d
    {
        friend class collisionCalc;
    public:
        world2d() = default;
        ~world2d() = default;

        int makePolygon(const double mass, const std::vector<v2> &vertices, const v2 &pos, const bool statics);
        int makeRect(const double mass, double w, double h, const v2 &pos, const bool statics);
        int makeCircle(const double mass, double r, const v2 &pos, const bool statics);
        void world2d::makeRevoluteJoint(body2d::ptr &a, body2d::ptr &b, const v2 &anchor);

        void step(Helper2d * helper);
        void clear();
        void init();
        void scene(int i);
        void makeBound();

        body2d * findBody(const v2 & pos);               //查找点所在图形
        void offset(const v2 & pt, const v2 & offset);   //计算点所在图形受力变换
        void mouse(const v2 & pt, bool down);            //鼠标坐标捕获
        void motion(const v2 & pt);                      //鼠标移动矢量

        void setHelper(Helper2d * helper);
        void enableGravity();

    public:
        static QTime lastClock;
        static double dt;
        static double dtInv;
        static v2 gravity;   //重力系数

    private:
        Helper2d * helper;

        bool mouse_drag{ false };       //鼠标拖动
        v2 globalDrag;                  //鼠标点击点
        v2 globalDragOffset;            //鼠标移动矢量

        std::vector<body2d::ptr> bodies;
        std::vector<body2d::ptr> staticBodies;
        std::vector<joint::ptr> joints;

        std::unordered_map<uint32_t, collision> collisions;     //hashmap

        uint16_t globalId = 1;
        int sleepNum = 0;
    };
}

#endif //GRAPHICS2D_WORLD2D_H