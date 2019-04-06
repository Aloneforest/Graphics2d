#ifndef GRAPHICS2D_JOINT2D_H
#define GRAPHICS2D_JOINT2D_H

#include "body2d.h"

class Helper2d;

namespace lib2d
{
    //关节：两个刚体间关系基类
    class joint
    {
    public:
        using ptr = std::shared_ptr<joint>;

        joint(body2d::ptr &_a, body2d::ptr &_b) : a(_a), b(_b) {}
        joint(const body2d &) = delete;
        joint &operator= (const joint &) = delete;

        virtual void prepare() = 0;                             //预处理
        virtual void update() = 0;                              //更新
        virtual void draw(Helper2d * helper) = 0;               //绘制

        body2d::ptr a;
        body2d::ptr b;
    };

    //旋转关节
    class revoluteJoint : public joint
    {
    public:
        revoluteJoint(body2d::ptr &_a, body2d::ptr &_b, const v2 & _anchor);
        revoluteJoint(const revoluteJoint &) = delete;
        revoluteJoint &operator= (const revoluteJoint &) = delete;

        void prepare() override;
        void update() override;
        void draw(Helper2d * helper) override;      //绘制旋转半径

        v2 worldAnchorA() const;                    //物体A锚点世界坐标
        v2 worldAnchorB() const;                    //物体B锚点世界坐标

        v2 anchor;                  //锚点的世界坐标
        v2 localAchorA;             //锚点相对物体A的本地坐标
        v2 localAchorB;             //锚点相对物体A的本地坐标
        v2 ra;                      //物体A定轴旋转半径（锚点当前相对与物体A重心所在坐标）
        v2 rb;                      //物体B定轴旋转半径
        m2 mass;                    //质量矩阵
        v2 p;                       //冲量
        v2 pAcc;                    //冲量累计
        v2 bias;                    //补偿
    };
}
#endif //GRAPHICS2D_JOINT2D_H