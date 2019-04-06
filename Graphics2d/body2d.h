#ifndef GRAPHICS2D_BODY2D_H
#define GRAPHICS2D_BODY2D_H

#include "math2d.h"

class Helper2d;

namespace lib2d
{
    struct AABB 
    {
        v2 boundMin, boundMax;                      //外包矩阵
    };

    class body2d
    {
    public:
        using ptr = std::shared_ptr<body2d>;

        body2d(uint16_t _id, double _mass, v2 _pos) : id(_id), mass(_mass), pos(_pos) {}
        body2d(const body2d &) = delete;                            //禁止拷贝
        body2d &operator= (const body2d &) = delete;                //禁止赋值

        virtual void drag(const v2 & pt, const v2 & offset) = 0;     //拖拽物体施加力矩
        virtual bool contains(const v2 & pt) = 0;                   //判断点的包含关系

        virtual void impulse(const v2 & p, const v2 & r) = 0;       //受力更新，计算力矩
        virtual body2dType type() const = 0;                        //返回刚体类型

        virtual v2 min() const = 0;
        virtual v2 max() const = 0;

        virtual void update(int) = 0;                               //更新状态
        virtual void draw(Helper2d * helper) = 0;                   //画图

        virtual v2 edge(const size_t idx) const = 0;

        v2 rotate(const v2 &v) const;                               //返回向量v经过angle角度旋转后的向量
        v2 world() const;                                           //返回刚体重心世界坐标

        bool isStatic{ false };
        int collNum{ 0 };           //碰撞次数
        uint16_t id{ 0 };           //ID
        doubleInv mass{ 0 };        //质量
        v2 pos;                     //世界坐标
        v2 center;                  //重心
        v2 V;                       //速度
        double angle{ 0 };          //角度
        double angleV{ 0 };         //角速度
        doubleInv inertia{ 0 };     //转动惯量
        double f{ 1 };              //滑动/静摩擦系数
        m2 R;                       //旋转矩阵
        v2 F;                       //受力
        v2 Fa;                      //受力（合力）
        double M{ 0 };              //力矩
        AABB aabb;                                  //外包矩阵
        std::vector<v2> vertices;                   //多边形顶点（本地坐标）
        std::vector<v2> verticesWorld;              //多边形顶点（世界坐标）
    };

    class polygon2d : public body2d
    {
    public:
        polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices);

        double calcPolygonArea();                               //计算多边形面积
        v2 calcPolygonCentroid();                               //计算多边形重心
        double calcPolygonInertia(double mass);                 //计算多边形转动变量
        void calcPolygonBounds();                               //计算多边形外包矩阵
        bool containsInBound(const v2 & pt);                    //判断点是否在多边形外包矩阵内
        bool containsInPolygon(const v2 & pt);                  //判断点是否在多边形内

        void drag(const v2 & pt, const v2 & offset) override;   //拖拽物体
        bool contains(const v2 & pt) override;                  //判断相交

        void init(const std::vector<v2> &_vertices);
        void setStatic();                                       //静态物体初始化
        void impulse(const v2 & p, const v2 & r) override;      //根据动量更新受力和力矩
        body2dType type() const override;

        v2 min() const override;
        v2 max() const override;

        void update(int n) override;
        void draw(Helper2d * helper) override;

        v2 edge(const size_t idx) const override;                        //向量|idx+1, idx|

    };

    class circle2d : public body2d
    {
    public:
        circle2d(uint16_t _id, double _mass, v2 _pos, double _r);

        void calcCircleBounds();                               //计算多边形外包矩阵

        void drag(const v2 & pt, const v2 & offset) override;   //拖拽物体
        bool contains(const v2 & pt) override;                  //判断相交

        void init();
        void setStatic();                                       //静态物体初始化
        void impulse(const v2 & p, const v2 & r) override;      //根据动量更新受力和力矩
        body2dType type() const override;

        v2 min() const override;
        v2 max() const override;

        void update(int n) override;
        void draw(Helper2d * helper) override;

        v2 edge(const size_t idx) const override;                        //向量|idx+1, idx|

        double r;
    };
}

#endif //GRAPHICS2D_BODY2D_H