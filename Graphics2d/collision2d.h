#ifndef GRAPHICS2D_COLLISION2D_H
#define GRAPHICS2D_COLLISION2D_H

#include "body2d.h"
#include "joint2d.h"

class Helper2d;

namespace lib2d
{
    class world2d;

    //接触点
    struct contact
    {
        v2 pos;                     //位置
        v2 ra, rb;                  //重心到接触点的向量，计算力矩
        double sep{ 0 };            //重叠距离
        double massNormal{ 0 };
        double massTangent{ 0 };
        double bias{ 0 };           //补偿碰撞物体速度
        double pn{ 0 };             //累计法向冲量
        double pt{ 0 };             //累计切向冲量
        int idxA{ 0 };              //交点所属边的索引+1 （B为正，A为负）
        int idxB{ 0 };

        contact(v2 _pos, size_t _index) :pos(_pos), idxA(_index), idxB(_index) {}

        bool operator == (const contact & other) const;
        bool operator != (const contact & other) const;
    };

    //碰撞结构
    struct collision
    {
        std::vector<contact> contacts;
        body2d::ptr bodyA;                  //碰撞物体A     弱指针？？？？？
        body2d::ptr bodyB;                  //碰撞物体B
        size_t idxA;                        //离B物体最近的A物体的边
        size_t idxB;                        //离A物体最近的B物体的边
        double satA;                        //最大间隙长度
        double satB;
        v2 N;                               //法线
    };

    //碰撞计算
    class collisionCalc
    {
    public:
        collisionCalc() = default;
        ~collisionCalc() = default;

        static uint32_t makeId(uint16_t a, uint16_t b);     //两多边形合成ID

        static bool separatingAxis(const body2d::ptr &bodyA, const body2d::ptr &bodyB, size_t &idx, double &sat);                   //分离轴算法：遍历矩阵A所有边，将矩阵B所有顶点投影到边的法线上，若投影长度最小值为负，则相交
        static bool boundCollition(const body2d::ptr &bodyA, const body2d::ptr &bodyB);                                             //外包矩阵相交判定：若两矩阵中心点相隔距离大于两矩阵边长之和的一半，则不相交

        static size_t incidentEdge(const v2 & N, polygon2d::ptr bodyPtr);                                                           //查找最小间隙索引
        static size_t clip(std::vector<contact> & out, const std::vector<contact> & in, size_t i, const v2 & p1, const v2 & p2);    //Sutherland-Hodgman（多边形裁剪），计算除碰撞边以外其他边对另一物体的碰撞边的裁剪
        static void collisionUpdate(collision & coll, const collision & oldColl);                                                   //碰撞更新
        static bool solveCollition(collision & coll);                                                                               //计算碰撞

        static void collisionDetection(world2d &world);                                                                             //碰撞检测
        static void collisionDetection(const body2d::ptr &bodyA, const body2d::ptr &bodyB, world2d &world);                         //两物体间的碰撞检测，构造碰撞结构

        static void collisionPrepare(collision & coll);                                                                             //碰撞计算准备，计算相关系数
        static void collisionStateUpdate(collision & coll);                                                                         //更新物体碰撞后的状态

        static void drawCollision(Helper2d * helper, const collision & coll);                                                       //绘制碰撞
    };
}

#endif //GRAPHICS2D_COLLISION2D_H