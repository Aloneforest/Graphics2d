#ifndef GRAPHICS2D_COLLISION2D_H
#define GRAPHICS2D_COLLISION2D_H

#include "body2d.h"
#include "joint2d.h"

class Helper2d;

namespace lib2d
{
    class world2d;

    //�Ӵ���
    struct contact
    {
        v2 pos;                     //λ��
        v2 ra, rb;                  //���ĵ��Ӵ������������������
        double sep{ 0 };            //�ص�����
        double massNormal{ 0 };
        double massTangent{ 0 };
        double bias{ 0 };           //������ײ�����ٶ�
        double pn{ 0 };             //�ۼƷ������
        double pt{ 0 };             //�ۼ��������
        int idxA{ 0 };              //���������ߵ�����+1 ��BΪ����AΪ����
        int idxB{ 0 };

        contact(v2 _pos, size_t _index) :pos(_pos), idxA(_index), idxB(_index) {}

        bool operator == (const contact & other) const;
        bool operator != (const contact & other) const;
    };

    //��ײ�ṹ
    struct collision
    {
        std::vector<contact> contacts;
        body2d::ptr bodyA;                  //��ײ����A     ��ָ�룿��������
        body2d::ptr bodyB;                  //��ײ����B
        size_t idxA;                        //��B���������A����ı�
        size_t idxB;                        //��A���������B����ı�
        double satA;                        //����϶����
        double satB;
        v2 N;                               //����
    };

    //��ײ����
    class collisionCalc
    {
    public:
        collisionCalc() = default;
        ~collisionCalc() = default;

        static uint32_t makeId(uint16_t a, uint16_t b);     //������κϳ�ID

        static bool separatingAxis(const body2d::ptr &bodyA, const body2d::ptr &bodyB, size_t &idx, double &sat);                   //�������㷨����������A���бߣ�������B���ж���ͶӰ���ߵķ����ϣ���ͶӰ������СֵΪ�������ཻ
        static bool boundCollition(const body2d::ptr &bodyA, const body2d::ptr &bodyB);                                             //��������ཻ�ж��������������ĵ�����������������߳�֮�͵�һ�룬���ཻ

        static size_t incidentEdge(const v2 & N, polygon2d::ptr bodyPtr);                                                           //������С��϶����
        static size_t clip(std::vector<contact> & out, const std::vector<contact> & in, size_t i, const v2 & p1, const v2 & p2);    //Sutherland-Hodgman������βü������������ײ�����������߶���һ�������ײ�ߵĲü�
        static void collisionUpdate(collision & coll, const collision & oldColl);                                                   //��ײ����
        static bool solveCollition(collision & coll);                                                                               //������ײ

        static void collisionDetection(world2d &world);                                                                             //��ײ���
        static void collisionDetection(const body2d::ptr &bodyA, const body2d::ptr &bodyB, world2d &world);                         //����������ײ��⣬������ײ�ṹ

        static void collisionPrepare(collision & coll);                                                                             //��ײ����׼�����������ϵ��
        static void collisionStateUpdate(collision & coll);                                                                         //����������ײ���״̬

        static void drawCollision(Helper2d * helper, const collision & coll);                                                       //������ײ
    };
}

#endif //GRAPHICS2D_COLLISION2D_H