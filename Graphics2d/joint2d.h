#ifndef GRAPHICS2D_JOINT2D_H
#define GRAPHICS2D_JOINT2D_H

#include "body2d.h"

class Helper2d;

namespace lib2d
{
    //�ؽڣ�����������ϵ����
    class joint
    {
    public:
        using ptr = std::shared_ptr<joint>;

        joint(body2d::ptr &_a, body2d::ptr &_b) : a(_a), b(_b) {}
        joint(const body2d &) = delete;
        joint &operator= (const joint &) = delete;

        virtual void prepare() = 0;                             //Ԥ����
        virtual void update() = 0;                              //����
        virtual void draw(Helper2d * helper) = 0;               //����

        body2d::ptr a;
        body2d::ptr b;
    };

    //��ת�ؽ�
    class revoluteJoint : public joint
    {
    public:
        revoluteJoint(body2d::ptr &_a, body2d::ptr &_b, const v2 & _anchor);
        revoluteJoint(const revoluteJoint &) = delete;
        revoluteJoint &operator= (const revoluteJoint &) = delete;

        void prepare() override;
        void update() override;
        void draw(Helper2d * helper) override;      //������ת�뾶

        v2 worldAnchorA() const;                    //����Aê����������
        v2 worldAnchorB() const;                    //����Bê����������

        v2 anchor;                  //ê�����������
        v2 localAchorA;             //ê���������A�ı�������
        v2 localAchorB;             //ê���������A�ı�������
        v2 ra;                      //����A������ת�뾶��ê�㵱ǰ���������A�����������꣩
        v2 rb;                      //����B������ת�뾶
        m2 mass;                    //��������
        v2 p;                       //����
        v2 pAcc;                    //�����ۼ�
        v2 bias;                    //����
    };
}
#endif //GRAPHICS2D_JOINT2D_H