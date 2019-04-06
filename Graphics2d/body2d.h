#ifndef GRAPHICS2D_BODY2D_H
#define GRAPHICS2D_BODY2D_H

#include "math2d.h"

class Helper2d;

namespace lib2d
{
    struct AABB 
    {
        v2 boundMin, boundMax;                      //�������
    };

    class body2d
    {
    public:
        using ptr = std::shared_ptr<body2d>;

        body2d(uint16_t _id, double _mass, v2 _pos) : id(_id), mass(_mass), pos(_pos) {}
        body2d(const body2d &) = delete;                            //��ֹ����
        body2d &operator= (const body2d &) = delete;                //��ֹ��ֵ

        virtual void drag(const v2 & pt, const v2 & offset) = 0;     //��ק����ʩ������
        virtual bool contains(const v2 & pt) = 0;                   //�жϵ�İ�����ϵ

        virtual void impulse(const v2 & p, const v2 & r) = 0;       //�������£���������
        virtual body2dType type() const = 0;                        //���ظ�������

        virtual v2 min() const = 0;
        virtual v2 max() const = 0;

        virtual void update(int) = 0;                               //����״̬
        virtual void draw(Helper2d * helper) = 0;                   //��ͼ

        virtual v2 edge(const size_t idx) const = 0;

        v2 rotate(const v2 &v) const;                               //��������v����angle�Ƕ���ת�������
        v2 world() const;                                           //���ظ���������������

        bool isStatic{ false };
        int collNum{ 0 };           //��ײ����
        uint16_t id{ 0 };           //ID
        doubleInv mass{ 0 };        //����
        v2 pos;                     //��������
        v2 center;                  //����
        v2 V;                       //�ٶ�
        double angle{ 0 };          //�Ƕ�
        double angleV{ 0 };         //���ٶ�
        doubleInv inertia{ 0 };     //ת������
        double f{ 1 };              //����/��Ħ��ϵ��
        m2 R;                       //��ת����
        v2 F;                       //����
        v2 Fa;                      //������������
        double M{ 0 };              //����
        AABB aabb;                                  //�������
        std::vector<v2> vertices;                   //����ζ��㣨�������꣩
        std::vector<v2> verticesWorld;              //����ζ��㣨�������꣩
    };

    class polygon2d : public body2d
    {
    public:
        polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices);

        double calcPolygonArea();                               //�����������
        v2 calcPolygonCentroid();                               //������������
        double calcPolygonInertia(double mass);                 //��������ת������
        void calcPolygonBounds();                               //���������������
        bool containsInBound(const v2 & pt);                    //�жϵ��Ƿ��ڶ�������������
        bool containsInPolygon(const v2 & pt);                  //�жϵ��Ƿ��ڶ������

        void drag(const v2 & pt, const v2 & offset) override;   //��ק����
        bool contains(const v2 & pt) override;                  //�ж��ཻ

        void init(const std::vector<v2> &_vertices);
        void setStatic();                                       //��̬�����ʼ��
        void impulse(const v2 & p, const v2 & r) override;      //���ݶ�����������������
        body2dType type() const override;

        v2 min() const override;
        v2 max() const override;

        void update(int n) override;
        void draw(Helper2d * helper) override;

        v2 edge(const size_t idx) const override;                        //����|idx+1, idx|

    };

    class circle2d : public body2d
    {
    public:
        circle2d(uint16_t _id, double _mass, v2 _pos, double _r);

        void calcCircleBounds();                               //���������������

        void drag(const v2 & pt, const v2 & offset) override;   //��ק����
        bool contains(const v2 & pt) override;                  //�ж��ཻ

        void init();
        void setStatic();                                       //��̬�����ʼ��
        void impulse(const v2 & p, const v2 & r) override;      //���ݶ�����������������
        body2dType type() const override;

        v2 min() const override;
        v2 max() const override;

        void update(int n) override;
        void draw(Helper2d * helper) override;

        v2 edge(const size_t idx) const override;                        //����|idx+1, idx|

        double r;
    };
}

#endif //GRAPHICS2D_BODY2D_H