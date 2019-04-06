#ifndef GRAPHICS2D_MATH2D_H
#define GRAPHICS2D_MATH2D_H

class Helper2d;

namespace lib2d
{
    //��ά����
    struct v2
    {
        double x{ 0 }, y{ 0 };

        v2() = default;
        v2(double _x, double _y);
        v2(const v2 &v) = default;
        v2 &operator= (const v2 &v) = default;

        inline v2 operator* (double d) const;
        inline v2 operator/ (double d) const;
        inline v2 operator+ (double d) const;
        inline v2 operator- (double d) const;
        inline v2 operator+ (const v2 &v) const;
        inline v2 operator- (const v2 &v) const;
        inline v2 &operator+= (const v2 &v);
        inline v2 &operator-= (const v2 &v);
        inline v2 operator- () const;
        friend inline v2 operator* (double d, const v2 &v);

        double cross(const v2 &v) const;            // ���
        double dot(const v2 &v) const;              // ���

        double magnitude() const;                   // �����ĳ���
        double magnitudeSquare() const;             //ƽ����

        v2 normalize() const;                       // ��������
        v2 normal() const;                          // ��������
        v2 N() const;

        bool zero(double d) const;                  // �ж����������Ƿ�Ϊ0

        v2 rotate(double theta) const;              // ��ת
    };

    //��ά����
    struct m2
    {
        double x1{ 1 }, y1{ 0 }, x2{ 0 }, y2{ 1 };

        m2() = default;
        m2(double _x1, double _y1, double _x2, double _y2);
        m2(const m2 &m) = default;
        m2 &operator= (const m2 &m) = default;
        m2(double d);

        m2 operator+ (const m2 &m) const;           //�������
        v2 operator* (const v2 &v) const;           //�������
        m2 operator* (double d) const;              //����
        friend m2 operator* (double d, const m2 &m);

        const m2 rotate(double theta);              //������ת����
        v2 rotate(const v2 &v)const;
        double det() const;                         //����ʽ��ֵ
        m2 inv() const;                             //����ʽ����
    };

    //����������
    struct doubleInv
    {
        double value{ 0 };                          //ֵ
        double inv{ 0 };                            //����

        explicit doubleInv(double v);                //��������ʾ���ù��캯����
        void set(double v);
    };
}

#endif