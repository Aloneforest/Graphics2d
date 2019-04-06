#ifndef GRAPHICS2D_MATH2D_H
#define GRAPHICS2D_MATH2D_H

class Helper2d;

namespace lib2d
{
    //二维向量
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

        double cross(const v2 &v) const;            // 叉乘
        double dot(const v2 &v) const;              // 点乘

        double magnitude() const;                   // 向量的长度
        double magnitudeSquare() const;             //平方和

        v2 normalize() const;                       // 方向向量
        v2 normal() const;                          // 法线向量
        v2 N() const;

        bool zero(double d) const;                  // 判断向量长度是否为0

        v2 rotate(double theta) const;              // 旋转
    };

    //二维矩阵
    struct m2
    {
        double x1{ 1 }, y1{ 0 }, x2{ 0 }, y2{ 1 };

        m2() = default;
        m2(double _x1, double _y1, double _x2, double _y2);
        m2(const m2 &m) = default;
        m2 &operator= (const m2 &m) = default;
        m2(double d);

        m2 operator+ (const m2 &m) const;           //矩阵相加
        v2 operator* (const v2 &v) const;           //矩阵相乘
        m2 operator* (double d) const;              //数乘
        friend m2 operator* (double d, const m2 &m);

        const m2 rotate(double theta);              //构造旋转矩阵
        v2 rotate(const v2 &v)const;
        double det() const;                         //行列式的值
        m2 inv() const;                             //行列式求逆
    };

    //浮点数倒数
    struct doubleInv
    {
        double value{ 0 };                          //值
        double inv{ 0 };                            //倒数

        explicit doubleInv(double v);                //（必须显示调用构造函数）
        void set(double v);
    };
}

#endif