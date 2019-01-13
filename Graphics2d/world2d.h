#ifndef GRAPHICS2D_WORLD2D_H
#define GRAPHICS2D_WORLD2D_H

class Helper2d;

namespace lib2d
{
	static const auto inf = std::numeric_limits<double>::infinity();

	//二维向量
	struct v2
	{
		double x{ 0 }, y{ 0 };

		v2() = default;
		v2(double _x, double _y);
		v2(const v2 &v) = default;
		v2 &operator= (const v2 &v) = default;

		v2 operator* (double d) const;
		v2 operator/ (double d) const;
		v2 operator+ (double d) const;
		v2 operator- (double d) const;
		v2 operator+ (const v2 &v) const;
		v2 operator- (const v2 &v) const;
		v2 &operator+= (const v2 &v);
		v2 &operator-= (const v2 &v);
		v2 operator- () const;
		friend v2 operator* (double d, const v2 &v);

		double cross(const v2 &v) const;		// 叉乘
		double dot(const v2 &v) const;			// 点乘

		double magnitude() const;				// 向量的长度
		double magnitudeSquare() const;

		v2 normalize() const;					// 方向向量
		v2 normal() const;						// 法线向量
		v2 N() const;

		bool zero(double d) const;				// 判断向量长度是否为0

		v2 rotate(double theta) const;			// 旋转
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

		m2 operator+ (const m2 &m) const;		//矩阵相加
		v2 operator* (const v2 &v) const;		//矩阵相乘
		m2 operator* (double d) const;			//数乘
		friend m2 operator* (double d, const m2 &m);

		const m2 rotate(double theta);			//构造旋转矩阵
		v2 rotate(const v2 &v)const;
		double det() const;						//行列式的值
		m2 inv() const;							//行列式求逆
	};

	class body2d
	{
	public:
		using ptr = std::shared_ptr<body2d>;

		body2d(uint16_t _id, double _mass) : id(_id), mass(_mass) {}
		body2d(const body2d &) = delete;				            //禁止拷贝
		body2d &operator= (const body2d &) = delete;	            //禁止赋值

        virtual void drag(const v2 &pt, const v2 & offset) = 0;     //拖拽物体
        virtual bool contains(const v2 & pt) = 0;                   //计算碰撞

		virtual void update(int) = 0;
		virtual void draw(Helper2d * help2d) = 0;

		uint16_t id{ 0 };	    //ID
		double mass{ 0 };	    //质量
		v2 pos;				    //世界坐标
		v2 center;			    //重心
		v2 V;				    //速度
        double angle{ 0 };	    //角度
        double angleV{ 0 };	    //角速度
        double inertia{ 0 };    //转动惯量
		m2 R;				    //旋转矩阵
		v2 F;				    //受力
	};

	class polygon2d : public body2d
	{
	public:
		polygon2d(uint16_t _id, double _mass, const std::vector<v2> &_vertices);

		double calsPolygonArea()		//计算多边形面积
		{
            double area = 0;
            auto size = vertices.size();
            for (size_t i = 0; i < size; ++i)
            {
                auto j = (i + 1) % size;
                area += vertices[i].cross(vertices[j]);     //叉乘计算面积
            }
            return area / 2;
		}

		v2 calsPolygonCentroid()		            //计算多边形重心
		{
            v2 gc;
            auto size = vertices.size();
            for (size_t i = 0; i < size; ++i)
            {
                auto j = (i + 1) % size;                                            //多边形重心 = （各三角形重心 * 其面积） / 总面积
                gc += (vertices[i] + vertices[j]) * vertices[i].cross(vertices[j]); //三角形重心 = 两向量之和 / 3
            }
            return gc / 6.0 / calsPolygonArea();
		}

		double calsPolygonInertia()		            //计算多边形转动变量
		{
            double acc0, acc1;
            auto size = vertices.size();
            for (size_t i = 0; i < size; ++i)       //转动惯量 = m / 6 * Σ((Pn+1 x Pn)(Pn+1·Pn+1 + Pn+1·Pn + Pn·Pn))/（Σ(Pn+1 x Pn)）
            {
                auto a = vertices[i], b = vertices[(i + 1) % size];
                auto _cross = std::abs(a.cross(b));
                acc0 += _cross * (a.dot(a) + b.dot(b) + a.dot(b));
                acc1 += _cross;
            }
            return mass * acc0 / 6 / acc1;
		}

		void calsPolygonBounds()		            //计算多边形外包矩阵
		{
            boundMin = boundMax = verticesWorld[0];
            for (size_t i = 1; i < verticesWorld.size(); ++i)
            {
                boundMin.x = std::min(boundMin.x, verticesWorld[i].x);
                boundMin.y = std::min(boundMin.y, verticesWorld[i].y);
                boundMax.x = std::min(boundMax.x, verticesWorld[i].x);
                boundMax.y = std::min(boundMax.y, verticesWorld[i].y);
            }
		}

		bool containsInBound(const v2 & pt)			//判断点是否在多边形外包矩阵内
		{
            return boundMin.x < pt.x &&
                boundMax.x > pt.x &&
                boundMin.y < pt.y &&
                boundMax.y > pt.y;
		}

		bool containsInPolygon(const v2 & pt)		//判断点是否在多边形内
		{
            const auto size = verticesWorld.size();
            if (size < 3) return false;
            if ((pt - verticesWorld[0]).cross(verticesWorld[1] - verticesWorld[0]) > 0)
                return false;
            if ((pt - verticesWorld[0]).cross(verticesWorld[size - 1] - verticesWorld[0]) < 0)
                return false;

            int i = 2, j = size - 1;
            int line = -1;

            while (i <= j)
            {
                int mid = (i + j) >> 1;

                if ((pt-verticesWorld[0]).cross(verticesWorld[mid] - verticesWorld[0]) > 0)
                {
                    line = mid;
                    j = mid - 1;
                }
                else
                {
                    i = mid + 1;
                }
		    }
            return (pt - verticesWorld[line - 1]).cross(verticesWorld[line] - verticesWorld[line - 1]) < 0;
        }

        bool contains(const v2 & pt) override
        {
            return containsInBound(pt) && containsInPolygon(pt);    //先判断是否在外包框内，再判断是否在多边形内
        }

        void drag(const v2 & pt, const v2 & offset) override
        {
            V += 1.0 / mass * offset;                                       //速度 += 力矩 / 质量
            angleV += 1.0 / inertia * (pt - pos - center).cross(offset);    //角速度 += 转动半径 x 力矩 /角速度
        }

		void init();

		void update(int n) override;
		void draw(Helper2d * helper) override;

		std::vector<v2> vertices;			//多边形顶点（本地坐标）
		std::vector<v2> verticesWorld;		//多边形顶点（世界坐标）
		v2 boundMin, boundMax;				//外包矩阵
	};

	class world2d
	{
	public:
		world2d() = default;
		~world2d() = default;

		polygon2d * makePolygon(double mass, const std::vector<v2> &vertices, const v2 &pos);
		polygon2d * makeRect(double mass, double w, double h, const v2 &pos);

		void step(Helper2d * helper);
		void clear();
		void init();

        body2d * findBody(const v2 & pos)
        {
            auto body = std::find_if(bodies.begin(), bodies.end(), [&](auto & b)
            {
                return b->contains(pos);
            });
            if (body != bodies.end())
            {
                return (*body).get();
            }
            return nullptr;
        }

        void offset(const v2 & pt, const v2 & offset)
        {
            auto body = findBody(pt);
            if (body)
            {
                body->drag(pt, offset);
            }

        }

        void mouse(const v2 & pt, bool down)        //鼠标坐标捕获
        {
            if (true == down)
            {
                mouse_drag = true;
                global_drag_offset.x = 0;
                global_drag_offset.y = 0;
            }
            else
            {
                mouse_drag = false;
                global_drag_offset.x = (pt.x - global_drag_offset.x);
                global_drag_offset.y = (pt.y - global_drag_offset.y);
                offset(global_drag, global_drag_offset);
                global_drag.x = pt.x;
                global_drag.y = pt.y;
            }
        }

        void motion(const v2 & pt)                  //鼠标移动矢量
        {
            if (mouse_drag)
            {
                global_drag_offset.x = (pt.x - global_drag_offset.x);
                global_drag_offset.y = (pt.x - global_drag_offset.y);
            }
        }


		void setHelper(Helper2d * helper);

	public:
		static QTime last_clock;
		static double dt;
		static double dt_inv;

	private:
		Helper2d * helper;

        bool mouse_drag{ false };
        v2 global_drag;
        v2 global_drag_offset;

		std::vector<body2d::ptr> bodies;
        uint16_t global_id = 1;

	};
}

#endif