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

		body2d(uint16_t _id, double _mass, v2 _pos) : id(_id), mass(_mass), pos(_pos) {}
		body2d(const body2d &) = delete;				            //禁止拷贝
		body2d &operator= (const body2d &) = delete;	            //禁止赋值

        virtual void drag(const v2 &pt, const v2 & offset) = 0;     //拖拽物体施加力矩
        virtual bool contains(const v2 & pt) = 0;                   //判断点的包含关系

		virtual void update(const v2, int) = 0;                               //更新状态
		virtual void draw(Helper2d * help2d) = 0;                   //画图

        int collNum;            //碰撞次数
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
		polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices);

        double calcPolygonArea();		            //计算多边形面积
        v2 calcPolygonCentroid();		            //计算多边形重心
        double calcPolygonInertia();		        //计算多边形转动变量
        void calcPolygonBounds();		            //计算多边形外包矩阵
        bool containsInBound(const v2 & pt);		//判断点是否在多边形外包矩阵内
        bool containsInPolygon(const v2 & pt);		//判断点是否在多边形内

        void drag(const v2 & pt, const v2 & offset) override;
        bool contains(const v2 & pt) override;

		void init();
        void setStatic();                 //静态物体初始化

		void update(const v2 gravity, int n) override;
		void draw(Helper2d * helper) override;

		std::vector<v2> vertices;			//多边形顶点（本地坐标）
		std::vector<v2> verticesWorld;		//多边形顶点（世界坐标）
		v2 boundMin, boundMax;				//外包矩阵
        bool isStatic;
	};

    struct contact
    {
        v2 pos;
    };

    struct collision
    {
        std::array<contact, 2> contacts;
        std::shared_ptr<body2d> bodyA;  //碰撞物体A
        std::shared_ptr<body2d> bodyB;  //碰撞物体B
        size_t idxA;
        size_t idxB;
    };

	class world2d
	{
        friend class collisionCalc;
	public:
		world2d() = default;
		~world2d() = default;

		polygon2d * makePolygon(const double mass, const std::vector<v2> &vertices, const v2 &pos, const bool statics);
		polygon2d * makeRect(const double mass, double w, double h, const v2 &pos, const bool statics);

		void step(Helper2d * helper);
		void clear();
		void init();

        body2d * findBody(const v2 & pos);               //查找点所在图形
        void offset(const v2 & pt, const v2 & offset);   //计算点所在图形受力变换
        void mouse(const v2 & pt, bool down);            //鼠标坐标捕获
        void motion(const v2 & pt);                      //鼠标移动矢量

		void setHelper(Helper2d * helper);

	public:
		static QTime last_clock;
		static double dt;
		static double dt_inv;


	private:
		Helper2d * helper;

        bool mouse_drag{ false };       //鼠标拖动
        v2 global_drag;                 //鼠标点击点
        v2 global_drag_offset;          //鼠标移动矢量

		std::vector<body2d::ptr> bodies;
        std::vector<body2d::ptr> static_bodies;

        std::unordered_map<uint32_t, collision> collisions;     //hashmap

        uint16_t global_id = 1;

        v2 gravity{ 0, -0.2 };   //重力系数
	};

    class collisionCalc
    {
    public:
        collisionCalc() = default;
        ~collisionCalc() = default;

        static uint32_t makeId(uint16_t a, uint16_t b)
        {
            return std::min(a, b) << 16 | std::max(a, b);
        }

        static bool separatingAxis()   //分离轴算法
        {

        }

        static bool boundCollition()   //外包矩阵相交判定
        {

        }

        static void collisionDetection(const body2d::ptr &bodyA, const body2d::ptr &bodyB, world2d &world)
        {
            double satA, satB;
            size_t idxA, idxB;
            auto id = makeId(bodyA->id, bodyB->id);

            if (!boundCollition() || separatingAxis() || separatingAxis())
            {
                auto prev = world.collisions.find(id);
                if (prev != world.collisions.end())
                {
                    bodyA->collNum--;
                    bodyB->collNum--;
                }
                return;
            }

            auto prev = world.collisions.find(id);
            if (prev == world.collisions.end())
            {
                collision c;
                c.bodyA = bodyA;
                c.bodyB = bodyB;
                c.idxA = idxA;
                c.idxB = idxB;
                world.collisions.insert(std::make_pair(id, c));
                bodyA->collNum++;
                bodyB->collNum++;
            }
            else
            {

            }
        }

        static void collisionDetection(world2d &world)
        {
            auto size = world.bodies.size();
            for (size_t i = 0; i < size; ++i)
            {
                for (size_t j = 0; j < size; ++j)
                {
                    collisionDetection(world.bodies[i], world.bodies[j], world);
                }
                for (auto &body : world.static_bodies)
                {
                    collisionDetection(world.bodies[i], body, world);
                }
            }
            return;
        }
    };
}

#endif