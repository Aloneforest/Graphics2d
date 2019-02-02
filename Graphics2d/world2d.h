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

		virtual void update(const v2, int) = 0;                     //更新状态
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
        void setStatic();                   //静态物体初始化

		void update(const v2 gravity, int n) override;
		void draw(Helper2d * helper) override;

        v2 edge(const size_t idx) const;           //向量|idx+1, idx|

        //v2 & vertexx(size_t idx)
        //{
        //    return verticesWorld[idx % verticesWorld.size()];
        //}

        //size_t index(size_t idx) const
        //{
        //    return idx % verticesWorld.size();
        //}

		std::vector<v2> vertices;			//多边形顶点（本地坐标）
		std::vector<v2> verticesWorld;		//多边形顶点（世界坐标）
		v2 boundMin, boundMax;				//外包矩阵
        bool isStatic;
	};

    struct contact
    {
        v2 pos;                     //位置
        v2 ra, rb;                  //重心到接触点的向量
        double sep{ 0 };            //重叠距离
        double massNormal{ 0 };
        double massTangent{ 0 };
        double bias{ 0 };
        double pn{ 0 };             //法向冲量
        double pt{ 0 };             //切向冲量
        int idxA{ 0 };              //交点
        int idxB{ 0 };
        
        contact (v2 _pos, size_t _index):pos(_pos), idxA(_index), idxB(_index){}

        bool operator == (const contact & other) const
        {
            if (idxA == other.idxA && idxB == other.idxB)
            {
                return true;
            }
            return idxA == other.idxB && idxB == other.idxA;
        }

        bool operator != (const contact & other) const
        {
            return !(*this == other);
        }

    };

    struct collision
    {
        std::vector<contact> contacts;
        body2d::ptr bodyA;  //碰撞物体A     弱指针？？？？？
        body2d::ptr bodyB;  //碰撞物体B
        size_t idxA;    //出现最大间隙的边
        size_t idxB;
        double satA;    //最大间隙长度
        double satB;
        v2 N;           //法线
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
		static QTime lastClock;
		static double dt;
		static double dtInv;


	private:
		Helper2d * helper;

        bool mouse_drag{ false };       //鼠标拖动
        v2 globalDrag;                 //鼠标点击点
        v2 globalDragOffset;          //鼠标移动矢量

		std::vector<body2d::ptr> bodies;
        std::vector<body2d::ptr> static_bodies;

        std::unordered_map<uint32_t, collision> collisions;     //hashmap

        uint16_t globalId = 1;

        v2 gravity{ 0, -0.2 };   //重力系数
	};

    class collisionCalc
    {
    public:
        collisionCalc() = default;
        ~collisionCalc() = default;

        static uint32_t makeId(uint16_t a, uint16_t b);

        static bool separatingAxis(const body2d::ptr &bodyA, const body2d::ptr &bodyB, size_t &idx, double &sat);   //分离轴算法：遍历矩阵A所有边，将矩阵B所有顶点投影到边的法线上，若投影长度最小值为负，则相交
        static bool boundCollition(const body2d::ptr &bodyA, const body2d::ptr &bodyB);                             //外包矩阵相交判定：若两矩阵中心点相隔距离大于两矩阵边长之和的一半，则不相交
        
        static bool solveCollition(collision & c)   //计算碰撞
        {
            if (c.satA < c.satB)
            {
                std::swap(c.bodyA, c.bodyB);
                std::swap(c.idxA, c.idxB);
                std::swap(c.satA, c.satB);
            }

            auto bodyA = std::dynamic_pointer_cast<polygon2d>(c.bodyA);
            auto bodyB = std::dynamic_pointer_cast<polygon2d>(c.bodyB);

            c.N = bodyA->edge(c.idxA).normal();
            //c.idxB = incidentEdge();

            decltype(c.contacts) contacts;          //获取类型

            contacts.emplace_back(bodyB->verticesWorld[c.idxB % bodyB->verticesWorld.size()], c.idxB % bodyB->verticesWorld.size() + 1);
            contacts.emplace_back(bodyB->verticesWorld[c.idxB % bodyB->verticesWorld.size() + 1 ], c.idxB % bodyB->verticesWorld.size() + 1 + 1);
            auto tmp = contacts;

        }

        static void collisionDetection(const body2d::ptr &bodyA, const body2d::ptr &bodyB, world2d &world);
        static void collisionDetection(world2d &world);
    };
}

#endif