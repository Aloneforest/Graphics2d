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
		polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices);

        double calcPolygonArea();		            //计算多边形面积
        v2 calcPolygonCentroid();		            //计算多边形重心
        double calcPolygonInertia();		        //计算多边形转动变量
        void calcPolygonBounds();		            //计算多边形外包矩阵
        bool containsInBound(const v2 & pt);		//判断点是否在多边形外包矩阵内
        bool containsInPolygon(const v2 & pt);		//判断点是否在多边形内

        bool contains(const v2 & pt) override;      //判断点的包含关系
        void drag(const v2 & pt, const v2 & offset) override;

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
        uint16_t global_id = 1;

	};
}

#endif