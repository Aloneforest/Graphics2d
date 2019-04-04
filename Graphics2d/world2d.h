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

    enum body2dType
    {
        POLYGON,                                    //多边形
        CIRCLE                                      //圆
    };

    //-------------------------------------------------

    class body2d
    {
    public:
        using ptr = std::shared_ptr<body2d>;

        body2d(uint16_t _id, double _mass, v2 _pos) : id(_id), mass(_mass), pos(_pos) {}
        body2d(const body2d &) = delete;                            //禁止拷贝
        body2d &operator= (const body2d &) = delete;                //禁止赋值

        virtual void drag(const v2 & pt, const v2 & offset) = 0;     //拖拽物体施加力矩
        virtual bool contains(const v2 & pt) = 0;                   //判断点的包含关系

        virtual void impulse(const v2 & p, const v2 & r) = 0;       //受力更新，计算力矩
        virtual body2dType type() const = 0;                        //返回刚体类型

        virtual v2 min() const = 0;
        virtual v2 max() const = 0;

        virtual void update(int) = 0;                               //更新状态
        virtual void draw(Helper2d * helper) = 0;                   //画图

        virtual v2 edge(const size_t idx) const = 0;

        v2 rotate(const v2 &v) const;                               //返回向量v经过angle角度旋转后的向量
        v2 world() const;                                           //返回刚体重心世界坐标

        bool isStatic{ false };
        int collNum{ 0 };           //碰撞次数
        uint16_t id{ 0 };           //ID
        doubleInv mass{ 0 };        //质量
        v2 pos;                     //世界坐标
        v2 center;                  //重心
        v2 V;                       //速度
        double angle{ 0 };          //角度
        double angleV{ 0 };         //角速度
        doubleInv inertia{ 0 };     //转动惯量
        double f{ 1 };              //滑动/静摩擦系数
        m2 R;                       //旋转矩阵
        v2 F;                       //受力
        v2 Fa;                      //受力（合力）
        double M{ 0 };              //力矩
        v2 boundMin, boundMax;                      //外包矩阵
        std::vector<v2> vertices;                   //多边形顶点（本地坐标）
        std::vector<v2> verticesWorld;              //多边形顶点（世界坐标）
    };

    class polygon2d : public body2d
    {
    public:
        polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices);

        double calcPolygonArea();                               //计算多边形面积
        v2 calcPolygonCentroid();                               //计算多边形重心
        double calcPolygonInertia(double mass);                 //计算多边形转动变量
        void calcPolygonBounds();                               //计算多边形外包矩阵
        bool containsInBound(const v2 & pt);                    //判断点是否在多边形外包矩阵内
        bool containsInPolygon(const v2 & pt);                  //判断点是否在多边形内

        void drag(const v2 & pt, const v2 & offset) override;   //拖拽物体
        bool contains(const v2 & pt) override;                  //判断相交

        void init(const std::vector<v2> &_vertices);
        void setStatic();                                       //静态物体初始化
        void impulse(const v2 & p, const v2 & r) override;      //根据动量更新受力和力矩
        body2dType type() const override;

        v2 min() const override;
        v2 max() const override;

        void update(int n) override;
        void draw(Helper2d * helper) override;

        v2 edge(const size_t idx) const override;                        //向量|idx+1, idx|

    };

    class circle2d : public body2d
    {
    public:
        circle2d(uint16_t _id, double _mass, v2 _pos, double _r);

        //double calcPolygonArea();                               //计算多边形面积
        //v2 calcPolygonCentroid();                               //计算多边形重心
        //double calcPolygonInertia(double mass);                 //计算多边形转动变量
        //void calcPolygonBounds();                               //计算多边形外包矩阵
        //bool containsInBound(const v2 & pt);                    //判断点是否在多边形外包矩阵内
        //bool containsInPolygon(const v2 & pt);                  //判断点是否在多边形内

        void drag(const v2 & pt, const v2 & offset) override;   //拖拽物体
        bool contains(const v2 & pt) override;                  //判断相交

        void init();
        void setStatic();                                       //静态物体初始化
        void impulse(const v2 & p, const v2 & r) override;      //根据动量更新受力和力矩
        body2dType type() const override;

        v2 min() const override;
        v2 max() const override;

        void update(int n) override;
        void draw(Helper2d * helper) override;

        v2 edge(const size_t idx) const override;                        //向量|idx+1, idx|

        double r;
    };

    //关节：两个刚体间关系基类
    class joint
    {
    public:
        using ptr = std::shared_ptr<joint>;

        joint(body2d::ptr &_a, body2d::ptr &_b) : a(_a), b(_b) {}
        joint(const body2d &) = delete;
        joint &operator= (const joint &) = delete;

        virtual void prepare() = 0;                             //预处理
        virtual void update() = 0;                              //更新
        virtual void draw(Helper2d * helper) = 0;               //绘制

        body2d::ptr a;
        body2d::ptr b;
    };

    //旋转关节
    class revoluteJoint : public joint
    {
    public:
        revoluteJoint(body2d::ptr &_a, body2d::ptr &_b, const v2 & _anchor);
        revoluteJoint(const revoluteJoint &) = delete;
        revoluteJoint &operator= (const revoluteJoint &) = delete;

        void prepare() override;
        void update() override;
        void draw(Helper2d * helper) override;      //绘制旋转半径

        v2 worldAnchorA() const;                    //物体A锚点世界坐标
        v2 worldAnchorB() const;                    //物体B锚点世界坐标

        v2 anchor;                  //锚点的世界坐标
        v2 localAchorA;             //锚点相对物体A的本地坐标
        v2 localAchorB;             //锚点相对物体A的本地坐标
        v2 ra;                      //物体A定轴旋转半径（锚点当前相对与物体A重心所在坐标）
        v2 rb;                      //物体B定轴旋转半径
        m2 mass;                    //质量矩阵
        v2 p;                       //冲量
        v2 pAcc;                    //冲量累计
        v2 bias;                    //补偿
    };

    //----------------------------------------------------------

    //接触点
    struct contact
    {
        v2 pos;                     //位置
        v2 ra, rb;                  //重心到接触点的向量，计算力矩
        double sep{ 0 };            //重叠距离
        double massNormal{ 0 };
        double massTangent{ 0 };
        double bias{ 0 };
        double pn{ 0 };             //累计法向冲量
        double pt{ 0 };             //累计切向冲量
        int idxA{ 0 };              //交点所属边的索引+1 （B为正，A为负）
        int idxB{ 0 };
        
        contact (v2 _pos, size_t _index):pos(_pos), idxA(_index), idxB(_index){}

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

    //计算类型
    enum UpdateType
    {
        INIT_FORCE_AND_TORQUE,                  //初始化力和力矩
        CALC_VELOCITY_AND_ANGULAR_VELOCITY,     //计算速度和角速度
        CALC_DISPLACEMENT_AND_ANGLE,            //计算位移和角度
        ADD_GRAVITY,                            //添加重力
        RESET_NET_FORCE                         //重设合外力
    };

    //屏幕
    class world2d
    {
        friend class collisionCalc;
    public:
        world2d() = default;
        ~world2d() = default;

        int makePolygon(const double mass, const std::vector<v2> &vertices, const v2 &pos, const bool statics);
        int makeRect(const double mass, double w, double h, const v2 &pos, const bool statics);
        int makeCircle(const double mass, double r, const v2 &pos, const bool statics);
        void world2d::makeRevoluteJoint(body2d::ptr &a, body2d::ptr &b, const v2 &anchor);

        void step(Helper2d * helper);
        void clear();
        void init();
        void scene(int i);
        void makeBound();

        body2d * findBody(const v2 & pos);               //查找点所在图形
        void offset(const v2 & pt, const v2 & offset);   //计算点所在图形受力变换
        void mouse(const v2 & pt, bool down);            //鼠标坐标捕获
        void motion(const v2 & pt);                      //鼠标移动矢量

        void setHelper(Helper2d * helper);

    public:
        static QTime lastClock;
        static double dt;
        static double dtInv;
        static v2 gravity;   //重力系数

    private:
        Helper2d * helper;

        bool mouse_drag{ false };       //鼠标拖动
        v2 globalDrag;                  //鼠标点击点
        v2 globalDragOffset;            //鼠标移动矢量

        std::vector<body2d::ptr> bodies;
        std::vector<body2d::ptr> staticBodies;
        std::vector<joint::ptr> joints;

        std::unordered_map<uint32_t, collision> collisions;     //hashmap

        uint16_t globalId = 1;
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
        static size_t clip(std::vector<contact> & out, const std::vector<contact> & in, size_t i, const v2 & p1, const v2 & p2);    //Sutherland-Hodgman（多边形裁剪）
        static void collisionUpdate(collision & coll, const collision & oldColl);                                                   //碰撞更新
        static bool solveCollition(collision & coll);                                                                               //计算碰撞

        static void collisionDetection(world2d &world);                                                                             //碰撞检测
        static void collisionDetection(const body2d::ptr &bodyA, const body2d::ptr &bodyB, world2d &world);                         //两物体间的碰撞检测，构造碰撞结构

        static void collisionPrepare(collision & coll);                                                                             //碰撞计算准备，计算相关系数
        static void collisionStateUpdate(collision & coll);                                                                         //更新物体碰撞后的状态

        static void drawCollision(Helper2d * helper, const collision & coll);                                                       //绘制碰撞
    };
}

#endif