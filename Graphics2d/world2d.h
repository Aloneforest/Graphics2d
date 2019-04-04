#ifndef GRAPHICS2D_WORLD2D_H
#define GRAPHICS2D_WORLD2D_H

class Helper2d;

namespace lib2d
{
    static const auto inf = std::numeric_limits<double>::infinity();

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

    enum body2dType
    {
        POLYGON,                                    //�����
        CIRCLE                                      //Բ
    };

    //-------------------------------------------------

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
        v2 boundMin, boundMax;                      //�������
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

        //double calcPolygonArea();                               //�����������
        //v2 calcPolygonCentroid();                               //������������
        //double calcPolygonInertia(double mass);                 //��������ת������
        //void calcPolygonBounds();                               //���������������
        //bool containsInBound(const v2 & pt);                    //�жϵ��Ƿ��ڶ�������������
        //bool containsInPolygon(const v2 & pt);                  //�жϵ��Ƿ��ڶ������

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

    //----------------------------------------------------------

    //�Ӵ���
    struct contact
    {
        v2 pos;                     //λ��
        v2 ra, rb;                  //���ĵ��Ӵ������������������
        double sep{ 0 };            //�ص�����
        double massNormal{ 0 };
        double massTangent{ 0 };
        double bias{ 0 };
        double pn{ 0 };             //�ۼƷ������
        double pt{ 0 };             //�ۼ��������
        int idxA{ 0 };              //���������ߵ�����+1 ��BΪ����AΪ����
        int idxB{ 0 };
        
        contact (v2 _pos, size_t _index):pos(_pos), idxA(_index), idxB(_index){}

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

    //��������
    enum UpdateType
    {
        INIT_FORCE_AND_TORQUE,                  //��ʼ����������
        CALC_VELOCITY_AND_ANGULAR_VELOCITY,     //�����ٶȺͽ��ٶ�
        CALC_DISPLACEMENT_AND_ANGLE,            //����λ�ƺͽǶ�
        ADD_GRAVITY,                            //�������
        RESET_NET_FORCE                         //���������
    };

    //��Ļ
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

        body2d * findBody(const v2 & pos);               //���ҵ�����ͼ��
        void offset(const v2 & pt, const v2 & offset);   //���������ͼ�������任
        void mouse(const v2 & pt, bool down);            //������겶��
        void motion(const v2 & pt);                      //����ƶ�ʸ��

        void setHelper(Helper2d * helper);

    public:
        static QTime lastClock;
        static double dt;
        static double dtInv;
        static v2 gravity;   //����ϵ��

    private:
        Helper2d * helper;

        bool mouse_drag{ false };       //����϶�
        v2 globalDrag;                  //�������
        v2 globalDragOffset;            //����ƶ�ʸ��

        std::vector<body2d::ptr> bodies;
        std::vector<body2d::ptr> staticBodies;
        std::vector<joint::ptr> joints;

        std::unordered_map<uint32_t, collision> collisions;     //hashmap

        uint16_t globalId = 1;
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
        static size_t clip(std::vector<contact> & out, const std::vector<contact> & in, size_t i, const v2 & p1, const v2 & p2);    //Sutherland-Hodgman������βü���
        static void collisionUpdate(collision & coll, const collision & oldColl);                                                   //��ײ����
        static bool solveCollition(collision & coll);                                                                               //������ײ

        static void collisionDetection(world2d &world);                                                                             //��ײ���
        static void collisionDetection(const body2d::ptr &bodyA, const body2d::ptr &bodyB, world2d &world);                         //����������ײ��⣬������ײ�ṹ

        static void collisionPrepare(collision & coll);                                                                             //��ײ����׼�����������ϵ��
        static void collisionStateUpdate(collision & coll);                                                                         //����������ײ���״̬

        static void drawCollision(Helper2d * helper, const collision & coll);                                                       //������ײ
    };
}

#endif