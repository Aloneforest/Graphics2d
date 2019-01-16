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

		double cross(const v2 &v) const;		// ���
		double dot(const v2 &v) const;			// ���

		double magnitude() const;				// �����ĳ���
		double magnitudeSquare() const;

		v2 normalize() const;					// ��������
		v2 normal() const;						// ��������
		v2 N() const;

		bool zero(double d) const;				// �ж����������Ƿ�Ϊ0

		v2 rotate(double theta) const;			// ��ת
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

		m2 operator+ (const m2 &m) const;		//�������
		v2 operator* (const v2 &v) const;		//�������
		m2 operator* (double d) const;			//����
		friend m2 operator* (double d, const m2 &m);

		const m2 rotate(double theta);			//������ת����
		v2 rotate(const v2 &v)const;
		double det() const;						//����ʽ��ֵ
		m2 inv() const;							//����ʽ����
	};

	class body2d
	{
	public:
		using ptr = std::shared_ptr<body2d>;

		body2d(uint16_t _id, double _mass, v2 _pos) : id(_id), mass(_mass), pos(_pos) {}
		body2d(const body2d &) = delete;				            //��ֹ����
		body2d &operator= (const body2d &) = delete;	            //��ֹ��ֵ

        virtual void drag(const v2 &pt, const v2 & offset) = 0;     //��ק����ʩ������
        virtual bool contains(const v2 & pt) = 0;                   //�жϵ�İ�����ϵ

		virtual void update(const v2, int) = 0;                               //����״̬
		virtual void draw(Helper2d * help2d) = 0;                   //��ͼ

        int collNum;            //��ײ����
		uint16_t id{ 0 };	    //ID
		double mass{ 0 };	    //����
		v2 pos;				    //��������
		v2 center;			    //����
		v2 V;				    //�ٶ�
        double angle{ 0 };	    //�Ƕ�
        double angleV{ 0 };	    //���ٶ�
        double inertia{ 0 };    //ת������
		m2 R;				    //��ת����
		v2 F;				    //����
	};

	class polygon2d : public body2d
	{
	public:
		polygon2d(uint16_t _id, double _mass, v2 _pos, const std::vector<v2> &_vertices);

        double calcPolygonArea();		            //�����������
        v2 calcPolygonCentroid();		            //������������
        double calcPolygonInertia();		        //��������ת������
        void calcPolygonBounds();		            //���������������
        bool containsInBound(const v2 & pt);		//�жϵ��Ƿ��ڶ�������������
        bool containsInPolygon(const v2 & pt);		//�жϵ��Ƿ��ڶ������

        void drag(const v2 & pt, const v2 & offset) override;
        bool contains(const v2 & pt) override;

		void init();
        void setStatic();                 //��̬�����ʼ��

		void update(const v2 gravity, int n) override;
		void draw(Helper2d * helper) override;

		std::vector<v2> vertices;			//����ζ��㣨�������꣩
		std::vector<v2> verticesWorld;		//����ζ��㣨�������꣩
		v2 boundMin, boundMax;				//�������
        bool isStatic;
	};

    struct contact
    {
        v2 pos;
    };

    struct collision
    {
        std::array<contact, 2> contacts;
        std::shared_ptr<body2d> bodyA;  //��ײ����A
        std::shared_ptr<body2d> bodyB;  //��ײ����B
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

        body2d * findBody(const v2 & pos);               //���ҵ�����ͼ��
        void offset(const v2 & pt, const v2 & offset);   //���������ͼ�������任
        void mouse(const v2 & pt, bool down);            //������겶��
        void motion(const v2 & pt);                      //����ƶ�ʸ��

		void setHelper(Helper2d * helper);

	public:
		static QTime last_clock;
		static double dt;
		static double dt_inv;


	private:
		Helper2d * helper;

        bool mouse_drag{ false };       //����϶�
        v2 global_drag;                 //�������
        v2 global_drag_offset;          //����ƶ�ʸ��

		std::vector<body2d::ptr> bodies;
        std::vector<body2d::ptr> static_bodies;

        std::unordered_map<uint32_t, collision> collisions;     //hashmap

        uint16_t global_id = 1;

        v2 gravity{ 0, -0.2 };   //����ϵ��
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

        static bool separatingAxis()   //�������㷨
        {

        }

        static bool boundCollition()   //��������ཻ�ж�
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