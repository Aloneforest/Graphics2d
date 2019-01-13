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

        virtual void drag(const v2 &pt, const v2 & offset) = 0;     //��ק����
        virtual bool contains(const v2 & pt) = 0;                   //������ײ

		virtual void update(int) = 0;
		virtual void draw(Helper2d * help2d) = 0;

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

        bool contains(const v2 & pt) override;      //�жϵ�İ�����ϵ
        void drag(const v2 & pt, const v2 & offset) override;

		void init();

		void update(int n) override;
		void draw(Helper2d * helper) override;

		std::vector<v2> vertices;			//����ζ��㣨�������꣩
		std::vector<v2> verticesWorld;		//����ζ��㣨�������꣩
		v2 boundMin, boundMax;				//�������
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
        uint16_t global_id = 1;

	};
}

#endif