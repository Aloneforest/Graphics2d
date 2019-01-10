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
		double magnitude_square() const;

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
		using ptr = std::unique_ptr<body2d>;

		body2d(uint16_t _id, double _mass) : id(_id), mass(_mass) {}
		body2d(const body2d &) = delete;				//��ֹ����
		body2d &operator= (const body2d &) = delete;	//��ֹ��ֵ

		virtual void update(int) = 0;
		virtual void draw(Helper2d * help2d) = 0;

		uint16_t id{ 0 };	//ID
		double mass{ 0 };	//����
		v2 pos;				//λ��
		v2 center;			//����
		v2 V;				//�ٶ�
		double angle{ 0 };	//�Ƕ�
		double angleV{ 0 };	//���ٶ�
		m2 R;				//��ת����
		v2 F;				//����
	};

	class polygon2d : public body2d
	{
	public:
		polygon2d(uint16_t _id, double _mass, const std::vector<v2> &_vertices);

		void update(int n) override;
		void draw(Helper2d * helper) override;

		std::vector<v2> vertices;
		std::vector<v2> verticesWorld;
	};

	class world2d
	{
	public:
		world2d() = default;
		~world2d() = default;

		polygon2d * make_polygon(double mass, const std::vector<v2> &vertices, const v2 &pos);
		polygon2d * make_rect(double mass, double w, double h, const v2 &pos);

		void step(Helper2d * helper);
		void clear();
		void init();

		void set_helper(Helper2d * helper);

	public:
		static QTime last_clock;

	private:
		Helper2d * helper;

		std::vector<body2d::ptr> bodies;
		uint16_t global_id = 1;
	};
}

#endif