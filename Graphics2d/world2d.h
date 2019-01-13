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

		body2d(uint16_t _id, double _mass) : id(_id), mass(_mass) {}
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
		polygon2d(uint16_t _id, double _mass, const std::vector<v2> &_vertices);

		double calsPolygonArea()		//�����������
		{
            double area = 0;
            auto size = vertices.size();
            for (size_t i = 0; i < size; ++i)
            {
                auto j = (i + 1) % size;
                area += vertices[i].cross(vertices[j]);     //��˼������
            }
            return area / 2;
		}

		v2 calsPolygonCentroid()		            //������������
		{
            v2 gc;
            auto size = vertices.size();
            for (size_t i = 0; i < size; ++i)
            {
                auto j = (i + 1) % size;                                            //��������� = �������������� * ������� / �����
                gc += (vertices[i] + vertices[j]) * vertices[i].cross(vertices[j]); //���������� = ������֮�� / 3
            }
            return gc / 6.0 / calsPolygonArea();
		}

		double calsPolygonInertia()		            //��������ת������
		{
            double acc0, acc1;
            auto size = vertices.size();
            for (size_t i = 0; i < size; ++i)       //ת������ = m / 6 * ��((Pn+1 x Pn)(Pn+1��Pn+1 + Pn+1��Pn + Pn��Pn))/����(Pn+1 x Pn)��
            {
                auto a = vertices[i], b = vertices[(i + 1) % size];
                auto _cross = std::abs(a.cross(b));
                acc0 += _cross * (a.dot(a) + b.dot(b) + a.dot(b));
                acc1 += _cross;
            }
            return mass * acc0 / 6 / acc1;
		}

		void calsPolygonBounds()		            //���������������
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

		bool containsInBound(const v2 & pt)			//�жϵ��Ƿ��ڶ�������������
		{
            return boundMin.x < pt.x &&
                boundMax.x > pt.x &&
                boundMin.y < pt.y &&
                boundMax.y > pt.y;
		}

		bool containsInPolygon(const v2 & pt)		//�жϵ��Ƿ��ڶ������
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
            return containsInBound(pt) && containsInPolygon(pt);    //���ж��Ƿ���������ڣ����ж��Ƿ��ڶ������
        }

        void drag(const v2 & pt, const v2 & offset) override
        {
            V += 1.0 / mass * offset;                                       //�ٶ� += ���� / ����
            angleV += 1.0 / inertia * (pt - pos - center).cross(offset);    //���ٶ� += ת���뾶 x ���� /���ٶ�
        }

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

        void mouse(const v2 & pt, bool down)        //������겶��
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

        void motion(const v2 & pt)                  //����ƶ�ʸ��
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