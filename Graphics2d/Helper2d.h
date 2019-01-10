#pragma once
#include "stdafx.h"
#include "world2d.h"


class Helper2d : public QObject
{
	Q_OBJECT

public:
	Helper2d();
	~Helper2d();

	void paint(QPainter * painter, QRect eventRect);

	void clear();
	void paint_line(int x1, int y1, int x2, int y2);
	void paint_polygon(const std::vector<lib2d::v2> &v);
	void paint_text(int x, int y, const QString & str);

	QSize get_size();
	QRect get_rect();

	static QPointF world2screen(const lib2d::v2 &v);

private slots:
	void exec(QString &str);

private:
	QSize size;
	QRect rect;
	static QSizeF mid;

	QPainter * painter;
	QBrush background{ QColor(Qt::black) };
	QPen drag{ QColor(Qt::yellow) };
	QFont text{ "Consolas", 15, 40 };

	lib2d::world2d world;
};
