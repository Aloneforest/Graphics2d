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
    void paintPoint(lib2d::v2 pos, const QPen drag);
	void paintLine(lib2d::v2 a, lib2d::v2 b, const QPen drag);
	void paintPolygon(const std::vector<lib2d::v2> &v);
	void paintText(lib2d::v2 v, const QString & str);

    lib2d::world2d & getWorld();
	QSize getSize();
	QRect getRect();

    static QPointF world2screen(const lib2d::v2 & v);       //世界坐标到屏幕坐标 
    static lib2d::v2 screen2world(const QPointF & pt);      //屏幕坐标到世界坐标 世界坐标 [-1,1]

    QPen dragYellow{ QColor(Qt::yellow) };
    QPen dragRed{ QColor(Qt::red) };
    QPen dragWhite{ QColor(Qt::white)};

private slots:
	void exec(QString &str);

private:
	QSize size;
	QRect rect;
	static QSizeF mid;

	QPainter * painter;
	QBrush background{ QColor(Qt::black) };
	QFont text{ "Consolas", 15, 40 };

	lib2d::world2d world;
};
