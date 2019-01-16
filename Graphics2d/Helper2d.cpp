#include "stdafx.h"

#include "Helper2d.h"

QSizeF Helper2d::mid;

Helper2d::Helper2d()
{
	world.setHelper(this);
	world.init();
}

Helper2d::~Helper2d()
{

}

void Helper2d::paint(QPainter * painter, QRect eventRect)
{
	this->rect = eventRect;
	this->size = rect.size();
	this->mid = size * 0.5;
	this->painter = painter;
	world.step(this);
}

void Helper2d::clear()
{
	painter->fillRect(rect, background);
}

void Helper2d::paintLine(lib2d::v2 a, lib2d::v2 b)
{
    painter->setPen(drag);
    auto pointA = world2screen(a);
    auto pointB = world2screen(b);
	painter->drawLine(QLineF(pointA, pointB));
}

void Helper2d::paintPolygon(const std::vector<lib2d::v2> &v)
{
	static std::vector<QPointF> vp;
	vp.clear();
	vp.resize(v.size());
	std::transform(v.begin(), v.end(), vp.begin(), world2screen);	//第四个参数需要声明为静态函数
	painter->setPen(drag);
	painter->drawPolygon(vp.data(), (int)vp.size());
}

void Helper2d::paintText(lib2d::v2 v, const QString & str)
{
	painter->setPen(drag);
	painter->setFont(text);
	painter->drawText(v.x, v.y, str);
}


lib2d::world2d & Helper2d::getWorld()
{
    return world;
}

QSize Helper2d::getSize()
{
	return size;
}

QRect Helper2d::getRect()
{
	return rect;
}

QPointF Helper2d::world2screen(const lib2d::v2 & v)
{
	return QPointF(v.x * mid.width() + mid.width(), -v.y * mid.height() + mid.height());
}

lib2d::v2 Helper2d::screen2world(const QPointF & pt)
{
    return lib2d::v2((pt.x() -  mid.width() ) / mid.width(), (mid.height() - pt.y() ) / mid.height());
}

void Helper2d::exec(QString & str)
{

}