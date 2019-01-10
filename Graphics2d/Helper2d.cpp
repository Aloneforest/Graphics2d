#include "stdafx.h"

#include "Helper2d.h"

QSizeF Helper2d::mid;

Helper2d::Helper2d()
{
	world.set_helper(this);
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

void Helper2d::paint_line(int x1, int y1, int x2, int y2)
{
	painter->setPen(drag);
	painter->drawLine(QLineF(QPointF(x1, y1), QPointF(x2, y2)));
}

void Helper2d::paint_polygon(const std::vector<lib2d::v2> &v)
{
	static std::vector<QPointF> vp;
	vp.clear();
	vp.resize(v.size());
	std::transform(v.begin(), v.end(), vp.begin(), world2screen);	//第四个参数需要声明为静态函数
	painter->setPen(drag);
	painter->drawPolygon(vp.data(), (int)vp.size());
}

void Helper2d::paint_text(int x, int y, const QString & str)
{
	painter->setPen(drag);
	painter->setFont(text);
	painter->drawText(x, y, str);
}

QSize Helper2d::get_size()
{
	return size;
}

QRect Helper2d::get_rect()
{
	return rect;
}

QPointF Helper2d::world2screen(const lib2d::v2 & v)
{
	return QPointF((v.x + 1.0) * mid.width(), -v.y * mid.width() + mid.height());
}

void Helper2d::exec(QString & str)
{

}