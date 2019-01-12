#include "Helper2d.h"

Helper2d::Helper2d()
{

}

Helper2d::~Helper2d()
{

}

void Helper2d::paint(QPainterPtr painterPtr, QRect eventRect)
{
	this->rect = eventRect;
	this->size = rect.size();
	this->painterPtr = painterPtr;
	world.step(this);
}

void Helper2d::clear()
{
	painterPtr->fillRect(rect, background);
}

void Helper2d::paint_line(int x1, int y1, int x2, int y2)
{
	painterPtr->setPen(drag);
	painterPtr->drawLine(QLineF(QPointF(x1, y1), QPointF(x2, y2)));
}

void Helper2d::paint_text(int x, int y, const QString & str)
{
	painterPtr->setPen(drag);
	painterPtr->setFont(text);
	painterPtr->drawText(x, y, str);
}

QSize Helper2d::get_size()
{
	return size;
}

QRect Helper2d::get_rect()
{
	return rect;
}

void Helper2d::exec(QString & str)
{

}