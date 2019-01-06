#include "Helper2d.h"

Helper2d::Helper2d()
{

}

Helper2d::~Helper2d()
{

}

void Helper2d::paint(QPainterPtr painterPtr, QRect eventRect)
{
	rect = eventRect;
	size = rect.size();
	this->painterPtr = painterPtr;

	paint_line(rect.topLeft().x(), rect.topLeft().y(), rect.bottomRight().x(), rect.bottomRight().y()); //²âÊÔ
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

void Helper2d::exec(QString & str)
{
	if ("line" == str)
	{

	}
	else if ("text" == str)
	{

	}
	else
	{

	}
}