#include "Imp2d.h"

Imp2d::Imp2d(QWidget *parent) : QOpenGLWidget(parent)
{
	auto timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(1000 / 60);;
}

Imp2d::~Imp2d()
{

}

Helper2d * Imp2d::get_helper()
{
	return &helper;
}

//void Imp2d::animate()
//{
//	update();
//}

void Imp2d::paintEvent(QPaintEvent * event)
{
	auto painterPtr = QPainterPtr(new QPainter());
	painterPtr->begin(this);
	painterPtr->setRenderHint(QPainter::Antialiasing); //启用反走样，告诉QPainter用不同颜色强度绘制边框以减少视觉扭曲，这种扭曲一般
	helper.paint(painterPtr, event->rect());
	painterPtr->end();
}