#include "stdafx.h"

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
	QPainter painter;
	painter.begin(this);
	painter.setRenderHint(QPainter::Antialiasing); //���÷�����������QPainter�ò�ͬ��ɫǿ�Ȼ��Ʊ߿��Լ����Ӿ�Ť��������Ť��һ��
	helper.paint(&painter, event->rect());
	painter.end();
}