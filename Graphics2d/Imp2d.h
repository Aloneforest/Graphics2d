#pragma once
#include "stdafx.h"

#include <QtWidgets\QOpenGlWidget>
#include "Helper2d.h"

class Imp2d : public QOpenGLWidget
{
	Q_OBJECT

public:
	Imp2d(QWidget *parent = NULL);
	~Imp2d();

	Helper2d * getHelper();

//public slots:
//	void animate();

protected:
    void paintEvent(QPaintEvent * event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent * event) Q_DECL_OVERRIDE;      //鼠标点击
    void mouseReleaseEvent(QMouseEvent * event) Q_DECL_OVERRIDE;    //鼠标释放
    void mouseMoveEvent(QMouseEvent * event) Q_DECL_OVERRIDE;       //鼠标移动

private:
	Helper2d helper;
};