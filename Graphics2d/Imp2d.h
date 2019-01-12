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
    void mousePressEvent(QMouseEvent * event) Q_DECL_OVERRIDE;      //�����
    void mouseReleaseEvent(QMouseEvent * event) Q_DECL_OVERRIDE;    //����ͷ�
    void mouseMoveEvent(QMouseEvent * event) Q_DECL_OVERRIDE;       //����ƶ�

private:
	Helper2d helper;
};