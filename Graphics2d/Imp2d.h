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

	Helper2d * get_helper();

//public slots:
//	void animate();

protected:
	void paintEvent(QPaintEvent * event) Q_DECL_OVERRIDE;

private:
	Helper2d helper;
};