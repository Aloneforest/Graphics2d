#pragma once
#include "stdafx.h"


class Helper2d : public QObject
{
	Q_OBJECT

public:
	Helper2d();
	~Helper2d();

	void paint(QPainterPtr painterPtr, QRect eventRect);

	void clear();
	void paint_line(int x1, int y1, int x2, int y2);
	void paint_text(int x, int y, const QString & str);

	QSize get_size();

private slots:
	void exec(QString &str);

private:
	QSize size;
	QRect rect;
	QPainterPtr painterPtr;

	QBrush background{ QColor(Qt::black) };
	QPen drag{ QColor(Qt::yellow) };
	QFont text{ "Consolas", 15, 40 };

};

