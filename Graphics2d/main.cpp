#include "Graphics2d.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Graphics2d w;
	w.show();
	return a.exec();
}
