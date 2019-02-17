#include "stdafx.h"
#include "Graphics2d.h"

Graphics2d::Graphics2d(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	connect(ui.console, SIGNAL(returnPressed()), this, SLOT(execLispCommand()));
	connect(this, SIGNAL(send_command(QString &)), ui.imp->getHelper(), SLOT(exec(QString &)));
}


void Graphics2d::execLispCommand()
{
	auto code = ui.console->text();
    ui.console->clear();
	if (code.isEmpty())
		return;
	if ("exit" == code)
	{
		close();
		return;
	}
	emit send_command(code);
}