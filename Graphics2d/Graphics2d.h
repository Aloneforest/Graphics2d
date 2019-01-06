#pragma once
#include "stdafx.h"

#include <QtWidgets/QMainWindow>
#include "ui_Graphics2d.h"

class Graphics2d : public QMainWindow
{
	Q_OBJECT

public:
	Graphics2d(QWidget *parent = Q_NULLPTR);

signals:
	void send_command(QString &);

private:
	Ui::Graphics2dClass ui;

private slots:
	void execLispCommand();
};