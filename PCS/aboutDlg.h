#ifndef ABOUTDLG_H
#define ABOUTDLG_H

#include <QDialog>
#include "ui_aboutDlg.h"

class aboutDlg : public QDialog, public Ui::aboutDlg
{
	Q_OBJECT

public:
	aboutDlg(QWidget *parent = 0);
	~aboutDlg();
};

#endif // ABOUTDLG_H
