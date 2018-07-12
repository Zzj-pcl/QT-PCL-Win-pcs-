#ifndef ONPCS_H
#define ONPCS_H

#include <QWidget>
#include "ui_OnPCS.h"


class OnPCS : public QWidget, public Ui::OnPCS
{
	Q_OBJECT

public:
	OnPCS(QWidget *parent = 0);
	~OnPCS();


};

#endif // ONPCS_H
