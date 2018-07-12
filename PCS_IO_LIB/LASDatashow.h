#ifndef LASDATASHOW_H
#define LASDATASHOW_H

#include <QWidget>
#include "ui_LASDatashow.h"

class LASDatashow : public QWidget, public Ui::LASDatashow
{
	Q_OBJECT

public:
	LASDatashow(QWidget *parent = 0);
	~LASDatashow();
};

#endif // LASDATASHOW_H
