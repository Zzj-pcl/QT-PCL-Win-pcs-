#include "pcsTimer.h"

#include <QTime>
#include <assert.h>

static pcsSingle<QTime> timer;

void pcsTimer::Init()
{
	if (!timer.instance)
	{
		timer.instance = new QTime();
		timer.instance->start();
	}
}

int pcsTimer::Seconds()
{
	assert(timer.instance && timer.instance->isValid());
	return (timer.instance ? timer.instance->elapsed() / 1000 : 0);
}

int pcsTimer::MilliSeconds()
{
	assert(timer.instance && timer.instance->isValid());
	return (timer.instance ? timer.instance->elapsed() : 0);
}