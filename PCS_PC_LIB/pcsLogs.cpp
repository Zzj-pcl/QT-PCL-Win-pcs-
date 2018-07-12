#include "pcsLogs.h"

#include <assert.h>

static char buffer[4096]; //格式字符串生成缓冲区
static pcsLogs* instance = 0;

pcsLogs* pcsLogs::TheInstance()
{
	return instance;
}

void pcsLogs::RegisterInstance(pcsLogs* LogInstance)
{
	instance = LogInstance;
}

bool pcsLogs::Print(const char* format, ...)
{
	if (instance)
	{
		va_list args;
		va_start(args, format);
		_vsnprintf(buffer, (size_t)4096, format, args);
		va_end(args);
		instance->DisplayMessage(QString(buffer), LOG_Standard);
	}
	return true;
}

bool pcsLogs::Print(const QString& message)
{
	if (instance)
		instance->DisplayMessage(message, LOG_Standard);
	return true;
}

bool pcsLogs::Warning(const char* format, ...)
{
	if (instance)
	{
		va_list args;
		va_start(args, format);
		_vsnprintf(buffer, (size_t)4096, format, args);
		va_end(args);
		instance->DisplayMessage(QString(buffer), LOG_Warning);
	}
	return false;
}

bool pcsLogs::Warning(const QString& message)
{
	if (instance)
		instance->DisplayMessage(message, LOG_Warning);
	return false;
}

bool pcsLogs::Error(const char* format, ...)
{
	if (instance)
	{
		va_list args;
		va_start(args, format);
		_vsnprintf(buffer, (size_t)4096, format, args);
		va_end(args);
		instance->DisplayMessage(QString(buffer), LOG_Error);
	}
	return false;
}

bool pcsLogs::Error(const QString& message)
{
	if (instance)
		instance->DisplayMessage(message, LOG_Error);
	return false;
}

bool pcsLogs::PrintDebug(const char* format, ...)
{
	if (instance)
	{
		va_list args;
		va_start(args, format);
		_vsnprintf(buffer, (size_t)4096, format, args);
		va_end(args);
		instance->DisplayMessage(QString(buffer), LOG_Standard_Debug);
	}
	return true;
}

bool pcsLogs::PrintDebug(const QString& message)
{
	if (instance)
		instance->DisplayMessage(message, LOG_Standard_Debug);
	return true;
}

bool pcsLogs::WarningDebug(const char* format, ...)
{
	if (instance)
	{
		va_list args;
		va_start(args, format);
		_vsnprintf(buffer, (size_t)4096, format, args);
		va_end(args);
		instance->DisplayMessage(QString(buffer), LOG_Warning_Debug);
	}
	return false;
}

bool pcsLogs::WarningDebug(const QString& message)
{
	if (instance)
		instance->DisplayMessage(message, LOG_Warning_Debug);
	return false;
}

bool pcsLogs::ErrorDebug(const char* format, ...)
{
	if (instance)
	{
		va_list args;
		va_start(args, format);
		_vsnprintf(buffer, (size_t)4096, format, args);
		va_end(args);
		instance->DisplayMessage(QString(buffer), LOG_Error_Debug);
	}
	return false;
}

bool pcsLogs::ErrorDebug(const QString& message)
{
	if (instance)
		instance->DisplayMessage(message, LOG_Error_Debug);
	return false;
}
