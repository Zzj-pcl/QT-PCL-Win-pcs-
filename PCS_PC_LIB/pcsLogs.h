#ifndef PCSLOGS_H
#define PCSLOGS_H

#include "pcs_pc_lib_global.h"
#include <string.h>
#include <stdio.h>
#include <QString.h>


//主日志文件
class PCS_PC_LIB_EXPORT pcsLogs
{
public:
	virtual~pcsLogs(){};

	static pcsLogs* TheInstance(); //用于返回静态实例
	static void RegisterInstance(pcsLogs* LogInstance); //记录该实例
	static bool Print(const char *format, ...); //输出格式化消息
	static bool Print(const QString& messge); //字符串输出
	static bool PrintDebug(const char *format, ...); //调试模式输出
	static bool PrintDebug(const QString& message);
	static bool Warning(const char *format, ...); //输出格式化警告模式
	static bool Warning(const QString& message);
	static bool WarningDebug(const char *format, ...); //调试模式警告消息
	static bool WarningDebug(const QString& message);
	static bool Error(const char *format, ...);  //输出格式化错误模式
	static bool Error(const QString& message);
	static bool ErrorDebug(const char *format, ...);
	static bool ErrorDebug(const QString& message);

protected:
	//消息提示级别
	enum MessageLevel
	{
		LOG_Standard = 0, /**< Standard message (Print) **/
		LOG_Standard_Debug = 1, /**< Standard message - debug only (PrintDebug) **/
		LOG_Warning = 2, /**< Warning message (Warning) **/
		LOG_Warning_Debug = 3, /**< Warning message - debug only (WarningDebug) **/
		LOG_Error = 4, /**< Error message (Error) **/
		LOG_Error_Debug = 5, /**< Error message - debug only (ErrorDebug) **/
	};
	//通用消息提示方法 
	void virtual DisplayMessage(const QString& message, MessageLevel level) = 0;
};

#endif // PCSLOGS_H
