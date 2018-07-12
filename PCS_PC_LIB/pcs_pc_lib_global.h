#ifndef PCS_PC_LIB_GLOBAL_H
#define PCS_PC_LIB_GLOBAL_H

#include <QtCore/qglobal.h>

#ifdef PCS_PC_LIB_LIB
# define PCS_PC_LIB_EXPORT Q_DECL_EXPORT
#else
# define PCS_PC_LIB_EXPORT Q_DECL_IMPORT
#endif

#endif // PCS_PC_LIB_GLOBAL_H
