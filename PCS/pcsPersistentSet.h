#ifndef PCSPERSISTENTSET_H
#define PCSPERSISTENTSET_H

#include <QString>

class PCS;

class pcsPS
{
public:
	static inline const QString Load() { return "Load"; }
	static inline const QString Save() { return "Save"; }
	static inline const QString CurrentPath() { return "CurrentPath"; }
	static inline const QString SelectedInputFilter() { return "SelectedInputFilter"; }
	static inline const QString pcsMainWindow() { return "pcsMainWindow"; }

};

#endif // PCSPERSISTENTSET_H
