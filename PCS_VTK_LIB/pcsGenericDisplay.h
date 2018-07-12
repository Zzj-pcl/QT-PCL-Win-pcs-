#ifndef PCSGENERICDISPLAY_H
#define PCSGENERICDISPLAY_H

//Qt
#include <QImage>
#include <QString>
#include <QFont>

class pcsGenericDisplay
{
public:
	virtual void redraw() = 0;
	virtual void toBeRefreshed() = 0;
	virtual void refresh() = 0;
	virtual void invalidateViewport() = 0;
	virtual unsigned getTexture(const QImage& image) = 0;
	virtual void releaseTexture(unsigned texID) = 0;
	virtual QFont getTextDisplayFont() const = 0;

	virtual bool supportVTKVersion(unsigned VTKVersionFlag) = 0;

	enum TextAlign {
		ALIGN_HLEFT = 1,
		ALIGN_HMIDDLE = 2,
		ALIGN_HRIGHT = 4,
		ALIGN_VTOP = 8,
		ALIGN_VMIDDLE = 16,
		ALIGN_VBOTTOM = 32,
		ALIGN_DEFAULT = 1 | 8
	};

	virtual void displayText(QString text,
								int x,
								int y,
								unsigned char align = ALIGN_DEFAULT,
								float bkgAlpha = 0,
								const unsigned char* rgbColor = 0,
								const QFont* font = 0) = 0;

};

#endif // PCSGENERICDISPLAY_H
