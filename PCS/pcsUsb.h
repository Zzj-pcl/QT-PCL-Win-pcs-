#ifndef PCSUSB_H
#define PCSUSB_H

#include <string.h>
#include <QDialog>


#define CONST_WAVE_MAX	4
#define CONST_SIZE_MAX	8388608	//8M 字节
#define CONST_USB_SIZE		512*127	 //USBDEV单次读写数据最大值

#define CONST_WRBUF_SIZE	8388608	 //8M 字节
#define CONST_RDBUF_SIZE	8388608	 //8M 字节
#define CONST_PROBUF_SIZE	8388608	 //8M 字节

#define IDM_ABOUTBOX                    0x0010
#define IDD_ABOUTBOX                    100
#define IDS_ABOUTBOX                    101
#define IDD_USBDEVDEMO_DIALOG           102
#define IDR_MAINFRAME                   128
#define IDC_CURSOR_DRAG                 129
#define IDC_CURSOR_PIN                  130
#define IDC_VIEW                        1000
#define IDC_BTN_OPENUSBDEV              1001
#define IDC_BTN_RDUSB                   1002
#define IDC_BTN_CLOSEUSBDEV             1003
#define IDC_BTN_WRUSB                   1004
#define IDC_BTN_TESTUSB                 1005
#define IDC_BTN_ENUMUSBDEV              1006
#define IDC_STATUS                      1008
#define IDC_RAD_BYTE                    1009
#define IDC_RAD_WORD                    1010
#define IDC_BTN_UARTTX                  1011
#define IDC_BTN_UARTRX                  1012
#define IDC_EDIT_UARTTX                 1013
#define IDC_EDIT_UARTRX                 1014
#define IDC_COM_BAUDRATE                1015
#define IDC_BTN_GETUSBTYPE              1016
#define IDC_EDIT_USBRDSIZE              1017
#define IDC_STATIC_CNT                  1018
#define IDC_STATIC_ENUMNUB              1020
#define IDC_EDIT_USBDEVSEL              1022
#define IDC_BTN_USBDEVSEL               1023
#define IDC_STATIC_USBTYPE              1025
#define IDC_BTN_CAP                     1028
#define IDC_STATIC_VIEW                 1029

class pcsUsb : public QDialog
{
public:
	pcsUsb();
	~pcsUsb();


public slots:
	void UsbCheck();
	void GetUsbType();
};

#endif // PCSUSB_H
