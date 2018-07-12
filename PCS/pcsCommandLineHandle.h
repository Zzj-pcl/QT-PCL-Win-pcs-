#ifndef PCSCommandLINEHANDLE_H
#define PCSCommandLINEHANDLE_H

//Qt
#include <QString>
#include <QStringList>
#include <vector>

class QDialog;

class pcsCommandLineHandle
{
public:
	static int Parse(int nargs, char** args);

protected:

	bool CommandLoad(QStringList& arguments);
	//bool CommandSubsample(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool CommandCurvature(QStringList& arguments, QDialog* parent = 0);
	bool CommandDensity(QStringList& arguments, QDialog* parent = 0);
	bool CommandApproxDensity(QStringList& arguments, QDialog* parent = 0);
	bool CommandSFGradient(QStringList& arguments, QDialog* parent = 0);
	bool CommandRoughness(QStringList& arguments, QDialog* parent = 0);
	//bool CommandSampleMesh(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool CommandBundler(QStringList& arguments);
	bool CommandDist(QStringList& arguments, bool cloud2meshDist, QDialog* parent = 0);
	bool CommandFilterSFByValue(QStringList& arguments);
	bool CommandMergeClouds(QStringList& arguments);
	//bool CommandStatTest(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool CommandBestFitPlane(QStringList& arguments);
	bool CommandCrop(QStringList& arguments);
	bool CommandCrop2D(QStringList& arguments);
	bool MatchBBCenters(QStringList& arguments);
	bool CommandICP(QStringList& arguments, QDialog* parent = 0);
	bool CommandChangeCloudOutputFormat(QStringList& arguments);
	bool CommandChangeMeshOutputFormat(QStringList& arguments);
	bool SetActiveSF(QStringList& arguments);

private:
	pcsCommandLineHandle();
	~pcsCommandLineHandle();


};

#endif // PCSCommandLINEHANDLE_H
