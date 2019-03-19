#ifndef CERELINKTHREAD_H
#define CERELINKTHREAD_H

#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include "lsl_cpp.h"
#include "PSMoveClient_CAPI.h"

class PSMoveThread : public QThread
{
    Q_OBJECT

public:
	PSMoveThread(QObject *parent = 0);
    ~PSMoveThread();

    void initPSMS(double srate);                         // Starts the thread. Passes parameters from GUI to OpenVRThread member variables.
    void startStreams(
		QStringList streamDeviceList = QStringList(),
		bool doIMU = true, bool doIMU_raw = true,
		bool doPos = true, bool doPos_raw = true);  // Starts IMU and/or position streams for all devices.

	bool m_bControllerStreamActive = false;

signals:
    void psmsConnected(bool result);              // Emitted after successful PSMS initialization.
    void deviceListUpdated(QStringList deviceList); // Emitted after a new device is detected.
    void outletsStarted(bool result);				// Emitted after LSL outlets are created.

protected:
    void run() override;

private:
    bool connectToPSMS();     // Initialize PSMS. If successful, device scanning will begin.
    void refreshControllerList();   // Scan for devices.
	void acquireControllers();
    bool createOutlets();       // Create the outlets.
	bool pollAndPush();

    QMutex mutex;
    QWaitCondition condition;
    bool abort;
    double m_srate;                                    // Desired pose sampling rate.
    bool m_bGoOutlets;								// Request to start streams has been made.
	bool m_bIMU = true;
	bool m_bIMU_raw = true;
	bool m_bPos = true;
	bool m_bPos_raw = true;
    std::vector<uint32_t> m_deviceIndices;          // List of found devices indices.
    std::vector<uint32_t> m_streamDeviceIndices;    // List of device indices for streams.
    std::vector<lsl::stream_outlet> m_IMUOutlets;
	int m_channelCount_IMU;
    std::vector<lsl::stream_outlet> m_PosOutlets;
	int m_channelCount_Pos;
	uint64_t m_pushCounter;
	double m_startTime;
	std::vector<PSMController *> m_controllerViews;
	std::vector<int> m_lastSeqNums;
};

#endif // CERELINKTHREAD_H
