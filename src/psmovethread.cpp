#include "psmovethread.h"
#include <QDebug>
#include <cmath>

enum runPhase {
	phase_startLink,
	phase_scanForDevices,
	phase_createOutlets,
	phase_transferData,
	phase_shutdown
};

QString GetControllerString(PSMController *pctrl) {
	QString devString = QString::number(pctrl->ControllerID);
	devString.append(":");
	devString.append(pctrl->ControllerState.PSMoveState.DeviceSerial);
	return devString;
}

void PrintData() {
	double clk = lsl::local_clock();
	qDebug() << std::fmod(1000.0 * clk, 1000) << ", ";
}


PSMoveThread::PSMoveThread(QObject *parent)
	: QThread(parent), abort(false), m_srate(lsl::IRREGULAR_RATE), m_bGoOutlets(false),
	  m_pushCounter(0) {
	// Any other initializations
}

PSMoveThread::~PSMoveThread() {
	mutex.lock();
	abort = true;		 // Tell run() loop to stop.
	condition.wakeOne(); // In case thread is sleeping
	mutex.unlock();
	wait();
}

void PSMoveThread::initPSMS(int srate) {
	QMutexLocker locker(&mutex);
	// Set member variables passed in as arguments.
	this->m_srate = srate;

	if (!isRunning()) {
		start(HighPriority);
	} else {
		qDebug() << "PSMThread is already running. Disconnecting...";
		this->abort = true;
	}
}

void PSMoveThread::startStreams(QStringList streamDeviceList,
	bool doIMU, bool doPos,
	bool doPos, bool doPos_raw) {
	// Responds to event on main thread.
	std::vector<uint32_t> newStreamDeviceIndices;
	if (!this->m_bGoOutlets) {
		if (streamDeviceList.length() == 0) {
			// No devices were selected. Stream all devices.
			newStreamDeviceIndices = m_deviceIndices;
		} else {
			for (QStringList::iterator it = streamDeviceList.begin(); it != streamDeviceList.end();
				 ++it) {
				QStringList pieces = it->split(":");
				QString strIx = pieces.value(0);
				newStreamDeviceIndices.push_back(strIx.toInt());
			}
		}
	}
	// let the running thread know that it's time to toggle the outlets.
	this->mutex.lock();
	this->m_bIMU = doIMU;
	this->m_bIMU_raw = doIMU_raw;
	this->m_bPos = doPos;
	this->m_bPos_raw = doPos_raw;
	this->m_bGoOutlets = !this->m_bGoOutlets;
	this->m_streamDeviceIndices = newStreamDeviceIndices;
	this->mutex.unlock();
}

bool PSMoveThread::connectToPSMS() {
	PSMResult res = PSM_Initialize(
		PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, PSM_DEFAULT_TIMEOUT);
	if (res != PSMResult_Success) {
		qDebug() << "Unable to init PSMove Client";
		return false;
	}

	return true;
}

void PSMoveThread::refreshControllerList() {
	auto list = new PSMControllerList;
	PSMResult res = PSM_GetControllerList(list, PSM_DEFAULT_TIMEOUT);
	if (res == PSMResult_Success) {
		QStringList controllerList;
		std::vector<uint32_t> newControllerIndices;
		for (size_t i = 0; i < list->count; i++) {
			PSMController *p_controller = PSM_GetController(list->controller_id[i]);
			newControllerIndices.push_back(p_controller->ControllerID);
			controllerList << GetControllerString(p_controller);
		}

		if (newControllerIndices.size() != m_deviceIndices.size() ||
			newControllerIndices != m_deviceIndices) {
			m_deviceIndices = newControllerIndices;
			emit deviceListUpdated(controllerList);
		}
	}
}

bool PSMoveThread::createOutlets() {
	// Safely copy member variables to local variables.
	this->mutex.lock();
	int desiredSRate = this->m_srate;
	std::vector<uint32_t> devInds = this->m_streamDeviceIndices;
	bool doIMU = this->m_bIMU;
	bool doIMU_raw = this->m_bIMU_raw;
	bool doPos = this->m_bPos;
	bool doPos_raw = this->m_bPos_raw;
	this->mutex.unlock();

	// Each device has up to 2 streams: IMU and Position, with the following channels.
	QStringList imuChanLabels;
	// See PSMPSMoveCalibratedSensorData
	imuChanLabels
		<< "Accel.x" << "Accel.y" << "Accel.z"
		<< "Gyro.x" << "Gyro.y" << "Gyro.z"
		<< "Meg.x" << "Mag.y" << "Mag.z"
		<< "timestamp";
	if (doIMU_raw)
	{
		// TODO: Add more channels imu_raw
	}
	
	QStringList posChanLabels;
	posChanLabels << "Pos.x"
				  << "Pos.y"
				  << "Pos.z";
	if (doPos_raw)
	{
		// TODO: Add more channels for pos_raw
	}

	for (auto it = devInds.begin(); it < devInds.end(); it++) {
		PSMController *p_controller = PSM_GetController(it);
		QString ctrl_name = GetControllerString(p_controller);
		// Controller flags
		int ctrl_flags = 0;

		if (doIMU) {
			QString imu_stream_id = "PSMoveIMU" << ctrl_name;
			lsl::stream_info imuInfo("PSMoveIMU", "MoCap", imuChanLabels.size(), desiredSRate,
				lsl::cf_float32, ctrl_name.toStdString());
			// Append device meta-data
			imuInfo.desc()
				.append_child("acquisition")
				.append_child_value("manufacturer", "Sony")
				.append_child_value("model", "PlayStation Move");
			// Append channel info
			lsl::xml_element imuInfoChannels = imuInfo.desc().append_child("channels");
			QString devStr = QString::number(*it);
			devStr += "_";
			for (int imu_ix = 0; imu_ix < imuChanLabels.size(); imu_ix++) {
				QString chLabel = devStr;
				chLabel.append(imuChanLabels[imu_ix]);
				imuInfoChannels.append_child("channel")
					.append_child_value("label", chLabel.toStdString())
					.append_child_value("type", "IMU")
					.append_child_value("unit", "various");
			}
			this->mutex.lock();
			this->m_IMUOutlets.push_back(lsl::stream_outlet(imuInfo));
			this->mutex.unlock();

			ctrl_flags |= PSMStreamFlags_includeCalibratedSensorData;
			if (doIMU_raw)
				ctrl_flags |= PSMStreamFlags_includeRawSensorData;
		}
		if (doPos) {
			QString pos_stream_id = "PSMovePosition" << ctrl_name;
			lsl::stream_info posInfo("PSMovePosition", "MoCap", posChanLabels.size(), desiredSRate,
				lsl::cf_float32, pos_stream_id.toStdString());
			// Append device meta-data
			posInfo.desc()
				.append_child("acquisition")
				.append_child_value("manufacturer", "Sony")
				.append_child_value("model", "PlayStation Move");
			// Append channel info
			lsl::xml_element posInfoChannels = posInfo.desc().append_child("channels");
			QString devStr = QString::number(*it);
			devStr += "_";
			for (int pos_ix = 0; pos_ix < posChanLabels.size(); pos_ix++) {
				QString chLabel = devStr;
				chLabel.append(posChanLabels[pos_ix]);
				posInfoChannels.append_child("channel")
					.append_child_value("label", chLabel.toStdString())
					.append_child_value("type", "Position")
					.append_child_value("unit", "cm");
			}
			this->mutex.lock();
			this->m_PosOutlets.push_back(lsl::stream_outlet(posInfo));
			this->mutex.unlock();
			ctrl_flags |= PSMStreamFlags_includePositionData;
			if (doPos_raw)
				ctrl_flags |= PSMStreamFlags_includeRawTrackerData;
		}

		PSMRequestID request_id;
		PSM_AllocateControllerListener(it);
		m_controllerViews.push_back(PSM_GetController(it));
		m_lastSeqNums.push_back(-1);
		PSM_StartControllerDataStreamAsync(it, ctrl_flags, &request_id);
		PSM_RegisterCallback(request_id, handle_acquire_controller, this);
	}

	return true;
}

bool PSMoveThread::pollAndPush() {
	bool b_pushedAny = false;

	this->mutex.lock();
	int desiredSRate = this->m_srate;
	std::vector<uint32_t> devInds = this->m_streamDeviceIndices;
	this->mutex.unlock();

	// TODO: Async polling of devices.
	if (m_isControllerStreamActive)
	{
		for (size_t dev_ix = 0; dev_ix < m_controllerViews.size(); dev_ix++) {
			if (m_controllerViews[dev_ix].OutputSequenceNum != m_lastSeqNums[dev_ix])
			{
				// TODO: Build IMU data chunk.
				if (m_bIMU)
				{
					const PSMPSMoveCalibratedSensorData &calibratedSensorData =
						m_controllerViews[dev_ix].ControllerState.PSMoveState.CalibratedSensorData;
					// TODO: Copy data to IMU data chunk.
				}
				if (m_bIMU_raw)
				{
					const PSMPSMoveRawSensorData &rawSensorData =
						m_controllerViews[dev_ix].ControllerState.PSMoveState.RawSensorData;
					// TODO: Copy data to IMU data chunk
				}
				// TODO: Push IMU data chunk.
				
				// TODO: Build Pos data chunk.
				if (m_bPos) {
					const PSMPSMoveCalibratedSensorData &calibratedSensorData =
						m_controllerViews[dev_ix].ControllerState.PSMoveState.CalibratedSensorData;
					// TODO: Copy data to IMU data chunk.
				}
				if (m_bPos_raw) {
					const PSMRawTrackerData &rawTrackerData =
						m_controllerViews[dev_ix].ControllerState.PSMoveState.RawTrackerData;
					// TODO: Copy data to Pos data chunk
				}
				// TODO: Push IMU data chunk.
				
				m_lastSeqNums[dev_ix] = m_controllerViews[dev_ix].OutputSequenceNum;
				b_pushedAny = true;
			}
		}
	}
	
	// PSM_StartControllerDataStream


	return b_pushedAny;
}

void PSMoveThread::run() {
	runPhase phase = phase_startLink;

	// Thread-safe copy member variables to local variables.
	this->mutex.lock();
	this->mutex.unlock();

	forever {
		this->mutex.lock();
		if (this->abort) phase = phase_shutdown;
		this->mutex.unlock();

		switch (phase) {
		case phase_startLink:
			if (connectToPSMS()) {
				emit psmsConnected(true);
				phase = phase_scanForDevices;
			} else {
				phase = phase_shutdown;
			}
			break;
		case phase_scanForDevices:
			if (this->m_bGoOutlets) {
				phase = phase_createOutlets;
			} else {
				refreshControllerList();
				this->msleep(100);
			}
			break;
		case phase_createOutlets:
			if (!m_isControllerStreamActive) {
				if (createOutlets()) {
					emit outletsStarted(true);
					m_startTime = lsl::local_clock();
				} else {
					phase = phase_shutdown;
				}
			} else {
				phase = phase_transferData;
			}
			break;
		case phase_transferData:
			if (!this->m_bGoOutlets) {
				qDebug() << "Instructed to stop streaming.";
				this->mutex.lock();
				this->m_IMUOutlets.clear();
				this->m_PosOutlets.clear();
				phase = phase_scanForDevices;
				this->mutex.unlock();
				emit outletsStarted(false);
				break;
			}
			if (!pollAndPush()) { this->usleep(1); }
			break;
		case phase_shutdown:
			this->mutex.lock();
			this->m_IMUOutlets.clear();
			this->m_PosOutlets.clear();
			this->mutex.unlock();
			emit outletsStarted(false);
			PSM_Shutdown();
			emit psmsConnected(false);
			return;
			break;
		}
	}
}


void handle_acquire_controller(const PSMResponseMessage *response, void *userdata) {
	PSMoveThread *thisPtr = reinterpret_cast<PSMoveThread *>(userdata);

	if (response->result_code == PSMResult_Success) {
		thisPtr->m_isControllerStreamActive = true;
		// thisPtr->m_lastControllerSeqNum = -1;
		// Wait for the first controller packet to show up...
	}
}