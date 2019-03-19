#include "psmovethread.h"
#include <QDebug>
#include <cmath>
#include <iostream>

enum runPhase {
	phase_startLink,
	phase_scanForDevices,
	phase_waitForControllers,
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

void handle_acquire_controller(const PSMResponseMessage *response, void *userdata) {
	PSMoveThread *thisPtr = reinterpret_cast<PSMoveThread *>(userdata);

	if (response->result_code == PSMResult_Success) {
		thisPtr->m_bControllerStreamActive = true;
		// thisPtr->m_lastControllerSeqNum = -1;
		// Wait for the first controller packet to show up...
	}
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

void PSMoveThread::initPSMS(double srate) {
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

void PSMoveThread::startStreams(
	QStringList streamDeviceList, bool doIMU, bool doIMU_raw, bool doPos, bool doPos_raw) {
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

void PSMoveThread::acquireControllers() {
	this->mutex.lock();
	std::vector<uint32_t> devInds = this->m_streamDeviceIndices;
	bool doIMU = this->m_bIMU;
	bool doIMU_raw = this->m_bIMU_raw;
	bool doPos = this->m_bPos;
	bool doPos_raw = this->m_bPos_raw;
	this->mutex.unlock();

	// Controller flags
	int ctrl_flags = 0;
	if (doIMU) ctrl_flags |= PSMStreamFlags_includeCalibratedSensorData;
	if (doIMU_raw) ctrl_flags |= PSMStreamFlags_includeRawSensorData;
	if (doPos) ctrl_flags |= PSMStreamFlags_includePositionData;
	if (doPos_raw) ctrl_flags |= PSMStreamFlags_includeRawTrackerData;

	for (auto it = devInds.begin(); it < devInds.end(); it++) {
		PSMControllerID ctrl_id(*it);
		PSMRequestID request_id;
		PSM_AllocateControllerListener(ctrl_id);
		m_controllerViews.push_back(PSM_GetController(ctrl_id));
		m_lastSeqNums.push_back(-1);
		PSM_StartControllerDataStreamAsync(ctrl_id, ctrl_flags, &request_id);
		PSM_RegisterCallback(request_id, handle_acquire_controller, this);
	}
}

bool PSMoveThread::createOutlets() {
	// Safely copy member variables to local variables.
	this->mutex.lock();
	// double desiredSRate = this->m_srate;
	double desiredSRate = lsl::IRREGULAR_RATE;
	std::vector<uint32_t> devInds = this->m_streamDeviceIndices;
	bool doIMU = this->m_bIMU;
	bool doIMU_raw = this->m_bIMU_raw;
	bool doPos = this->m_bPos;
	bool doPos_raw = this->m_bPos_raw;
	this->mutex.unlock();

	// Each device has up to 2 streams: IMU and Position, with the following channels.
	QStringList imuChanLabels;
	if (doIMU) {
		imuChanLabels << "timestamp"
					  << "Accel.x"
					  << "Accel.y"
					  << "Accel.z"
					  << "Gyro.x"
					  << "Gyro.y"
					  << "Gyro.z"
					  << "Mag.x"
					  << "Mag.y"
					  << "Mag.z";
	}
	if (doIMU_raw) {
		imuChanLabels << "raw_timestamp"
					  << "raw_Accel.x"
					  << "raw_Accel.y"
					  << "raw_Accel.z"
					  << "raw_Gyro.x"
					  << "raw_Gyro.y"
					  << "raw_Gyro.z"
					  << "raw_Mag.x"
					  << "raw_Mag.y"
					  << "raw_Mag.z";
	}
	m_channelCount_IMU = imuChanLabels.size();

	QStringList posChanLabels;
	if (doPos) {
		posChanLabels << "Pos.orient_w"
					  << "Pos.orient_x"
					  << "Pos.orient_y"
					  << "Pos.orient_z"
					  << "Pos.x"
					  << "Pos.y"
					  << "Pos.z";
	}

	if (doPos_raw) {
		posChanLabels << "RelativePosition.x"
					  << "RelativePosition.y"
					  << "RelativePosition.z";
	}
	m_channelCount_Pos = posChanLabels.size();

	for (auto it = devInds.begin(); it < devInds.end(); it++) {
		PSMControllerID ctrl_id(*it);
		PSMController *p_controller = PSM_GetController(ctrl_id);
		QString ctrl_name = GetControllerString(p_controller);
		
		if (doIMU || doIMU_raw) {
			QString imu_stream_id = QString("PSMoveIMU") + ctrl_name;
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
			for (int imu_ix = 0; imu_ix < m_channelCount_IMU; imu_ix++) {
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
		}
		if (doPos || doPos_raw) {
			QString pos_stream_id = QString("PSMovePosition") + ctrl_name;
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
			for (int pos_ix = 0; pos_ix < m_channelCount_Pos; pos_ix++) {
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
		}
	}

	return true;
}

bool PSMoveThread::pollAndPush() {
	bool b_pushedAny = false;

	this->mutex.lock();
	double desiredSRate = this->m_srate;
	std::vector<uint32_t> devInds = this->m_streamDeviceIndices;
	this->mutex.unlock();

	// See if devices have new data.
	for (size_t dev_ix = 0; dev_ix < m_controllerViews.size(); dev_ix++) {
		if (m_controllerViews[dev_ix]->OutputSequenceNum != m_lastSeqNums[dev_ix]) {
			qDebug() << "Found new data.";
			int n_seqs = m_controllerViews[dev_ix]->OutputSequenceNum - m_lastSeqNums[dev_ix];
			if (n_seqs > 1) {
				qDebug() << "I didn't expect so many! - " << n_seqs;
			}

			// Build IMU data sample/chunk.
			
			if (m_bIMU || m_bIMU_raw) {
				std::vector<float> imu_sample(m_channelCount_IMU);
				int imu_c_off = 0;
				if (m_bIMU) {
					const PSMPSMoveCalibratedSensorData &calibSens =
						m_controllerViews[dev_ix]->ControllerState.PSMoveState.CalibratedSensorData;
					// Copy data to IMU data sample.
					imu_sample[imu_c_off + 0] = (float)calibSens.TimeInSeconds;
					imu_sample[imu_c_off + 1] = calibSens.Accelerometer.x;
					imu_sample[imu_c_off + 2] = calibSens.Accelerometer.y;
					imu_sample[imu_c_off + 3] = calibSens.Accelerometer.z;
					imu_sample[imu_c_off + 4] = calibSens.Gyroscope.x;
					imu_sample[imu_c_off + 5] = calibSens.Gyroscope.y;
					imu_sample[imu_c_off + 6] = calibSens.Gyroscope.z;
					imu_sample[imu_c_off + 7] = calibSens.Magnetometer.x;
					imu_sample[imu_c_off + 8] = calibSens.Magnetometer.y;
					imu_sample[imu_c_off + 9] = calibSens.Magnetometer.z;
					imu_c_off += 10;
				}
				if (m_bIMU_raw) {
					const PSMPSMoveRawSensorData &rawSens =
						m_controllerViews[dev_ix]->ControllerState.PSMoveState.RawSensorData;
					// Copy raw data to IMU data chunk
					imu_sample[imu_c_off + 0] = rawSens.TimeInSeconds;
					imu_sample[imu_c_off + 1] = rawSens.Accelerometer.x;
					imu_sample[imu_c_off + 2] = rawSens.Accelerometer.y;
					imu_sample[imu_c_off + 3] = rawSens.Accelerometer.z;
					imu_sample[imu_c_off + 4] = rawSens.Gyroscope.x;
					imu_sample[imu_c_off + 5] = rawSens.Gyroscope.y;
					imu_sample[imu_c_off + 6] = rawSens.Gyroscope.z;
					imu_sample[imu_c_off + 7] = rawSens.Magnetometer.x;
					imu_sample[imu_c_off + 8] = rawSens.Magnetometer.y;
					imu_sample[imu_c_off + 9] = rawSens.Magnetometer.z;
					imu_c_off += 10;
				}
				// Push IMU data chunk.
				m_IMUOutlets[dev_ix].push_sample(imu_sample.data());
			}
			

			// Build Pos data sample / chunk.
			if (m_bPos || m_bPos_raw)
			{
				std::vector<float> pos_sample(m_channelCount_Pos);
				int pos_c_off = 0;
				if (m_bPos) {
					const PSMPosef &poseData =
						m_controllerViews[dev_ix]->ControllerState.PSMoveState.Pose;
					pos_sample[pos_c_off + 0] = poseData.Orientation.w;
					pos_sample[pos_c_off + 1] = poseData.Orientation.x;
					pos_sample[pos_c_off + 2] = poseData.Orientation.y;
					pos_sample[pos_c_off + 3] = poseData.Orientation.z;
					pos_sample[pos_c_off + 4] = poseData.Position.x;
					pos_sample[pos_c_off + 5] = poseData.Position.y;
					pos_sample[pos_c_off + 6] = poseData.Position.z;
					pos_c_off += 7;
				}
				if (m_bPos_raw) {
					const PSMRawTrackerData &rawTrackerData =
						m_controllerViews[dev_ix]->ControllerState.PSMoveState.RawTrackerData;
					pos_sample[pos_c_off + 0] = rawTrackerData.RelativePositionCm.x;
					pos_sample[pos_c_off + 1] = rawTrackerData.RelativePositionCm.y;
					pos_sample[pos_c_off + 2] = rawTrackerData.RelativePositionCm.z;
					pos_c_off += 3;
				}
				// Push Pos data sample/chunk.
				m_PosOutlets[dev_ix].push_sample(pos_sample.data());
			}
			m_lastSeqNums[dev_ix] = m_controllerViews[dev_ix]->OutputSequenceNum;
			b_pushedAny = true;
		}
	}
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
				acquireControllers();
				phase = phase_waitForControllers;
			} else {
				// Continuously search for new controllers to appear.
				refreshControllerList();
				this->msleep(250);
			}
			break;
		case phase_waitForControllers:
			PSM_Update();
			if (this->m_bControllerStreamActive) phase = phase_createOutlets;
			break;
		case phase_createOutlets:
			if (createOutlets()) {
				emit outletsStarted(true);
				m_startTime = lsl::local_clock();
				phase = phase_transferData;
			} else {
				phase = phase_shutdown;
			}
			break;
		case phase_transferData:
			PSM_Update();
			// If we are no longer running the outlets, we need to destroy them.
			if (!this->m_bGoOutlets) {
				qDebug() << "Instructed to stop streaming.";
				this->mutex.lock();
				this->m_IMUOutlets.clear();
				this->m_PosOutlets.clear();
				phase = phase_scanForDevices;
				this->mutex.unlock();
				emit outletsStarted(false);
				break;
			} else if (!pollAndPush()) {
				this->usleep(1);
			}
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
