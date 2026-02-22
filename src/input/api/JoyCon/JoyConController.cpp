#include "input/api/JoyCon/JoyConController.h"

#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

JoyConController::JoyConController(std::string path, Side side)
	: Controller(path, side == Side::Left ? "Joy-Con (L)" : "Joy-Con (R)")
	, m_path(path)
	, m_side(side)
{
	// Gyro-only controller: always enable motion so the user doesn't have
	// to manually tick "Use motion" in settings.
	m_settings.motion = true;

	if (open_device())
	{
		m_thread_running.store(true);
		m_read_thread = std::thread(&JoyConController::read_thread_func, this);
	}
}

JoyConController::~JoyConController()
{
	m_thread_running.store(false);
	if (m_read_thread.joinable())
		m_read_thread.join();
	close_device();
}

bool JoyConController::open_device()
{
	m_handle = hid_open_path(m_path.c_str());
	if (!m_handle)
	{
		cemuLog_log(LogType::Force, "JoyCon: failed to open HID device at {}", m_path);
		return false;
	}

	// Use blocking mode while sending init subcommands
	hid_set_nonblocking(m_handle, 0);

	// Enable IMU (subcommand 0x40, arg 0x01 = enable)
	send_subcommand(0x40, {0x01});
	std::this_thread::sleep_for(50ms);

	// Set input report mode to 0x30 (standard full report — includes IMU)
	send_subcommand(0x03, {0x30});
	std::this_thread::sleep_for(50ms);

	// Switch to non-blocking for the read loop
	hid_set_nonblocking(m_handle, 1);

	m_connected.store(true);
	cemuLog_log(LogType::Force, "JoyCon: connected {} — calibrating gyro bias for ~1 second (hold still)",
		m_side == Side::Left ? "Left Joy-Con" : "Right Joy-Con");
	return true;
}

void JoyConController::close_device()
{
	if (m_handle)
	{
		hid_close(m_handle);
		m_handle = nullptr;
	}
	m_connected.store(false);
}

bool JoyConController::send_subcommand(uint8_t subcmd, const std::vector<uint8_t>& data)
{
	if (!m_handle)
		return false;

	// Output report 0x01: [report_id, counter, rumble_l(4), rumble_r(4), subcmd, data...]
	std::vector<uint8_t> buf;
	buf.reserve(11 + data.size());
	buf.push_back(0x01);                                    // report ID
	buf.push_back(m_packet_counter++ & 0x0F);              // global packet counter (low nibble)
	// Neutral rumble data (L then R)
	buf.insert(buf.end(), {0x00, 0x01, 0x40, 0x40});
	buf.insert(buf.end(), {0x00, 0x01, 0x40, 0x40});
	buf.push_back(subcmd);
	buf.insert(buf.end(), data.begin(), data.end());

	return hid_write(m_handle, buf.data(), buf.size()) >= 0;
}

void JoyConController::request_recalibration()
{
	// Reset bias accumulators, then let parse_imu count down kCalibSamples frames.
	// Also reset the Mahony filter so the orientation starts fresh from neutral.
	{
		std::lock_guard lock(m_motion_mutex);
		m_calib_sum_gx = m_calib_sum_gy = m_calib_sum_gz = 0.0f;
		m_motion_handler = WiiUMotionHandler{};
		m_last_sample    = MotionSample{};
		m_last_imu_time  = {};
	}
	m_calib_remaining.store(kCalibSamples);
	cemuLog_log(LogType::Force, "JoyCon: recalibration requested — hold still for ~1 second");
}

void JoyConController::read_thread_func()
{
	constexpr int kBufSize = 64;
	uint8_t buf[kBufSize];
	bool logged_first = false;
	int report_count  = 0;

	while (m_thread_running.load())
	{
		if (!m_handle)
		{
			std::this_thread::sleep_for(100ms);
			continue;
		}

		int res = hid_read_timeout(m_handle, buf, kBufSize, 100 /*ms*/);
		if (res < 0)
		{
			cemuLog_log(LogType::Force, "JoyCon: HID read error, device disconnected");
			m_connected.store(false);
			std::this_thread::sleep_for(500ms);
			continue;
		}
		if (res == 0)
			continue; // timeout, no data

		// Log first received report for diagnosis
		if (!logged_first)
		{
			logged_first = true;
			cemuLog_log(LogType::Force, "JoyCon: first report received: ID=0x{:02X} len={}", buf[0], res);
		}

		// If Joy-Con is still sending simple 0x3F reports (not in 0x30 mode yet),
		// resend the mode-switch command.
		if (buf[0] == 0x3F && report_count < 5)
		{
			cemuLog_log(LogType::Force, "JoyCon: received 0x3F report, resending mode switch to 0x30");
			send_subcommand(0x40, {0x01}); // Enable IMU
			std::this_thread::sleep_for(30ms);
			send_subcommand(0x03, {0x30}); // Set report mode
		}

		++report_count;

		// Standard full report with IMU = 0x30, 49 bytes
		if (res >= 49 && buf[0] == 0x30)
			parse_imu(buf, (size_t)res);
	}
}

void JoyConController::parse_imu(const uint8_t* report, size_t len)
{
	if (len < 49)
		return;

	const auto now = std::chrono::high_resolution_clock::now();

	// Helper: read little-endian int16_t
	auto r16 = [](const uint8_t* p) -> int16_t {
		return static_cast<int16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
	};

	// Process all 3 IMU sub-samples contained in each 0x30 report.
	// IMU data starts at byte 13 (header: 1 report_id + 12 bytes frame info).
	for (int s = 0; s < 3; ++s)
	{
		const uint8_t* imu = report + 13 + s * 12;

		// Raw values
		const float raw_gx = r16(imu + 6)  * GYRO_RAD_PER_UNIT;
		const float raw_gy = r16(imu + 8)  * GYRO_RAD_PER_UNIT;
		const float raw_gz = r16(imu + 10) * GYRO_RAD_PER_UNIT;

		// --- Calibration phase: accumulate samples to estimate static bias ---
		const int remaining = m_calib_remaining.load();
		if (remaining > 0)
		{
			m_calib_sum_gx += raw_gx;
			m_calib_sum_gy += raw_gy;
			m_calib_sum_gz += raw_gz;

			const int new_remaining = remaining - 1;
			m_calib_remaining.store(new_remaining);

			if (new_remaining == 0)
			{
				// Calibration done: compute bias as mean
				m_bias_gx = m_calib_sum_gx / kCalibSamples;
				m_bias_gy = m_calib_sum_gy / kCalibSamples;
				m_bias_gz = m_calib_sum_gz / kCalibSamples;
				cemuLog_log(LogType::Force,
					"JoyCon: calibration done. Bias = ({:.5f}, {:.5f}, {:.5f}) rad/s",
					m_bias_gx, m_bias_gy, m_bias_gz);
			}
			// Don't feed motion data during calibration
			continue;
		}

		// --- Normal operation: subtract bias, feed into Mahony filter ---

		// Bias-corrected gyro (rad/s)
		const float gx_c = raw_gx - m_bias_gx;
		const float gy_c = raw_gy - m_bias_gy;
		const float gz_c = raw_gz - m_bias_gz;

		// Accelerometer (g)
		const float ax = r16(imu + 0) * ACC_G_PER_UNIT;
		const float ay = r16(imu + 2) * ACC_G_PER_UNIT;
		const float az = r16(imu + 4) * ACC_G_PER_UNIT;

		// Compute deltaTime from wall clock (shared across 3 sub-samples)
		float sub_dt;
		if (m_last_imu_time == std::chrono::high_resolution_clock::time_point{})
		{
			m_last_imu_time = now;
			sub_dt = 1.0f / 180.0f; // nominal 180 Hz sub-sample rate
		}
		else
		{
			const float elapsed = std::chrono::duration<float>(now - m_last_imu_time).count();
			sub_dt = std::min(elapsed, 0.1f) / 3.0f;
			if (s == 0)
				m_last_imu_time = now;
		}

		// Axis mapping for Right Joy-Con held in portrait (joystick at top):
		//   gx_c = Joy-Con X → pitch  (tilt forward/back)
		//   gz_c = Joy-Con Z → yaw    (rotate left/right)
		//   gy_c = Joy-Con Y → roll   (tilt sideways)
		std::lock_guard lock(m_motion_mutex);
		m_motion_handler.processMotionSample(sub_dt,
			gx_c, gz_c, gy_c,   // gx=pitch, gy=yaw, gz=roll
			ax,   az,   ay);    // accx, accy, accz
		m_last_sample = m_motion_handler.getMotionSample();
	}
}

MotionSample JoyConController::get_motion_sample()
{
	std::lock_guard lock(m_motion_mutex);
	return m_last_sample;
}

std::string JoyConController::get_button_name(uint64 /*button*/) const
{
	return "(Joy-Con gyro-only)";
}

ControllerState JoyConController::raw_state()
{
	// Gyro-only controller — no buttons/sticks exposed
	return {};
}
