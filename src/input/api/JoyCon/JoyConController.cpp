#include "input/api/JoyCon/JoyConController.h"

#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// HID device management
// ---------------------------------------------------------------------------

bool JoyConController::open_device()
{
	m_handle = hid_open_path(m_path.c_str());
	if (!m_handle)
	{
		cemuLog_log(LogType::Force, "JoyCon: failed to open HID device at {}", m_path);
		return false;
	}

	hid_set_nonblocking(m_handle, 0);

	send_subcommand(0x40, {0x01}); // Enable IMU
	std::this_thread::sleep_for(50ms);

	send_subcommand(0x03, {0x30}); // Set report mode to 0x30 (full, with IMU)
	std::this_thread::sleep_for(50ms);

	hid_set_nonblocking(m_handle, 1);

	m_connected.store(true);
	cemuLog_log(LogType::Force, "JoyCon: connected {} — hold still ~1 sec for auto-calibration",
		m_side == Side::Left ? "Left" : "Right");
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

	std::vector<uint8_t> buf;
	buf.reserve(11 + data.size());
	buf.push_back(0x01);                       // report ID
	buf.push_back(m_packet_counter++ & 0x0F); // packet counter
	buf.insert(buf.end(), {0x00, 0x01, 0x40, 0x40}); // neutral rumble L
	buf.insert(buf.end(), {0x00, 0x01, 0x40, 0x40}); // neutral rumble R
	buf.push_back(subcmd);
	buf.insert(buf.end(), data.begin(), data.end());

	return hid_write(m_handle, buf.data(), buf.size()) >= 0;
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

void JoyConController::request_recalibration()
{
	{
		std::lock_guard lock(m_motion_mutex);
		// Reset all calibration accumulators
		m_calib_sum_gx = m_calib_sum_gy = m_calib_sum_gz = 0.0f;
		m_calib_sum_ax = m_calib_sum_ay = m_calib_sum_az = 0.0f;
		// Reset reference to identity (will be recomputed when calibration finishes)
		m_ref_q = {1.0f, 0.0f, 0.0f, 0.0f};
		// Reset Mahony filter so orientation starts fresh
		m_motion_handler = WiiUMotionHandler{};
		m_last_sample    = MotionSample{};
		m_last_imu_time  = {};
	}
	m_calib_remaining.store(kCalibSamples);
	cemuLog_log(LogType::Force, "JoyCon: recalibration requested — hold still in desired neutral position");
}

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------

// Rotate vector (vx,vy,vz) by unit quaternion q = {w, x, y, z}
// Rodrigues: v' = v + 2w*(q_xyz×v) + 2*(q_xyz×(q_xyz×v))
void JoyConController::rotate_by_quat(const std::array<float,4>& q,
                                       float vx, float vy, float vz,
                                       float& rx, float& ry, float& rz)
{
	const float w = q[0], qx = q[1], qy = q[2], qz = q[3];

	// t = 2 * cross(q_xyz, v)
	const float tx = 2.0f * (qy * vz - qz * vy);
	const float ty = 2.0f * (qz * vx - qx * vz);
	const float tz = 2.0f * (qx * vy - qy * vx);

	// v' = v + w*t + cross(q_xyz, t)
	rx = vx + w * tx + qy * tz - qz * ty;
	ry = vy + w * ty + qz * tx - qx * tz;
	rz = vz + w * tz + qx * ty - qy * tx;
}

// Compute quaternion that rotates unit vector (fx,fy,fz) → (tx,ty,tz)
std::array<float,4> JoyConController::quat_from_to(float fx, float fy, float fz,
                                                     float tx, float ty, float tz)
{
	// cross product = rotation axis (unnormalized), dot = cos(angle)
	float ax = fy * tz - fz * ty;
	float ay = fz * tx - fx * tz;
	float az = fx * ty - fy * tx;
	float dot = fx * tx + fy * ty + fz * tz;
	float sin_half = std::sqrt(std::max(0.0f, (1.0f - dot) * 0.5f));
	float cos_half = std::sqrt(std::max(0.0f, (1.0f + dot) * 0.5f));

	float axis_len = std::sqrt(ax * ax + ay * ay + az * az);
	if (axis_len < 1e-6f)
	{
		// Vectors are parallel (or anti-parallel)
		if (dot > 0.0f)
			return {1.0f, 0.0f, 0.0f, 0.0f}; // identity
		else
			return {0.0f, 1.0f, 0.0f, 0.0f}; // 180° around X
	}

	float s = sin_half / axis_len;
	return {cos_half, ax * s, ay * s, az * s};
}

// ---------------------------------------------------------------------------
// Read thread
// ---------------------------------------------------------------------------

void JoyConController::read_thread_func()
{
	constexpr int kBufSize = 64;
	uint8_t buf[kBufSize];
	bool logged_first  = false;
	int  report_count  = 0;

	while (m_thread_running.load())
	{
		if (!m_handle)
		{
			std::this_thread::sleep_for(100ms);
			continue;
		}

		int res = hid_read_timeout(m_handle, buf, kBufSize, 100);
		if (res < 0)
		{
			cemuLog_log(LogType::Force, "JoyCon: HID read error, device disconnected");
			m_connected.store(false);
			std::this_thread::sleep_for(500ms);
			continue;
		}
		if (res == 0)
			continue;

		if (!logged_first)
		{
			logged_first = true;
			cemuLog_log(LogType::Force, "JoyCon: first report ID=0x{:02X} len={}", buf[0], res);
		}

		// If Joy-Con hasn't switched to 0x30 mode yet, resend init commands
		if (buf[0] == 0x3F && report_count < 5)
		{
			cemuLog_log(LogType::Force, "JoyCon: still in 0x3F mode, resending mode switch");
			send_subcommand(0x40, {0x01});
			std::this_thread::sleep_for(30ms);
			send_subcommand(0x03, {0x30});
		}

		++report_count;

		if (res >= 49 && buf[0] == 0x30)
			parse_imu(buf, (size_t)res);
	}
}

// ---------------------------------------------------------------------------
// IMU parsing  (core logic)
// ---------------------------------------------------------------------------

void JoyConController::parse_imu(const uint8_t* report, size_t len)
{
	if (len < 49)
		return;

	// Helper: read little-endian int16_t
	auto r16 = [](const uint8_t* p) -> int16_t {
		return static_cast<int16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
	};

	// --- Compute deltaTime ONCE for all 3 sub-samples ---
	// The 0x30 report contains 3 IMU samples covering the elapsed period since
	// the previous report.  Update the clock BEFORE the sub-sample loop so that
	// every sub-sample gets sub_dt = elapsed/3 (not 0 for samples 1 and 2).
	const auto now = std::chrono::high_resolution_clock::now();
	float sub_dt;
	{
		std::lock_guard lock(m_motion_mutex);
		if (m_last_imu_time == std::chrono::high_resolution_clock::time_point{})
		{
			// First call: use nominal rate (Joy-Con sends at ~60 Hz, 3 sub-samples each)
			sub_dt = 1.0f / 180.0f;
		}
		else
		{
			const float elapsed = std::chrono::duration<float>(now - m_last_imu_time).count();
			sub_dt = std::min(elapsed, 0.1f) / 3.0f;
		}
		m_last_imu_time = now; // update ONCE before the loop
	}

	// --- Process all 3 sub-samples ---
	// IMU data starts at byte 13 (1 report_id + 12 bytes frame header).
	for (int s = 0; s < 3; ++s)
	{
		const uint8_t* imu = report + 13 + s * 12;

		const float raw_gx = r16(imu + 6)  * GYRO_RAD_PER_UNIT;
		const float raw_gy = r16(imu + 8)  * GYRO_RAD_PER_UNIT;
		const float raw_gz = r16(imu + 10) * GYRO_RAD_PER_UNIT;

		const float raw_ax = r16(imu + 0)  * ACC_G_PER_UNIT;
		const float raw_ay = r16(imu + 2)  * ACC_G_PER_UNIT;
		const float raw_az = r16(imu + 4)  * ACC_G_PER_UNIT;

		// --- Calibration phase: accumulate samples ---
		const int remaining = m_calib_remaining.load();
		if (remaining > 0)
		{
			// Accumulate raw gyro for bias estimation
			m_calib_sum_gx += raw_gx;
			m_calib_sum_gy += raw_gy;
			m_calib_sum_gz += raw_gz;

			// Accumulate mapped accel for reference quaternion
			// Mapping: (ax, az, ay) → Mahony frame (same as normal operation below)
			m_calib_sum_ax += raw_ax;
			m_calib_sum_ay += raw_az; // note: az→ay in Mahony frame
			m_calib_sum_az += raw_ay; // note: ay→az in Mahony frame

			const int new_rem = remaining - 1;
			m_calib_remaining.store(new_rem);

			if (new_rem == 0)
			{
				// --- Gyro bias ---
				m_bias_gx = m_calib_sum_gx / kCalibSamples;
				m_bias_gy = m_calib_sum_gy / kCalibSamples;
				m_bias_gz = m_calib_sum_gz / kCalibSamples;

				// --- Reference quaternion from gravity direction ---
				// Average mapped accel = gravity direction in the Mahony frame
				// for the current physical Joy-Con position.
				// We compute the quaternion that rotates this direction to (0,0,1)
				// so the Mahony filter treats the calibration position as "neutral".
				float gx = m_calib_sum_ax / kCalibSamples;
				float gy = m_calib_sum_ay / kCalibSamples;
				float gz = m_calib_sum_az / kCalibSamples;
				const float glen = std::sqrt(gx*gx + gy*gy + gz*gz);
				if (glen > 0.01f) { gx /= glen; gy /= glen; gz /= glen; }

				std::lock_guard lock(m_motion_mutex);
				m_ref_q = quat_from_to(gx, gy, gz, 0.0f, 0.0f, 1.0f);

				// Reset the Mahony filter so it converges from the new reference
				m_motion_handler = WiiUMotionHandler{};
				m_last_imu_time  = {};

				cemuLog_log(LogType::Force,
					"JoyCon: calibration done. "
					"Bias=({:.4f},{:.4f},{:.4f}) rad/s  "
					"GravDir=({:.3f},{:.3f},{:.3f})",
					m_bias_gx, m_bias_gy, m_bias_gz, gx, gy, gz);
			}
			continue; // Don't feed motion data during calibration
		}

		// --- Normal operation ---

		// 1. Subtract gyro bias
		const float gx_c = raw_gx - m_bias_gx;
		const float gy_c = raw_gy - m_bias_gy;
		const float gz_c = raw_gz - m_bias_gz;

		// 2. Apply Joy-Con axis mapping for Right Joy-Con held in portrait
		//    (joystick at top, SL/SR on sides)
		//    Joy-Con X → pitch,  Joy-Con Z → yaw,  Joy-Con Y → roll
		float mg_x = gx_c, mg_y = gz_c, mg_z = gy_c; // gyro: pitch/yaw/roll
		float ma_x = raw_ax, ma_y = raw_az, ma_z = raw_ay; // accel

		// 3. Apply reference quaternion so calibration position = neutral
		float rg_x, rg_y, rg_z;
		float ra_x, ra_y, ra_z;
		{
			std::lock_guard lock(m_motion_mutex);
			rotate_by_quat(m_ref_q, mg_x, mg_y, mg_z, rg_x, rg_y, rg_z);
			rotate_by_quat(m_ref_q, ma_x, ma_y, ma_z, ra_x, ra_y, ra_z);

			// 4. Feed into Mahony filter (sub_dt already computed above)
			m_motion_handler.processMotionSample(sub_dt,
				rg_x, rg_y, rg_z,   // gyro:  pitch, yaw, roll (rad/s)
				ra_x, ra_y, ra_z);   // accel: x, y, z (g)

			m_last_sample = m_motion_handler.getMotionSample();
		}
	}
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

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
	return {}; // gyro-only, no buttons/sticks
}
