#pragma once

#include "input/api/JoyCon/JoyConControllerProvider.h"
#include "input/api/Controller.h"
#include "input/motion/MotionHandler.h"
#include "input/motion/MotionSample.h"

#include <hidapi/hidapi.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

// Joy-Con controller connected over Bluetooth.
// Gyro-only: sends IMU enable commands, reads 0x30 reports in a background thread,
// and provides MotionSample for the VPAD motion system.
// No button mapping is required — use_motion() is the only relevant feature.
class JoyConController : public Controller<JoyConControllerProvider>
{
public:
	enum class Side { Left, Right };

	JoyConController(std::string path, Side side);
	~JoyConController() override;

	std::string_view api_name() const override { return "JoyCon"; }
	InputAPI::Type   api()      const override { return InputAPI::JoyCon; }

	bool is_connected()          override { return m_connected.load(); }
	bool has_motion()            override { return true; }
	bool has_axis() const        override { return false; }

	std::string get_button_name(uint64 button) const override;
	MotionSample get_motion_sample() override;

	// Calibration: while holding the Joy-Con still in the desired neutral position,
	// samples gyro bias AND captures the gravity vector as the orientation reference.
	// After ~1 second the Joy-Con's current physical position becomes "zero" for the game.
	void request_recalibration();
	bool is_calibrating() const { return m_calib_remaining > 0; }

protected:
	ControllerState raw_state() override;

private:
	bool open_device();
	void close_device();
	bool send_subcommand(uint8_t subcmd, const std::vector<uint8_t>& data = {});
	void read_thread_func();
	void parse_imu(const uint8_t* report, size_t len);

	std::string       m_path;
	Side              m_side;
	hid_device*       m_handle = nullptr;
	std::atomic<bool> m_connected{false};
	uint8_t           m_packet_counter = 0;

	std::thread       m_read_thread;
	std::atomic<bool> m_thread_running{false};

	std::mutex           m_motion_mutex;
	WiiUMotionHandler    m_motion_handler;
	MotionSample         m_last_sample;
	std::chrono::high_resolution_clock::time_point m_last_imu_time{};

	// --- Gyro bias calibration ---
	static constexpr int kCalibSamples = 180; // ~1 second at 180 sub-frames/sec
	std::atomic<int> m_calib_remaining{kCalibSamples};
	float m_calib_sum_gx = 0, m_calib_sum_gy = 0, m_calib_sum_gz = 0;
	float m_bias_gx = 0, m_bias_gy = 0, m_bias_gz = 0;

	// Nintendo Vendor ID + Joy-Con Product IDs
	static constexpr uint16_t VENDOR_ID  = 0x057E;
	static constexpr uint16_t LEFT_PID   = 0x2006;
	static constexpr uint16_t RIGHT_PID  = 0x2007;

	// Default IMU sensitivity (factory settings)
	// Accel:  ±8 G  → 1/4096 G per LSB  ≈ 0.000244 G
	// Gyro:   ±2000 dps → 1/16.4 dps/LSB → × π/180 ≈ 0.001065 rad/s per LSB
	static constexpr float ACC_G_PER_UNIT     = 0.000244f;
	static constexpr float GYRO_RAD_PER_UNIT  = 0.001065f;
};
