#pragma once

#include "input/api/Mouse/MouseControllerProvider.h"
#include "input/api/Controller.h"
#include "input/motion/MotionHandler.h"
#include "input/motion/MotionSample.h"

#include <atomic>
#include <chrono>

// Mouse controller that provides analog stick values from mouse movement
// Used to map mouse to right stick (camera control) for keyboard+mouse gameplay
class MouseController : public Controller<MouseControllerProvider>
{
public:
	MouseController();

	std::string_view api_name() const override
	{
		return "Mouse";
	}
	InputAPI::Type api() const override { return InputAPI::Mouse; }

	bool is_connected() override { return m_connected; }

	// Mouse provides axis values (rotation from mouse delta)
	bool has_axis() const override { return true; }

	// No buttons on mouse for this mapping (just movement)
	std::string get_button_name(uint64 button) const override;

	// Settings
	struct MouseSettings
	{
		float sensitivity = 1.0f;      // Mouse sensitivity multiplier
		float deadzone = 0.01f;        // Minimum delta to register
		bool invertX = false;          // Invert X axis
		bool invertY = false;          // Invert Y axis
		bool use_raw_input = true;     // Use raw mouse input if available
	};

	static MouseSettings& get_mouse_settings() { return s_mouse_settings; }
	static void set_mouse_settings(const MouseSettings& settings) { s_mouse_settings = settings; }

	// Gyro mode: how the mouse gyro simulation is activated
	enum class GyroMode : int
	{
		Always = 0, // Active whenever mouse capture is active
		Toggle = 1, // A key press toggles gyro on/off
		Hold   = 2, // Gyro active only while a key is held
	};

	struct GyroSettings
	{
		GyroMode mode        = GyroMode::Hold; // Default: off until user binds a key
		uint32   key         = 0;              // Key used for Toggle/Hold (0 = none)
		float    sensitivity = 1.0f;           // Gyro sensitivity multiplier
	};

	static GyroSettings& get_gyro_settings() { return s_gyro_settings; }
	static void set_gyro_settings(const GyroSettings& settings) { s_gyro_settings = settings; }

	// Returns true if gyro simulation should be active this frame (handles all 3 modes).
	// Has side effects (Toggle edge detection) — call once per frame from update_motion().
	static bool is_gyro_active();

	// Side-effect-free read of the last result of is_gyro_active().
	// Safe to call from get_rotation() without disturbing Toggle state.
	static bool is_gyro_on() { return s_gyro_active_cache.load(); }

	// Returns the current MotionSample built from accumulated mouse deltas.
	// Must be called once per frame from the emulation thread.
	static MotionSample get_gyro_sample();

	// Toggle mouse capture (when true, mouse is captured and controls right stick)
	static bool is_capture_active() { return s_capture_active; }
	static void set_capture_active(bool active) { s_capture_active = active; }

	// Toggle key (e.g., right mouse button or a keyboard key)
	static uint32 get_toggle_key() { return s_toggle_key; }
	static void set_toggle_key(uint32 key) { s_toggle_key = key; }

	// Process raw mouse movement (called from window event handler)
	static void on_mouse_move(int32_t deltaX, int32_t deltaY);
	static void on_mouse_wheel(float delta);

	// Mouse button states (set from MainWindow when captured)
	static void set_left_button(bool down) { s_left_button.store(down); }
	static void set_right_button(bool down) { s_right_button.store(down); }
	static bool is_left_button_down() { return s_left_button.load(); }
	static bool is_right_button_down() { return s_right_button.load(); }

	// Get current mouse state as right stick values (hardcoded, no mapping needed)
	// Returns {x, y} in [-1, 1] range for direct injection into get_rotation()
	static glm::vec2 get_current_rotation();

	// Get mouse wheel as forward/backward movement for left stick Y
	// Returns value in [-1, 1] that decays over time
	static float get_wheel_axis();

protected:
	ControllerState raw_state() override;

private:
	void update_mouse_delta();

	bool m_connected = true;
	
	// Accumulated mouse deltas (thread-safe via atomic)
	static std::atomic<float> s_delta_x;
	static std::atomic<float> s_delta_y;
	static std::atomic<float> s_wheel_delta;

	static MouseSettings s_mouse_settings;
	static std::atomic<bool> s_capture_active;
	static std::atomic<bool> s_left_button;
	static std::atomic<bool> s_right_button;
	static uint32 s_toggle_key; // Key that toggles mouse capture
	static std::atomic<float> s_wheel_axis; // Decaying wheel value for left stick

	// Gyro state (all accessed only from emulation thread except the deltas)
	static GyroSettings s_gyro_settings;
	static std::atomic<bool> s_gyro_enabled;       // Current on/off state for Toggle mode
	static std::atomic<bool> s_gyro_active_cache;  // Last result of is_gyro_active(), for is_gyro_on()
	static bool s_gyro_prev_key_down;              // Previous key state for Toggle edge detection
	static std::atomic<float> s_gyro_delta_x;      // Accumulated mouse X delta for gyro (separate from stick)
	static std::atomic<float> s_gyro_delta_y;      // Accumulated mouse Y delta for gyro
	static WiiUMotionHandler s_motion_handler;     // Mahony-based sensor fusion (emulation thread only)
	static std::chrono::high_resolution_clock::time_point s_last_gyro_time; // For deltaTime calculation

	std::mutex m_delta_mutex;
	float m_current_dx = 0.0f;
	float m_current_dy = 0.0f;
	float m_current_wheel = 0.0f;

	// For time-based smoothing
	std::chrono::high_resolution_clock::time_point m_last_update{};
	float m_smoothing_x = 0.0f;
	float m_smoothing_y = 0.0f;

	static constexpr float kMouseScale = 0.03f; // Scale factor for mouse to stick
	static constexpr float kSmoothingFactor = 0.5f; // Lower = more smoothing
};
