#pragma once

#include "input/api/Mouse/MouseControllerProvider.h"
#include "input/api/Controller.h"

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
		static_assert(to_string(InputAPI::Mouse) == "Mouse");
		return to_string(InputAPI::Mouse);
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
		float deadzone = 0.05f;        // Minimum delta to register
		bool invertX = false;          // Invert X axis
		bool invertY = false;          // Invert Y axis
		bool use_raw_input = true;     // Use raw mouse input if available
	};

	static MouseSettings& get_mouse_settings() { return s_mouse_settings; }
	static void set_mouse_settings(const MouseSettings& settings) { s_mouse_settings = settings; }

	// Toggle mouse capture (when true, mouse is captured and controls right stick)
	static bool is_capture_active() { return s_capture_active; }
	static void set_capture_active(bool active) { s_capture_active = active; }

	// Toggle key (e.g., right mouse button or a keyboard key)
	static uint32 get_toggle_key() { return s_toggle_key; }
	static void set_toggle_key(uint32 key) { s_toggle_key = key; }

	// Process raw mouse movement (called from window event handler)
	static void on_mouse_move(int32_t deltaX, int32_t deltaY);
	static void on_mouse_wheel(float delta);

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
	static uint32 s_toggle_key; // Key that toggles mouse capture

	std::mutex m_delta_mutex;
	float m_current_dx = 0.0f;
	float m_current_dy = 0.0f;
	float m_current_wheel = 0.0f;

	// For time-based smoothing
	std::chrono::high_resolution_clock::time_point m_last_update{};
	float m_smoothing_x = 0.0f;
	float m_smoothing_y = 0.0f;

	static constexpr float kMouseScale = 0.005f; // Scale factor for mouse to stick
	static constexpr float kSmoothingFactor = 0.3f; // Lower = more smoothing
};
