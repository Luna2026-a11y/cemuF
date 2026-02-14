#include "input/api/Mouse/MouseController.h"
#include "WindowSystem.h"
#include <cmath>

// Static members
std::atomic<float> MouseController::s_delta_x{0.0f};
std::atomic<float> MouseController::s_delta_y{0.0f};
std::atomic<float> MouseController::s_wheel_delta{0.0f};
MouseController::MouseSettings MouseController::s_mouse_settings{};
std::atomic<bool> MouseController::s_capture_active{false};
uint32 MouseController::s_toggle_key = 0; // 0 = no toggle key set

MouseController::MouseController()
	: base_type("mouse", "Mouse for Camera")
{
	m_connected = true;
	m_last_update = std::chrono::high_resolution_clock::now();
}

std::string MouseController::get_button_name(uint64 button) const
{
	// Mouse doesn't have buttons exposed here (mouse clicks are handled by WindowSystem)
	return fmt::format("Mouse Button {}", button);
}

void MouseController::on_mouse_move(int32 deltaX, int32 deltaY)
{
	if (!s_capture_active.load())
		return;

	// Accumulate deltas (atomic for thread safety)
	MouseController::s_delta_x.fetch_add(static_cast<float>(deltaX));
	MouseController::s_delta_y.fetch_add(static_cast<float>(deltaY));
}

void MouseController::on_mouse_wheel(float delta)
{
	if (!s_capture_active.load())
		return;

	MouseController::s_wheel_delta.fetch_add(delta);
}

void MouseController::update_mouse_delta()
{
	// Take ownership of accumulated deltas
	const float dx = s_delta_x.exchange(0.0f);
	const float dy = s_delta_y.exchange(0.0f);
	const float wheel = s_wheel_delta.exchange(0.0f);

	// Apply settings
	const auto& settings = s_mouse_settings;
	
	float scale_x = settings.sensitivity * kMouseScale;
	float scale_y = settings.sensitivity * kMouseScale;

	if (settings.invertX)
		scale_x = -scale_x;
	if (settings.invertY)
		scale_y = -scale_y;

	// Calculate acceleration curve (quadratic for finer small movements)
	float target_x = dx * scale_x;
	float target_y = dy * scale_y;

	// Apply deadzone
	if (std::abs(target_x) < settings.deadzone)
		target_x = 0.0f;
	if (std::abs(target_y) < settings.deadzone)
		target_y = 0.0f;

	// Smoothing (exponential moving average)
	m_smoothing_x = m_smoothing_x * (1.0f - kSmoothingFactor) + target_x * kSmoothingFactor;
	m_smoothing_y = m_smoothing_y * (1.0f - kSmoothingFactor) + target_y * kSmoothingFactor;

	m_current_dx = m_smoothing_x;
	m_current_dy = m_smoothing_y;
	m_current_wheel = wheel * 0.1f; // Scale wheel for triggers

	m_last_update = std::chrono::high_resolution_clock::now();
}

ControllerState MouseController::raw_state()
{
	ControllerState result{};

	if (WindowSystem::GetWindowInfo().debugger_focused)
		return result;

	// Check if capture should be active
	// Can be toggled via:
	// 1. Explicit set_capture_active()
	// 2. Toggle key held
	// 3. Mouse in window with focus
	bool should_capture = s_capture_active.load();

	if (s_toggle_key != 0)
	{
		if (WindowSystem::IsKeyDown(s_toggle_key))
			should_capture = true;
	}

	if (!should_capture)
	{
		// Reset deltas when not capturing
		s_delta_x.store(0.0f);
		s_delta_y.store(0.0f);
		return result;
	}

	// Update mouse deltas
	update_mouse_delta();

	// Store rotation values (maps to right stick)
	// Clamp to [-1, 1] range
	result.rotation.x = std::max(-1.0f, std::min(1.0f, m_current_dx));
	result.rotation.y = std::max(-1.0f, std::min(1.0f, m_current_dy));

	// Wheel can be mapped to triggers
	result.trigger.x = std::max(0.0f, std::min(1.0f, -m_current_wheel));
	result.trigger.y = std::max(0.0f, std::min(1.0f, m_current_wheel));

	return result;
}
