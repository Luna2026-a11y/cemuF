#include "input/api/Mouse/MouseController.h"
#include "WindowSystem.h"
#include <cmath>
#include <numbers>

// Static members — mouse camera
std::atomic<float> MouseController::s_delta_x{0.0f};
std::atomic<float> MouseController::s_delta_y{0.0f};
std::atomic<float> MouseController::s_wheel_delta{0.0f};
MouseController::MouseSettings MouseController::s_mouse_settings{};
std::atomic<bool> MouseController::s_capture_active{false};
std::atomic<bool> MouseController::s_left_button{false};
std::atomic<bool> MouseController::s_right_button{false};
uint32 MouseController::s_toggle_key = 0;
std::atomic<float> MouseController::s_wheel_axis{0.0f};

// Static members — gyro simulation
MouseController::GyroSettings MouseController::s_gyro_settings{};
std::atomic<bool> MouseController::s_gyro_enabled{false};
std::atomic<bool> MouseController::s_gyro_active_cache{false};
bool MouseController::s_gyro_prev_key_down = false;
std::atomic<float> MouseController::s_gyro_delta_x{0.0f};
std::atomic<float> MouseController::s_gyro_delta_y{0.0f};
WiiUMotionHandler MouseController::s_motion_handler{};
std::chrono::high_resolution_clock::time_point MouseController::s_last_gyro_time{};

MouseController::MouseController()
	: Controller("mouse-0", "Mouse")
{
	m_connected = true;
	m_last_update = std::chrono::high_resolution_clock::now();
}

std::string MouseController::get_button_name(uint64 button) const
{
	return fmt::format("Mouse Button {}", button);
}

void MouseController::on_mouse_move(int32_t deltaX, int32_t deltaY)
{
	if (!s_capture_active.load())
		return;

	// Stick camera deltas (consumed by get_current_rotation)
	MouseController::s_delta_x.fetch_add(static_cast<float>(deltaX));
	MouseController::s_delta_y.fetch_add(static_cast<float>(deltaY));

	// Gyro deltas (consumed independently by get_gyro_sample)
	MouseController::s_gyro_delta_x.fetch_add(static_cast<float>(deltaX));
	MouseController::s_gyro_delta_y.fetch_add(static_cast<float>(deltaY));
}

void MouseController::on_mouse_wheel(float delta)
{
	if (!s_capture_active.load())
		return;

	MouseController::s_wheel_delta.fetch_add(delta);

	// Add impulse to wheel axis (clamped to [-1, 1])
	float current = s_wheel_axis.load();
	float target = std::max(-1.0f, std::min(1.0f, current + delta * 0.5f));
	s_wheel_axis.store(target);
}

void MouseController::update_mouse_delta()
{
	const float dx = s_delta_x.exchange(0.0f);
	const float dy = s_delta_y.exchange(0.0f);
	const float wheel = s_wheel_delta.exchange(0.0f);

	const auto& settings = s_mouse_settings;
	
	float scale_x = settings.sensitivity * kMouseScale;
	float scale_y = settings.sensitivity * kMouseScale;

	if (settings.invertX)
		scale_x = -scale_x;
	if (settings.invertY)
		scale_y = -scale_y;

	float target_x = dx * scale_x;
	float target_y = dy * scale_y;

	if (std::abs(target_x) < settings.deadzone)
		target_x = 0.0f;
	if (std::abs(target_y) < settings.deadzone)
		target_y = 0.0f;

	m_smoothing_x = m_smoothing_x * (1.0f - kSmoothingFactor) + target_x * kSmoothingFactor;
	m_smoothing_y = m_smoothing_y * (1.0f - kSmoothingFactor) + target_y * kSmoothingFactor;

	m_current_dx = m_smoothing_x;
	m_current_dy = m_smoothing_y;
	m_current_wheel = wheel * 0.1f;

	m_last_update = std::chrono::high_resolution_clock::now();
}

glm::vec2 MouseController::get_current_rotation()
{
	if (!s_capture_active.load())
		return {0.0f, 0.0f};

	const float dx = s_delta_x.exchange(0.0f);
	const float dy = s_delta_y.exchange(0.0f);

	const auto& settings = s_mouse_settings;

	float scale = settings.sensitivity * kMouseScale;

	float rx = dx * scale;
	float ry = dy * scale;

	if (settings.invertX)
		rx = -rx;
	if (settings.invertY)
		ry = -ry;

	// Clamp to [-1, 1]
	rx = std::max(-1.0f, std::min(1.0f, rx));
	ry = std::max(-1.0f, std::min(1.0f, ry));

	return {rx, ry};
}

float MouseController::get_wheel_axis()
{
	if (!s_capture_active.load())
		return 0.0f;

	float val = s_wheel_axis.load();

	// Decay toward zero (smooth release)
	if (std::abs(val) > 0.01f)
		s_wheel_axis.store(val * 0.85f); // decay ~15% per frame
	else
		s_wheel_axis.store(0.0f);

	return std::max(-1.0f, std::min(1.0f, val));
}

bool MouseController::is_gyro_active()
{
	bool result = false;

	if (s_capture_active.load())
	{
		const auto& gs = s_gyro_settings;

		switch (gs.mode)
		{
		case GyroMode::Always:
			result = true;
			break;

		case GyroMode::Toggle:
			if (gs.key != 0)
			{
				const bool pressed = WindowSystem::IsKeyDown(gs.key);
				const bool prev    = s_gyro_prev_key_down;
				s_gyro_prev_key_down = pressed;
				if (pressed && !prev) // rising edge → toggle
					s_gyro_enabled = !s_gyro_enabled.load();
			}
			result = s_gyro_enabled.load();
			break;

		case GyroMode::Hold:
			result = (gs.key != 0) && WindowSystem::IsKeyDown(gs.key);
			break;
		}
	}

	// Cache pour is_gyro_on() (lecture sans effet de bord depuis get_rotation)
	s_gyro_active_cache.store(result);
	return result;
}

MotionSample MouseController::get_gyro_sample()
{
	// Rate-limit : on ne recalcule le filtre Mahony qu'à 120hz max.
	// VPADRead peut être appelé des centaines de fois par seconde selon le jeu ;
	// sans ce garde-fou, le filtre quaternion tournerait en boucle et tuerait le CPU.
	static MotionSample s_cached_sample{};
	constexpr float kMinDeltaTime = 1.0f / 120.0f; // 120hz max

	const auto now = std::chrono::high_resolution_clock::now();

	// Premier appel : initialiser l'horloge et retourner un sample vide
	if (s_last_gyro_time == std::chrono::high_resolution_clock::time_point{})
	{
		s_last_gyro_time = now;
		return s_cached_sample;
	}

	const float elapsed = std::chrono::duration<float>(now - s_last_gyro_time).count();

	// Pas encore le moment : on rend le dernier sample calculé
	if (elapsed < kMinDeltaTime)
		return s_cached_sample;

	// Clamp à 100ms pour éviter les grands sauts après une pause
	const float deltaTime = std::min(elapsed, 0.1f);
	s_last_gyro_time = now;

	// --- Consommer les deltas gyro (séparés des deltas stick) ---
	const float dx = s_gyro_delta_x.exchange(0.0f);
	const float dy = s_gyro_delta_y.exchange(0.0f);

	// --- Convertir pixels → vitesse angulaire (rad/s) ---
	// base_scale ≈ 0.001 rad/px : traverser 1920px ≈ 110°
	const float scale = s_gyro_settings.sensitivity * 0.001f;
	const float gx = dy * scale / deltaTime; // pitch : souris haut/bas
	const float gy = dx * scale / deltaTime; // yaw   : souris gauche/droite
	constexpr float gz = 0.0f;              // roll  : pas de roulis avec la souris

	// --- Vecteur gravité de référence (GamePad à plat, écran vers le haut) ---
	constexpr float accx = 0.0f;
	constexpr float accy = 0.0f;
	constexpr float accz = 1.0f;

	// --- Intégrer dans le filtre Mahony et mettre en cache ---
	s_motion_handler.processMotionSample(deltaTime, gx, gy, gz, accx, accy, accz);
	s_cached_sample = s_motion_handler.getMotionSample();
	return s_cached_sample;
}

ControllerState MouseController::raw_state()
{
	ControllerState result{};

	if (WindowSystem::GetWindowInfo().debugger_focused)
		return result;

	bool should_capture = s_capture_active.load();

	if (s_toggle_key != 0)
	{
		if (WindowSystem::IsKeyDown(s_toggle_key))
			should_capture = true;
	}

	if (!should_capture)
	{
		s_delta_x.store(0.0f);
		s_delta_y.store(0.0f);
		return result;
	}

	update_mouse_delta();

	result.rotation.x = std::max(-1.0f, std::min(1.0f, m_current_dx));
	result.rotation.y = std::max(-1.0f, std::min(1.0f, m_current_dy));

	result.trigger.x = std::max(0.0f, std::min(1.0f, -m_current_wheel));
	result.trigger.y = std::max(0.0f, std::min(1.0f, m_current_wheel));

	return result;
}
