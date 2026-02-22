#pragma once

#include <wx/dialog.h>
#include <wx/timer.h>
#include <wx/slider.h>
#include <wx/choice.h>
#include <wx/textctrl.h>

#include "input/api/Controller.h"
#include "input/api/Mouse/MouseController.h"

class wxCheckBox;
class wxStaticText;
class wxInputDraw;

class DefaultControllerSettings : public wxDialog
{
public:
	DefaultControllerSettings(wxWindow* parent, const wxPoint& position, ControllerPtr controller);
	~DefaultControllerSettings();

private:
	void update_settings();
	static wxString gyro_key_name(uint32 vk);

	ControllerPtr m_controller;
	ControllerBase::Settings m_settings;
	float m_rumble_backup;

	wxTimer* m_timer;
	std::optional<std::chrono::steady_clock::time_point> m_rumble_time{};

	wxSlider* m_axis_deadzone, *m_axis_range;
	wxSlider* m_rotation_deadzone, *m_rotation_range;
	wxSlider* m_trigger_deadzone, *m_trigger_range;
	wxSlider* m_rumble;

	wxCheckBox* m_use_motion = nullptr;

	// Mouse gyro settings (visible only when controller api == Mouse)
	wxChoice*   m_gyro_mode        = nullptr;
	wxTextCtrl* m_gyro_key_text    = nullptr;
	wxSlider*   m_gyro_sensitivity = nullptr;
	uint32      m_gyro_key_code    = 0; // Windows VK code

	// Joy-Con gyro settings (visible only when controller api == JoyCon)
	wxStaticText* m_joycon_calib_status = nullptr;

	wxInputDraw* m_axis_draw, * m_rotation_draw, *m_trigger_draw;

	void on_timer(wxTimerEvent& event);
	void on_close(wxCloseEvent& event);
	void on_deadzone_change(wxCommandEvent& event);
	void on_range_change(wxCommandEvent& event);
	void on_rumble_change(wxCommandEvent& event);
	void on_gyro_mode_change(wxCommandEvent& event);
	void on_gyro_key_press(wxKeyEvent& event);
};
