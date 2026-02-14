#pragma once

#include "input/api/ControllerProvider.h"

class MouseControllerProvider : public ControllerProviderBase
{
public:
	MouseControllerProvider();

	InputAPI::Type api() const override { return InputAPI::Mouse; }
	std::string_view api_name() const override { return "Mouse"; }

	bool is_init() const override { return m_init; }

	ControllerProviderSettings* get_settings() override { return nullptr; }

	std::vector<ControllerPtr> get_controllers() override;

private:
	bool m_init = true;
};
