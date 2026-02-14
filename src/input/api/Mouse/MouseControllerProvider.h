#pragma once

#include "input/api/ControllerProvider.h"

class MouseControllerProvider : public ControllerProviderBase
{
public:
	MouseControllerProvider();

	inline static InputAPI::Type kAPIType = InputAPI::Mouse;
	InputAPI::Type api() const override { return kAPIType; }

	std::vector<std::shared_ptr<ControllerBase>> get_controllers() override;

private:
	bool m_init = true;
};
