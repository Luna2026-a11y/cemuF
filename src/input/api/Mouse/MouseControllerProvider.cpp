#include "input/api/Mouse/MouseControllerProvider.h"
#include "input/api/Mouse/MouseController.h"

MouseControllerProvider::MouseControllerProvider()
{
	m_init = true;
}

std::vector<std::shared_ptr<ControllerBase>> MouseControllerProvider::get_controllers()
{
	// Mouse is always available (if window has focus)
	return { std::make_shared<MouseController>() };
}
