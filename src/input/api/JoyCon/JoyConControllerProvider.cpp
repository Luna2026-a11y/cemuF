#include "input/api/JoyCon/JoyConControllerProvider.h"
#include "input/api/JoyCon/JoyConController.h"

#include <hidapi/hidapi.h>

static constexpr uint16_t JOYCON_VID  = 0x057E;
static constexpr uint16_t LEFT_PID    = 0x2006;
static constexpr uint16_t RIGHT_PID   = 0x2007;

std::vector<std::shared_ptr<ControllerBase>> JoyConControllerProvider::get_controllers()
{
	std::vector<std::shared_ptr<ControllerBase>> result;

	hid_init();

	auto enumerate = [&](uint16_t pid, JoyConController::Side side)
	{
		auto* devs = hid_enumerate(JOYCON_VID, pid);
		for (auto* it = devs; it != nullptr; it = it->next)
		{
			try
			{
				result.push_back(std::make_shared<JoyConController>(it->path, side));
			}
			catch (const std::exception& ex)
			{
				cemuLog_log(LogType::Force, "JoyCon: failed to create controller for {}: {}",
					it->path, ex.what());
			}
		}
		hid_free_enumeration(devs);
	};

	enumerate(LEFT_PID,  JoyConController::Side::Left);
	enumerate(RIGHT_PID, JoyConController::Side::Right);

	return result;
}
