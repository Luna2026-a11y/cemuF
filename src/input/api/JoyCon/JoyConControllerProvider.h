#pragma once

#include "input/api/ControllerProvider.h"

// Provides Joy-Con controllers found via Bluetooth HID (hidapi).
// Each connected Joy-Con (Left 0x2006 / Right 0x2007) becomes one JoyConController.
// The controller is gyro-only: has_motion()=true, no button mapping required.
class JoyConControllerProvider : public ControllerProviderBase
{
public:
	inline static InputAPI::Type kAPIType = InputAPI::JoyCon;
	InputAPI::Type api() const override { return kAPIType; }

	// Scans for connected Joy-Cons via hidapi and returns one controller per device.
	std::vector<std::shared_ptr<ControllerBase>> get_controllers() override;
};
