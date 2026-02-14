import re

with open('/home/stagiaire/.openclaw/workspace/cemu/src/input/emulated/VPADController.cpp', 'r') as f:
    content = f.read()

# Find the position after the switch cases and before "bool mapping_updated"
# The closing braces of the switch are: 	}
# }

# Look for the pattern: 	}
# }

# bool mapping_updated = false;
old_pattern = '''\t\tbreak;
\t}
}

bool mapping_updated = false;'''

new_text = '''\t\tbreak;
\t}
\tcase InputAPI::Mouse:
\t{
\t\t// Mouse controller provides rotation values directly for right stick (camera)
\t\t// No buttons, just the rotation axis
\t\tmapping =
\t\t{
\t\t\t// Mouse rotation maps to right stick for camera control
\t\t\t{kButtonId_StickR_Up, kRotationYN},
\t\t\t{kButtonId_StickR_Down, kRotationYP},
\t\t\t{kButtonId_StickR_Left, kRotationXN},
\t\t\t{kButtonId_StickR_Right, kRotationXP},
\t\t};
\t\tbreak;
\t}
}

bool mapping_updated = false;'''

if old_pattern in content:
    content = content.replace(old_pattern, new_text)
    with open('/home/stagiaire/.openclaw/workspace/cemu/src/input/emulated/VPADController.cpp', 'w') as f:
        f.write(content)
    print("Successfully patched VPADController.cpp")
else:
    print("Pattern not found - checking what's there...")
    # Try to find similar patterns
    import subprocess
    result = subprocess.run(['grep', '-n', 'bool mapping_updated', '/home/stagiaire/.openclaw/workspace/cemu/src/input/emulated/VPADController.cpp'], capture_output=True, text=True)
    print(f"mapping_updated found at line: {result.stdout.strip()}")
