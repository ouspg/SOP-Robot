# FacsAU.msg
# Defines an Action Unit activation command for the FACS expression system.
# Multiple AUs can be blended simultaneously by the FACS node.

# FACS Action Unit number (e.g., 1=inner brow raise, 6=cheek raise, 12=lip pull)
int32 au_id

# 0.0 = neutral, 1.0 = full activation
float64 intensity

# Arbitration: higher priority overrides lower (3 > 2 > 1)
int32 priority

# Transition duration in nanoseconds
int64 duration_nanosecs  
