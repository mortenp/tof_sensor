# Multiple targets par zone
This example shows the possibility of VL53L5CX to get/set params. It initializes the VL53L5CX ULD, set a configuration, and starts ranging.
## Get started
You need to increase the main stack size else you will get a stack overflow error.
Go to Component Config -> ESP System settings and increase the Main task stack size to at least `7168`.

