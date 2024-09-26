# Enables some extra cmake features
cmake_minimum_required(VERSION 3.1)

# Defines the project. The version should match the version number of the board
# this code is for. The build system will generally guess <project_name>.toml
# as the serial config file, but it can also deal with many cases where the names
# don't match
project(bmb VERSION 6.0)

# Overrides the automatic serial config detection
# Generally, the build system can infer the serial config
# from your project name, so this shouldn't usually be necessary
#set(AVR_SERIAL_CONF test_device.toml)

# Overrides the default clock frequency of the device
# 32MHz is the default, so most projects shouldn't need this
# If your board changes clock to something else, this var must
# be set, or serial will not work correctly
#set(AVR_CLOCK_FREQ 32000000)

avr_add_serial_executable(bmb bmb.c)
