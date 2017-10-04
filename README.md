# ACTS-Testing template branch
This branch contains a cpp ready template that can be used to create new esp-idf projects from.

## Project features
The project template contains several elements to test a basic setup using the esp32.
Here is a list of project features

- CPP11x ready template
- Setup of a wifi network using the esp32
- Small example to get up to speed

## Remaining tasks
- [x] Make cpp ready
- [ ] Make a wifi template
- [ ] Add an example
- [ ] Add documentation

## General
### platformio settings
It is recommended to turn verbose output on when compiling.
This can be done by inserting the following command within a platformio terminal
```
platformio settings set force_verbose 1
```
To read more about platformio command settings:
http://docs.platformio.org/en/latest/userguide/cmd_settings.html

#### platformio.ini
In the ini file, the monitor_baud is set to 115200 bps
```
monitor_baud = 115200
```
