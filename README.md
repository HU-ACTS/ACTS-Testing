# ACTS-Testing
Repository for experimental code and examples.
This repository features branches that are not linked in a direct way.
Instead each branch offers a test case but it also includes a template branch that can be used to setup a working environment project that uses platformio and esp-idf.

## Project Setup
### Required software
- Linux, Mac or Windows
- [Atom IDE](https://atom.io/)
- [Platformio](http://platformio.org/), a plugin that runs on top of Atom
- [ESP-IDF](https://esp-idf.readthedocs.io/en/latest/get-started/index.html)

### Required Hardware
- ESP32

### Installation
1. Have a computer that has the latest version of Linux (distro), Mac or Windows installed.
2. Install the ESP-IDF toolchain. For a very well written guide on how to this: https://esp-idf.readthedocs.io/en/latest/get-started/index.html.
3. Install Atom: https://atom.io/
4. Install the Platformio plugin on top of Atom: http://platformio.org/
Note: Both Atom and Platformio might take a while to install.

## Configuration
### Configuration of Atom/Platformio
When you try to launch Atom, it automagicly launches as platformio (since it is a plugin of atom).
To configure a project that builds for esp32 and runs on it, the platform.ini file requires the following config:
(REQUIRES MORE DETAILED CONFIGURATION EXPLAINATION)

```
platform = espressif32
board = featheresp32
framework = espidf
```

You could also head to the template branch and copy paste the project from there by going to the tab PlatformIO and then selecting open project from that page.

#### Installation of packages
To see installed packages or install new packages, go to the tab ```File -> Settings ```. From there navigate to Packages.

The following packages are recommended to install in atom:
Build (0.68.0+) by noseglid
Busy (0.7.0+) by noseglid 
file-icons (2.1.12+) by file-icons
language-ini (1.19.0+) by jacobbednarz
linter (2.2.0+) by steelbrain (Has cow powers!)
linter-gcc (0.7.1+) by hebaishi
linter-ui-default (1.6.10+) by steelbrain
markdown-preview-plus(2.4.10+) by atom-community
### Configuration of ESP-IDF
ESP-IDF should be configured correctly if it is installed correctly.

### Running your project
~Good luck developing more content from the ACTS Team and me.
## More reads

## Links/References




