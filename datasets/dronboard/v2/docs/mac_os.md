# Mac OS Notes

## Setup
I needed to follow PX4's [Mac OS Development Environment](https://docs.px4.io/master/en/dev_setup/dev_env_mac.html) setup instructions but with modifications.

The commands must be prefixed with `arch -x86_64` because I'm using Apple's ARM CPU. The PX4 development setup doesn't work on ARM yet.

The PX4 devs might fix the underlying problem in the future. But for now, here's what I ran (including the workarounds):

>Note: When testing in a Mac OS VM, the terminal didn't launch with rosetta until I ran:
>```bash
>softwareupdate --install-rosetta
>```

```bash
arch -x86_64 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
echo 'eval "$(arch -x86_64 /usr/local/bin/brew shellenv)"' >> "$HOME/.zprofile"
```

Before trying to build/run PX4 or Gazebo, you must set these aliases:
```
alias pip3="arch -x86_64 /usr/bin/pip3"
alias python3="arch -x86_64 /usr/bin/python3"
alias brew="arch -x86_64 /usr/local/Homebrew/bin/brew"
```

Or you can add this to your `.zprofile`:
```zsh
echo 'alias pip3="arch -x86_64 /usr/bin/pip3"' >> $HOME/.zprofile
echo 'alias python3="arch -x86_64 /usr/bin/python3"' >> $HOME/.zprofile
echo 'alias brew="arch -x86_64 /usr/local/Homebrew/bin/brew"' >> $HOME/.zprofile
```

```bash
brew tap PX4/px4
brew install px4-dev

python3 -m pip install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema

# this is a work around
brew unlink tbb
brew uninstall --force tbb
brew install tbb@2020
brew link tbb@2020

brew install --cask temurin
brew install --cask xquartz
brew install px4-sim-gazebo

git clone --recurse-submodules --branch v1.13.0 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot/Tools/setup
# export PATH="$PATH:$HOME/Library/Python/3.8/bin"
echo 'export PATH="$PATH:$HOME/Library/Python/3.8/bin"' >> "$HOME/.zprofile"

source macos.sh
```

### Install QGroundControl

[Here's the download page at qgroundcontrol.com](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).

Getting it to run on Mac OS is a little tricky because the program is not signed which causes a problem.

Be sure to read the notes below the `.dmg` download link.

## Patches!

### PX4
Without these changes PX4 didn't compile. Also, since the goal is to run mavros in a container, I updated the mavlink networking config too. Finally I changed a few parameters so that the hardware-tests will run.  To apply the patch:

1. Download or copy the [`macos-fixes.patch`](patch-files/macos-fixes.patch) file to your computer.
2. Apply the patch:
    ```bash
    cd PX4-Autopilot
    # note you might need to cp the patch file to the working directory:
    # cp /path/to/macos-fixes.patch "$PWD/macos-fixes.patch"
    git apply macos-fixes.patch
    ```

You may run into an error where PX4 won't compile because it is unable to find Qt. To resolve this issue, confirm where you have Qt installed. Using brew with arch=i386 (intel, or rosetta on arm64), it is likely installed in the following location:
```bash
/user/local/opt
```
You can then set the Qt5_DIR environment variable to the location of the Qt5Config.cmake under the Qt5 application directory. ie:
```bash
export Qt5_DIR="/usr/local/opt/qt@5/lib/cmake/Qt5/"
```

### sitl_gazebo

The drone is dark gray and the background is dark gray. This makes the drone too hard to see (especially while flying).
This patch makes the drone red.

1. Download or copy the [`red-iris.patch`](patch-files/red-iris.patch) file to your computer.
2. Apply the patch
    ```bash
    cd PX4-Autopilot/Tools/sitl_gazebo
    # you might need to copy the patch file to this working directory
    # cp /path/to/red-iris.patch . 
    git apply red-iris.patch
    ```


## Running PX4 and Gazebo

Start gazebo and PX4 using the terminal:

```bash
cd PX4-Autopilot
# Fly near Notre Dame (most missions expect)
export PX4_HOME_LAT=41.70573086523541
export PX4_HOME_LON=-86.24421841999177
export PX4_HOME_ALT=218.566
make px4_sitl gazebo
```

Note: the Typhoon H480's camera doesn't work and it causes the terminal to fill with a repeating error message. So the commands above will simulate the Iris.


Sometimes it's helpful to simulate in a 3D environment: 
```bash
cd PX4-Autopilot
# Fly at Baylands Park near San Jose 37.413534 -121.996561
export PX4_HOME_LAT=37.413534
export PX4_HOME_LON=-121.996561
export PX4_HOME_ALT=1.3
make px4_sitl gazebo___baylands
```
Note: this is slower