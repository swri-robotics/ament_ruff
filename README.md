# `ament_ruff`

Python linting and formatting based on [ruff](https://github.com/astral-sh/ruff) with `ament` integration.

## Usage

### `ament_ruff`

See [here](ament_ruff/doc/index.rst).

### `ament_cmake_ruff`

See [here](ament_cmake_ruff/doc/index.rst).

## Installation

Install ruff first:

```bash
pip install ruff

# You may have to set your path to include ruff's binary location:
export PATH=$PATH:$HOME/.local/bin

# Or do so permanently in bash
echo "export PATH=$PATH:$HOME/.local/bin" >> .bashrc
```

Clone this repository into your workspace and build it:

```bash
# make a workspace if you don't have one
mkdir -p ~/ws/ament_ruff/src
cd ~/ws/ament_ruff/src
git clone https://github.com/swri-robotics/ament_ruff.git

cd ..
colcon build
source install/setup.bash
```