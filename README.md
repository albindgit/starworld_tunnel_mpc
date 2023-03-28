# Starworld tunnel-MPC


## Installation
Clone repo and initialize submodule
```
git clone https://github.com/albindgit/starworld_tunnel_mpc.git
cd starworld_tunnel_mpc/starworlds
git submodule init
git submodule update
cd ..
```
Create and activate virtual environment
```
python -m venv venv
. venv/bin/activate
```
Install package
```
pip install -e .
pip install -e starworlds
```
