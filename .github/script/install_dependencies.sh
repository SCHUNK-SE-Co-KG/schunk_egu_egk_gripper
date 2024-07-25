#/usr/bin/bash
cd $HOME
apt-get install -y curl libcurl4-openssl-dev

# Python dependencies
python_deps="fastapi uvicorn httpx requests coverage"
os_name=$(lsb_release -cs)

case $os_name in
  jammy) # Ubuntu 22.04
    pip install --user $python_deps
    ;;
  kinetic) # Ubuntu 24.04
    pip install --break-system-packages $python_deps
    ;;
  *) # Newer
    pip install --break-system-packages $python_deps
    ;;
esac
