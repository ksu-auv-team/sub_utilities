sudo apt update
sudo apt install curl libhdf5-serial-dev hdf5-tools
sudo apt install htop nmap vim
sudo apt install python3-pip
sudo apt-get install python3-pandas

#sudo apt purge evolution*
sudo pip3 install -U pip

sudo apt-get install zlib1g-dev zip libjpeg8-dev libhdf5-dev hdf5-tools
sudo pip3 install -U numpy==1.16 scipy==1.1.0 grpcio==1.22.1 absl-py py-cpuinfo psutil portpicker grpcio six mock requests gast==0.2.2 h5py astor termcolor jupyter
sudo pip3 install protobuf==3.6.1
sudo pip3 install tqdm
sudo pip3 install sklearn

sudo pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v42 tensorflow_gpu==1.13.1+nv19.4
sudo pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v43 tensorflow_gpu==1.15.0+nv20.1