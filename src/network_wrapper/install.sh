echo "Clone this repo and place it where you would like to have all the detection/model API installed and run this script in the root of that directory."

echo "Install jetson nano swap memory of 6GB. Reboot to take effect"
./install_scripts/install_nano_swap.sh

echo "Install Tensorflow and dependencies"
./install_scripts/install_deps.sh

echo "Install TF models and detection API"
./install_scripts/install_tf_models.sh

sudo apt install curl
curl https://bootstrap.pypa.io/pip/3.6/get-pip.py | sudo python3

echo "done!"
