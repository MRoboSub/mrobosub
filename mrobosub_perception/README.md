# How to install torch, torchvision on fresh jetson

Updated Apr 2026 by Muskaan Mittal (muskaan@umich.edu)

Sorry I don't have more detailed instructions, installing this drove me insane.

Basically, you have to install cuda (if cuda wasn't already installed when the jetpack was), torch, and torchvision

## STEP 1. Find out Jetpack version
At time of writing doc, we are using jetpack 6.1; which works well with CUDA 12.6

## STEP 2: Install appropriate CUDA

## STEP 3: Install appropriate torch. You CANNOT do pip install torch
I used the instructions here: https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
and found out the appropriate version of torch using some hunting

the export TORCH_INSTALL url came out to https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/ (... insert appropriate version)

## STEP 4: Install other dependencies and test torch installation
Test installation by running this:
python3 -c "import torch; print(torch.__version__)"
python3 -c "import torch; print(torch.cuda.is_available())"

You'll probably run into a whole host of python packages you don't have. Just sudo apt/ pip install them.

## STEP 5: Install torchvision
There are pre-built wheels but there were none that matched CUDA 12.6, so I built from source which took about 30 min

These are the instructions Claude gave me; some iteration of them worked successfully:

### Install dependencies
sudo apt-get install -y libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev

### Clone torchvision
git clone https://github.com/pytorch/vision.git
cd vision
git checkout tags/v0.20.0

### Build and install
sudo python3 setup.py install

You can test whether this worked by opening up a python shell and making sure "import torchvision" runs successfully.