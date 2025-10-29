# TODO

apt-get install python3-rosdep
sudo rosdep init

rosdep update

rosdep install --from-paths src -y --ignore-src

sudo apt-get install luarocks

luarocks install dkjson
luarocks install jsonschema


# Install new version 1.89 of boost (for lockfree triple buffer)
#This does not eliminate the current version of Boost (safe to do!)
cd ~/Downloads
wget -O boost_1_89_0.tar.gz 'https://sourceforge.net/projects/boost/files/boost/1.89.0/boost_1_89_0.tar.gz/download'
tar -xvzf "boost_1_89_0.tar.gz"
rm boost_1_89_0.tar.gz
cd boost_1_89_0/
sudo mkdir -p /opt/boost/1.89.0
sudo chown $USER:$USER /opt/boost/1.89.0 #Install new boost in parallel without changing currently installed BOOST version
./bootstrap.sh --prefix=/opt/boost/1.89.0
./b2 -j$(nproc) install
# Clean downloaded files
cd ~/Downloads
sudo rm -rf ~/Downloads/boost_1_89_0/
