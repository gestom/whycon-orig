#cleanup
rm -rf dists
mkdir dists
rm -rf obj-x86_64-linux-gnu
rm -rf debian

#create the package 
bloom-generate rosdebian --ros-distro kinetic 
source ../../devel/setup.bash
fakeroot debian/rules binary
#organise the repo 

#repo reconstruction
cd dists
mv ../../*.deb .
mkdir -p xenial/main/binary-amd64
mkdir -p xenial/main/binary-i386
cp ../*.deb .
cp ../keyFile .

#sign the files
sudo dpkg-sig --sign builder ros-kinetic-whycon-ros_0.0.1-0xenial_amd64.deb

cd xenial
dpkg-scanpackages . /dev/null  > main/binary-i386/Packages

#create package list
mv ../ros-kinetic-whycon-ros_0.0.1-0xenial_amd64.deb .
dpkg-scanpackages . /dev/null  > main/binary-amd64/Packages
mv ros-kinetic-whycon-ros_0.0.1-0xenial_amd64.deb ../../

#create releases 
apt-ftparchive release . > Release
gpg --clearsign -o InRelease Release
gpg -abs -o Release.gpg Release
cd ../../ 
scp *.deb tkrajnik@labe.felk.cvut.cz:/home/doktor/tkrajnik/htdocs/packages/
scp -r dists tkrajnik@labe.felk.cvut.cz:/home/doktor/tkrajnik/htdocs/packages/
