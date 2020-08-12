#!/bin/sh

set -e

FILES='lib_inav_flow lib_inav_baro lib_inav_gps lib_inav_rangefinder'

if [ $# -ne 1 ]; then
	echo "no version input exit"
	exit
else
	echo "version:" $1
	sleep 2
fi

echo '\033[0;32;1m'
echo "Start"
echo '\033[0m'

rm -rf *
git reset --hard
git pull

git checkout develop
rm -rf *
git reset --hard
git pull

echo '\033[0;32;1m'
echo "Copy Files"
echo '\033[0m'

rm -rf ../release_backup
mkdir ../release_backup
cp -r * ../release_backup/

git checkout release

rm -rf * 
cp -r ../release_backup/* ./

echo '\033[0;32;1m'
echo "Build"
echo '\033[0m'

make ARCH=EVAB2

mkdir -p bin
for obj in ${FILES};do
    cp objs/${obj}.o bin/
done

make clean

for file in ${FILES};do
    rm -f libraries/${file}.c
done

rm -f doc/*.docx

echo '\033[0;32;1m'
echo "Done"
echo '\033[0m'

GITID=$(git rev-parse develop)
git add -A *
git commit -m $1"   commit id:("$GITID")"

echo '\033[0;32;1m'
echo "Release Ready"
echo '\033[0m'

