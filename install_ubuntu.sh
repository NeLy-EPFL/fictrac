#!/bin/sh

echo
echo "+------------------------------+"
echo "|    FicTrac install script    |"
echo "+------------------------------+"
echo

# Get Ubuntu version	
ver="$(lsb_release -sr)"
echo "Found Ubuntu version $ver"

if [ "$ver" = "22.04" ] || [ "$ver" = "20.04" ]; then
	FICTRAC_DIR="$(dirname "$0")"
	if [ ! -d "$FICTRAC_DIR/build" ]; then
		echo
		echo "+-- Installing dependencies ---+"
		echo
		sudo apt update
		sudo apt install -y gcc g++ cmake libavcodec-dev libnlopt-dev libboost-dev libopencv-dev
	else
		echo "Previous build detected, so skipping dependency installation."
	fi

	echo
	echo "+-- Creating build directory --+"
	echo
	cd "$FICTRAC_DIR"	# make sure we are in fictrac dir
	if [ -d ./build ]; then
		echo "Removing existing build dir"
		rm -r ./build
	fi
	mkdir build
	if [ -d ./build ]; then
		echo "Created build dir"
		cd ./build
	else
		echo "Uh oh, something went wrong attempting to create the build dir!"
		exit
	fi
	
	echo
	echo "+-- Generating build files ----+"
	echo
	cmake ..
	
	echo
	echo "+-- Building FicTrac ----------+"
	echo
	cmake --build . --config Release --parallel $(nproc) --clean-first
	
	cd ..
	if [ -f ./bin/fictrac ]; then
		echo
		echo "FicTrac built successfully!"
		echo
	else
		echo
		echo "Hmm... something seems to have gone wrong - can't find FicTrac executable."
		echo
	fi
fi
