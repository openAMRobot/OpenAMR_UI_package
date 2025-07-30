#!/bin/bash

# Compiles the UI for the OpenAMR project and moves it to the ROS2 package.

path=$(pwd) #Store current directory path
echo "Starting build"
cd $path/robotui
npm run build
echo "Build sucessfully completed"
# Rename the built files for compatibility with the HTML template
cd $path/robotui/build/static/js
fileA=$(ls | grep -E  'main' | grep -v '.map' | grep -v '.txt') #Find the main file, excluding map files and the old main.js
echo "Renaming $fileA to main.js" #We do not rename the map file as we'd then need to straight up modify the main.js file
mv $fileA main.js
# Prepare the ROS2 package's UI folder to receive the files
cd $path/openamr_ui_package/openamr_ui_package/static/js
fileB=$(ls | grep -E  'main.[a-z0-9].js.map') #Find the map file
echo "Removing old files"
rm $fileB
rm main.js #The name of the old file is assumed to be main.js because compiled with this file (and also because it works with the HTML template)
# Copy the new files to the ROS2 package
cp $path/robotui/build/static/js/main.js $path/openamr_ui_package/openamr_ui_package/static/js/
cp $path/robotui/build/static/js/$fileA.map $path/openamr_ui_package/openamr_ui_package/static/js/
# Build package (Because the map file's name has changed)
cd $path/../../../
colcon build --packages-select openamr_ui_package --symlink-install
echo "Build completed successfully"