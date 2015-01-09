#!/bin/bash
sudo -v

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

BUILD_DIR="$DIR/build"
echo Code will be built in $BUILD_DIR
read -p "Press any key to continue." -n1 -s
echo -e '\n'

# Fixes USB permissions issue
if !(groups $USER | grep &>/dev/null '\bdialout\b'); then
  sudo adduser $USER dialout
  RESTART=true
fi

if [ -d "$BUILD_DIR" ]; then
  cd build
else
  mkdir build
  cd build
fi

cmake ..
make

INSTALL_DIR=/opt/kmeliface
if [ ! -d "$INSTALL_DIR" ]; then
  read -p "Do you want to install the kqi driver to $INSTALL_DIR? [Y/n] " var
  if [[ $var == 'Y' ]] || [[ $var == 'y' ]]; then
    sudo mkdir /opt/kmeliface
    sudo cp -r $DIR/* $INSTALL_DIR/
    sudo rm $INSTALL_DIR/install.sh
  fi
fi

if [ "$RESTART" == true ]; then
  echo "Reboot is required. Restart now? [Y/n]"
  read var
  if [[ $var == 'Y' ]] || [[ $var == 'y' ]]; then
    sudo shutdown -r now
  fi
else
  echo Done.
fi
