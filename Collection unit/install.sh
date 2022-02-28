### Install Docker ###
sudo chmod +x ./.install/docker-install.sh
sudo source ./.install/docker-install.sh

### Configure Kiosk ###
sudo cp ./.install/autostart ~/.config/lxsession/LXDE-pi/autostart
sudo cp ./.install/start-kiosk.sh ~/start-kiosk.sh
sudo chmod +x ~/start-kiosk.sh

### Configure Screensaver ###
sudo chmod +x ./.install/configure-screensaver.sh
sudo source ./.install/configure-screensaver.sh

### Moving files to the right directory ###
sudo cp ./.install/docker-compose.yaml ~/docker-compose.yaml
sudo mv ./install.sh ./.install/install.sh

### Start Docker containers ###
sudo docker-compose up -d

### Reboot ###
sudo reboot now