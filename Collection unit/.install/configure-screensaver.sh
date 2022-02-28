sudo echo "disable_splash=1" >> /boot/config.txt
sudo sed -i 's/console=tty1/console=tty3/g' /boot/cmdline.txt
sudo echo "splash quiet plymouth.ignore-serial-consoles logo.nologo vt.global_cursor_default=0" >> /boot/cmdline.txt
sudo cp ./splash.png /usr/share/plymouth/themes/pix/splash.png