sudo cp ./80-can.network /lib/systemd/network/
sudo systemctl start systemd-networkd.service
sudo systemctl enable systemd-networkd.service
sudo cp evpi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start evpi.service   
sudo systemctl enable evpi.service  
