# Go1 Free-Dog SDK - Modified for Simple High-Level Control

## Origin
This is a repository forked from: https://github.com/Bin4ry/free-dog-sdk.git


## How to Use

### Highlevel
To run a high-level script, simple follow these steps:

 1. Connect to the dog via WiFi
 2. Make sure you own WiFi IP is 192.168.12.14 (the Dog should assign you this IP automatically!), if not change the connection object accordingly!
 3. Run the following command:
```
python3 calci_startup.py
```
This startup script puts the dog into a standing state no matter of it's original state. Use this if the dog is laying down, it will stand. If it was already standing, it will move slightly.
 4. Run one of the other control scripts:
```
python3 calci_pose.py
python3 calci_pushup.py
python3 calci_dance.py
```
If everything it working correctly, the robot will follow the direction of the control script! :) 
5. When you're done, run the terminate control script. This will make the dog lay back down and go into idle state:
```
python3 calci_terminate.py
```


### Lowlevel (Untested)
There are three LowLevel examples currently hidden in 'extra':

 - example_position(lowlevel).py -> The dog will do positioning with the front right leg.
 - example_torque(lowlevel).py --> The dog will put a torque to the front right leg, you can try to push against the torque to see it in action. (Don't go too hard!)
 - example_velocity(lowlevel).py--> The dog will control the velocity of the front left leg.


#### Prerequisite to be able to run the Lowlevel commands via WiFi
> Execute the following code on the robot's Raspberry Pi：
```
sudo vi /etc/sysctl.conf
```
Remove the comment in front of net.ipv4.ip_forward=1
```
sudo sysctl -p
sudo iptables -F
sudo iptables -t nat -F
sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i wlan1 -o eth0 -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan1 -j ACCEPT
```
Do the following on your own laptop：
```
sudo route add default gw 192.168.12.1
```

After that your can run the Lowlevel examples via WiFi!

#### Run the Lowlevel Example
To communicate to the dog via Lowlevel you need to put the dog into Lowlevel mode first. This will make sure the Highlevel functions of the dog will be disabled. To do that you need to use the RC. On the RC to the following sequence:

```
L2 + A
L2 + A
L2 + B
L1 + L2 + START
```

The dog should be lying on the floor now and you should HEAR that the dog is more silent than before.

Now to run the examples you should put your dog on the back and fold in all legs. This is the safest way!
Run the example of your choice like this:

```
example_velocity(lowlevel).py
```

The dog should move now, fully controlled via Lowlevel. Congratulations :)




## Birds of a Feather?
Looking for Quadruped friends? Stop by "TheDogPound - Animal control for stray robot dogs" Slack group, and join #faux-level and #unitree for support assistance.
https://join.slack.com/t/robotdogs/shared_invite/zt-24ep8mqn4-1p42Aq7owRv9klLI~3C5Pw