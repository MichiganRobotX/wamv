# Setting up the network

##############################
###   SET IPTABLES RULES   ###
##############################
# Set default policies
sudo iptables -P INPUT DROP
sudo iptables -P FORWARD DROP

# Accept all localhost traffic (local traffic to the system itself)
sudo iptables -A INPUT -i lo -j ACCEPT

# Accept all traffic coming from the local subnets.
sudo iptables -A INPUT -i enp0s31f6 -j ACCEPT

# Accept traffic, coming from the WAN interface, that is related to established connections
sudo iptables -A INPUT -i wlp1s0 -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

# Forward requests from the local subnets.
sudo iptables -A FORWARD -i enp0s31f6 -o wlp1s0 -j ACCEPT
sudo iptables -A FORWARD -i wlp1s0 -o enp0s31f6 -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

# Let traffic on the interfaces which we are doing routing for go out to the
# Internet masqueraded as our Internet-connected interface.
sudo iptables -t nat -A POSTROUTING -o wlp1s0 -j MASQUERADE
##################################
###   END SET IPTABLES RULES   ###
##################################
