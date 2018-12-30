# Setting up the network

The current setup consists of three different subnets. These are managed by the
base-station computer, which has three different NICs (network interface cards):

- One wireless NIC. Provides WAN access (Internet connectivity). Acts as the
  default gateway for the internal subnets. As most wireless routers provide
  their clients with an IP address via DHCP, this card's IP address is not
  statically set like the other two.

- Two wired NICs. Both are set up on LANs, one for the WolvMarine and one for
  local communication to the judges during the competition. The judge's network
  is unknown until the time of the competition, so must be set up quickly on
  the first day. The WolvMarine's local network is currently assigned to the
  `192.168.1.0` network with mask `255.255.255.0`.

So, the basic setup runs a router on the base-station computer which handles
any out-of-network requests from the two internal networks. None of the
computers utilize the uncomplicated fire wall (ufw) that ships with Ubuntu.
This program is purged. Then, the base-station computer sets some `iptables`
rules to handle and route requests. The remote computer should drop any
requests that it didn't receive from the base-station computer.

This setup was forwards Internet connectivity of the base-station computer to
the remote computer, so that software updates can be performed easily.

## Components of the `192.168.1.0/24` network

- `192.168.1.101` : The radio access point on the base-station side.
- `192.168.1.102` : The radio point-to-point station on the remote side.
- `192.168.1.103` : The remote computer, running on the robot.
- `192.168.1.104` : The base-station computer.
- `192.168.1.106` : The cRIO computer, running on the robot, currently
  communicating on port `50000`.
- `192.168.1.201` : The stern Velodyne LiDAR, running on the robot, currently
  communicating on port `2368`.
- `192.168.1.202` : The bow Velodyne LiDAR, running on the robot, currently
  communicating on port `2360`.

## Setting up ssh

  After setting up ssh keys, password-based login should be disabled.

## Setting up `iptables` on the base-station computer

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
