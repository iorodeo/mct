echo -e "\n\nLoading simple rc.firewall-iptables version $FWVER..\n"
DEPMOD=/sbin/depmod
MODPROBE=/sbin/modprobe

EXTIF="eth0"
INTIF1="eth1"
INTIF2="eth2"
INTIF3="eth3"
INTIF4="eth4"

echo "   External Interface:  $EXTIF"
echo "   Internal Interface 1:  $INTIF1"
echo "   Internal Interface 2:  $INTIF2"
echo "   Internal Interface 3:  $INTIF3"
echo "   Internal Interface 4:  $INTIF4"

#======================================================================
#== No editing beyond this line is required for initial MASQ testing == 
echo -en "   loading modules: "
echo "  - Verifying that all kernel modules are ok"
$DEPMOD -a
echo "----------------------------------------------------------------------"
echo -en "ip_tables, "
$MODPROBE ip_tables
echo -en "nf_conntrack, " 
$MODPROBE nf_conntrack
echo -en "nf_conntrack_ftp, " 
$MODPROBE nf_conntrack_ftp
echo -en "nf_conntrack_irc, " 
$MODPROBE nf_conntrack_irc
echo -en "iptable_nat, "
$MODPROBE iptable_nat
echo -en "nf_nat_ftp, "
$MODPROBE nf_nat_ftp
echo "----------------------------------------------------------------------"
echo -e "   Done loading modules.\n"
echo "   Enabling forwarding.."
echo "1" > /proc/sys/net/ipv4/ip_forward
echo "   Enabling DynamicAddr.."
echo "1" > /proc/sys/net/ipv4/ip_dynaddr 
echo "   Clearing any existing rules and setting default policy.."

iptables-restore <<-EOF
*nat
-A POSTROUTING -o "$EXTIF" -j MASQUERADE
COMMIT
*filter
:INPUT ACCEPT [0:0]
:FORWARD DROP [0:0]
:OUTPUT ACCEPT [0:0]
-A FORWARD -i "$EXTIF" -o "$INTIF1" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT 
-A FORWARD -i "$EXTIF" -o "$INTIF2" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT 
-A FORWARD -i "$EXTIF" -o "$INTIF3" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT 
-A FORWARD -i "$EXTIF" -o "$INTIF4" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT 
-A FORWARD -i "$INTIF1" -o "$EXTIF" -j ACCEPT
-A FORWARD -i "$INTIF2" -o "$EXTIF" -j ACCEPT
-A FORWARD -i "$INTIF3" -o "$EXTIF" -j ACCEPT
-A FORWARD -i "$INTIF4" -o "$EXTIF" -j ACCEPT

-A FORWARD -j LOG
COMMIT
EOF

echo -e "\nrc.firewall-iptables v$FWVER done.\n"
