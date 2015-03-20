#!/bin/bash
apt-get install python3-PyQt4 -y
a=`pwd`
echo "#!/bin/bash
cd $a
python3 $a/main.py" > launcher.sh

chmod +x launcher.sh

echo ""
echo "You can open the software by executing launcher.sh."
echo "You can move this file where you want, it will still work."
