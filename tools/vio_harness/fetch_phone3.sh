#!/bin/bash
# fetch_phone3.sh <indoor1|indoor2|outdoor1|outdoor2|advio15|advio20>  -> external/vio3/phone/<seq>/{cam0,mav0,...} (images regenerated; csv fixtures reused from external/vio2/phone/<seq>)
R=/home/nybo/github/pose-validation; PY=$R/external/gnss/venv/bin/python; T=$R/tools/gnss_harness; O=$R/external/vio3/phone/$1
mkdir -p $O
case $1 in
 indoor1) $PY $T/mobilegvio_to_euroc.py Indoor-1 $O 1e9 --every 2 ;;
 indoor2) $PY $T/mobilegvio_to_euroc.py Indoor-2 $O 1e9 --every 2 ;;
 outdoor1) $PY $T/mobilegvio_to_euroc.py Outdoor-1 $O 1e9 --every 2 ;;
 outdoor2) $PY $T/mobilegvio_to_euroc.py Outdoor-2 $O 450 --every 2 ;;
 advio15|advio20) n=${1#advio}; [ -f $R/external/vio3/phone/advio-$n.zip ] || curl -sL -o $R/external/vio3/phone/advio-$n.zip "https://zenodo.org/records/1476931/files/advio-$n.zip?download=1"
   $PY $T/advio_to_euroc.py $R/external/vio3/phone/advio-$n.zip $O --every 2; rm -f $R/external/vio3/phone/advio-$n.zip ;;
esac
$PY $T/make_layout.py $O $R/external/vio2/phone/$1/imu0/data.csv
echo FETCH_DONE
