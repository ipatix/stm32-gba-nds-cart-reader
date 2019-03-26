#!/bin/bash

set -ue

echo '__attribute__((aligned(4))) const unsigned char nds_cart_key[0x1048] = {'
dd if=biosnds7.rom of=/dev/stdout iflag=skip_bytes,count_bytes skip=48 count=4168 | xxd -i
echo '};'
echo '__attribute__((aligned(4))) const unsigned char dsi_cart_key[0x1048] = {'
dd if=biosdsi7.rom of=/dev/stdout iflag=skip_bytes,count_bytes skip=50896 count=4168 | xxd -i
echo '};'
