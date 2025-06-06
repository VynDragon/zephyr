if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <arg1>"
    exit 1
fi

python3 /home/vdragon/dev/DUO/duo-buildroot-sdk-v2/fsbl/plat/cv181x/fiptool.py -v genfip fip.bin \
--CHIP_CONF='chip_conf' \
--NOR_INFO='FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF' \
--NAND_INFO='00000000' \
--BL2=$1
