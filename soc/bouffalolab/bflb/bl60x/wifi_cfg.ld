

_wifi_cfg_start = .;
. = ALIGN(4);
KEEP(*(.wifi.cfg.entry))
_wifi_cfg_end = .;

_wifi_cfg_size = _wifi_cfg_end - _wifi_cfg_start;

PROVIDE( _ld_bl_static_cfg_entry_start = _wifi_cfg_start );
PROVIDE( _ld_bl_static_cfg_entry_end   = _wifi_cfg_end );
