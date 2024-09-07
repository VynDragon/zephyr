
SECTION_PROLOGUE(.wifi_ram,,)
{
	_wifi_ram_start = .;
	KEEP(*(.wifi_ram))
	_wifi_ram_end = .;
} GROUP_LINK_IN(ROMABLE_REGION)
_wifi_ram_size = _wifi_ram_end - _wifi_ram_start;
