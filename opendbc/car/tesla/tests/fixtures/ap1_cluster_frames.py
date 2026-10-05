"""Stock AP1 cluster frames from bus 2 (src 2) of the user's AP1 rlogs.

Sampled from two AP1 Model S captures on the box: the 7-segment frog_ap1
drive and the 2026-10-04 BogPilot drive. Raw 8-byte payloads of these three
cluster ids only: no VIN, GPS, or dongle id. Over all 10218 src-2 frames of 0x399 / 0x389 / 0x239 in those logs,
every set bit lies inside a tesla_can.dbc signal, and every 0x399 / 0x389
checksum matches (addr lo + addr hi + bytes 0..6) & 0xFF.
"""

AUTOPILOT_STATUS_HEX = (
  "0105000000007012",
  "0105000200009034",
  "010500030000d075",
  "01050500000030d7",
  "0105050000007017",
  "0105050000c0a007",
  "010505000400f09b",
  "01050503080020d2",
  "01050503080040f2",
  "010505030800f0a2",
  "0125000000004002",
  "0125000000009052",
  "012500000000d092",
  "012500000000f0b2",
  "01250003000030f5",
  "012500030000d095",
  "020500000000a043",
  "0205050000c0b018",
  "0205050000c0c028",
  "0205050008005000",
  "0205050008009040",
  "02250000000010d3",
  "0225000000004003",
  "022505000000b078",
)

DAS_STATUS2_HEX = (
  "000003003080104f",
  "000003003080407f",
  "000003003080b0ef",
  "000003003080d00f",
  "000003003080e01f",
  "000003003080f02f",
  "0000440030808000",
  "00200000108080bc",
  "002000003080308c",
  "00200004308060c0",
  "00208000308020fc",
  "0020e0003080609c",
  "00380004308070e8",
  "0038e00430801068",
  "4e0044001080a04e",
  "4e004400308010de",
  "4e0044043080a072",
  "4e20c000308090fa",
  "ff0144001180a001",
  "ff0144001180f051",
  "ff0144003180c041",
  "ff0144003180e061",
  "ff014428338030db",
  "ff2100043180f051",
)

DAS_LANES_HEX = (
  "1031667bffff0aa0",
  "201a68bdff000080",
  "20215eabff0000e0",
  "202cd6ff79650040",
  "203375e7ff000040",
  "20346579009b02d0",
  "203669ff000600a0",
  "20375ea9ff0000c0",
  "203f6190a76e0080",
  "203f637d7b7c02d0",
  "20405f7c7a800ab0",
  "2040678bd87c0220",
  "20406a7c0f6e0a70",
  "300c000069ff0080",
  "30326981ff0006c0",
  "303f6b7c7f820270",
  "303f6c837ed40200",
  "3040816c00ff0210",
  "4004005200b80090",
  "4011000000b80020",
  "402c796700a60030",
  "403a664a23900000",
  "403f637a1e9f0280",
  "403f677c695c0250",
)

# Stock periods seen in those logs (median, ms).
STOCK_PERIOD_MS = {0x399: 500, 0x389: 500, 0x239: 100}

# 0x3e9 DAS_bodyControls: stock frames set bits 22, 23, 26 and 28, which no
# tesla_can.dbc signal covers (mask below). Reason 0x3e9 is not rebuilt.
DAS_BODY_CONTROLS_UNCOVERED_BITS_SEEN = 0x0000000014c00000
