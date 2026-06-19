Simple BLE scan sketch showing how to make use of the central role API to identify surrounding peripheral BLE devices.

Sample output:

Sasquatch STM32WB5MMG BLE scanner
----------------------------------
Scanner address: 80:e1:26:b3:09:b7
Scan duplicates: no
RTC date now: 26-06-19
RTC time now: 11:38:11.629

Scanning for nearby BLE advertisers...

Report #1
  RTC date:         26-06-19
  RTC time:         11:38:40.279
  Uptime:           32131 ms
  Address:          60:3e:8f:0b:4f:dc  (type 1)
  Address type:     random static
  RSSI:             -67 dBm
  Signal:           medium
  Packet kind:      advertisement, connectable
  Event flags:      connectable advertisement 
  Adv length:       17 bytes
  Raw adv data:     02011A020A0C0AFF4C001005411C9B1363
  TX power:         12 dBm
  Manufacturer ID:  004C  (Apple)
  Manufacturer data:1005411C9B1363

Report #2
  RTC date:         26-06-19
  RTC time:         11:38:42.180
  Uptime:           34010 ms
  Address:          49:2e:3e:8e:ee:8b  (type 1)
  Address type:     random static
  RSSI:             -101 dBm
  Signal:           weak
  Packet kind:      advertisement, connectable
  Event flags:      connectable advertisement 
  Adv length:       17 bytes
  Raw adv data:     02011A020A120AFF4C00100535185638F5
  TX power:         18 dBm
  Manufacturer ID:  004C  (Apple)
  Manufacturer data:100535185638F5

Report #3
  RTC date:         26-06-19
  RTC time:         11:38:42.958
  Uptime:           34763 ms
  Address:          c4:76:33:ff:ab:0e  (type 1)
  Address type:     random static
  RSSI:             -100 dBm
  Signal:           weak
  Packet kind:      advertisement, non-connectable
  Event flags:      advertisement 
  Adv length:       8 bytes
  Raw adv data:     07FF4C0012020000
  Manufacturer ID:  004C  (Apple)
  Manufacturer data:12020000

Report #4
  RTC date:         26-06-19
  RTC time:         11:38:48.980
  Uptime:           40875 ms
  Address:          6a:39:82:30:87:31  (type 1)
  Address type:     random static
  RSSI:             -101 dBm
  Signal:           weak
  Packet kind:      advertisement, non-connectable
  Event flags:      advertisement 
  Adv length:       17 bytes
  Raw adv data:     02011A0DFF4C00160800991C9572EC1FD5
  Manufacturer ID:  004C  (Apple)
  Manufacturer data:160800991C9572EC1FD5

