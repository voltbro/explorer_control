#!/usr/bin/env zx

await $`sudo ip link set can0 up type can bitrate 500000 dbitrate 4000000 restart-ms 1000 berr-reporting on fd on`
await $`sudo ip link set can1 up type can bitrate 500000 dbitrate 4000000 restart-ms 1000 berr-reporting on fd on`

console.log("CANFD-HAT is configured")
