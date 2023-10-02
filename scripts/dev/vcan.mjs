#!/usr/bin/env zx

await $`sudo ip link add dev vcan0 type vcan`
await $`sudo ip link set up vcan0`
console.log("vcan0 is working")
