"""Quick BLE scanner — prints everything seen for 10 seconds."""
import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for 10 seconds...")
    devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
    for device, adv in devices.values():
        print(f"\n  Name:    {device.name!r}")
        print(f"  Address: {device.address}")
        print(f"  RSSI:    {adv.rssi} dBm")
        print(f"  UUIDs:   {adv.service_uuids}")
        print(f"  MfgData: {adv.manufacturer_data}")

asyncio.run(main())
