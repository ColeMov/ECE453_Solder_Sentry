"""Quick BLE scan dump — verify the board is visible to bleak."""
import asyncio
from bleak import BleakScanner

NUS = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"


async def main():
    print("scanning 12 s with adv data...")
    devs = await BleakScanner.discover(timeout=12.0, return_adv=True)
    print(f"found {len(devs)} devices\n")
    hits = []
    for addr, (d, adv) in devs.items():
        svcs = [s.lower() for s in (adv.service_uuids or [])]
        nus_match = NUS in svcs
        name = d.name or adv.local_name
        if nus_match or (name and "ECE453".lower() in name.lower()):
            hits.append((addr, d, adv, nus_match))
            continue

    print(f"=== potential ECE453 matches: {len(hits)} ===")
    for addr, d, adv, nus_match in hits:
        print(f"  {addr}")
        print(f"    name={d.name!r}  local={adv.local_name!r}")
        print(f"    rssi={adv.rssi}  nus={'YES' if nus_match else 'no'}")
        print(f"    svcs={adv.service_uuids}")
        print(f"    mfr={adv.manufacturer_data}")

    if not hits:
        print("  none. dumping all named devices for sanity:")
        for addr, (d, adv) in devs.items():
            name = d.name or adv.local_name
            if name:
                print(f"    {addr}  name={name!r}  rssi={adv.rssi}")


if __name__ == "__main__":
    asyncio.run(main())
