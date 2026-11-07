# `system/` — OS-level bring-up

| File | Purpose |
|---|---|
| `can-setup.sh` | Sets `can0` **and** `can1` to 1 Mbit/s, `txqueuelen 1000`, brings them up and prints a summary. Mirror of the version deployed at `/usr/local/bin/can-setup.sh`. |
| `can-interface.service` | systemd one-shot unit that runs the script at boot (installed in `/etc/systemd/system/`). |

Install / verify steps: [`../docs/HARDWARE_SETUP.md`](../docs/HARDWARE_SETUP.md) §3.
