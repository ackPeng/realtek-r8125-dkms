# Realtek r8125 DKMS

![Static Badge](https://img.shields.io/badge/RELEASE-V9.016.01-blue)

This directory extends Realtek upstream `r8125-9.016.01` source with Debian DKMS packaging files.

## Build Debian package

Install build dependencies:

```bash
sudo apt install devscripts debmake debhelper build-essential dkms dh-dkms
```

Build package:

```bash
dpkg-buildpackage -b -rfakeroot -us -uc
```

The original upstream readme is provided as `REALTEK_README.txt`.
