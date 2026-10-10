# Linux 6.18 for the Nokia N9

This is linux-stable **v6.18.55** with the changes that let the Nokia N9 (RM-696) run its stock
Harmattan PR1.3 userspace, unmodified: Nokia's drivers and interfaces brought forward, power
management as Nokia's kernel did it, and fixes to mainline bugs found on the way. Each change is one
commit on branch `rm696`, with its reason in the message: `git log --no-merges v6.18.55..rm696`
(they started on v6.18.53; stable releases come in as merges of their tags).

The configuration is `omap2plus_defconfig` with `arch/arm/configs/rm696.config` on top.

## Building

A bootable N9 image needs more than this tree: the SGX GPU driver
([pvr_sgx_rm696](https://github.com/zsavajji/pvr_sgx_rm696), built against it as external modules), WLAN
firmware and the initramfs. The build kit, [rm696_mainline_port](https://github.com/zsavajji/rm696_mainline_port), builds
all of it into one file NOLO boots:

```sh
git clone --depth 1 --branch rm696 https://github.com/zsavajji/linux_mainline_rm696
git clone https://github.com/zsavajji/pvr_sgx_rm696
git clone https://github.com/zsavajji/rm696_mainline_port
cd rm696_mainline_port && RELEASE=1 kernel/build.sh     # -> build/zImage-n9-release
```

The kit's documentation, online at https://zsavajji.github.io/rm696_mainline_port/, covers installing, the rescue SSH server, what works and
what each change is for.

Linux's own README is `README`.

## Licence

GPL-2.0, as Linux (`COPYING`); Nokia's drivers, Aegis and the DSP bridge are GPL-2.0 as well.
