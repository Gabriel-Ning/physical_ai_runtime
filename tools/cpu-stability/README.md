# CPU stability guard

This guard applies conservative settings for this machine's Intel Core
i9-14900K:

- select the `power-saver` profile;
- disable Intel Turbo Boost through `intel_pstate`;
- set the Intel P-state performance range to 15-80%;
- select the `power` energy/performance preference.

It does not change NVIDIA clocks, GPU power limits, CUDA visibility, or GPU
compute settings. CPU-side data loading and preprocessing remain subject to
the separately configured user-session CPU quota.

Install and apply immediately:

```bash
sudo ./tools/cpu-stability/install.sh
```

Inspect the active values:

```bash
/usr/local/sbin/cpu-stability-guard status
```

Remove the service and restore the prior Ubuntu-style balanced defaults:

```bash
sudo ./tools/cpu-stability/install.sh --uninstall
```

The installer only writes two system files:
`/usr/local/sbin/cpu-stability-guard` and
`/etc/systemd/system/cpu-stability.service`.
