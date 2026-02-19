# Trivy HIGH/CRITICAL Schwachstellen-Report

**Datum:** 2026-02-19
**Image:** schunk-gripper:scan (gebaut aus Dockerfile)
**Base-Image:** ros:humble-ros-core (Ubuntu 22.04 Jammy)

| Schweregrad | Anzahl |
|-------------|--------|
| CRITICAL    | 0 |
| HIGH        | 5 |
| **Gesamt**  | **5** |

## HIGH (5)

| Paket | CVE | Installiert | Fix verfügbar | Beschreibung |
|-------|-----|-------------|---------------|--------------|
| linux-libc-dev | [CVE-2024-35870](https://avd.aquasec.com/nvd/cve-2024-35870) | 5.15.0-170.180 | — | kernel: smb: client: fix UAF in smb2_reconnect_server() |
| linux-libc-dev | [CVE-2024-53179](https://avd.aquasec.com/nvd/cve-2024-53179) | 5.15.0-170.180 | — | kernel: smb: client: fix use-after-free of signing key |
| linux-libc-dev | [CVE-2025-37849](https://avd.aquasec.com/nvd/cve-2025-37849) | 5.15.0-170.180 | — | kernel: KVM: arm64: Tear down vGIC on failed vCPU creation |
| linux-libc-dev | [CVE-2025-37899](https://avd.aquasec.com/nvd/cve-2025-37899) | 5.15.0-170.180 | — | kernel: ksmbd: fix use-after-free in session logoff |
| linux-libc-dev | [CVE-2025-38118](https://avd.aquasec.com/nvd/cve-2025-38118) | 5.15.0-170.180 | — | kernel: Linux kernel: Bluetooth MGMT use-after-free vulnerability allows privile |

---
*Automatisch erstellt von GitHub Actions (Trivy Docker-Image-Scan Workflow).*
