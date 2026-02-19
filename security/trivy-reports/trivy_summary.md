# Trivy Docker-Image-Scan – Zusammenfassung

**Datum:** 2026-02-19
**Image:** schunk-gripper:scan (gebaut aus Dockerfile)
**Base-Image:** ros:humble-ros-core (Ubuntu 22.04 Jammy)
**Scanner:** Trivy (GitHub Actions)

## Schwachstellen (Image-Scan)

| Schweregrad | Anzahl |
|-------------|--------|
| CRITICAL    | 0 |
| HIGH        | 5 |
| MEDIUM      | 1522 |
| LOW         | 158 |
| **Gesamt**  | **1685** |

## Dockerfile-Misconfigurations

Gefundene Misconfigurations: **2**

## Reports

- [trivy_image_report.json](trivy_image_report.json) – Vollständiger Image-Scan (JSON)
- [trivy_image_report.txt](trivy_image_report.txt) – Image-Scan (Tabelle)
- [trivy_config_report.json](trivy_config_report.json) – Dockerfile-Config-Scan (JSON)
- [trivy_config_report.txt](trivy_config_report.txt) – Dockerfile-Config-Scan (Tabelle)

---
*Automatisch erstellt von GitHub Actions (Trivy Docker-Image-Scan Workflow).*
