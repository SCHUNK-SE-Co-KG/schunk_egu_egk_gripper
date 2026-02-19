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

## Repository-Quellcode-Scan (Nicht-Docker-Abhängigkeiten)

Gefundene Schwachstellen im Repo: **0** (davon HIGH/CRITICAL: **0**)

Siehe **[trivy_repo_scan.md](trivy_repo_scan.md)** für Details inkl. Abgleich mit Docker-Image.

## Reports

- **[trivy_high_critical.md](trivy_high_critical.md) – HIGH/CRITICAL Schwachstellen (Markdown)**
- **[trivy_repo_scan.md](trivy_repo_scan.md) – Repository-Quellcode-Scan (Nicht-Docker)**
- [trivy_high_critical.json](trivy_high_critical.json) – HIGH/CRITICAL Schwachstellen (JSON)
- [trivy_high_critical.txt](trivy_high_critical.txt) – HIGH/CRITICAL Schwachstellen (Tabelle)
- [trivy_image_report.json](trivy_image_report.json) – Vollständiger Image-Scan (JSON)
- [trivy_image_report.txt](trivy_image_report.txt) – Image-Scan (Tabelle)
- [trivy_repo_scan.json](trivy_repo_scan.json) – Repository-Scan (JSON)
- [trivy_repo_scan.txt](trivy_repo_scan.txt) – Repository-Scan (Tabelle)
- [trivy_config_report.json](trivy_config_report.json) – Dockerfile-Config-Scan (JSON)
- [trivy_config_report.txt](trivy_config_report.txt) – Dockerfile-Config-Scan (Tabelle)

---
*Automatisch erstellt von GitHub Actions (Trivy Docker-Image-Scan Workflow).*
