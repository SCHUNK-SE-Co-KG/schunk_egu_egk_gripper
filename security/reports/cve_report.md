# CVE-Scan Report – schunk_mechatronic_gripper

**Scan-Zeitpunkt:** 2026-02-18T14:28:52Z
**Repository:** SCHUNK-SE-Co-KG/schunk_mechatronic_gripper
**Abhängigkeiten geprüft:** 19
**Schwachstellen gefunden:** 29

> 29 Schwachstelle(n) gefunden!

## Zusammenfassung nach Ökosystem

| Ökosystem | Abhängigkeiten | Schwachstellen |
|-----------|---------------|----------------|
| PyPI (Python) | 13 | 29 |
| ROS 2 | 6 | 0 |
| **Gesamt** | **19** | **29** |

## Geprüfte Abhängigkeiten

### Python (PyPI)

| Paket | Version | Quelle |
|-------|---------|--------|
| setuptools | – | schunk_gripper_driver\setup.py |
| pymodbus | – | schunk_gripper_driver\setup.py |
| fastapi | – | schunk_gripper_dummy\setup.py |
| uvicorn | – | schunk_gripper_dummy\setup.py |
| requests | – | schunk_gripper_dummy\setup.py |
| python-multipart | – | schunk_gripper_dummy\setup.py |
| pyserial | 3.5 | schunk_gripper_library\setup.py |
| httpx | 0.28.1 | schunk_gripper_library\setup.py |
| pytest | 6.2.5 | schunk_gripper_library\setup.py |
| netifaces2 | 0.0.22 | schunk_gripper_library\setup.py |
| empy | 3.3.4 | schunk_gripper_library\setup.py |
| catkin_pkg | 1.1.0 | schunk_gripper_library\setup.py |
| lark | 1.1.1 | schunk_gripper_library\setup.py |

### ROS 2

| Paket | Ökosystem | Quelle | Upstream |
|-------|-----------|--------|----------|
| rclpy | ROS | schunk_gripper_driver\package.xml | [ros2/rclpy](https://github.com/ros2/rclpy) |
| launch | ROS | schunk_gripper_driver\package.xml | [ros2/launch](https://github.com/ros2/launch) |
| launch_ros | ROS | schunk_gripper_driver\package.xml | [ros2/launch_ros](https://github.com/ros2/launch_ros) |
| std_srvs | ROS | schunk_gripper_driver\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| sensor_msgs | ROS | schunk_gripper_driver\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| std_msgs | ROS | schunk_gripper_interfaces\package.xml | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |

## Gefundene Schwachstellen

### GHSA-27x4-j476-jp5f

- **Paket:** PyPI:setuptools
- **CVSS-Score:** 8.4 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:H/PR:N/UI:R/S:C/C:H/I:H/A:H
- **CVE:** CVE-2013-1633
- **Beschreibung:** Setuptools vulnerable to Man-in-the-middle attacks
- **Fix-Version:** 0.7
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2013-1633
  - https://github.com/pypa/advisory-database/tree/main/vulns/setuptools/PYSEC-2013-22.yaml
  - https://github.com/pypa/setuptools
  - https://pypi.python.org/pypi/setuptools/0.9.8#changes
  - http://www.reddit.com/r/Python/comments/17rfh7/warning_dont_use_pip_in_an_untrusted_network_a

### GHSA-5rjg-fvgr-3xxf

- **Paket:** PyPI:setuptools
- **CVSS-Score:** 7.7 (KRITISCH)
- **Schweregrad:** CVSS:4.0/AV:N/AC:L/AT:N/PR:N/UI:N/VC:N/VI:H/VA:N/SC:N/SI:N/SA:N/E:P
- **CVE:** CVE-2025-47273
- **Beschreibung:** setuptools has a path traversal vulnerability in PackageIndex.download that leads to Arbitrary File Write
- **Fix-Version:** 78.1.1
- **Referenzen:**
  - https://github.com/pypa/setuptools/security/advisories/GHSA-5rjg-fvgr-3xxf
  - https://nvd.nist.gov/vuln/detail/CVE-2025-47273
  - https://github.com/pypa/setuptools/issues/4946
  - https://github.com/pypa/setuptools/commit/250a6d17978f9f6ac3ac887091f2d32886fbbb0b
  - https://github.com/pypa/advisory-database/tree/main/vulns/setuptools/PYSEC-2025-49.yaml

### GHSA-cx63-2mw6-8hw5

- **Paket:** PyPI:setuptools
- **CVSS-Score:** 7.5 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:R/S:U/C:H/I:H/A:H
- **CVE:** CVE-2024-6345
- **Beschreibung:** setuptools vulnerable to Command Injection via package URL
- **Fix-Version:** 70.0.0
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2024-6345
  - https://github.com/pypa/setuptools/pull/4332
  - https://github.com/pypa/setuptools/commit/88807c7062788254f654ea8c03427adc859321f0
  - https://github.com/pypa/setuptools
  - https://huntr.com/bounties/d6362117-ad57-4e83-951f-b8141c6e7ca5

### GHSA-r9hx-vwmv-q579

- **Paket:** PyPI:setuptools
- **CVSS-Score:** 8.7 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:N/A:H
- **CVE:** CVE-2022-40897
- **Beschreibung:** pypa/setuptools vulnerable to Regular Expression Denial of Service (ReDoS)
- **Fix-Version:** 65.5.1
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2022-40897
  - https://github.com/pypa/setuptools/issues/3659
  - https://github.com/pypa/setuptools/commit/43a9c9bfa6aa626ec2a22540bea28d2ca77964be
  - https://setuptools.pypa.io/en/latest
  - https://security.netapp.com/advisory/ntap-20240621-0006

### PYSEC-2013-22

- **Paket:** PyPI:setuptools
- **CVSS-Score:** 8.4 (KRITISCH)
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2013-1633
- **Beschreibung:** easy_install in setuptools before 0.7 uses HTTP to retrieve packages from the PyPI repository, and does not perform integrity checks on package contents, which allows man-in-the-middle attackers to execute arbitrary code via a crafted response to the default use of the product.
- **Fix-Version:** 0.7
- **Referenzen:**
  - http://www.reddit.com/r/Python/comments/17rfh7/warning_dont_use_pip_in_an_untrusted_network_a/
  - https://pypi.python.org/pypi/setuptools/0.9.8#changes

### PYSEC-2022-43012

- **Paket:** PyPI:setuptools
- **CVSS-Score:** 8.7 (KRITISCH)
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2022-40897
- **Beschreibung:** Python Packaging Authority (PyPA) setuptools before 65.5.1 allows remote attackers to cause a denial of service via HTML in a crafted package or custom PackageIndex page. There is a Regular Expression Denial of Service (ReDoS) in package_index.py.
- **Fix-Version:** 43a9c9bfa6aa626ec2a22540bea28d2ca77964be
- **Referenzen:**
  - https://github.com/pypa/setuptools/blob/fe8a98e696241487ba6ac9f91faa38ade939ec5d/setuptools/package_index.py#L200
  - https://pyup.io/posts/pyup-discovers-redos-vulnerabilities-in-top-python-packages/
  - https://github.com/pypa/setuptools/compare/v65.5.0...v65.5.1
  - https://github.com/pypa/setuptools/commit/43a9c9bfa6aa626ec2a22540bea28d2ca77964be
  - https://pyup.io/vulnerabilities/CVE-2022-40897/52495/

### PYSEC-2025-49

- **Paket:** PyPI:setuptools
- **CVSS-Score:** 7.7 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:L/UI:N/S:U/C:H/I:H/A:H
- **CVE:** CVE-2025-47273
- **Beschreibung:** setuptools is a package that allows users to download, build, install, upgrade, and uninstall Python packages. A path traversal vulnerability in `PackageIndex` is present in setuptools prior to version 78.1.1. An attacker would be allowed to write files to arbitrary locations on the filesystem with
- **Fix-Version:** 250a6d17978f9f6ac3ac887091f2d32886fbbb0b
- **Referenzen:**
  - https://github.com/pypa/setuptools/security/advisories/GHSA-5rjg-fvgr-3xxf
  - https://lists.debian.org/debian-lts-announce/2025/05/msg00035.html
  - https://github.com/pypa/setuptools/issues/4946
  - https://github.com/pypa/setuptools/commit/250a6d17978f9f6ac3ac887091f2d32886fbbb0b
  - https://github.com/pypa/setuptools/blob/6ead555c5fb29bc57fe6105b1bffc163f56fd558/setuptools/package_index.py#L810C1-L825C88

### GHSA-8h2j-cgx8-6xv7

- **Paket:** PyPI:fastapi
- **CVSS-Score:** 8.8 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:L/I:H/A:N
- **CVE:** CVE-2021-32677
- **Beschreibung:** Cross-Site Request Forgery (CSRF) in FastAPI
- **Fix-Version:** 0.65.2
- **Referenzen:**
  - https://github.com/tiangolo/fastapi/security/advisories/GHSA-8h2j-cgx8-6xv7
  - https://nvd.nist.gov/vuln/detail/CVE-2021-32677
  - https://github.com/tiangolo/fastapi/commit/fa7e3c996edf2d5482fff8f9d890ac2390dede4d
  - https://github.com/pypa/advisory-database/tree/main/vulns/fastapi/PYSEC-2021-100.yaml
  - https://github.com/tiangolo/fastapi

### PYSEC-2021-100

- **Paket:** PyPI:fastapi
- **CVSS-Score:** 8.8 (KRITISCH)
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2021-32677
- **Beschreibung:** FastAPI is a web framework for building APIs with Python 3.6+ based on standard Python type hints. FastAPI versions lower than 0.65.2 that used cookies for authentication in path operations that received JSON payloads sent by browsers were vulnerable to a Cross-Site Request Forgery (CSRF) attack. In
- **Fix-Version:** fa7e3c996edf2d5482fff8f9d890ac2390dede4d
- **Referenzen:**
  - https://github.com/tiangolo/fastapi/commit/fa7e3c996edf2d5482fff8f9d890ac2390dede4d
  - https://github.com/tiangolo/fastapi/security/advisories/GHSA-8h2j-cgx8-6xv7
  - https://lists.fedoraproject.org/archives/list/package-announce@lists.fedoraproject.org/message/MATAWX25TYKNEKLDMKWNLYDB34UWTROA/

### PYSEC-2024-38

- **Paket:** PyPI:fastapi
- **CVSS-Score:** 7.5 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:N/A:H
- **CVE:** CVE-2024-24762
- **Beschreibung:** FastAPI is a web framework for building APIs with Python 3.8+ based on standard Python type hints. When using form data, `python-multipart` uses a Regular Expression to parse the HTTP `Content-Type` header, including options. An attacker could send a custom-made `Content-Type` option that is very di
- **Fix-Version:** 9d34ad0ee8a0dfbbcce06f76c2d5d851085024fc
- **Referenzen:**
  - https://github.com/tiangolo/fastapi/security/advisories/GHSA-qf9m-vfgh-m389
  - https://github.com/tiangolo/fastapi/commit/9d34ad0ee8a0dfbbcce06f76c2d5d851085024fc
  - https://github.com/tiangolo/fastapi/releases/tag/0.109.1

### GHSA-33c7-2mpw-hg34

- **Paket:** PyPI:uvicorn
- **CVSS-Score:** 8.7 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:H/A:N
- **CVE:** CVE-2020-7694
- **Beschreibung:** Log injection in uvicorn
- **Fix-Version:** 0.11.7
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2020-7694
  - https://github.com/encode/uvicorn/issues/723
  - https://github.com/encode/uvicorn/commit/895807f94ea9a8e588605c12076b7d7517cda503
  - https://github.com/encode/uvicorn
  - https://github.com/pypa/advisory-database/tree/main/vulns/uvicorn/PYSEC-2020-150.yaml

### GHSA-f97h-2pfx-f59f

- **Paket:** PyPI:uvicorn
- **CVSS-Score:** 8.7 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:H/A:N
- **CVE:** CVE-2020-7695
- **Beschreibung:** HTTP response splitting in uvicorn
- **Fix-Version:** 0.11.7
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2020-7695
  - https://github.com/encode/uvicorn
  - https://github.com/pypa/advisory-database/tree/main/vulns/uvicorn/PYSEC-2020-151.yaml
  - https://snyk.io/vuln/SNYK-PYTHON-UVICORN-570471

### PYSEC-2020-150

- **Paket:** PyPI:uvicorn
- **CVSS-Score:** 8.7 (KRITISCH)
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2020-7694
- **Beschreibung:** This affects all versions of package uvicorn. The request logger provided by the package is vulnerable to ASNI escape sequence injection. Whenever any HTTP request is received, the default behaviour of uvicorn is to log its details to either the console or a log file. When attackers request crafted
- **Fix-Version:** 0.11.7
- **Referenzen:**
  - https://snyk.io/vuln/SNYK-PYTHON-UVICORN-575560
  - https://github.com/encode/uvicorn
  - https://github.com/advisories/GHSA-33c7-2mpw-hg34

### PYSEC-2020-151

- **Paket:** PyPI:uvicorn
- **CVSS-Score:** 8.7 (KRITISCH)
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2020-7695
- **Beschreibung:** Uvicorn before 0.11.7 is vulnerable to HTTP response splitting. CRLF sequences are not escaped in the value of HTTP headers. Attackers can exploit this to add arbitrary headers to HTTP responses, or even return an arbitrary response body, whenever crafted input is used to construct HTTP headers.
- **Fix-Version:** 0.11.7
- **Referenzen:**
  - https://snyk.io/vuln/SNYK-PYTHON-UVICORN-570471
  - https://github.com/encode/uvicorn
  - https://github.com/advisories/GHSA-f97h-2pfx-f59f

### GHSA-652x-xj99-gmcc

- **Paket:** PyPI:requests
- **CVSS-Score:** 6.9
- **Schweregrad:** CVSS:4.0/AV:N/AC:L/AT:N/PR:N/UI:N/VC:L/VI:N/VA:N/SC:N/SI:N/SA:N
- **CVE:** CVE-2014-1830
- **Beschreibung:** Exposure of Sensitive Information to an Unauthorized Actor in Requests
- **Fix-Version:** 2.3.0
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2014-1830
  - https://github.com/kennethreitz/requests/issues/1885
  - https://github.com/psf/requests/issues/1885
  - https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=733108
  - https://github.com/psf/requests

### GHSA-9hjg-9r4m-mvj7

- **Paket:** PyPI:requests
- **CVSS-Score:** 5.3
- **Schweregrad:** CVSS:3.1/AV:N/AC:H/PR:N/UI:R/S:U/C:H/I:N/A:N
- **CVE:** CVE-2024-47081
- **Beschreibung:** Requests vulnerable to .netrc credentials leak via malicious URLs
- **Fix-Version:** 2.32.4
- **Referenzen:**
  - https://github.com/psf/requests/security/advisories/GHSA-9hjg-9r4m-mvj7
  - https://nvd.nist.gov/vuln/detail/CVE-2024-47081
  - https://github.com/psf/requests/pull/6965
  - https://github.com/psf/requests/commit/96ba401c1296ab1dda74a2365ef36d88f7d144ef
  - https://github.com/psf/requests

### GHSA-9wx4-h78v-vm56

- **Paket:** PyPI:requests
- **CVSS-Score:** 5.6
- **Schweregrad:** CVSS:3.1/AV:L/AC:H/PR:H/UI:R/S:U/C:H/I:H/A:N
- **CVE:** CVE-2024-35195
- **Beschreibung:** Requests `Session` object does not verify requests after making first request with verify=False
- **Fix-Version:** 2.32.0
- **Referenzen:**
  - https://github.com/psf/requests/security/advisories/GHSA-9wx4-h78v-vm56
  - https://nvd.nist.gov/vuln/detail/CVE-2024-35195
  - https://github.com/psf/requests/pull/6655
  - https://github.com/psf/requests/commit/a58d7f2ffb4d00b46dca2d70a3932a0b37e22fac
  - https://github.com/psf/requests

### GHSA-cfj3-7x9c-4p3h

- **Paket:** PyPI:requests
- **CVSS-Score:** 6.9
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:L/I:N/A:N
- **CVE:** CVE-2014-1829
- **Beschreibung:** Exposure of Sensitive Information to an Unauthorized Actor in Requests
- **Fix-Version:** 2.3.0
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2014-1829
  - https://github.com/kennethreitz/requests/issues/1885
  - https://github.com/psf/requests/issues/1885
  - https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=733108
  - https://github.com/advisories/GHSA-cfj3-7x9c-4p3h

### GHSA-j8r2-6x86-q33q

- **Paket:** PyPI:requests
- **CVSS-Score:** 6.1
- **Schweregrad:** CVSS:3.1/AV:N/AC:H/PR:N/UI:R/S:C/C:H/I:N/A:N
- **CVE:** CVE-2023-32681
- **Beschreibung:** Unintended leak of Proxy-Authorization header in requests
- **Fix-Version:** 2.31.0
- **Referenzen:**
  - https://github.com/psf/requests/security/advisories/GHSA-j8r2-6x86-q33q
  - https://nvd.nist.gov/vuln/detail/CVE-2023-32681
  - https://github.com/psf/requests/commit/74ea7cf7a6a27a4eeb2ae24e162bcc942a6706d5
  - https://github.com/psf/requests
  - https://github.com/psf/requests/releases/tag/v2.31.0

### GHSA-pg2w-x9wp-vw92

- **Paket:** PyPI:requests
- **CVSS-Score:** 5.0
- **Schweregrad:** MODERATE
- **CVE:** CVE-2015-2296
- **Beschreibung:** Python Requests Session Fixation
- **Fix-Version:** 2.6.0
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2015-2296
  - https://github.com/kennethreitz/requests/commit/3bd8afbff29e50b38f889b2f688785a669b9aafc
  - https://github.com/psf/requests/commit/3bd8afbff29e50b38f889b2f688785a669b9aafc
  - https://github.com/psf/requests
  - https://github.com/pypa/advisory-database/tree/main/vulns/requests/PYSEC-2015-17.yaml

### GHSA-x84v-xcm2-53pg

- **Paket:** PyPI:requests
- **CVSS-Score:** 7.5 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:N/A:N
- **CVE:** CVE-2018-18074
- **Beschreibung:** Insufficiently Protected Credentials in Requests
- **Fix-Version:** 2.20.0
- **Referenzen:**
  - https://nvd.nist.gov/vuln/detail/CVE-2018-18074
  - https://github.com/requests/requests/issues/4716
  - https://github.com/requests/requests/pull/4718
  - https://github.com/requests/requests/commit/c45d7c49ea75133e52ab22a8e9e13173938e36ff
  - https://access.redhat.com/errata/RHSA-2019:2035

### PYSEC-2014-13

- **Paket:** PyPI:requests
- **CVSS-Score:** 6.9
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2014-1829
- **Beschreibung:** Requests (aka python-requests) before 2.3.0 allows remote servers to obtain a netrc password by reading the Authorization header in a redirected request.
- **Fix-Version:** 2.3.0
- **Referenzen:**
  - https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=733108
  - https://github.com/kennethreitz/requests/issues/1885
  - http://www.ubuntu.com/usn/USN-2382-1
  - http://www.debian.org/security/2015/dsa-3146
  - http://www.mandriva.com/security/advisories?name=MDVSA-2015:133

### PYSEC-2014-14

- **Paket:** PyPI:requests
- **CVSS-Score:** 6.9
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2014-1830
- **Beschreibung:** Requests (aka python-requests) before 2.3.0 allows remote servers to obtain sensitive information by reading the Proxy-Authorization header in a redirected request.
- **Fix-Version:** 2.3.0
- **Referenzen:**
  - https://github.com/kennethreitz/requests/issues/1885
  - https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=733108
  - http://www.debian.org/security/2015/dsa-3146
  - http://www.mandriva.com/security/advisories?name=MDVSA-2015:133
  - http://advisories.mageia.org/MGASA-2014-0409.html

### PYSEC-2015-17

- **Paket:** PyPI:requests
- **CVSS-Score:** –
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2015-2296
- **Beschreibung:** The resolve_redirects function in sessions.py in requests 2.1.0 through 2.5.3 allows remote attackers to conduct session fixation attacks via a cookie without a host value in a redirect.
- **Fix-Version:** 3bd8afbff29e50b38f889b2f688785a669b9aafc
- **Referenzen:**
  - http://www.openwall.com/lists/oss-security/2015/03/15/1
  - http://www.ubuntu.com/usn/USN-2531-1
  - http://www.openwall.com/lists/oss-security/2015/03/14/4
  - https://github.com/kennethreitz/requests/commit/3bd8afbff29e50b38f889b2f688785a669b9aafc
  - https://warehouse.python.org/project/requests/2.6.0/

### PYSEC-2018-28

- **Paket:** PyPI:requests
- **CVSS-Score:** 7.5 (KRITISCH)
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2018-18074
- **Beschreibung:** The Requests package before 2.20.0 for Python sends an HTTP Authorization header to an http URI upon receiving a same-hostname https-to-http redirect, which makes it easier for remote attackers to discover credentials by sniffing the network.
- **Fix-Version:** c45d7c49ea75133e52ab22a8e9e13173938e36ff
- **Referenzen:**
  - https://github.com/requests/requests/pull/4718
  - https://github.com/requests/requests/issues/4716
  - https://github.com/requests/requests/commit/c45d7c49ea75133e52ab22a8e9e13173938e36ff
  - https://bugs.debian.org/910766
  - https://usn.ubuntu.com/3790-1/

### PYSEC-2023-74

- **Paket:** PyPI:requests
- **CVSS-Score:** 6.1
- **Schweregrad:** UNKNOWN
- **CVE:** CVE-2023-32681
- **Beschreibung:** Requests is a HTTP library. Since Requests 2.3.0, Requests has been leaking Proxy-Authorization headers to destination servers when redirected to an HTTPS endpoint. This is a product of how we use `rebuild_proxies` to reattach the `Proxy-Authorization` header to requests. For HTTP connections sent t
- **Fix-Version:** 74ea7cf7a6a27a4eeb2ae24e162bcc942a6706d5
- **Referenzen:**
  - https://github.com/psf/requests/security/advisories/GHSA-j8r2-6x86-q33q
  - https://github.com/psf/requests/releases/tag/v2.31.0
  - https://github.com/psf/requests/commit/74ea7cf7a6a27a4eeb2ae24e162bcc942a6706d5
  - https://lists.fedoraproject.org/archives/list/package-announce@lists.fedoraproject.org/message/AW7HNFGYP44RT3DUDQXG2QT3OEV2PJ7Y/

### GHSA-2jv5-9r88-3w3p

- **Paket:** PyPI:python-multipart
- **CVSS-Score:** 7.5 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:N/A:H
- **CVE:** CVE-2024-24762
- **Beschreibung:** python-multipart vulnerable to Content-Type Header ReDoS
- **Fix-Version:** 0.0.7
- **Referenzen:**
  - https://github.com/Kludex/python-multipart/security/advisories/GHSA-2jv5-9r88-3w3p
  - https://nvd.nist.gov/vuln/detail/CVE-2024-24762
  - https://github.com/github/advisory-database/pull/4829
  - https://github.com/Kludex/python-multipart/commit/20f0ef6b4e4caf7d69a667c54dff57fe467109a4
  - https://github.com/encode/starlette/commit/13e5c26a27f4903924624736abd6131b2da80cc5

### GHSA-59g5-xgcq-4qw3

- **Paket:** PyPI:python-multipart
- **CVSS-Score:** 8.7 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:N/A:H
- **CVE:** CVE-2024-53981
- **Beschreibung:** Denial of service (DoS) via deformation `multipart/form-data` boundary
- **Fix-Version:** 0.0.18
- **Referenzen:**
  - https://github.com/Kludex/python-multipart/security/advisories/GHSA-59g5-xgcq-4qw3
  - https://nvd.nist.gov/vuln/detail/CVE-2024-53981
  - https://github.com/Kludex/python-multipart/commit/c4fe4d3cebc08c660e57dd709af1ffa7059b3177
  - https://github.com/Kludex/python-multipart

### GHSA-wp53-j4wj-2cfg

- **Paket:** PyPI:python-multipart
- **CVSS-Score:** 8.6 (KRITISCH)
- **Schweregrad:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:L/I:H/A:L
- **CVE:** CVE-2026-24486
- **Beschreibung:** Python-Multipart has Arbitrary File Write via Non-Default Configuration
- **Fix-Version:** 0.0.22
- **Referenzen:**
  - https://github.com/Kludex/python-multipart/security/advisories/GHSA-wp53-j4wj-2cfg
  - https://nvd.nist.gov/vuln/detail/CVE-2026-24486
  - https://github.com/Kludex/python-multipart/commit/9433f4bbc9652bdde82bbe380984e32f8cfc89c4
  - https://github.com/Kludex/python-multipart
  - https://github.com/Kludex/python-multipart/releases/tag/0.0.22

---
*Automatisch generiert von `security/cve_scanner.py` via [OSV.dev](https://osv.dev) und GitHub Advisory Database.*
