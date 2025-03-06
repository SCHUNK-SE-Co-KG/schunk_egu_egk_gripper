# Run tests locally

We use `socat` to set-up pseudo terminals for a connection between the driver library and a minimalist Modbus server.
Install that with
```bash
sudo apt-get install socat
```

Additionally, inside the dummy's package, install test-related dependencies with

```bash
pip install --user pytest httpx coverage
```

You can run the tests either directly in this folder with `pytest .` or with more output through

```bash
coverage run -m pytest .
coverage report  # for console output
coverage html    # for web-based output
```

## Modbus unit testing

Special test fixtures will create two pseudo-terminals that are connected with each other,
such as `/dev/pts/14` and `/dev/pts/15`.
The tests then use this pseudo serial connection to setup a Modbus server/client communication like this:

Modbus Client <-> /dev/pts/15 <-> /dev/pts/14 <-> Modbus Server
