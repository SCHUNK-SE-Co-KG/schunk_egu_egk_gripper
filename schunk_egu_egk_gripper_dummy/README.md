# Schunk EGU/EGK Dummy
A minimalist protocol simulator for system tests.

## Install
Use a virtual environment and
```bash
pip install fastapi uvicorn
```

## Getting started
1. In a new terminal, start the dummy
    ```bash
    uvicorn main:server --port 8000 --reload
    ```

## Run tests locally

Inside a virtual environment (.venv)

```bash
pip install pytest coverage
```

And then inside this repository
```bash
coverage run -m pytest tests/
coverage report
```
