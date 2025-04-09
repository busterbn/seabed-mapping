init:
    python -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    cp patch.py .venv/lib/python3.13/site-packages/bps_oculus/core.py