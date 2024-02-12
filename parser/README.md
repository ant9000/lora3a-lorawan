Example usage for Chirpstack:
```
sudo apt install -y build-essential python3-venv python3-dev
python3 -m venv venv
. venv/bin/activate
pip install -r requirements.txt
mosquitto_sub -h hostname -p port -u username -P password -t application/\# -F "%J" | \
  jq --unbuffered -r '.payload.data + ""' | ./parser.py
```
