#!/bin/bash

# Activate Python virtual environment (if needed)
source ~/myenv/bin/activate

# Start MAVProxy and get GPS data using Python
python3 get_gps.py

echo "Startup process complete!"




