SKETCH = arduino

compile:
	arduino-cli compile $(SKETCH)

upload:
	arduino-cli compile --upload $(SKETCH)

monitor:
	arduino-cli monitor -c baudrate=115200 $(SKETCH)
