build:
	platformio run

upload:
	platformio run --target upload

watch:
	platformio run --target upload
	platformio device monitor

clean:
	platformio run --target clean

test:
	platformio test

monitor:
	platformio device monitor
