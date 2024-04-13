all: main.c
	avr-gcc -mmcu=attiny2313a -DF_CPU=16000000 -Os -o main.elf -I. main.c -std=c11
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

flash: all
	avrdude -v -p t2313 -c stk500v1 -b 19200 -U main.hex -P /dev/ttyUSB0

fuses:
	avrdude -v -p t2313 -c stk500v1 -b 19200 -P /dev/ttyUSB0 -U lfuse:w:0xEE:m 	-U hfuse:w:0xDF:m 	-U efuse:w:0xFF:m 

flashall: all fuses

clean:
	-rm main.elf main.hex
