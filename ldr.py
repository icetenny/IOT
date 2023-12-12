import gpiozero

ldr = gpiozero.Button(21)
led = gpiozero.LED(27)

while True:
    print(ldr.is_pressed)
    if ldr.is_pressed:
        led.off()
    else:
        led.on()
