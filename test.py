from time import sleep
import serial
import pygame

ser = serial.Serial('COM4', 19200)


def write(s):
    ser.write(s.encode('ascii'))


sleep(3)
write('E')
sleep(.2)
write('H')
sleep(15)

pygame.init()
screen = pygame.display.set_mode([1900, 1100])

running = True

while running:
    # Look at every event in the queue
    for event in pygame.event.get():
        # Did the user hit a key?
        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = pygame.mouse.get_pos()
            print('M %.2f %.2f' % ((1920 - x) * 330.0 / 1200, -y * 330.0 / 1200))
            write('M %.2f %.2f' % ((1920 - x) * 330.0 / 1200, -y * 330.0 / 1200))
            sleep(.1)
            write('T')

        # Did the user click the window close button? If so, stop the loop.
        elif event.type == pygame.QUIT:
            running = False
