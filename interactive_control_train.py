#!/usr/bin/env python
"""Interactive control for the car"""
import time
import io
import pygame
import pygame.font
import pygame.camera
import pygame.image
import sys
import configuration
"""
import helpers.motor_driver as motor_driver_helper
import helpers.image as image_helper
"""
UP = LEFT = DOWN = RIGHT = ACCELERATE = DECELERATE = False

def get_keys():
    """Returns a tuple of (UP, DOWN, LEFT, RIGHT, change, ACCELERATE,
    DECELERATE, stop) representing which keys are UP or DOWN and
    whether or not the key states changed.
    """
    change = False
    stop = False
    key_to_global_name = {
        pygame.K_LEFT: 'LEFT',
        pygame.K_RIGHT: 'RIGHT',
        pygame.K_UP: 'UP',
        pygame.K_DOWN: 'DOWN',
        pygame.K_ESCAPE: 'QUIT',
        pygame.K_q: 'QUIT',
        pygame.K_w: 'ACCELERATE',
        pygame.K_s: 'DECELERATE'
    }
    for event in pygame.event.get():
        if event.type in {pygame.K_q, pygame.K_ESCAPE}:
            stop = True
        elif event.type in {pygame.KEYDOWN, pygame.KEYUP}:
            down = (event.type == pygame.KEYDOWN)
            change = (event.key in key_to_global_name)
            if event.key in key_to_global_name:
                globals()[key_to_global_name[event.key]] = down
    return (UP, DOWN, LEFT, RIGHT, change, ACCELERATE, DECELERATE, stop)


def interactive_control():
	command = 'idle'
	duty_cycle = configuration.INITIAL_PWM_DUTY_CYCLE
	while True:
		up_key, down, left, right, change, accelerate, decelerate, stop = get_keys()
		if stop:
			break
		if accelerate:
			duty_cycle = duty_cycle + 3 if (duty_cycle + 3) <= 100 else duty_cycle
			motor_driver_helper.change_pwm_duty_cycle(pwm, duty_cycle)
			print("speed: " + str(duty_cycle))
		if decelerate:
			duty_cycle = duty_cycle - 3 if (duty_cycle - 3) >= 0 else duty_cycle
			motor_driver_helper.change_pwm_duty_cycle(pwm, duty_cycle)
			print("speed: " + str(duty_cycle))
		if change:
			command = 'idle'
			motor_driver_helper.set_idle_mode()
			if up_key:
				command = 'forward'
				print(duty_cycle)
				motor_driver_helper.set_forward_mode()
			elif down:
				command = 'reverse'
				motor_driver_helper.set_reverse_mode()

			append = lambda x: command + '_' + x if command != 'idle' else x

			if left:
				command = append('left')
				motor_driver_helper.set_left_mode()
			elif right:
				command = append('right')
				motor_driver_helper.set_right_mode()
		print(command)
		stream = io.BytesIO()
		camera.capture(stream, format='jpeg', use_video_port=True)
		image_helper.save_image_with_direction(stream, command)
		stream.flush()

		clock.tick(30)
	pygame.quit()

def setup_interactive_control():
    """Setup the Pygame Interactive Control Screen"""
    pygame.init()
def convert_to_gs(surf):
    width, height = surf.get_size()
    for x in range(width):
        for y in range(height):
            red, green, blue, alpha = surf.get_at((x, y))
            average = (red + green + blue) // 3
            gs_color = (average, average, average, alpha)
            surf.set_at((x, y), gs_color)
def main():
	setup_interactive_control()
	pygame.camera.init()

	cameras = pygame.camera.list_cameras()

	print "Using camera %s ..." % cameras[0]

	webcam = pygame.camera.Camera(cameras[0], (480,320) , "RGB")
	webcam.start()

	# grab first frame
	img = webcam.get_image()

	WIDTH = img.get_width()
	HEIGHT = img.get_height()

	screen = pygame.display.set_mode( ( WIDTH, HEIGHT ) )
	pygame.display.set_caption("pyGame Camera View")
   	while True :
		for e in pygame.event.get() :
			if e.type == pygame.QUIT :
				sys.exit()
		# draw frame
		screen.blit(img, (0,0))
		pygame.display.flip()
		# grab next frame    
		img = webcam.get_image()
		convert_to_gs(img)


if __name__ == '__main__':
    main()
