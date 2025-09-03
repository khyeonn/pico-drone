import serial
import math
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *


ser = serial.Serial('COM3', 115200, timeout=1)

vertices = [
    [-0.75, -0.5, -0.25],
    [0.75, -0.5, -0.25],
    [0.75, 0.5, -0.25],
    [-0.75, 0.5, -0.25],
    [-0.75, -0.5, 0.25],
    [0.75, -0.5, 0.25],
    [0.75, 0.5, 0.25],
    [-0.75, 0.5, 0.25]
]

edges = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7)
]


def draw_cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()


def read_serial():
    line = ser.readline()
    if line:
        data = line.decode('utf-8').strip().split()
        values = [val.split('=')[1] for val in data][0:6]
        if len(values) == 6:
            try:
                ax, ay, az, gx, gy, gz = map(float, values)
                return ax, ay, az, gx, gy, gz
            except ValueError:
                return None
    return None


def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    while True:
        ax, ay, az, gx, gy, gz = read_serial()
        if ax is not None:
            # Calculate pitch and roll from accelerometer data
            roll = math.atan2(ay, az)
            pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

            # Clear the screen and apply transformations
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glPushMatrix()

            # Apply rotations based on pitch and roll
            glRotatef(pitch * 180.0 / math.pi, 1.0, 0.0, 0.0)  # Pitch rotation
            glRotatef(roll * 180.0 / math.pi, 0.0, 1.0, 0.0)   # Roll rotation

            # Draw the cube
            draw_cube()

            glPopMatrix()
            pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return


if __name__ == "__main__":  
    main()